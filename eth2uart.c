#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <fcntl.h>
#include <sys/mman.h>

/* ================= TCP 配置 ================= */

#define TCP_LISTEN_PORT   9876
#define TCP_RECV_SIZE    4096    /* 网络接收块大小 */

/* ================= 帧头 ================= */

static const uint8_t FRAME_HEAD[4] = {0xFA, 0xFA, 0x00, 0x01};

/* ================= FPGA 寄存器 ================= */

#define REG_BASE_ADDR    0x40000000
#define REG_MAP_SIZE     0x1000

#define RX_ADDR_OFF      0x0C
#define RX_LEN_OFF       0x10
#define RX_TRIG_OFF      0x14

/* ================================================= */

static int recv_all(int fd, uint8_t *buf, size_t len)
{
    size_t got = 0;
    while (got < len) {
        ssize_t ret = recv(fd, buf + got, len - got, 0);
        if (ret <= 0)
            return -1;
        got += ret;
    }
    return 0;
}

int main(void)
{
    int listen_fd = -1;
    int conn_fd   = -1;
    int mem_fd    = -1;

    struct sockaddr_in local_addr;
    socklen_t addr_len = sizeof(local_addr);

    /* FPGA 寄存器 */
    void *reg_map = NULL;
    volatile uint32_t *reg_rx_addr;
    volatile uint32_t *reg_rx_len;
    volatile uint32_t *reg_rx_trig;

    /* DDR */
    void *ddr_map = NULL;
    uint8_t *ddr_ptr;

    /* 网络缓冲 */
    uint8_t hdr[12];
    uint8_t net_buf[TCP_RECV_SIZE];

    /* ================= 打开 /dev/mem ================= */

    mem_fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (mem_fd < 0) {
        perror("open /dev/mem");
        goto cleanup;
    }

    reg_map = mmap(NULL,
                   REG_MAP_SIZE,
                   PROT_READ | PROT_WRITE,
                   MAP_SHARED,
                   mem_fd,
                   REG_BASE_ADDR);
    if (reg_map == MAP_FAILED) {
        perror("mmap reg");
        goto cleanup;
    }

    reg_rx_addr = (volatile uint32_t *)((uint8_t *)reg_map + RX_ADDR_OFF);
    reg_rx_len  = (volatile uint32_t *)((uint8_t *)reg_map + RX_LEN_OFF);
    reg_rx_trig = (volatile uint32_t *)((uint8_t *)reg_map + RX_TRIG_OFF);

    /* ================= TCP Server ================= */

    listen_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (listen_fd < 0) {
        perror("socket");
        goto cleanup;
    }

    memset(&local_addr, 0, sizeof(local_addr));
    local_addr.sin_family = AF_INET;
    local_addr.sin_port   = htons(TCP_LISTEN_PORT);
    local_addr.sin_addr.s_addr = INADDR_ANY;

    if (bind(listen_fd,
             (struct sockaddr *)&local_addr,
             sizeof(local_addr)) < 0) {
        perror("bind");
        goto cleanup;
    }

    if (listen(listen_fd, 1) < 0) {
        perror("listen");
        goto cleanup;
    }

    printf("========================================\n");
    printf(" TCP → DDR → FPGA (Streaming)\n");
    printf(" Port : %d\n", TCP_LISTEN_PORT);
    printf("========================================\n");
    printf("[INFO] Waiting for TCP connection...\n");

    conn_fd = accept(listen_fd,
                     (struct sockaddr *)&local_addr,
                     &addr_len);
    if (conn_fd < 0) {
        perror("accept");
        goto cleanup;
    }

    printf("[INFO] TCP client connected\n");

    /* ================= 主循环：一帧一帧处理 ================= */

    while (1) {

        /* ---------- 1. 接收帧头 ---------- */
        if (recv_all(conn_fd, hdr, 12) < 0) {
            printf("[WARN] TCP closed while reading header\n");
            break;
        }

        if (memcmp(hdr, FRAME_HEAD, 4) != 0) {
            printf("[ERROR] Invalid frame head\n");
            break;
        }

        uint32_t len =
            (hdr[4] << 24) | (hdr[5] << 16) |
            (hdr[6] << 8)  |  hdr[7];

        uint32_t addr =
            (hdr[8] << 24) | (hdr[9] << 16) |
            (hdr[10] << 8) |  hdr[11];

        printf("[FRAME] len=%u (%.2f MB), addr=0x%08X\n",
               len, len / 1024.0 / 1024.0, addr);

        /* ---------- 2. mmap DDR ---------- */
        ddr_map = mmap(NULL,
                       len,
                       PROT_READ | PROT_WRITE,
                       MAP_SHARED,
                       mem_fd,
                       addr);
        if (ddr_map == MAP_FAILED) {
            perror("mmap ddr");
            break;
        }

        ddr_ptr = (uint8_t *)ddr_map;

        /* ---------- 3. 流式接收 payload 并写 DDR ---------- */
        uint32_t left = len;
        uint32_t written = 0;

        while (left > 0) {
            ssize_t n = recv(conn_fd,
                             net_buf,
                             left > TCP_RECV_SIZE ?
                             TCP_RECV_SIZE : left,
                             0);
            if (n <= 0) {
                printf("[ERROR] TCP closed during payload\n");
                goto cleanup;
            }

            memcpy(ddr_ptr, net_buf, n);
            ddr_ptr += n;
            left    -= n;
            written += n;
        }

        munmap(ddr_map, len);
        ddr_map = NULL;

        printf("[DDR] write done: %u bytes\n", written);

        /* ---------- 4. 完整帧完成后再通知 FPGA ---------- */
        *reg_rx_len  = len;
        *reg_rx_addr = addr;

        *reg_rx_trig = 0;
        usleep(1);
        *reg_rx_trig = 1;

        printf("[FPGA] TRIG=1 LEN=0x%08X ADDR=0x%08X\n",
               len, addr);
    }

cleanup:
    if (ddr_map && ddr_map != MAP_FAILED)
        munmap(ddr_map, 0);
    if (reg_map && reg_map != MAP_FAILED)
        munmap(reg_map, REG_MAP_SIZE);
    if (conn_fd >= 0)
        close(conn_fd);
    if (listen_fd >= 0)
        close(listen_fd);
    if (mem_fd >= 0)
        close(mem_fd);

    return 0;
}
