#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <errno.h>

/* ================= TCP 配置 ================= */

#define TCP_LISTEN_PORT   9876
#define TCP_RECV_SIZE     4096    /* 网络接收块大小 */

/* ================= FPGA 寄存器 ================= */

#define REG_BASE_ADDR     0x40000000
#define REG_MAP_SIZE      0x1000

/* ================= 多路配置 ================= */

#define NUM_CHANNELS      5

typedef struct {
    uint8_t  frame_head[4];   /* 该通道的帧头 */
    uint32_t rx_addr_off;     /* RX_ADDR 寄存器偏移 */
    uint32_t rx_len_off;      /* RX_LEN  寄存器偏移 */
    uint32_t rx_trig_off;     /* RX_TRIG 寄存器偏移 */
} rx_channel_cfg_t;

/* TODO: 按你的实际协议/寄存器映射修改这里 */
static rx_channel_cfg_t g_rx_cfg[NUM_CHANNELS] = {
    /*   frame_head                addr   len    trig */
    { {0xFA, 0xFA, 0x00, 0x01},  0x00,  0x04,  0x08 },  /* CH0 */
    { {0xFA, 0xFA, 0x00, 0x02},  0x0C,  0x10,  0x14 },  /* CH1 */
    { {0xFA, 0xFA, 0x00, 0x03},  0x18,  0x1C,  0x20 },  /* CH2 */
    { {0xFA, 0xFA, 0x00, 0x04},  0x24,  0x28,  0x2C },  /* CH3 */
    { {0xFA, 0xFA, 0x00, 0x05},  0x30,  0x34,  0x38 },  /* CH4 */
};

/* ================================================= */

static int recv_all(int fd, uint8_t *buf, size_t len)
{
    size_t got = 0;
    while (got < len) {
        ssize_t ret = recv(fd, buf + got, len - got, 0);
        if (ret == 0) return -1;           /* peer closed */
        if (ret < 0) {
            if (errno == EINTR) continue;
            return -1;
        }
        got += (size_t)ret;
    }
    return 0;
}

static int find_channel_by_head(const uint8_t *hdr4)
{
    for (int i = 0; i < NUM_CHANNELS; i++) {
        if (memcmp(hdr4, g_rx_cfg[i].frame_head, 4) == 0)
            return i;
    }
    return -1;
}

static uint32_t be32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) |
           ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] <<  8) |
           ((uint32_t)p[3] <<  0);
}

int main(void)
{
    int listen_fd = -1;
    int conn_fd   = -1;
    int mem_fd    = -1;

    struct sockaddr_in local_addr;
    socklen_t addr_len = sizeof(local_addr);

    /* FPGA 寄存器映射 */
    void *reg_map = NULL;

    /* DDR 映射 */
    void   *ddr_map = NULL;
    size_t  ddr_map_len = 0;

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
        reg_map = NULL;
        goto cleanup;
    }

    /* ================= TCP Server ================= */

    listen_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (listen_fd < 0) {
        perror("socket");
        goto cleanup;
    }

    int opt = 1;
    setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

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
    printf(" TCP → DDR → FPGA (5-Channel, Configurable)\n");
    printf(" Port : %d\n", TCP_LISTEN_PORT);
    printf(" Channels: %d\n", NUM_CHANNELS);
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

        int ch = find_channel_by_head(hdr);
        if (ch < 0) {
            printf("[ERROR] Unknown frame head: %02X %02X %02X %02X\n",
                   hdr[0], hdr[1], hdr[2], hdr[3]);
            break;
        }

        rx_channel_cfg_t *cfg = &g_rx_cfg[ch];

        uint32_t len  = be32(&hdr[4]);
        uint32_t addr = be32(&hdr[8]);

        printf("[FRAME][CH%d] len=%u (%.2f MB), addr=0x%08X\n",
               ch, len, len / 1024.0 / 1024.0, addr);

        if (len == 0) {
            printf("[ERROR] len=0\n");
            break;
        }

        /* ---------- 2. mmap DDR ---------- */
        ddr_map_len = (size_t)len;
        ddr_map = mmap(NULL,
                       ddr_map_len,
                       PROT_READ | PROT_WRITE,
                       MAP_SHARED,
                       mem_fd,
                       (off_t)addr);
        if (ddr_map == MAP_FAILED) {
            perror("mmap ddr");
            ddr_map = NULL;
            ddr_map_len = 0;
            break;
        }

        uint8_t *ddr_ptr = (uint8_t *)ddr_map;

        /* ---------- 3. 流式接收 payload 并写 DDR ---------- */
        uint32_t left = len;
        uint32_t written = 0;

        while (left > 0) {
            size_t to_read = (left > TCP_RECV_SIZE) ? TCP_RECV_SIZE : left;

            ssize_t n = recv(conn_fd, net_buf, to_read, 0);
            if (n == 0) {
                printf("[ERROR] TCP closed during payload\n");
                goto cleanup;
            }
            if (n < 0) {
                if (errno == EINTR) continue;
                perror("recv payload");
                goto cleanup;
            }

            memcpy(ddr_ptr, net_buf, (size_t)n);
            ddr_ptr += (size_t)n;
            left    -= (uint32_t)n;
            written += (uint32_t)n;
        }

        munmap(ddr_map, ddr_map_len);
        ddr_map = NULL;
        ddr_map_len = 0;

        printf("[DDR][CH%d] write done: %u bytes\n", ch, written);

        /* ---------- 4. 完整帧完成后再通知 FPGA ---------- */
        volatile uint32_t *reg_rx_addr =
            (volatile uint32_t *)((uint8_t *)reg_map + cfg->rx_addr_off);
        volatile uint32_t *reg_rx_len  =
            (volatile uint32_t *)((uint8_t *)reg_map + cfg->rx_len_off);
        volatile uint32_t *reg_rx_trig =
            (volatile uint32_t *)((uint8_t *)reg_map + cfg->rx_trig_off);

        *reg_rx_len  = len;
        *reg_rx_addr = addr;

        *reg_rx_trig = 0;
        usleep(1);
        *reg_rx_trig = 1;

        printf("[FPGA][CH%d] TRIG=1 LEN=0x%08X ADDR=0x%08X (off: addr=0x%X len=0x%X trig=0x%X)\n",
               ch, len, addr,
               cfg->rx_addr_off, cfg->rx_len_off, cfg->rx_trig_off);
    }

cleanup:
    if (ddr_map)
        munmap(ddr_map, ddr_map_len);

    if (reg_map)
        munmap(reg_map, REG_MAP_SIZE);

    if (conn_fd >= 0)
        close(conn_fd);

    if (listen_fd >= 0)
        close(listen_fd);

    if (mem_fd >= 0)
        close(mem_fd);

    return 0;
}
