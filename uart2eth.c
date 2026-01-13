#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <errno.h>

/* ================= 寄存器定义 ================= */

#define REG_BASE_ADDR        0x40000000
#define REG_MAP_SIZE         0x1000

#define REG_DDR_ADDR_OFF     0x00
#define REG_DDR_SIZE_OFF     0x04
#define REG_TRIG_OFF         0x08

/* ================= 安全限制 ================= */

#define DDR_MAX_SIZE         (2 * 1024 * 1024)

/* ================= TCP 配置 ================= */

#define DEST_IP              "192.168.3.121"
#define DEST_PORT            9875
#define TCP_PKT_SIZE         1024

/* ================================================= */

static int tcp_send_all(int sockfd, const uint8_t *buf, size_t len)
{
    size_t sent = 0;

    while (sent < len) {
        ssize_t ret = send(sockfd, buf + sent, len - sent, 0);
        if (ret < 0) {
            perror("[ERROR] send");
            return -1;
        }
        if (ret == 0) {
            printf("[ERROR] send returned 0 (peer closed)\n");
            return -1;
        }
        sent += ret;
    }
    return 0;
}

int main(void)
{
    int mem_fd  = -1;
    int sockfd  = -1;

    void *reg_map = NULL;
    void *ddr_map = NULL;

    volatile uint32_t *reg_ddr_addr;
    volatile uint32_t *reg_ddr_size;
    volatile uint32_t *reg_trig;

    uint8_t  *ddr_ptr = NULL;
    uint32_t last_trig = 0;
    uint32_t last_ddr_size = 0;

    struct sockaddr_in dest_addr;

    /* 打开 /dev/mem */
    mem_fd = open("/dev/mem", O_RDONLY | O_SYNC);
    if (mem_fd < 0) {
        perror("open /dev/mem");
        goto cleanup;
    }

    /* 映射寄存器 */
    reg_map = mmap(NULL,
                   REG_MAP_SIZE,
                   PROT_READ,
                   MAP_SHARED,
                   mem_fd,
                   REG_BASE_ADDR);
    if (reg_map == MAP_FAILED) {
        perror("mmap reg");
        goto cleanup;
    }

    reg_ddr_addr = (volatile uint32_t *)((uint8_t *)reg_map + REG_DDR_ADDR_OFF);
    reg_ddr_size = (volatile uint32_t *)((uint8_t *)reg_map + REG_DDR_SIZE_OFF);
    reg_trig     = (volatile uint32_t *)((uint8_t *)reg_map + REG_TRIG_OFF);

    /* 创建 TCP socket */
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) {
        perror("socket");
        goto cleanup;
    }

    memset(&dest_addr, 0, sizeof(dest_addr));
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port   = htons(DEST_PORT);
    inet_pton(AF_INET, DEST_IP, &dest_addr.sin_addr);

    printf("========================================\n");
    printf(" UART2ETH DDR -> TCP Sender Started\n");
    printf(" REG_BASE     : 0x%08X\n", REG_BASE_ADDR);
    printf(" TRIG_REG     : 0x%08X\n", REG_BASE_ADDR + REG_TRIG_OFF);
    printf(" DEST         : %s:%d\n", DEST_IP, DEST_PORT);
    printf("========================================\n");

    printf("[INFO] Connecting to TCP server...\n");
    if (connect(sockfd,
                (struct sockaddr *)&dest_addr,
                sizeof(dest_addr)) < 0) {
        perror("connect");
        goto cleanup;
    }
    printf("[INFO] TCP connected OK\n");
    printf("[INFO] Waiting for trigger...\n");

    /* ================= 主循环 ================= */

    while (1) {

        uint32_t cur_trig = *reg_trig;

        /* 0 -> 1 上升沿触发 */
        if (last_trig == 0 && cur_trig == 1) {

            uint32_t ddr_phys_addr  = *reg_ddr_addr;
            uint32_t ddr_total_size = *reg_ddr_size;

            printf("[TRIGGER] addr=0x%08X size=%u\n",
                   ddr_phys_addr, ddr_total_size);

            if (ddr_phys_addr == 0 ||
                ddr_total_size == 0 ||
                ddr_total_size > DDR_MAX_SIZE ||
                (ddr_phys_addr & 0xFFF)) {

                printf("[WARN] Invalid DDR params, ignore trigger\n");
                last_trig = cur_trig;
                usleep(1000);
                continue;
            }

            if (ddr_map && ddr_map != MAP_FAILED) {
                munmap(ddr_map, last_ddr_size);
                ddr_map = NULL;
                ddr_ptr = NULL;
            }

            ddr_map = mmap(NULL,
                           ddr_total_size,
                           PROT_READ,
                           MAP_SHARED,
                           mem_fd,
                           ddr_phys_addr);
            if (ddr_map == MAP_FAILED) {
                perror("mmap ddr");
                last_trig = cur_trig;
                usleep(1000);
                continue;
            }

            ddr_ptr = (uint8_t *)ddr_map;
            last_ddr_size = ddr_total_size;

            printf("[SEND] TCP sending %u bytes...\n", ddr_total_size);

            size_t offset = 0;
            unsigned int pkt_cnt = 0;

            while (offset < ddr_total_size) {

                size_t send_len = TCP_PKT_SIZE;
                if (offset + send_len > ddr_total_size)
                    send_len = ddr_total_size - offset;

                if (tcp_send_all(sockfd,
                                 ddr_ptr + offset,
                                 send_len) < 0) {
                    printf("[ERROR] TCP send failed\n");
                    break;
                }

                offset += send_len;
                pkt_cnt++;
            }

            printf("[DONE] packets=%u total_bytes=%zu\n",
                   pkt_cnt, offset);
        }

        last_trig = cur_trig;
        usleep(1000);
    }

cleanup:
    if (ddr_map && ddr_map != MAP_FAILED)
        munmap(ddr_map, last_ddr_size);
    if (reg_map && reg_map != MAP_FAILED)
        munmap(reg_map, REG_MAP_SIZE);
    if (sockfd >= 0)
        close(sockfd);
    if (mem_fd >= 0)
        close(mem_fd);

    return 0;
}
