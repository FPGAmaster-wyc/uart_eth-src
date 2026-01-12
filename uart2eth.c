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

#define REG_DDR_ADDR_OFF     0x00    /* DDR 起始物理地址 */
#define REG_DDR_SIZE_OFF     0x04    /* DDR 数据长度（字节） */
#define REG_TRIG_OFF         0x08    /* 触发寄存器 */

/* ================= 安全限制 ================= */

#define DDR_MAX_SIZE         (1024 * 1024)   /* 最大 1MB */

/* ================= UDP 配置 ================= */

#define DEST_IP              "192.168.3.121"
#define DEST_PORT            9875
#define UDP_PKT_SIZE         1024

/* ================================================= */

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
    reg_trig     = (volatile uint32_t *)((uint8_t *)reg_map + REG_TRIG_OFF);
    reg_ddr_size = (volatile uint32_t *)((uint8_t *)reg_map + REG_DDR_SIZE_OFF);

    /* 创建 UDP socket */
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        perror("socket");
        goto cleanup;
    }

    memset(&dest_addr, 0, sizeof(dest_addr));
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port   = htons(DEST_PORT);
    inet_pton(AF_INET, DEST_IP, &dest_addr.sin_addr);

    printf("========================================\n");
    printf(" UART2ETH DDR -> UDP Sender Started\n");
    printf(" REG_BASE     : 0x%08X\n", REG_BASE_ADDR);
    printf(" TRIG_REG     : 0x%08X\n", REG_BASE_ADDR + REG_TRIG_OFF);
    printf(" DEST         : %s:%d\n", DEST_IP, DEST_PORT);
    printf("========================================\n");
    printf("[INFO] Waiting for trigger...\n");

    /* ================= 主循环 ================= */

    while (1) {

        uint32_t cur_trig = *reg_trig;

        /* 检测 0 -> 1 上升沿 */
        if (last_trig == 0 && cur_trig == 1) {

            /* 触发后读取 DDR 参数 */
            uint32_t ddr_phys_addr  = *reg_ddr_addr;
            uint32_t ddr_total_size = *reg_ddr_size;

            printf("[TRIGGER] addr=0x%08X size=%u\n",
                   ddr_phys_addr, ddr_total_size);

            /* 参数合法性检查 */
            if (ddr_phys_addr == 0 ||
                ddr_total_size == 0 ||
                ddr_total_size > DDR_MAX_SIZE ||
                (ddr_phys_addr & 0xFFF)) {

                printf("[WARN] Invalid DDR params, ignore this trigger\n");
                last_trig = cur_trig;
                usleep(1000);
                continue;
            }

            /* 如果 DDR 映射已存在，先释放 */
            if (ddr_map && ddr_map != MAP_FAILED) {
                munmap(ddr_map, last_ddr_size);
                ddr_map = NULL;
                ddr_ptr = NULL;
            }

            /* 映射新的 DDR 区域 */
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

            /* 发送 DDR 数据 */
            size_t offset = 0;
            unsigned int pkt_cnt = 0;

            while (offset < ddr_total_size) {
                size_t send_len = UDP_PKT_SIZE;
                if (offset + send_len > ddr_total_size)
                    send_len = ddr_total_size - offset;

                ssize_t ret = sendto(sockfd,
                                     ddr_ptr + offset,
                                     send_len,
                                     0,
                                     (struct sockaddr *)&dest_addr,
                                     sizeof(dest_addr));
                if (ret < 0) {
                    perror("sendto");
                    break;
                }

                offset += send_len;
                pkt_cnt++;

                usleep(1000);   /* 防止 UDP 丢包 */
            }

            printf("[DONE] packets=%u\n", pkt_cnt);
        }

        last_trig = cur_trig;
        usleep(1000);   /* 1ms 轮询 */
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
