#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <fcntl.h>
#include <sys/mman.h>

/* ================= UDP 配置 ================= */

#define UDP_LISTEN_PORT   9876
#define UDP_BUF_SIZE     2048

/* ================= 帧头 ================= */

/* FA FA 00 01 */
static const uint8_t FRAME_HEAD[4] = {0xFA, 0xFA, 0x00, 0x01};

/* ================= 缓存 ================= */

#define CACHE_BUF_SIZE   8192

/* ================= FPGA 寄存器 ================= */

#define REG_BASE_ADDR    0x40000000
#define REG_MAP_SIZE     0x1000

#define RX_ADDR_OFF      0x0C   /* 地址 */
#define RX_LEN_OFF       0x10   /* 长度 */
#define RX_TRIG_OFF      0x14   /* 触发 */

/* ================================================= */

int main(void)
{
    int sockfd;
    int mem_fd;

    uint8_t udp_buf[UDP_BUF_SIZE];
    uint8_t cache_buf[CACHE_BUF_SIZE];
    size_t  cache_len = 0;

    struct sockaddr_in local_addr;
    socklen_t addr_len = sizeof(local_addr);

    void *reg_map = NULL;
    volatile uint32_t *reg_rx_addr;
    volatile uint32_t *reg_rx_len;
    volatile uint32_t *reg_rx_trig;

    void *ddr_map = NULL;

    /* ================= 打开 /dev/mem ================= */

    mem_fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (mem_fd < 0) {
        perror("open /dev/mem");
        return -1;
    }

    /* 映射 FPGA 寄存器 */
    reg_map = mmap(NULL,
                   REG_MAP_SIZE,
                   PROT_READ | PROT_WRITE,
                   MAP_SHARED,
                   mem_fd,
                   REG_BASE_ADDR);
    if (reg_map == MAP_FAILED) {
        perror("mmap reg");
        close(mem_fd);
        return -1;
    }

    reg_rx_addr = (volatile uint32_t *)((uint8_t *)reg_map + RX_ADDR_OFF);
    reg_rx_len  = (volatile uint32_t *)((uint8_t *)reg_map + RX_LEN_OFF);
    reg_rx_trig = (volatile uint32_t *)((uint8_t *)reg_map + RX_TRIG_OFF);

    /* ================= UDP socket ================= */

    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        perror("socket");
        return -1;
    }

    memset(&local_addr, 0, sizeof(local_addr));
    local_addr.sin_family = AF_INET;
    local_addr.sin_port   = htons(UDP_LISTEN_PORT);
    local_addr.sin_addr.s_addr = INADDR_ANY;

    if (bind(sockfd,
             (struct sockaddr *)&local_addr,
             sizeof(local_addr)) < 0) {
        perror("bind");
        close(sockfd);
        return -1;
    }

    printf("========================================\n");
    printf(" UDP → DDR → FPGA Sender\n");
    printf(" Port      : %d\n", UDP_LISTEN_PORT);
    printf(" FrameHead : FA FA 00 01\n");
    printf("========================================\n");
	
    /* ================= 主循环 ================= */

    while (1) {
        ssize_t recv_len = recvfrom(sockfd,
                                    udp_buf,
                                    UDP_BUF_SIZE,
                                    0,
                                    (struct sockaddr *)&local_addr,
                                    &addr_len);
        if (recv_len <= 0)
            continue;

        /* 追加到缓存 */
        if (cache_len + recv_len > CACHE_BUF_SIZE) {
            cache_len = 0;
            continue;
        }

        memcpy(cache_buf + cache_len, udp_buf, recv_len);
        cache_len += recv_len;

        size_t i = 0;
        while (i + 12 <= cache_len) {

            /* 查找帧头 */
            if (memcmp(&cache_buf[i], FRAME_HEAD, 4) != 0) {
                i++;
                continue;
            }

            /* 大端解析 LEN */
            uint32_t len =
                (cache_buf[i + 4] << 24) |
                (cache_buf[i + 5] << 16) |
                (cache_buf[i + 6] << 8)  |
                cache_buf[i + 7];

            /* 大端解析 ADDR */
            uint32_t addr =
                (cache_buf[i + 8]  << 24) |
                (cache_buf[i + 9]  << 16) |
                (cache_buf[i + 10] << 8)  |
                cache_buf[i + 11];

            printf("[FRAME] len=%u (0x%08X), addr=0x%08X\n",
                   len, len, addr);

            /* payload 是否完整 */
            if (i + 12 + len > cache_len)
                break;

            /* 映射 DDR */
            ddr_map = mmap(NULL,
                           len,
                           PROT_READ | PROT_WRITE,
                           MAP_SHARED,
                           mem_fd,
                           addr);
            if (ddr_map == MAP_FAILED) {
                perror("mmap ddr");
                i += 12 + len;
                continue;
            }

            /* 写 DDR */
            memcpy(ddr_map, &cache_buf[i + 12], len);
            munmap(ddr_map, len);

            printf("[DDR] write %u bytes OK\n", len);

            /* ========= 通知 FPGA（严格按你要求的顺序） ========= */

            
            *reg_rx_len  = len;   /* 写长度 */
            *reg_rx_addr = addr;  /* 写地址 */
			
			*reg_rx_trig = 1;     /* 先拉高触发 */

            printf("[FPGA] TRIG=1 LEN=0x%08X ADDR=0x%08X\n",
                   len, addr);

            /* 跳过整帧 */
            i += 12 + len;
        }

        /* 清理缓存 */
        if (i > 0) {
            memmove(cache_buf, cache_buf + i, cache_len - i);
            cache_len -= i;
        }
    }

    close(sockfd);
    munmap(reg_map, REG_MAP_SIZE);
    close(mem_fd);
    return 0;
}
