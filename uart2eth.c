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

/* ================= 基本配置 ================= */

#define NUM_CHANNELS      5
#define FRAME_HEAD_LEN    4

#define REG_BASE_ADDR     0x40000000
#define REG_MAP_SIZE      0x1000

#define DDR_MAX_SIZE      (2 * 1024 * 1024)

#define DEST_IP           "192.168.3.121"
#define DEST_PORT         9875
#define TCP_PKT_SIZE      1024

/* trig 事件 FIFO 深度 */
#define TRIG_QUEUE_SIZE  32

/* ================= 通道配置结构 ================= */

typedef struct {
    uint8_t  frame_head[FRAME_HEAD_LEN]; /* 当前不使用，仅保留 */
    uint32_t tx_addr_off;
    uint32_t tx_len_off;
    uint32_t tx_trig_off;
} tx_channel_cfg_t;

/* ================= 5 路通道配置 ================= */

static tx_channel_cfg_t g_tx_cfg[NUM_CHANNELS] = {
    { {0xFA, 0xFA, 0x00, 0x01},  0x3C,  0x40,  0x44 },  /* CH0 */
    { {0xFA, 0xFA, 0x00, 0x02},  0x48,  0x4C,  0x50 },  /* CH1 */
    { {0xFA, 0xFA, 0x00, 0x03},  0x54,  0x58,  0x5C },  /* CH2 */
    { {0xFA, 0xFA, 0x00, 0x04},  0x60,  0x64,  0x68 },  /* CH3 */
    { {0xFA, 0xFA, 0x00, 0x05},  0x6C,  0x70,  0x74 },  /* CH4 */
};

/* ================= trig FIFO ================= */

typedef struct {
    int ch[TRIG_QUEUE_SIZE];
    int head;
    int tail;
    int count;
} trig_queue_t;

static trig_queue_t g_trig_q;

static int trig_queue_push(trig_queue_t *q, int ch)
{
    if (q->count >= TRIG_QUEUE_SIZE)
        return -1;

    q->ch[q->tail] = ch;
    q->tail = (q->tail + 1) % TRIG_QUEUE_SIZE;
    q->count++;
    return 0;
}

static int trig_queue_pop(trig_queue_t *q, int *ch)
{
    if (q->count == 0)
        return -1;

    *ch = q->ch[q->head];
    q->head = (q->head + 1) % TRIG_QUEUE_SIZE;
    q->count--;
    return 0;
}

/* ================= TCP 发送 ================= */

static int tcp_send_all(int sockfd, const uint8_t *buf, size_t len)
{
    size_t sent = 0;
    while (sent < len) {
        ssize_t ret = send(sockfd, buf + sent, len - sent, 0);
        if (ret <= 0) {
            perror("send");
            return -1;
        }
        sent += ret;
    }
    return 0;
}

/* ================= 主程序 ================= */

int main(void)
{
    int mem_fd = -1;
    int sockfd = -1;
    void *reg_map = NULL;

    uint32_t last_trig[NUM_CHANNELS] = {0};
    int sending = 0;

    struct sockaddr_in dest_addr;

    /* 打开 /dev/mem */
    mem_fd = open("/dev/mem", O_RDONLY | O_SYNC);
    if (mem_fd < 0) {
        perror("open /dev/mem");
        goto cleanup;
    }

    /* 映射寄存器 */
    reg_map = mmap(NULL, REG_MAP_SIZE,
                   PROT_READ, MAP_SHARED,
                   mem_fd, REG_BASE_ADDR);
    if (reg_map == MAP_FAILED) {
        perror("mmap reg");
        goto cleanup;
    }

    /* TCP socket */
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) {
        perror("socket");
        goto cleanup;
    }

    memset(&dest_addr, 0, sizeof(dest_addr));
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port   = htons(DEST_PORT);
    inet_pton(AF_INET, DEST_IP, &dest_addr.sin_addr);

    if (connect(sockfd,
                (struct sockaddr *)&dest_addr,
                sizeof(dest_addr)) < 0) {
        perror("connect");
        goto cleanup;
    }

    printf("[INFO] UART2ETH TX 5CH FIFO sender started\n");

    /* ================= 主循环 ================= */

    while (1) {

        /* ---------- A. trig 检测（不阻塞） ---------- */
        for (int ch = 0; ch < NUM_CHANNELS; ch++) {

            volatile uint32_t *reg_trig =
                (volatile uint32_t *)((uint8_t *)reg_map +
                                       g_tx_cfg[ch].tx_trig_off);

            uint32_t cur = *reg_trig;

            if (last_trig[ch] == 0 && cur == 1) {
                trig_queue_push(&g_trig_q, ch);
                printf("[EVENT] CH%d trig queued\n", ch);
            }

            last_trig[ch] = cur;
        }

        /* ---------- B. 若空闲则发送一个 ---------- */
        if (!sending && g_trig_q.count > 0) {

            int ch;
            if (trig_queue_pop(&g_trig_q, &ch) == 0) {

                tx_channel_cfg_t *cfg = &g_tx_cfg[ch];

                volatile uint32_t *reg_addr =
                    (volatile uint32_t *)((uint8_t *)reg_map + cfg->tx_addr_off);
                volatile uint32_t *reg_len  =
                    (volatile uint32_t *)((uint8_t *)reg_map + cfg->tx_len_off);

                uint32_t ddr_addr = *reg_addr;
                uint32_t ddr_len  = *reg_len;

                if (ddr_addr == 0 ||
                    ddr_len == 0 ||
                    ddr_len > DDR_MAX_SIZE ||
                    (ddr_addr & 0xFFF)) {
                    printf("[CH%d] invalid DDR params\n", ch);
                    continue;
                }

                sending = 1;

                void *ddr_map = mmap(NULL, ddr_len,
                                     PROT_READ, MAP_SHARED,
                                     mem_fd, ddr_addr);
                if (ddr_map == MAP_FAILED) {
                    perror("mmap ddr");
                    sending = 0;
                    continue;
                }

                uint8_t *ddr_ptr = (uint8_t *)ddr_map;
                size_t offset = 0;

                while (offset < ddr_len) {
                    size_t len = TCP_PKT_SIZE;
                    if (offset + len > ddr_len)
                        len = ddr_len - offset;

                    if (tcp_send_all(sockfd,
                                     ddr_ptr + offset,
                                     len) < 0)
                        break;

                    offset += len;
                }

                printf("[CH%d] SEND DONE (%zu bytes)\n", ch, offset);

                munmap(ddr_map, ddr_len);
                sending = 0;
            }
        }

        usleep(1000);
    }

cleanup:
    if (reg_map && reg_map != MAP_FAILED)
        munmap(reg_map, REG_MAP_SIZE);
    if (sockfd >= 0)
        close(sockfd);
    if (mem_fd >= 0)
        close(mem_fd);
    return 0;
}
