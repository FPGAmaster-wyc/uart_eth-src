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
#include <sys/stat.h>

/* ================= TCP 配置 ================= */

#define TCP_LISTEN_PORT   9876
#define TCP_RECV_SIZE     4096

/* ================= FPGA 寄存器 ================= */

#define REG_BASE_ADDR     0x40000000
#define REG_MAP_SIZE      0x1000

/* ================= 多路配置 ================= */

#define NUM_CHANNELS      5

#define SPECIAL_HEAD_06   0xFAFA0006   /* TCP → 文件 */
#define SPECIAL_HEAD_07   0xFAFA0007   /* 文件 → TCP */

#define LOG_DIR           "/run/media/mmcblk0p1/logs"

typedef struct {
    uint8_t  frame_head[4];
    uint32_t rx_addr_off;
    uint32_t rx_len_off;
    uint32_t rx_trig_off;
} rx_channel_cfg_t;

static rx_channel_cfg_t g_rx_cfg[NUM_CHANNELS] = {
    { {0xFA,0xFA,0x00,0x01}, 0x00, 0x04, 0x08 },
    { {0xFA,0xFA,0x00,0x02}, 0x0C, 0x10, 0x14 },
    { {0xFA,0xFA,0x00,0x03}, 0x18, 0x1C, 0x20 },
    { {0xFA,0xFA,0x00,0x04}, 0x24, 0x28, 0x2C },
    { {0xFA,0xFA,0x00,0x05}, 0x30, 0x34, 0x38 },
};

/* ================= 工具函数 ================= */

static int recv_all(int fd, uint8_t *buf, size_t len)
{
    size_t got = 0;
    while (got < len) {
        ssize_t r = recv(fd, buf + got, len - got, 0);
        if (r == 0) return -1;            /* 对端断开 */
        if (r < 0) {
            if (errno == EINTR) continue;
            perror("recv_all");
            return -1;
        }
        got += r;
    }
    return 0;
}

static uint32_t be32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) |
           ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] <<  8) |
           ((uint32_t)p[3]);
}

static int find_channel_by_head(const uint8_t *hdr4)
{
    for (int i = 0; i < NUM_CHANNELS; i++) {
        if (memcmp(hdr4, g_rx_cfg[i].frame_head, 4) == 0)
            return i;
    }
    return -1;
}

/* ================= FAFA0006：写文件 ================= */

static int save_bin_file(uint32_t addr, uint32_t len, int sock_fd)
{
    mkdir(LOG_DIR, 0755);

    char path[256];
    snprintf(path, sizeof(path), LOG_DIR "/0x%08X.bin", addr);

    int fd = open(path, O_CREAT | O_WRONLY | O_TRUNC, 0644);
    if (fd < 0) {
        perror("open bin");
        return -1;
    }

    uint8_t buf[TCP_RECV_SIZE];
    uint32_t left = len;

    while (left > 0) {
        ssize_t n = recv(sock_fd, buf,
                         left > TCP_RECV_SIZE ? TCP_RECV_SIZE : left, 0);
        if (n <= 0) {
            perror("recv bin");
            close(fd);
            return -1;
        }
        write(fd, buf, n);
        left -= n;
    }

    fsync(fd);
    close(fd);

    printf("[BIN][SAVE] %u bytes -> %s\n", len, path);
    return 0;
}

/* ================= FAFA0007：读文件并补 0 发送 ================= */

static int send_bin_file(uint32_t addr, uint32_t len, int sock_fd)
{
    char path[256];
    snprintf(path, sizeof(path), LOG_DIR "/0x%08X.bin", addr);

    int fd = open(path, O_RDONLY);
    if (fd < 0) {
        perror("open bin send");
        return -1;
    }

    struct stat st;
    fstat(fd, &st);

    uint32_t file_size = st.st_size;
    uint8_t buf[TCP_RECV_SIZE];
    uint32_t sent = 0;

    while (sent < file_size && sent < len) {
        ssize_t r = read(fd, buf, sizeof(buf));
        if (r <= 0) break;

        send(sock_fd, buf, r, 0);
        sent += r;
    }
    close(fd);

    memset(buf, 0, sizeof(buf));
    while (sent < len) {
        size_t z = (len - sent > sizeof(buf)) ? sizeof(buf) : (len - sent);
        send(sock_fd, buf, z, 0);
        sent += z;
    }

    printf("[BIN][SEND] addr=0x%08X total=%u\n", addr, len);
    return 0;
}

static int is_valid_head4(const uint8_t h[4])
{
    if (h[0] != 0xFA || h[1] != 0xFA) return 0;
    if (h[2] != 0x00) return 0;

    /* 01~07 都认为合法（其中 06/07 是特殊命令） */
    if (h[3] >= 0x01 && h[3] <= 0x07) return 1;
    return 0;
}

/* 在 TCP 字节流中查找一个“合法帧头的起点”，并读满 12 字节头 */
static int recv_frame_head(int fd, uint8_t hdr[12])
{
    uint8_t win[4] = {0};
    int filled = 0;

    while (1) {
        uint8_t b;
        ssize_t n = recv(fd, &b, 1, 0);
        if (n == 0) return -1; /* disconnect */
        if (n < 0) {
            if (errno == EINTR) continue;
            perror("recv_frame_head recv");
            return -1;
        }

        /* 滑动窗口凑 4 字节 */
        if (filled < 4) {
            win[filled++] = b;
        } else {
            memmove(&win[0], &win[1], 3);
            win[3] = b;
        }

        if (filled == 4 && is_valid_head4(win)) {
            /* 找到合法 head4，把它放进 hdr[0..3] */
            memcpy(hdr, win, 4);

            /* 再把剩余 8 字节头读满 */
            if (recv_all(fd, &hdr[4], 8) < 0)
                return -1;

            return 0;
        }
    }
}


/* ================= main ================= */

int main(void)
{
    int listen_fd, conn_fd, mem_fd;
    struct sockaddr_in addr;
    socklen_t addr_len = sizeof(addr);

    mem_fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (mem_fd < 0) {
        perror("open /dev/mem");
        return 1;
    }

    void *reg_map = mmap(NULL, REG_MAP_SIZE, PROT_READ | PROT_WRITE,
                         MAP_SHARED, mem_fd, REG_BASE_ADDR);
    if (reg_map == MAP_FAILED) {
        perror("mmap reg");
        return 1;
    }

    listen_fd = socket(AF_INET, SOCK_STREAM, 0);
    int opt = 1;
    setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(TCP_LISTEN_PORT);
    addr.sin_addr.s_addr = INADDR_ANY;

    bind(listen_fd, (struct sockaddr *)&addr, sizeof(addr));
    listen(listen_fd, 1);

    printf("[INFO] TCP listen %d\n", TCP_LISTEN_PORT);

re_accept:
    conn_fd = accept(listen_fd, (struct sockaddr *)&addr, &addr_len);
    printf("[INFO] TCP connected\n");

    while (1) {

        uint8_t hdr[12];
        if (recv_frame_head(conn_fd, hdr) < 0) {
			printf("[INFO] TCP disconnected\n");
			close(conn_fd);
			goto re_accept;
		}


        uint32_t head = be32(hdr);
        uint32_t len  = be32(&hdr[4]);
        uint32_t addr_ddr = be32(&hdr[8]);

        if (head == SPECIAL_HEAD_06) {
            if (save_bin_file(addr_ddr, len, conn_fd) < 0)
                printf("[WARN] save bin failed\n");
            continue;
        }

        if (head == SPECIAL_HEAD_07) {
            if (send_bin_file(addr_ddr, len, conn_fd) < 0)
                printf("[WARN] send bin failed\n");
            continue;
        }

        int ch = find_channel_by_head(hdr);
        if (ch < 0) {
            printf("[WARN] Unknown head %02X %02X %02X %02X\n",
                   hdr[0], hdr[1], hdr[2], hdr[3]);
            continue;
        }

        void *ddr_map = mmap(NULL, len, PROT_READ | PROT_WRITE,
                             MAP_SHARED, mem_fd, addr_ddr);
        if (ddr_map == MAP_FAILED) {
            perror("[ERROR] mmap ddr");
            continue;
        }

        uint8_t *p = ddr_map;
        uint32_t left = len;
        uint8_t buf[TCP_RECV_SIZE];

        while (left > 0) {
            ssize_t n = recv(conn_fd, buf,
                             left > sizeof(buf) ? sizeof(buf) : left, 0);
            if (n <= 0) {
                perror("[ERROR] recv payload");
                break;
            }
            memcpy(p, buf, n);
            p += n;
            left -= n;
        }

        munmap(ddr_map, len);

        if (left != 0) {
            printf("[WARN] payload incomplete, drop frame\n");
            continue;
        }

        rx_channel_cfg_t *cfg = &g_rx_cfg[ch];
        volatile uint32_t *ra =
            (uint32_t *)((uint8_t *)reg_map + cfg->rx_addr_off);
        volatile uint32_t *rl =
            (uint32_t *)((uint8_t *)reg_map + cfg->rx_len_off);
        volatile uint32_t *rt =
            (uint32_t *)((uint8_t *)reg_map + cfg->rx_trig_off);

        *rl = len;
        *ra = addr_ddr;
        *rt = 0;
        usleep(1);
        *rt = 1;
    }
}
