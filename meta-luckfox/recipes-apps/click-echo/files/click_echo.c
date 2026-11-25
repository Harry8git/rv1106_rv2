#include <arpa/inet.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/socket.h>
#include <unistd.h>

int main(void) {
    int s = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in rx = {
        .sin_family = AF_INET, .sin_port = htons(14602), .sin_addr.s_addr = htonl(INADDR_LOOPBACK)
    };
    struct sockaddr_in tx = {
        .sin_family = AF_INET, .sin_port = htons(14601), .sin_addr.s_addr = htonl(INADDR_LOOPBACK)
    };
    if (bind(s, (struct sockaddr *)&rx, sizeof(rx)) < 0) {
        perror("bind 14602");
        return 1;
    }

    printf("[click_echo] Listening on UDP 14602 -> echoing to 14601\n");
    uint8_t xy[4] = {0};
    uint8_t tmp[4];

    while (1) {
        if (recv(s, tmp, 4, MSG_DONTWAIT) == 4) {
            for (int i = 0; i < 4; i++) xy[i] = tmp[i];
            uint16_t x = ((uint16_t)xy[0] << 8) | xy[1];
            uint16_t y = ((uint16_t)xy[2] << 8) | xy[3];
            printf("[click_echo] Click received: x=%u, y=%u\n", x, y);
            // Send immediately (including 0,0 to clear crosshair)
            sendto(s, xy, 4, 0, (struct sockaddr *)&tx, sizeof(tx));
        }
        if (xy[0] | xy[1] | xy[2] | xy[3]) {
            sendto(s, xy, 4, 0, (struct sockaddr *)&tx, sizeof(tx));
        }
        usleep(50000); // 20 Hz refresh while active
    }
}