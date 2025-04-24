#include <string.h>
#include <stdio.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_system.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "driver/i2s.h"
#include "driver/gpio.h"
#include "lwip/sockets.h"
#include "lwip/igmp.h"

#define TAG "VOXSHARE"

#define SAMPLE_RATE     16000
#define I2S_NUM         I2S_NUM_0
#define I2S_DMA_BUF_LEN 1024
#define I2S_SAMPLE_BITS 16

#define MULTICAST_ADDR  "239.255.42.99"
#define MULTICAST_PORT  5005

#define BUTTON_GPIO     GPIO_NUM_0
#define LED_GPIO        GPIO_NUM_2

#define AUDIO_BLOCK_SIZE 320  // 10ms of audio at 16kHz mono
#define MAX_CLIENTS 16

static int udp_socket;
static struct sockaddr_in mcast_addr;
static volatile bool is_transmitting = false;

typedef struct {
    uint32_t ip_addr;
    int16_t buffer[AUDIO_BLOCK_SIZE];
    bool has_audio;
} client_audio_t;

client_audio_t clients[MAX_CLIENTS] = {0};

void add_or_update_client(uint32_t ip, int16_t *audio_data)
{
    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (clients[i].has_audio && clients[i].ip_addr == ip) {
            memcpy(clients[i].buffer, audio_data, sizeof(int16_t) * AUDIO_BLOCK_SIZE);
            return;
        }
    }
    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (!clients[i].has_audio) {
            clients[i].ip_addr = ip;
            memcpy(clients[i].buffer, audio_data, sizeof(int16_t) * AUDIO_BLOCK_SIZE);
            clients[i].has_audio = true;
            return;
        }
    }
}

void clear_all_client_audio()
{
    for (int i = 0; i < MAX_CLIENTS; i++) {
        clients[i].has_audio = false;
    }
}

void init_gpio()
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BUTTON_GPIO) | (1ULL << LED_GPIO),
        .mode = GPIO_MODE_INPUT_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_GPIO, 0);
}

void init_i2s()
{
    i2s_config_t i2s_config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_RX,
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_I2S_MSB,
        .intr_alloc_flags = 0,
        .dma_buf_count = 4,
        .dma_buf_len = I2S_DMA_BUF_LEN,
        .use_apll = false
    };

    i2s_pin_config_t pin_config = {
        .bck_io_num = 26,
        .ws_io_num = 25,
        .data_out_num = 22,
        .data_in_num = 23
    };

    i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_NUM, &pin_config);
}

void init_udp_socket()
{
    udp_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (udp_socket < 0) {
        ESP_LOGE(TAG, "Failed to create socket");
        return;
    }

    struct sockaddr_in local_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(MULTICAST_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY)
    };

    if (bind(udp_socket, (struct sockaddr *)&local_addr, sizeof(local_addr)) < 0) {
        ESP_LOGE(TAG, "Socket bind failed");
    }

    ip_mreq mreq = {
        .imr_multiaddr.s_addr = inet_addr(MULTICAST_ADDR),
        .imr_interface.s_addr = htonl(INADDR_ANY)
    };

    if (setsockopt(udp_socket, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq)) < 0) {
        ESP_LOGE(TAG, "Multicast group join failed");
    }

    bzero(&mcast_addr, sizeof(mcast_addr));
    mcast_addr.sin_family = AF_INET;
    mcast_addr.sin_addr.s_addr = inet_addr(MULTICAST_ADDR);
    mcast_addr.sin_port = htons(MULTICAST_PORT);
}

void audio_send_task(void *arg)
{
    int16_t buffer[AUDIO_BLOCK_SIZE];
    size_t bytes_read;

    while (1) {
        if (gpio_get_level(BUTTON_GPIO) == 0) {
            is_transmitting = true;
            gpio_set_level(LED_GPIO, 1);
            i2s_read(I2S_NUM, buffer, sizeof(buffer), &bytes_read, portMAX_DELAY);
            sendto(udp_socket, buffer, bytes_read, 0, (struct sockaddr *)&mcast_addr, sizeof(mcast_addr));
        } else {
            is_transmitting = false;
            gpio_set_level(LED_GPIO, 0);
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

void audio_receive_task(void *arg)
{
    struct sockaddr_in source_addr;
    socklen_t socklen = sizeof(source_addr);
    int16_t rx_buffer[AUDIO_BLOCK_SIZE];

    while (1) {
        int len = recvfrom(udp_socket, rx_buffer, sizeof(rx_buffer), 0,
                           (struct sockaddr *)&source_addr, &socklen);

        if (len > 0 && len == sizeof(rx_buffer)) {
            uint32_t sender_ip = source_addr.sin_addr.s_addr;

            if (!is_transmitting) {
                add_or_update_client(sender_ip, rx_buffer);
            }
        }

        static int64_t last_mixed = 0;
        int64_t now = esp_timer_get_time(); // in microseconds

        if (now - last_mixed >= 10 * 1000) {
            int32_t mix[AUDIO_BLOCK_SIZE] = {0};
            int16_t final_mix[AUDIO_BLOCK_SIZE] = {0};

            for (int i = 0; i < MAX_CLIENTS; i++) {
                if (clients[i].has_audio) {
                    for (int j = 0; j < AUDIO_BLOCK_SIZE; j++) {
                        mix[j] += clients[i].buffer[j];
                    }
                }
            }

            for (int j = 0; j < AUDIO_BLOCK_SIZE; j++) {
                if (mix[j] > INT16_MAX) mix[j] = INT16_MAX;
                if (mix[j] < INT16_MIN) mix[j] = INT16_MIN;
                final_mix[j] = (int16_t)mix[j];
            }

            size_t bytes_written;
            i2s_write(I2S_NUM, final_mix, sizeof(final_mix), &bytes_written, portMAX_DELAY);

            clear_all_client_audio();
            last_mixed = now;
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void app_main()
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    init_gpio();
    init_i2s();
    init_udp_socket();

    xTaskCreate(audio_send_task, "audio_send", 4096, NULL, 5, NULL);
    xTaskCreate(audio_receive_task, "audio_recv", 8192, NULL, 5, NULL);
}
