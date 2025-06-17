// Ver 0.2FIX
#include <string.h>
#include <stdio.h>
#include <limits.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"

#include "esp_system.h"
#include "esp_log.h"
#include "esp_event.h"
#include "nvs_flash.h"

#include "esp_eth.h"
#include "esp_netif.h"
#include "esp_eth_netif_glue.h"
#include "driver/gpio.h"
#include "driver/i2s_std.h"
#include "esp_timer.h"
#include "driver/spi_master.h"
#include "esp_mac.h"
#include "w5500.h"
#include "opus.h"
#include "esp_dsp.h"

// --- For beep sample generation ---
#include "math.h"

#define TAG "VOXSHARE"

#define SINUS_SEND_TO_I2S     0

// --- Configuration ---
#define PACKET_TYPE_PING      "PING"
#define PACKET_TYPE_PING_LEN  4
#define PACKET_TYPE_AUDIO     "AUD"
#define PACKET_TYPE_AUDIO_LEN 3
#define PING_INTERVAL_MS      2000 // 2 секунды как в Python

#define SAMPLE_RATE      16000
#define AUDIO_BLOCK_SIZE 320
#define MAX_CLIENTS      16
#define MIX_INTERVAL_MS  20

#define USE_OPUS_CODEC        1
#define OPUS_CHANNELS         1
#define OPUS_FRAME_SIZE       320 // 20ms at 16kHz
#define OPUS_MAX_PACKET_SIZE  400 // max 1275

static OpusEncoder *opus_encoder = NULL;
static OpusDecoder *opus_decoder = NULL;

#define CLIENT_TIMEOUT_MS 5000
#define CLIENT_NICK_SIZE  32

#define MULTICAST_ADDR "239.255.42.99"
#define MULTICAST_PORT 5005

#define I2S_NUM I2S_NUM_0
#define I2S_DMA_BUF_LEN 1024
#define I2S_BCK_IO      26
#define I2S_WS_IO       25
#define I2S_DATA_OUT_IO 22
#define I2S_DATA_IN_IO  19

#define BUTTON_GPIO GPIO_NUM_0
#define LED_GPIO    GPIO_NUM_2

// W5500 SPI Configuration
#define ETH_SPI_HOST SPI2_HOST
#define ETH_SPI_SCLK_GPIO 14
#define ETH_SPI_MISO_GPIO 12
#define ETH_SPI_MOSI_GPIO 13
#define ETH_CS_GPIO       5
#define ETH_INT_GPIO      4
#define ETH_RST_GPIO      16
#define ETH_SPI_CLOCK_MHZ 20 // Снижено с 40 до 10 для повышения стабильности SPI

// W5500 PHY Configuration
#define ETH_PHY_ADDR 1

#define DEVICE_PREFIX "ESP32_VSD_"

// --- Global Variables ---
static char device_nick[32];

static int udp_socket;
static struct sockaddr_in mcast_addr;
static volatile bool is_transmitting = false;
static i2s_chan_handle_t tx_handle;
static i2s_chan_handle_t rx_handle;
static esp_netif_t *s_eth_netif = NULL;
static esp_eth_handle_t s_eth_handle = NULL;
static esp_eth_netif_glue_handle_t s_eth_glue = NULL;
static volatile bool eth_reinitalizing = false;

// Хэндлы задач для их перезапуска
static TaskHandle_t audio_receive_task_handle = NULL;
static TaskHandle_t audio_mix_play_task_handle = NULL;
static TaskHandle_t audio_send_task_handle = NULL;
static TaskHandle_t ping_task_handle = NULL;

void start_network_tasks(); // Прототип новой функции

typedef struct {
    uint32_t ip_addr;
    int16_t buffer[AUDIO_BLOCK_SIZE];
    bool has_audio;
    int64_t last_seen;
    char nickname[CLIENT_NICK_SIZE];
} client_audio_t;

static client_audio_t clients[MAX_CLIENTS] = {0};
static SemaphoreHandle_t clients_mutex;

static EventGroupHandle_t network_event_group;
const int GOT_IP_BIT = BIT0;
const int REINIT_BIT = BIT1; // Новый бит для сигнала о перезапуске

// --- Function Prototypes ---
void ping_task(void *arg);
void add_or_update_client(uint32_t ip, int16_t *audio_data, const char* nickname);
void clear_inactive_clients();
void init_gpio();
void init_i2s();
esp_err_t init_ethernet();
void init_udp_socket();
void audio_send_task(void *arg);
void audio_receive_task(void *arg);
void audio_mix_play_task(void *arg);
void network_stack_reinit();
void network_supervisor_task(void *arg);
static void eth_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
static void got_ip_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);

// --- Ping sending ---
void ping_task(void *arg) {
    uint8_t ping_packet[128] = {0};
    // const char *device_nick = "ESP32_Device"; // Или конфигурируемое имя
    
    while (1) {
        // Формируем ping-пакет (4 байта тип + ник)
        memcpy(ping_packet, PACKET_TYPE_PING, PACKET_TYPE_PING_LEN);
        strncpy((char*)ping_packet + PACKET_TYPE_PING_LEN, 
               device_nick, 
               sizeof(ping_packet) - PACKET_TYPE_PING_LEN - 1);
        
        size_t ping_len = PACKET_TYPE_PING_LEN + strlen(device_nick);
        
        // Логирование для отладки
        ESP_LOGD(TAG, "Sending PING (%d bytes): type=%.4s, nick=%s", 
                ping_len, ping_packet, ping_packet + PACKET_TYPE_PING_LEN);
        
        int sent = sendto(udp_socket, ping_packet, ping_len, 0,
                          (struct sockaddr *)&mcast_addr, sizeof(mcast_addr));

        // Точно такая же логика отслеживания ошибок, как в audio_send_task
        static int tx_fail_count = 0;
        const int TX_FAIL_THRESHOLD = 10;

        if (sent < 0) {
            ESP_LOGW(TAG, "Ping sendto failed: errno=%d", errno);
            tx_fail_count++;
            if (tx_fail_count > TX_FAIL_THRESHOLD && !eth_reinitalizing) {
                ESP_LOGE(TAG, "Too many TX errors in PING task, signaling for network stack re-init...");
                xEventGroupSetBits(network_event_group, REINIT_BIT);
                tx_fail_count = 0;
                vTaskDelay(pdMS_TO_TICKS(5000)); // Пауза, чтобы супервизор успел сработать
            }
        } else {
            tx_fail_count = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(PING_INTERVAL_MS));
    }
}

// --- Client Management ---
void add_or_update_client(uint32_t ip, int16_t *audio_data, const char* nickname) {
    int first_empty = -1;
    char clean_nick[CLIENT_NICK_SIZE] = {0};
    
    // 1. Обработка nickname (улучшенная версия)
    if (nickname != NULL && nickname[0] != '\0') {  // Проверка на NULL и пустую строку
        strncpy(clean_nick, nickname, sizeof(clean_nick)-1);
        clean_nick[sizeof(clean_nick)-1] = '\0';
        
        // Дополнительная очистка от непечатных символов
        for (char *p = clean_nick; *p; p++) {
            if ((uint8_t)*p < 32 || (uint8_t)*p > 127) {
                *p = '_';
            }
        }
    } else {
        strncpy(clean_nick, "Unknown", sizeof(clean_nick)-1);
    }

    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (clients[i].ip_addr == ip) {
            // Обновление существующего клиента
            if (audio_data) {
                memcpy(clients[i].buffer, audio_data, sizeof(int16_t) * AUDIO_BLOCK_SIZE);
                clients[i].has_audio = true;
            }

            // Всегда обновляем nickname, если он предоставлен
            strncpy(clients[i].nickname, clean_nick, sizeof(clients[i].nickname)-1);
            clients[i].nickname[sizeof(clients[i].nickname)-1] = '\0';

            clients[i].last_seen = esp_timer_get_time();
            return;
        }

        if (first_empty == -1 && clients[i].ip_addr == 0) {
            first_empty = i;
        }
    }

    // Добавление нового клиента
    if (first_empty != -1) {
        clients[first_empty].ip_addr = ip;
        clients[first_empty].last_seen = esp_timer_get_time();
        
        strncpy(clients[first_empty].nickname, clean_nick, sizeof(clients[first_empty].nickname)-1);
        clients[first_empty].nickname[sizeof(clients[first_empty].nickname)-1] = '\0';
        
        if (audio_data) {
            memcpy(clients[first_empty].buffer, audio_data, sizeof(int16_t) * AUDIO_BLOCK_SIZE);
            clients[first_empty].has_audio = true;
        } else {
            memset(clients[first_empty].buffer, 0, sizeof(int16_t) * AUDIO_BLOCK_SIZE);
            clients[first_empty].has_audio = false;
        }
        
        ESP_LOGI(TAG, "New client added: %s (%s)", 
                inet_ntoa(*(struct in_addr *)&ip),
                clients[first_empty].nickname);
    } else {
        ESP_LOGW(TAG, "Max clients reached, ignoring new client. IP: %s", 
                inet_ntoa(*(struct in_addr *)&ip));
    }
}

void clear_inactive_clients() {
    int64_t now = esp_timer_get_time();
    uint32_t timed_out_ips[MAX_CLIENTS] = {0};
    char timed_out_nicks[MAX_CLIENTS][CLIENT_NICK_SIZE] = {0};
    int timed_out_count = 0;

    // --- Эта часть остается под мьютексом ---
    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (clients[i].ip_addr != 0 && (now - clients[i].last_seen > CLIENT_TIMEOUT_MS * 1000)) {
            if (timed_out_count < MAX_CLIENTS) {
                timed_out_ips[timed_out_count] = clients[i].ip_addr;
                strncpy(timed_out_nicks[timed_out_count], clients[i].nickname, CLIENT_NICK_SIZE -1);
                timed_out_nicks[timed_out_count][CLIENT_NICK_SIZE - 1] = '\0';
                timed_out_count++;
            }
            memset(&clients[i], 0, sizeof(client_audio_t));
        }
    }
    // --- Мьютекс будет освобожден после этого в вызывающей функции ---

    // --- Эта часть теперь ВНЕ мьютекса ---
    for (int i = 0; i < timed_out_count; i++) {
        ESP_LOGI(TAG, "Client timed out: %s (%s)",
               inet_ntoa(*(struct in_addr *)&timed_out_ips[i]),
               timed_out_nicks[i]);
    }
}

// --- Initialization ---
void init_gpio() {
    gpio_config_t io_conf_btn = {
        .pin_bit_mask = (1ULL << BUTTON_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf_btn);

    gpio_config_t io_conf_led = {
        .pin_bit_mask = (1ULL << LED_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf_led);
    gpio_set_level(LED_GPIO, 0);
}

void init_i2s() {
    i2s_chan_config_t chan_cfg = {
        .id = I2S_NUM,
        .role = I2S_ROLE_MASTER,
        .dma_desc_num = 10,
        .dma_frame_num = 240,
        .auto_clear = true,
    };

    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &tx_handle, &rx_handle));

    i2s_std_config_t std_cfg = {
        .clk_cfg = {
            .sample_rate_hz = SAMPLE_RATE,
            .clk_src = I2S_CLK_SRC_DEFAULT,
            .mclk_multiple = I2S_MCLK_MULTIPLE_384  
        },
        .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_24BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S_BCK_IO,
            .ws = I2S_WS_IO,
            .dout = I2S_DATA_OUT_IO,
            .din = I2S_DATA_IN_IO,
        },
    };

    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_handle, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_handle, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(tx_handle));
    ESP_ERROR_CHECK(i2s_channel_enable(rx_handle));
}

esp_err_t init_ethernet() {
    ESP_LOGI(TAG, "Initializing Ethernet...");
    esp_netif_config_t netif_cfg = ESP_NETIF_DEFAULT_ETH();
    s_eth_netif = esp_netif_new(&netif_cfg);

    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
    phy_config.phy_addr = ETH_PHY_ADDR;
    phy_config.reset_gpio_num = ETH_RST_GPIO;

    // gpio_install_isr_service(0); // Перемещено в app_main, чтобы не вызываться повторно

    spi_bus_config_t buscfg = {
        .miso_io_num = ETH_SPI_MISO_GPIO,
        .mosi_io_num = ETH_SPI_MOSI_GPIO,
        .sclk_io_num = ETH_SPI_SCLK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(ETH_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t devcfg = {
        .mode = 0,
        .clock_speed_hz = ETH_SPI_CLOCK_MHZ * 1000 * 1000,
        .spics_io_num = ETH_CS_GPIO,
        .queue_size = 64,  // Увеличено для большей надежности
    };

    eth_w5500_config_t w5500_config = ETH_W5500_DEFAULT_CONFIG(ETH_SPI_HOST, &devcfg);
    w5500_config.int_gpio_num = ETH_INT_GPIO;

    esp_eth_mac_t *mac = esp_eth_mac_new_w5500(&w5500_config, &mac_config);
    esp_eth_phy_t *phy = esp_eth_phy_new_w5500(&phy_config);

    // Кастомный MAC-адрес
    uint8_t custom_mac[6];
    esp_read_mac(custom_mac, ESP_MAC_WIFI_STA);
    custom_mac[0] = 0x02;
    custom_mac[5] += 1;
    ESP_ERROR_CHECK(mac->set_addr(mac, custom_mac));
    // Define device_nick based on device prexix + last 2 MAC bytes 
    snprintf(device_nick, sizeof(device_nick), DEVICE_PREFIX "%02X%02X", custom_mac[4], custom_mac[5]);


    esp_eth_config_t eth_config = ETH_DEFAULT_CONFIG(mac, phy);
    s_eth_handle = NULL; // Обнуляем перед установкой
    ESP_ERROR_CHECK(esp_eth_driver_install(&eth_config, &s_eth_handle));

    s_eth_glue = esp_eth_new_netif_glue(s_eth_handle);
    ESP_ERROR_CHECK(esp_netif_attach(s_eth_netif, s_eth_glue));

    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &got_ip_event_handler, NULL));

    // Включаем прием только нужных пакетов (отключаем promiscuous)
    mac->set_promiscuous(mac, false);

    ESP_ERROR_CHECK(esp_eth_start(s_eth_handle));

    ESP_LOGI(TAG, "Initializing Ethernet done.");
    return ESP_OK;
}

void init_udp_socket() {
    ESP_LOGI(TAG, "Initializing UDP Socket");

    udp_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (udp_socket < 0) {
        ESP_LOGE(TAG, "Failed to create UDP socket: errno %d", errno);
        return;
    }

    struct sockaddr_in local_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(MULTICAST_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY) // Принять с любого интерфейса
    };

    if (bind(udp_socket, (struct sockaddr *)&local_addr, sizeof(local_addr)) < 0) {
        ESP_LOGE(TAG, "Failed to bind UDP socket: errno %d", errno);
        close(udp_socket);
        udp_socket = -1;
        return;
    }

    // Присоединяемся к multicast-группе
    struct ip_mreq mreq = {
        .imr_multiaddr.s_addr = inet_addr(MULTICAST_ADDR),
        .imr_interface.s_addr = htonl(INADDR_ANY) // Можно заменить на IP устройства, если нужно
    };

    if (setsockopt(udp_socket, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq)) < 0) {
        ESP_LOGE(TAG, "Failed to join multicast group: errno %d", errno);
        close(udp_socket);
        udp_socket = -1;
        return;
    }

    // Отключаем приём собственных multicast-пакетов
    char loopch = 0;
    if (setsockopt(udp_socket, IPPROTO_IP, IP_MULTICAST_LOOP, &loopch, sizeof(loopch)) < 0) {
        ESP_LOGW(TAG, "Failed to disable multicast loopback: errno %d", errno);
    }

    // Опционально: выставим TTL для multicast
    // char ttl = 1;
    // setsockopt(udp_socket, IPPROTO_IP, IP_MULTICAST_TTL, &ttl, sizeof(ttl));

    // Адрес, на который будут отправляться пакеты
    bzero(&mcast_addr, sizeof(mcast_addr));
    mcast_addr.sin_family = AF_INET;
    mcast_addr.sin_addr.s_addr = inet_addr(MULTICAST_ADDR);
    mcast_addr.sin_port = htons(MULTICAST_PORT);

    ESP_LOGI(TAG, "UDP Socket Initialized for %s:%d", MULTICAST_ADDR, MULTICAST_PORT);
}

// --- Tasks ---
void audio_send_task(void *arg) {
    ESP_LOGI(TAG, "Free internal heap: %u", heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
 
    size_t raw_buf_size = AUDIO_BLOCK_SIZE * sizeof(uint32_t);
    uint32_t *raw_buf = heap_caps_malloc(raw_buf_size, MALLOC_CAP_8BIT);

    size_t pcm_buf_size = AUDIO_BLOCK_SIZE * sizeof(int16_t);
    int16_t *pcm_buf = heap_caps_malloc(pcm_buf_size, MALLOC_CAP_8BIT);    

    // size_t send_buf_size = OPUS_MAX_PACKET_SIZE + 3;
    uint8_t *send_buf = heap_caps_malloc(3 + OPUS_MAX_PACKET_SIZE, MALLOC_CAP_DEFAULT);

    ESP_LOGI(TAG, "Allocating buffers: RAW %p, PCM %p, SEND %p", raw_buf, pcm_buf, send_buf);

    int tx_fail_count = 0;
    const int TX_FAIL_THRESHOLD = 10; // Порог ошибок для перезапуска

    ESP_LOGI(TAG, "Audio Send Task Started");

    if (!pcm_buf || !send_buf) {
        ESP_LOGE(TAG, "Failed to allocate audio buffers!");
        vTaskDelete(NULL);
    }

    while (1) {
        if (gpio_get_level(BUTTON_GPIO) == 0) {
            if (!is_transmitting) {
                is_transmitting = true;
                gpio_set_level(LED_GPIO, 1);
                ESP_LOGI(TAG, "Started Transmitting");
            }

            size_t bytes_read;
            
            #if SINUS_SEND_TO_I2S
            // --- Generate SINUS to pcm_buf ---
                esp_err_t err = ESP_OK;
                bytes_read = pcm_buf_size;
                static float phase = 0;
                for (int i = 0; i < AUDIO_BLOCK_SIZE; i++) {
                     pcm_buf[i] = (int16_t)(10000 * sinf(phase));
                     phase += 2.0f * M_PI * 440.0f / SAMPLE_RATE; // 440 Гц
                     if (phase > 2.0f * M_PI) phase -= 2.0f * M_PI;
                }
            #else
                // it was: esp_err_t err = i2s_channel_read(rx_handle, pcm_buf, pcm_buf_size, &bytes_read, pdMS_TO_TICKS(300));
                esp_err_t err = i2s_channel_read(rx_handle, raw_buf, raw_buf_size, &bytes_read, portMAX_DELAY); 

                for (int i = 0; i < AUDIO_BLOCK_SIZE; i++) {
                //  Предположим, данные приходят MSB-first, 24 бита значащие
                    int32_t sample = (int32_t)(raw_buf[i] << 8);  // Приводим 24-бит к 32-бит со знаком
                    pcm_buf[i] = sample >> 16;  // Масштабируем до 16 бит
                }
            #endif

            // ESP_LOGI(TAG, "First 5 input samples: %d %d %d %d %d", pcm_buf[0], pcm_buf[1], pcm_buf[2], pcm_buf[3], pcm_buf[4]);

            if (err != ESP_OK || bytes_read != raw_buf_size) {
                ESP_LOGW(TAG, "I2S Read Error or Timeout: %d, bytes: %d", err, bytes_read);
                vTaskDelay(pdMS_TO_TICKS(50));
                continue;
            }

            send_buf[0] = 'A';
            send_buf[1] = 'U';
            send_buf[2] = 'D';

            #if USE_OPUS_CODEC
                int encoded_len = opus_encode(opus_encoder, pcm_buf, AUDIO_BLOCK_SIZE, send_buf + 3, OPUS_MAX_PACKET_SIZE);
            #else
                int encoded_len = AUDIO_BLOCK_SIZE * sizeof(int16_t);
                memcpy(send_buf + 3, pcm_buf, AUDIO_BLOCK_SIZE * sizeof(int16_t));
            #endif
             
            if (encoded_len <= 0 || encoded_len > OPUS_MAX_PACKET_SIZE) {
                ESP_LOGE(TAG, "Opus encoding failed or too big: %d", encoded_len);
                vTaskDelay(pdMS_TO_TICKS(50));
                continue;
            }

            // Check socket ready before sending
            assert(udp_socket >= 0);

            int sent = sendto(udp_socket, send_buf, encoded_len + 3, 0, (struct sockaddr *)&mcast_addr, sizeof(mcast_addr));

            if (sent < 0) {
                ESP_LOGW(TAG, "UDP sendto failed: errno=%d", errno);
                tx_fail_count++;
                if (tx_fail_count > TX_FAIL_THRESHOLD && !eth_reinitalizing) {
                    ESP_LOGE(TAG, "Too many TX errors in AUDIO task, signaling for network stack re-init...");
                    xEventGroupSetBits(network_event_group, REINIT_BIT);
                    tx_fail_count = 0;
                    vTaskDelay(pdMS_TO_TICKS(5000)); // Пауза, чтобы супервизор успел сработать
                }
                vTaskDelay(pdMS_TO_TICKS(50));
            } else {
                tx_fail_count = 0; // Сбрасываем счетчик при успешной отправке
            }

        } else {
            if (is_transmitting) {
                is_transmitting = false;
                gpio_set_level(LED_GPIO, 0);
                ESP_LOGI(TAG, "Stopped Transmitting");
            }
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
    free(raw_buf);
    free(pcm_buf);
    free(send_buf);
}

void audio_receive_task(void *arg) {
    struct sockaddr_in source_addr;
    socklen_t socklen = sizeof(source_addr);

    // Выделяем память в куче (DEFAULT достаточно)
    uint8_t *rx_buf = heap_caps_malloc(OPUS_MAX_PACKET_SIZE + 3, MALLOC_CAP_DEFAULT);
    int16_t *decoded_pcm = heap_caps_malloc(sizeof(int16_t) * AUDIO_BLOCK_SIZE, MALLOC_CAP_DEFAULT);

    esp_netif_ip_info_t ip_info;
    uint32_t local_ip_addr = 0;

    ESP_LOGI(TAG, "Audio Receive Task Started");

    if (!rx_buf || !decoded_pcm) {
        ESP_LOGE(TAG, "Failed to allocate memory for audio_receive_task");
        vTaskDelete(NULL);
    }

    if (s_eth_netif) {
        if (esp_netif_get_ip_info(s_eth_netif, &ip_info) == ESP_OK) {
            local_ip_addr = ip_info.ip.addr;
            ESP_LOGI(TAG, "Local IP: " IPSTR, IP2STR(&ip_info.ip));
        } else {
            ESP_LOGE(TAG, "Failed to get IP info");
        }
    } else {
        ESP_LOGE(TAG, "Ethernet netif handle is NULL");
    }

    while (1) {
        int64_t recv_start_time = esp_timer_get_time();
        int len = recvfrom(udp_socket, rx_buf, 3 + OPUS_MAX_PACKET_SIZE, 0,
                           (struct sockaddr *)&source_addr, &socklen);
        int64_t recv_end_time = esp_timer_get_time();

        ESP_LOGD(TAG, "recvfrom took %lld us", recv_end_time - recv_start_time);

        if (len < 0) {
            ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        if (len >= PACKET_TYPE_PING_LEN && memcmp(rx_buf, PACKET_TYPE_PING, PACKET_TYPE_PING_LEN) == 0) {
            char received_nick[32] = {0}; // Инициализация нулями
            int nick_len = len - PACKET_TYPE_PING_LEN; // Длина данных после заголовка
    
            // Копируем только фактически полученные данные
            if (nick_len > 0) {
               int copy_len = (nick_len < sizeof(received_nick)-1) ? nick_len : sizeof(received_nick)-1;
               memcpy(received_nick, rx_buf + PACKET_TYPE_PING_LEN, copy_len);
               received_nick[copy_len] = '\0'; // Гарантированное завершение строки
        
               // Очистка от непечатных символов
               for (char *p = received_nick; *p; p++) {
                   if ((uint8_t)*p < 32 || (uint8_t)*p > 127) *p = '_';
               }
            } else {
               strcpy(received_nick, "Unknown");
            }
    
            // Логирование для отладки
            ESP_LOGI(TAG, "Received PING from %s, nick: '%s' (len=%d)", 
                     inet_ntoa(*(struct in_addr *)&source_addr.sin_addr.s_addr),
                     received_nick,
                     nick_len);
    
            int64_t mutex_start_time = esp_timer_get_time();
            if (xSemaphoreTake(clients_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
               int64_t mutex_end_time = esp_timer_get_time();
               ESP_LOGD(TAG, "Mutex wait time (PING): %lld us", mutex_end_time - mutex_start_time);
               add_or_update_client(source_addr.sin_addr.s_addr, NULL, received_nick);
               xSemaphoreGive(clients_mutex);
            } else {
                ESP_LOGW(TAG, "Failed to take mutex for PING");
            }
        }
        else if (len > 3 && memcmp(rx_buf, PACKET_TYPE_AUDIO, 3) == 0) {

            #if USE_OPUS_CODEC
                int decoded_samples = opus_decode(opus_decoder, rx_buf + 3, len - 3, decoded_pcm, AUDIO_BLOCK_SIZE, 0);
            #else
                int decoded_samples = AUDIO_BLOCK_SIZE;
                memcpy(decoded_pcm, rx_buf + 3, AUDIO_BLOCK_SIZE * sizeof(int16_t));
            #endif

            // ESP_LOGI(TAG, "First 5 decoded samples: %d %d %d %d %d",
            //         decoded_pcm[0], decoded_pcm[1], decoded_pcm[2], decoded_pcm[3], decoded_pcm[4]);                                  

            if (decoded_samples == AUDIO_BLOCK_SIZE) {
                uint32_t sender_ip = source_addr.sin_addr.s_addr;

                // Игнорируем собственные пакеты, если передача активна
                if (!is_transmitting && sender_ip != local_ip_addr) {
                    int64_t mutex_start_time = esp_timer_get_time();
                    if (xSemaphoreTake(clients_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                        int64_t mutex_end_time = esp_timer_get_time();
                        ESP_LOGD(TAG, "Mutex wait time (AUDIO): %lld us", mutex_end_time - mutex_start_time);
                        add_or_update_client(sender_ip, decoded_pcm, NULL);
                        xSemaphoreGive(clients_mutex);
                    } else {
                        ESP_LOGW(TAG, "Failed to take mutex for AUDIO");
                    }
                }
            } else {
                ESP_LOGE(TAG, "Opus decode failed: %d", decoded_samples);
            }
        }

    }

    free(rx_buf);
    free(decoded_pcm);
}

void audio_mix_play_task(void *arg) {
    static int16_t mix_buffer[AUDIO_BLOCK_SIZE] = {0};
    static int16_t scaled_buffer[AUDIO_BLOCK_SIZE] = {0};
    static int32_t i2s_out_buf[AUDIO_BLOCK_SIZE] = {0};
    const size_t i2s_buf_size_bytes = sizeof(i2s_out_buf);

    const int max_shift = 2;  // Максимальное усиление (2^2 = x4)
    TickType_t xLastWakeTime = xTaskGetTickCount();
    ESP_LOGI(TAG, "Audio Mix/Play Task Started");

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(MIX_INTERVAL_MS));
        if (is_transmitting) continue;

        memset(mix_buffer, 0, sizeof(mix_buffer));
        int active_clients = 0;

        if (xSemaphoreTake(clients_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            clear_inactive_clients();
            for (int i = 0; i < MAX_CLIENTS; i++) {
                if (clients[i].ip_addr != 0 && clients[i].has_audio) {
                    active_clients++;
                    dsps_add_s16(clients[i].buffer, mix_buffer, mix_buffer, AUDIO_BLOCK_SIZE, 1, 1, 1, 0);
                    clients[i].has_audio = false;
                }
            }
            xSemaphoreGive(clients_mutex);
        } else {
            ESP_LOGW(TAG, "Mutex timeout in audio task!");
            continue;
        }

        int shift = (active_clients == 0) ? 0 : max_shift - __builtin_clz(active_clients | 1);
        shift = (shift < 0) ? 0 : (shift > max_shift) ? max_shift : shift;

        // Масштабирование и усиление через esp-dsp
        int16_t unity[AUDIO_BLOCK_SIZE];
        for (int i = 0; i < AUDIO_BLOCK_SIZE; i++) unity[i] = 1;

        dsps_mul_s16(mix_buffer, unity, scaled_buffer, AUDIO_BLOCK_SIZE, 1, 1, 1, shift);


        for (int i = 0; i < AUDIO_BLOCK_SIZE; i++) {
             int32_t s = scaled_buffer[i];
             if (s > INT16_MAX) s = INT16_MAX;
             if (s < INT16_MIN) s = INT16_MIN;
             i2s_out_buf[i] = s << 8;  // 24-бит для I2S
        }

        size_t bytes_written;
        esp_err_t err = i2s_channel_write(tx_handle, i2s_out_buf, i2s_buf_size_bytes, &bytes_written, portMAX_DELAY);
        if (err != ESP_OK || bytes_written != i2s_buf_size_bytes) {
            ESP_LOGE(TAG, "I2S error: %d, written: %zu", err, bytes_written);
        }
    }
}

void network_stack_reinit() {
    if (eth_reinitalizing) {
        return;
    }
    eth_reinitalizing = true;
    ESP_LOGE(TAG, "--- Starting Network Stack Re-initialization ---");

    // 1. Удаляем задачи
    ESP_LOGI(TAG, "Deleting network tasks...");
    if (audio_receive_task_handle) vTaskDelete(audio_receive_task_handle);
    if (audio_mix_play_task_handle) vTaskDelete(audio_mix_play_task_handle);
    if (audio_send_task_handle) vTaskDelete(audio_send_task_handle);
    if (ping_task_handle) vTaskDelete(ping_task_handle);
    audio_receive_task_handle = NULL;
    audio_mix_play_task_handle = NULL;
    audio_send_task_handle = NULL;
    ping_task_handle = NULL;


    // 2. Закрываем сокет
    if (udp_socket >= 0) {
        ESP_LOGI(TAG, "Closing UDP socket...");
        close(udp_socket);
        udp_socket = -1;
    }

    // 3. Правильная последовательность деинициализации
    if (s_eth_handle) {
        ESP_LOGI(TAG, "Stopping Ethernet...");
        esp_eth_stop(s_eth_handle);
        if (s_eth_glue) {
            esp_eth_del_netif_glue(s_eth_glue);
            s_eth_glue = NULL;
        }
        esp_eth_driver_uninstall(s_eth_handle);
        s_eth_handle = NULL;
    }
    // Освобождаем шину SPI после того, как драйвер ее отпустил
    ESP_LOGI(TAG, "Freeing SPI bus...");
    spi_bus_free(ETH_SPI_HOST);

    if (s_eth_netif) {
        esp_netif_destroy(s_eth_netif);
        s_eth_netif = NULL;
    }
    
    vTaskDelay(pdMS_TO_TICKS(500)); // Даем время на завершение

    // 4. Повторная инициализация
    ESP_LOGI(TAG, "Re-initializing Ethernet hardware...");
    ESP_ERROR_CHECK(init_ethernet());

    // 5. Ждем нового IP
    ESP_LOGI(TAG, "Waiting for new IP address...");
    xEventGroupClearBits(network_event_group, GOT_IP_BIT); // Очищаем старый флаг
    EventBits_t bits = xEventGroupWaitBits(network_event_group, GOT_IP_BIT, pdFALSE, pdTRUE, pdMS_TO_TICKS(30000)); // Ждем 30 сек
    
    // 6. Пересоздаем задачи
    if (bits & GOT_IP_BIT) {
        ESP_LOGI(TAG, "Got new IP. Restarting network tasks...");
        start_network_tasks();
    } else {
        ESP_LOGE(TAG, "Failed to get new IP address. Restarting...");
        esp_restart();
    }

    ESP_LOGI(TAG, "--- Network Stack Re-initialization Finished ---");
    eth_reinitalizing = false;
}

// --- Event Handlers ---
static void eth_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    uint8_t mac_addr[6] = {0};
    /* esp_eth_handle_t eth_handle = *(esp_eth_handle_t *)event_data; */

    switch (event_id) {
    case ETHERNET_EVENT_CONNECTED:
        esp_netif_get_mac(s_eth_netif, mac_addr);
        ESP_LOGI(TAG, "Ethernet Link Up");
        ESP_LOGI(TAG, "Ethernet HW Addr %02x:%02x:%02x:%02x:%02x:%02x",
                 mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
        break;
    case ETHERNET_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "Ethernet Link Down");
        xEventGroupClearBits(network_event_group, GOT_IP_BIT);
        break;
    case ETHERNET_EVENT_START:
        ESP_LOGI(TAG, "Ethernet Started");
        break;
    case ETHERNET_EVENT_STOP:
        ESP_LOGI(TAG, "Ethernet Stopped");
        break;
    default:
        break;
    }
}

static void got_ip_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    const esp_netif_ip_info_t *ip_info = &event->ip_info;

    ESP_LOGI(TAG, "Got IP Address");
    ESP_LOGI(TAG, "~~~~~~~~~~~");
    ESP_LOGI(TAG, "ETHIP:" IPSTR, IP2STR(&ip_info->ip));
    ESP_LOGI(TAG, "ETHMASK:" IPSTR, IP2STR(&ip_info->netmask));
    ESP_LOGI(TAG, "ETHGW:" IPSTR, IP2STR(&ip_info->gw));
    ESP_LOGI(TAG, "~~~~~~~~~~~");
    xEventGroupSetBits(network_event_group, GOT_IP_BIT);
}

// Beet test signal generation
void play_startup_beep() {
    const float frequency = 3000.0; // 1 kHz tone
    const float duration = 0.1f;    // 100 ms duration
    const int num_samples = (int)(SAMPLE_RATE * duration); // 1600 samples
    
    // Allocate buffer for samples
    int16_t *samples = malloc(num_samples * sizeof(int16_t));
    if (!samples) {
        ESP_LOGE(TAG, "Failed to allocate memory for beep samples");
        return;
    }
    
    // Generate sine wave
    for (int i = 0; i < num_samples; i++) {
        samples[i] = (int16_t)(32767 * sinf(2 * M_PI * frequency * i / SAMPLE_RATE));
    }
    
    // Send data through I2S
    size_t bytes_written = 0;
    esp_err_t ret = i2s_channel_write(tx_handle, samples, 
                                     num_samples * sizeof(int16_t), 
                                     &bytes_written, 
                                     portMAX_DELAY); //pdMS_TO_TICKS(100)
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2S write failed: %s", esp_err_to_name(ret));
    }
    
    free(samples);
    
    // Add small delay to ensure the beep finishes before continuing
    vTaskDelay(pdMS_TO_TICKS(110));
}

// --- Main Application ---
void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    network_event_group = xEventGroupCreate();
    clients_mutex = xSemaphoreCreateMutex();
    if (clients_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create clients mutex!");
        return;
    }

    init_gpio();
    // Устанавливаем службу ISR один раз при старте
    gpio_install_isr_service(0);

    init_i2s();

    // Checking I2S audio by making a beep test signal
    play_startup_beep();  
    
    // Prepearing OPUS
    #if USE_OPUS_CODEC

    int opus_err;
        ESP_LOGI(TAG, "Using OPUS codec.");
        opus_encoder = opus_encoder_create(SAMPLE_RATE, 1, OPUS_APPLICATION_AUDIO, &opus_err);
        if (opus_err != OPUS_OK) {
            ESP_LOGE(TAG, "Failed to create Opus encoder: %s", opus_strerror(opus_err));
            return;
        } else {
                ESP_LOGI(TAG, "OPUS encoder created.");
        }

        opus_decoder = opus_decoder_create(SAMPLE_RATE, 1, &opus_err);
        if (opus_err != OPUS_OK) {
            ESP_LOGE(TAG, "Failed to create Opus decoder: %s", opus_strerror(opus_err));
            return;
        } else {
                ESP_LOGI(TAG, "OPUS decoder created.");
        }
    #else
        ESP_LOGI(TAG, "Using PCM audio.");
    #endif

    // Init ethernet
    ESP_ERROR_CHECK(init_ethernet());

    ESP_LOGI(TAG, "Waiting for IP address...");
    EventBits_t bits = xEventGroupWaitBits(network_event_group, GOT_IP_BIT, pdFALSE, pdTRUE, portMAX_DELAY);

    if (bits & GOT_IP_BIT) {
        start_network_tasks();
    } else {
        ESP_LOGE(TAG, "Failed to get IP address. Halting.");
    }

    ESP_LOGI(TAG, "app_main finished setup.");
}

void start_network_tasks() {
    ESP_LOGI(TAG, "Initializing UDP and starting network tasks...");
    init_udp_socket();
    if (udp_socket >= 0) {
        xTaskCreate(audio_receive_task, "audio_recv", 28672, NULL, 6, &audio_receive_task_handle);
        xTaskCreate(audio_mix_play_task, "audio_mix", 28672, NULL, 7, &audio_mix_play_task_handle);
        xTaskCreate(audio_send_task, "audio_send", 28672, NULL, 6, &audio_send_task_handle);
        xTaskCreate(ping_task, "ping_task", 4096, NULL, 5, &ping_task_handle);
        xTaskCreate(network_supervisor_task, "net_supervisor", 4096, NULL, 10, NULL); // Создаем супервизора
        ESP_LOGI(TAG, "Network tasks started.");
    } else {
        ESP_LOGE(TAG, "UDP socket failed to initialize. Cannot start tasks.");
    }
    // После успешного старта всех задач, отключаем "шумные" логи драйвера, чтобы избежать дедлока в будущем.
    esp_log_level_set("w5500.mac", ESP_LOG_NONE);
}

void network_supervisor_task(void *arg) {
    while(1) {
        EventBits_t bits = xEventGroupWaitBits(network_event_group, REINIT_BIT, pdTRUE, pdFALSE, portMAX_DELAY);
        if ((bits & REINIT_BIT) != 0) {
            network_stack_reinit();
        }
    }
}