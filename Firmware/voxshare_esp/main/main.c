// Ver 0.1FIX
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

// --- For beep sample generation ---
#include "math.h"

#define TAG "VOXSHARE"

// --- OPUS ----
#include "opus.h"

#define OPUS_SAMPLE_RATE      16000
#define OPUS_CHANNELS         1
#define OPUS_FRAME_SIZE       320 // 20ms at 16kHz
#define OPUS_MAX_PACKET_SIZE  1275

static OpusEncoder *opus_encoder = NULL;
static OpusDecoder *opus_decoder = NULL;

// --- Configuration ---
#define SAMPLE_RATE 16000
#define AUDIO_BLOCK_SIZE 320
#define MAX_CLIENTS 16
#define MIX_INTERVAL_MS 20
#define CLIENT_TIMEOUT_INTERVALS 5

#define MULTICAST_ADDR "239.255.42.99"
#define MULTICAST_PORT 5005

#define I2S_NUM I2S_NUM_0
#define I2S_DMA_BUF_LEN 1024
#define I2S_BCK_IO 26
#define I2S_WS_IO 25
#define I2S_DATA_OUT_IO 22
#define I2S_DATA_IN_IO 19

#define BUTTON_GPIO GPIO_NUM_0
#define LED_GPIO GPIO_NUM_2

// W5500 SPI Configuration
#define ETH_SPI_HOST SPI2_HOST
#define ETH_SPI_SCLK_GPIO 14
#define ETH_SPI_MISO_GPIO 12
#define ETH_SPI_MOSI_GPIO 13
#define ETH_CS_GPIO 5
#define ETH_INT_GPIO 4
#define ETH_RST_GPIO 16
#define ETH_SPI_CLOCK_MHZ 4 

// W5500 PHY Configuration
#define ETH_PHY_ADDR 1

// --- Global Variables ---
static int udp_socket;
static struct sockaddr_in mcast_addr;
static volatile bool is_transmitting = false;
static i2s_chan_handle_t tx_handle;
static i2s_chan_handle_t rx_handle;
static esp_netif_t *s_eth_netif = NULL;

typedef struct {
    uint32_t ip_addr;
    int16_t buffer[AUDIO_BLOCK_SIZE];
    bool has_audio;
    int64_t last_seen;
} client_audio_t;

static client_audio_t clients[MAX_CLIENTS] = {0};
static SemaphoreHandle_t clients_mutex;

static EventGroupHandle_t network_event_group;
const int GOT_IP_BIT = BIT0;

// --- Function Prototypes ---
void add_or_update_client(uint32_t ip, int16_t *audio_data);
void clear_inactive_clients();
void init_gpio();
void init_i2s();
esp_err_t init_ethernet();
void init_udp_socket();
void audio_send_task(void *arg);
void audio_receive_task(void *arg);
void audio_mix_play_task(void *arg);
static void eth_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
static void got_ip_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);

// --- Client Management ---
void add_or_update_client(uint32_t ip, int16_t *audio_data) {
    int first_empty = -1;
    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (clients[i].ip_addr == ip) {
            memcpy(clients[i].buffer, audio_data, sizeof(int16_t) * AUDIO_BLOCK_SIZE);
            clients[i].has_audio = true;
            clients[i].last_seen = esp_timer_get_time();
            return;
        }
        if (first_empty == -1 && clients[i].ip_addr == 0) {
            first_empty = i;
        }
    }

    if (first_empty != -1) {
        clients[first_empty].ip_addr = ip;
        memcpy(clients[first_empty].buffer, audio_data, sizeof(int16_t) * AUDIO_BLOCK_SIZE);
        clients[first_empty].has_audio = true;
        clients[first_empty].last_seen = esp_timer_get_time();
        ESP_LOGI(TAG, "New client added: %s", inet_ntoa(*(struct in_addr *)&ip));
    } else {
        ESP_LOGW(TAG, "Max clients reached, ignoring new client.");
    }
}

void clear_inactive_clients() {
    int64_t now = esp_timer_get_time();
    int64_t timeout = MIX_INTERVAL_MS * CLIENT_TIMEOUT_INTERVALS * 1000;

    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (clients[i].ip_addr != 0 && (now - clients[i].last_seen > timeout)) {
            ESP_LOGI(TAG, "Client timed out: %s", inet_ntoa(*(struct in_addr *)&clients[i].ip_addr));
            clients[i].ip_addr = 0;
            // Флаг теперь сбрасывается только для неактивных клиентов.
            clients[i].has_audio = false;
        }
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
    // Before: i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM, I2S_ROLE_MASTER);
    // New (Define configuration with inreased buffers):
    i2s_chan_config_t chan_cfg = {
        .id = I2S_NUM,
        .role = I2S_ROLE_MASTER,
        .dma_desc_num = 10, // Increase number of descriptors (as about 6 by default)
        .dma_frame_num = 240, // Increase frame size (was about 120)
        .auto_clear = true,
    };

    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &tx_handle, &rx_handle));

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
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

    gpio_install_isr_service(0);

    spi_bus_config_t buscfg = {
        .miso_io_num = ETH_SPI_MISO_GPIO,
        .mosi_io_num = ETH_SPI_MOSI_GPIO,
        .sclk_io_num = ETH_SPI_SCLK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(ETH_SPI_HOST, &buscfg, 1)); //was SPI_DMA_CH_AUTO

    spi_device_interface_config_t devcfg = {
        .mode = 0,
        .clock_speed_hz = ETH_SPI_CLOCK_MHZ * 1000 * 1000,
        .spics_io_num = ETH_CS_GPIO,
        .queue_size = 20
    };

    eth_w5500_config_t w5500_config = ETH_W5500_DEFAULT_CONFIG(ETH_SPI_HOST, &devcfg);
    w5500_config.int_gpio_num = ETH_INT_GPIO;

    esp_eth_mac_t *mac = esp_eth_mac_new_w5500(&w5500_config, &mac_config);
    esp_eth_phy_t *phy = esp_eth_phy_new_w5500(&phy_config);

    // Create MAC address for w5500
    uint8_t custom_mac[6];
    esp_read_mac(custom_mac, ESP_MAC_WIFI_STA); //Read mac from WIFI
    custom_mac[0] = 0x02;  // Set local address
    custom_mac[5] += 1;    // Make it unique
    ESP_ERROR_CHECK(mac->set_addr(mac, custom_mac)); 

    esp_eth_config_t eth_config = ETH_DEFAULT_CONFIG(mac, phy);
    esp_eth_handle_t eth_handle = NULL;
    ESP_ERROR_CHECK(esp_eth_driver_install(&eth_config, &eth_handle));

    void* glue = esp_eth_new_netif_glue(eth_handle);
    ESP_ERROR_CHECK(esp_netif_attach(s_eth_netif, glue));
    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &got_ip_event_handler, NULL));
    ESP_ERROR_CHECK(esp_eth_start(eth_handle));

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
        .sin_addr.s_addr = htonl(INADDR_ANY)
    };
    if (bind(udp_socket, (struct sockaddr *)&local_addr, sizeof(local_addr)) < 0) {
        ESP_LOGE(TAG, "Failed to bind UDP socket: errno %d", errno);
        close(udp_socket);
        udp_socket = -1;
        return;
    }

    ip_mreq mreq = {
        .imr_multiaddr.s_addr = inet_addr(MULTICAST_ADDR),
        .imr_interface.s_addr = htonl(INADDR_ANY)
    };
    if (setsockopt(udp_socket, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq)) < 0) {
        ESP_LOGE(TAG, "Failed to join multicast group: errno %d", errno);
        close(udp_socket);
        udp_socket = -1;
        return;
    }

    char loopch = 0;
    if (setsockopt(udp_socket, IPPROTO_IP, IP_MULTICAST_LOOP, &loopch, sizeof(loopch)) < 0) {
        ESP_LOGW(TAG, "Failed to disable multicast loopback: errno %d", errno);
    }

    bzero(&mcast_addr, sizeof(mcast_addr));
    mcast_addr.sin_family = AF_INET;
    mcast_addr.sin_addr.s_addr = inet_addr(MULTICAST_ADDR);
    mcast_addr.sin_port = htons(MULTICAST_PORT);

    ESP_LOGI(TAG, "UDP Socket Initialized for %s:%d", MULTICAST_ADDR, MULTICAST_PORT);
}

// --- Tasks ---
void audio_send_task(void *arg) {
    int16_t pcm_buf[AUDIO_BLOCK_SIZE];
    uint8_t send_buf[3 + OPUS_MAX_PACKET_SIZE];
    size_t bytes_read;

    ESP_LOGI(TAG, "Audio Send Task Started");

    while (1) {
        if (gpio_get_level(BUTTON_GPIO) == 0) {
            if (!is_transmitting) {
                is_transmitting = true;
                gpio_set_level(LED_GPIO, 1);
                ESP_LOGI(TAG, "Started Transmitting");
            }

            esp_err_t err = i2s_channel_read(rx_handle, pcm_buf, sizeof(pcm_buf), &bytes_read, pdMS_TO_TICKS(300));
            if (err == ESP_OK && bytes_read == sizeof(pcm_buf)) {
                // Заголовок пакета
                send_buf[0] = 'A';
                send_buf[1] = 'U';
                send_buf[2] = 'D';

                int encoded_len = opus_encode(opus_encoder, pcm_buf, AUDIO_BLOCK_SIZE,
                                              send_buf + 3, OPUS_MAX_PACKET_SIZE);
                if (encoded_len > 0) {
                    int sent = sendto(udp_socket, send_buf, encoded_len + 3, 0,
                                      (struct sockaddr *)&mcast_addr, sizeof(mcast_addr));
                    if (sent < 0) {
                        ESP_LOGW(TAG, "UDP sendto failed: errno=%d", errno);
                        vTaskDelay(pdMS_TO_TICKS(50));
                    }
                } else {
                    ESP_LOGE(TAG, "Opus encoding failed: %d", encoded_len);
                }
            } else {
                ESP_LOGW(TAG, "I2S Read Error or Timeout: %d, bytes: %d", err, bytes_read);
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
}

void audio_receive_task(void *arg) {
    struct sockaddr_in source_addr;
    socklen_t socklen = sizeof(source_addr);
    uint8_t rx_buf[3 + OPUS_MAX_PACKET_SIZE];
    int16_t decoded_pcm[AUDIO_BLOCK_SIZE];
    esp_netif_ip_info_t ip_info;
    uint32_t local_ip_addr = 0;

    ESP_LOGI(TAG, "Audio Receive Task Started");

    if (s_eth_netif) {
        esp_netif_get_ip_info(s_eth_netif, &ip_info);
        local_ip_addr = ip_info.ip.addr;
    } else {
        ESP_LOGE(TAG, "Failed to get Ethernet netif handle");
    }

    while (1) {
        int len = recvfrom(udp_socket, rx_buf, sizeof(rx_buf), 0, (struct sockaddr *)&source_addr, &socklen);
        if (len < 0) {
            ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        if (len > 3 && rx_buf[0] == 'A' && rx_buf[1] == 'U' && rx_buf[2] == 'D') {
            int decoded_samples = opus_decode(opus_decoder, rx_buf + 3, len - 3,
                                              decoded_pcm, AUDIO_BLOCK_SIZE, 0);
            if (decoded_samples == AUDIO_BLOCK_SIZE) {
                uint32_t sender_ip = source_addr.sin_addr.s_addr;
                if (!is_transmitting && sender_ip != local_ip_addr) {
                    if (xSemaphoreTake(clients_mutex, portMAX_DELAY) == pdTRUE) {
                        add_or_update_client(sender_ip, decoded_pcm);
                        xSemaphoreGive(clients_mutex);
                    }
                }
            } else {
                ESP_LOGE(TAG, "Opus decode failed: %d", decoded_samples);
            }
        }
    }
}


void audio_mix_play_task(void *arg) {
    int32_t mix_buffer[AUDIO_BLOCK_SIZE];
    int16_t final_mix[AUDIO_BLOCK_SIZE];
    size_t bytes_written;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    ESP_LOGI(TAG, "Audio Mix/Play Task Started");

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(MIX_INTERVAL_MS));
        if (is_transmitting) {
            continue;
        }

        memset(mix_buffer, 0, sizeof(mix_buffer));
        int active_clients = 0;
        if (xSemaphoreTake(clients_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            // Сначала удаляем "протухших" клиентов
            clear_inactive_clients(); // Используйте исправленную версию из Варианта 1

            // Затем микшируем аудио от тех, кто прислал данные
            for (int i = 0; i < MAX_CLIENTS; i++) {
                if (clients[i].ip_addr != 0 && clients[i].has_audio) {
                    active_clients++;
                    for (int j = 0; j < AUDIO_BLOCK_SIZE; j++) {
                        mix_buffer[j] += clients[i].buffer[j];
                    }
                    // Сбрасываем флаг ПОСЛЕ того, как данные были использованы
                    clients[i].has_audio = false;
                }
            }
            xSemaphoreGive(clients_mutex);
        } else {
            ESP_LOGW(TAG, "Failed to get mutex in mix task!");
            continue;
        }

        if (active_clients > 0) {
            for (int j = 0; j < AUDIO_BLOCK_SIZE; j++) {
                if (mix_buffer[j] > INT16_MAX) mix_buffer[j] = INT16_MAX;
                if (mix_buffer[j] < INT16_MIN) mix_buffer[j] = INT16_MIN;
                final_mix[j] = (int16_t)mix_buffer[j];
            }
            i2s_channel_write(tx_handle, final_mix, sizeof(final_mix), &bytes_written, portMAX_DELAY);
            if (bytes_written != sizeof(final_mix)) {
                ESP_LOGW(TAG, "I2S Write Error or incomplete, bytes: %d", bytes_written);
            }
        } else {
            // Если активных клиентов не было, отправляем тишину
            memset(final_mix, 0, sizeof(final_mix));
            i2s_channel_write(tx_handle, final_mix, sizeof(final_mix), &bytes_written, pdMS_TO_TICKS(10));
        }
    }
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

    init_i2s();

    // Checking I2S audio by making a beep test signal
    play_startup_beep();  
    
    // Prepearing OPUS
    int err;
    opus_encoder = opus_encoder_create(SAMPLE_RATE, 1, OPUS_APPLICATION_AUDIO, &err);
    if (err != OPUS_OK) {
        ESP_LOGE(TAG, "Failed to create Opus encoder: %s", opus_strerror(err));
        return;
    }

    opus_decoder = opus_decoder_create(SAMPLE_RATE, 1, &err);
    if (err != OPUS_OK) {
        ESP_LOGE(TAG, "Failed to create Opus decoder: %s", opus_strerror(err));
        return;
    }

    // Init ethernet
    ESP_ERROR_CHECK(init_ethernet());

    ESP_LOGI(TAG, "Waiting for IP address...");
    EventBits_t bits = xEventGroupWaitBits(network_event_group, GOT_IP_BIT, pdFALSE, pdTRUE, portMAX_DELAY);

    if (bits & GOT_IP_BIT) {
        ESP_LOGI(TAG, "Got IP address. Initializing UDP...");
        init_udp_socket();
        if (udp_socket >= 0) {
            xTaskCreate(audio_receive_task, "audio_recv", 4096, NULL, 6, NULL);
            xTaskCreate(audio_mix_play_task, "audio_mix", 4096, NULL, 7, NULL);
            xTaskCreate(audio_send_task, "audio_send", 4096, NULL, 6, NULL);
        } else {
            ESP_LOGE(TAG, "UDP socket failed to initialize. Cannot start tasks.");
        }
    } else {
        ESP_LOGE(TAG, "Failed to get IP address. Halting.");
    }

    ESP_LOGI(TAG, "app_main finished setup.");
}