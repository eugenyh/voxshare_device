# ESP32-WROOM-32C Pin Assignment for Ethernet + I²S Audio + OLED + GPIO

| Peripheral          | Signal Name       | GPIO       | Pin | Notes                                      |
|---------------------|-------------------|------------|-----|--------------------------------------------|
| **Ethernet (RMII)** | REF_CLK           |            |     | 50MHz clock from external oscillator       |
|                     | TXD0              | GPIO19     | 31  | Transmit data                              |
|                     | TXD1              | GPIO22     | 36  |                                            |
|                     | TX_EN             | GPIO21     | 33  |                                            |
|                     | RXD0              | GPIO25     | 10  | Receive data                               |
|                     | RXD1              | GPIO26     | 11  |                                            |
|                     | CRS_DV            | GPIO27     | 12  | Carrier detect / receive data valid        |
|                     | MDC               | GPIO16     | 27  | PHY management clock                       |
|                     | MDIO              | GPIO17     | 28  | PHY management data                        |
| **OLED (I²C)**      | SDA               | GPIO4      | 26  | I²C data                                   |
|                     | SCL               | GPIO5      | 29  | I²C clock                                  |
| **I²S Shared**      | WS (LRCK)         | GPIO32     | 8   | Word select / Left-Right clock             |
|                     | SCK (BCLK)        | GPIO33     | 9   | Bit clock                                  |
| **I²S Microphone**  | SD (data in)      | GPIO34     | 6   | Data from microphone (input only GPIO)     |
| **I²S Amplifier**   | SD (data out)     | GPIO13     | 16  | Data to speaker amplifier                  |
| **User Button**     | Digital input     | GPIO14     | 13  | Use internal pull-up or external resistor  |
| **LED Indicator**   | Digital output    | GPIO15     | 23  | Connect LED with current-limiting resistor |
| **UART**            | RXD               | GPIO3      | 34  | UART receive (programming & debug)         |
| **UART**            | TXD               | GPIO1      | 35  | UART transmit (programming & debug)        |

## 🟢 Available GPIOs
|                     |                   |            |     | Notes                                      |
|---------------------|-------------------|------------|-----|--------------------------------------------|
|                     |                   | GPIO2      |     | May have onboard LED on some boards        |
|                     |                   | GPIO12     |     | ⚠️ Affects boot mode if HIGH during boot    |
|                     |                   | GPIO18     |     | General purpose I/O                        |
|                     |                   | GPIO23     |     | General purpose I/O                        |
|                     |                   | GPIO35     |     | Input only                                 |
|                     |                   | GPIO36     |     | Input only (SVP)                           |
|                     |                   | GPIO39     |     | Input only (SVN)                           |