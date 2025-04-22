# ESP32-WROOM-32C Pin Assignment for Ethernet + I²S Audio + OLED + GPIO

| Peripheral          | Signal Name       | GPIO       | Notes                                      |
|---------------------|-------------------|------------|--------------------------------------------|
| **Ethernet (RMII)** | REF_CLK           | GPIO0      | 50 MHz clock input from RTL8201F           |
|                     | TXD0              | GPIO19     | Transmit data                              |
|                     | TXD1              | GPIO22     |                                            |
|                     | TX_EN             | GPIO21     |                                            |
|                     | RXD0              | GPIO25     | Receive data                               |
|                     | RXD1              | GPIO26     |                                            |
|                     | CRS_DV            | GPIO27     | Carrier detect / receive data valid        |
|                     | MDC               | GPIO23     | PHY management clock                       |
|                     | MDIO              | GPIO18     | PHY management data                        |
| **OLED (I²C)**      | SDA               | GPIO4      | I²C data                                   |
|                     | SCL               | GPIO5      | I²C clock                                  |
| **I²S Shared**      | WS (LRCK)         | GPIO32     | Word select / Left-Right clock             |
|                     | SCK (BCLK)        | GPIO33     | Bit clock                                  |
| **I²S Microphone**  | SD (data in)      | GPIO34     | Data from microphone (input only GPIO)     |
| **I²S Amplifier**   | SD (data out)     | GPIO17     | Data to speaker amplifier                  |
| **User Button**     | Digital input     | GPIO16     | Use internal pull-up or external resistor  |
| **LED Indicator**   | Digital output    | GPIO15     | Connect LED with current-limiting resistor |

## 🟢 Available GPIOs

| GPIO   | Direction   | Notes                        |
|--------|-------------|------------------------------|
| GPIO12 | In/Out      | Available                    |
| GPIO13 | In/Out      | Available                    |
| GPIO14 | In/Out      | Available                    |
| GPIO35 | Input only  | Available (use as input)     |
| GPIO36 | Input only  | Available (use as input)     |
| GPIO39 | Input only  | Available (use as input)     |
