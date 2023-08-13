#ifndef MYLIB_SX127x_H_
#define MYLIB_SX127x_H_
#include "main.h"

/**************   SX127X register map  **************/

#define SX127X_REG_FIFO                         0x00
#define SX127X_REG_OP_MODE                      0x01
#define SX127X_REG_FRF_MSB                      0x06
#define SX127X_REG_FRF_MID                      0x07
#define SX127X_REG_FRF_LSB                      0x08
#define SX127X_REG_PA_CONFIG                    0x09
#define SX127X_REG_OCP                          0x0B
#define SX127X_REG_LNA                          0x0C
#define SX127X_REG_FIFO_ADDR_PTR                0x0D
#define SX127X_REG_FIFO_TX_BASE_ADDR            0x0E
#define SX127X_REG_FIFO_RX_BASE_ADDR            0x0F
#define SX127X_REG_FIFO_RX_CURRENT_ADDR         0x10
#define SX127X_REG_IRQ_FLAGS                    0x12
#define SX127X_REG_RX_NB_BYTES                  0x13
#define SX127X_REG_PKT_SNR_VALUE                0x19
#define SX127X_REG_PKT_RSSI_VALUE               0x1A
#define SX127X_REG_RSSI_VALUE                   0x1B
#define SX127X_REG_MODEM_CONFIG_1               0x1D
#define SX127X_REG_MODEM_CONFIG_2               0x1E
#define SX127X_REG_SYMB_TIMEOUT                 0x1F
#define SX127X_REG_PREAMBLE_MSB                 0x20
#define SX127X_REG_PREAMBLE_LSB                 0x21
#define SX127X_REG_PAYLOAD_LENGTH               0x22
#define SX127X_REG_MODEM_CONFIG_3               0x26
#define SX127X_REG_FREQ_ERROR_MSB               0x28
#define SX127X_REG_FREQ_ERROR_MID               0x29
#define SX127X_REG_FREQ_ERROR_LSB               0x2A
#define SX127X_REG_RSSI_WIDEBAND                0x2C
#define SX127X_REG_DETECTION_OPTIMIZE           0x31
#define SX127X_REG_INVERTIQ                     0x33
#define SX127X_REG_DETECTION_THRESHOLD          0x37
#define SX127X_REG_SYNC_WORD                    0x39
#define SX127X_REG_INVERTIQ2                    0x3B
#define SX127X_REG_DIO_MAPPING_1                0x40
#define SX127X_REG_VERSION                      0x42
#define SX127X_REG_TCXO                         0x4B
#define SX127X_REG_PA_DAC                       0x4D

/***************************************************************/

// Modem options
#define SX127X_FSK_MODEM                        0x00        // GFSK packet type
#define SX127X_LORA_MODEM                       0x01        // LoRa packet type
#define SX127X_OOK_MODEM                        0x02        // OOK packet type

// Long range mode and Modulation type
#define SX127X_LONG_RANGE_MODE                  0x80        // LoRa packet type
#define SX127X_MODULATION_OOK                   0x20        // OOK packet type
#define SX127X_MODULATION_FSK                   0x00        // GFSK packet type

// Devices modes
#define SX127X_MODE_SLEEP                       0x00        // sleep
#define SX127X_MODE_STDBY                       0x01        // standby
#define SX127X_MODE_TX                          0x03        // transmit
#define SX127X_MODE_RX_CONTINUOUS               0x05        // continuous receive
#define SX127X_MODE_RX_SINGLE                   0x06        // single receive
#define SX127X_MODE_CAD                         0x07        // channel activity detection (CAD)

// Rx operation mode
#define SX127X_RX_SINGLE                        0x000000    // Rx timeout duration: no timeout (Rx single mode)
#define SX127X_RX_CONTINUOUS                    0xFFFFFF    //                      infinite (Rx continuous mode)

// TX power options
#define SX127X_TX_POWER_RFO                     0x00        // output power is limited to +14 dBm
#define SX127X_TX_POWER_PA_BOOST                0x80        // output power is limited to +20 dBm

// RX gain options
#define SX127X_RX_GAIN_POWER_SAVING             0x00        // gain used in Rx mode: power saving gain (default)
#define SX127X_RX_GAIN_BOOSTED                  0x01        // boosted gain
#define SX127X_RX_GAIN_AUTO                     0x00        // option enable auto gain controller (AGC)

// Header type
#define SX127X_HEADER_EXPLICIT                  0x00        // explicit header mode
#define SX127X_HEADER_IMPLICIT                  0x01        // implicit header mode

// LoRa syncword
#define SX127X_SYNCWORD_LORAWAN                 0x34        // reserved LoRaWAN syncword

// Oscillator options
#define SX127X_OSC_CRYSTAL                      0x00        // crystal oscillator with external crystal
#define SX127X_OSC_TCXO                         0x10        // external clipped sine TCXO AC-connected to XTA pin

// DIO mapping
#define SX127X_DIO0_RX_DONE                     0x00        // set DIO0 interrupt for: RX done
#define SX127X_DIO0_TX_DONE                     0x40        //                         TX done
#define SX127X_DIO0_CAD_DONE                    0x80        //                         CAD done

// IRQ flags
#define SX127X_IRQ_CAD_DETECTED                 0x01        // Valid Lora signal detected during CAD operation
#define SX127X_IRQ_FHSS_CHANGE                  0x02        // FHSS change channel interrupt
#define SX127X_IRQ_CAD_DONE                     0x04        // channel activity detection finished
#define SX127X_IRQ_TX_DONE                      0x08        // packet transmission completed
#define SX127X_IRQ_HEADER_VALID                 0x10        // valid LoRa header received
#define SX127X_IRQ_CRC_ERR                      0x20        // wrong CRC received
#define SX127X_IRQ_RX_DONE                      0x40        // packet received
#define SX127X_IRQ_RX_TIMEOUT                   0x80        // waiting packet received timeout

// Rssi offset
#define SX127X_RSSI_OFFSET_LF                   164         // low band frequency RSSI offset
#define SX127X_RSSI_OFFSET_HF                   157         // high band frequency RSSI offset
#define SX1272_RSSI_OFFSET                      139         // frequency RSSI offset for SX1272
#define SX127X_BAND_THRESHOLD                   525E6       // threshold between low and high band frequency

#define SX126X_CR_4_8                           4

// Spreading Factor
#define SF7                                     0x07
#define SF8                                     0x08
#define SF9                                     0x09
#define SF10                                    0x0A
#define SF11                                    0x0B
#define SF12                                    0x0C

#define SX127X_BW_7                     0x00        // 7.81 kHz real
#define SX127X_BW_10                    0x01        // 10.42 kHz real
#define SX127X_BW_15                    0x02        // 15.63 kHz real
#define SX127X_BW_20                    0x03        // 20.83 kHz real
#define SX127X_BW_31                    0x04        // 31.25 kHz real
#define SX127X_BW_41                    0x05        // 41.67 kHz real
#define SX127X_BW_62                    0x06        // 62.50 kHz real
#define SX127X_BW_125                   0x07        // 125 kHz real
#define SX127X_BW_250                   0x08        // 250 kHz real
#define SX127X_BW_500                   0x09        // 500 kHz real

#define SX127X_CR_4_5                   0x01
#define SX127X_CR_4_6                   0x02
#define SX127X_CR_4_7                   0x03
#define SX127X_CR_4_8                   0x04

#define PA_OUTPUT_RFO_PIN               0
#define PA_OUTPUT_PA_BOOST_PIN          1

//------ POWER GAIN ------//
#define POWER_11db				0xF6
#define POWER_14db				0xF9
#define POWER_17db				0xFC
#define POWER_20db				0xFF

typedef struct sx127x_cfg{
    uint8_t StdbyConfig;        // SX126X_STDBY_RC
    uint8_t PacketType;         // SX126X_PACKET_TYPE_GFSK / SX126X_PACKET_TYPE_LORA
    uint32_t RfFreq;            // задаем частоту передатчика
    uint8_t Power;              // 1) - 17 (0xEF) to +14 (0x0E) dBm by step of 1 dB if low power PA is selected
                                // 2) - 9 (0xF7) to +22 (0x16) dBm by step of 1 dB if high power PA is selected
    uint8_t RampTime;           // SX126X_RAMP_800U
    uint8_t TX_Base_Address;    // 0
    uint8_t RX_Base_Address;    // 128
    uint8_t spredingFactor;     // фактор распространения SF
    uint8_t BandWidth;          // SX126X_BW_62
    uint8_t crcRate;            // SX126X_CR_4_7
    uint8_t LowDataRateOptimize; // 0x00
    uint16_t SyncWord;          //  0x3444 for Public Network, 0x1424 for Private Network
    uint16_t Preamble;       // 0x000C
    uint8_t payloadLength;
    uint8_t HeaderMode;         // 0x00 or 0x01
    uint8_t crcType;            // SX126X_CRC_OFF
    uint8_t InvertIq;           // 0x00
    uint8_t IrqStatus;
    uint8_t bufferIndex;
    // uint8_t CS_pin[2];       // GPIOC, GPIO_PIN_5
}sx127x_cfg;

uint8_t sx127x_Init(SPI_HandleTypeDef* hspi, sx127x_cfg* modem);
void LoRa_begin(SPI_HandleTypeDef* hspi);
void writeReg(SPI_HandleTypeDef* hspi, uint8_t address, uint8_t* pData, uint8_t len);
void setOCP(SPI_HandleTypeDef* hspi, uint8_t current);
void setPower(SPI_HandleTypeDef* hspi, sx127x_cfg* modem, uint8_t outputPin);
void setLNA(SPI_HandleTypeDef* hspi);
void setBandwidth_CRC(SPI_HandleTypeDef* hspi, sx127x_cfg* modem);
void setPreamble(SPI_HandleTypeDef* hspi, sx127x_cfg* modem);
void setStandby(SPI_HandleTypeDef* hspi);
void readReg(SPI_HandleTypeDef* hspi, uint8_t address, uint8_t* pData, uint8_t len);
void setFrequency(SPI_HandleTypeDef* hspi, sx127x_cfg* modem);
void setSpreadingFactor(SPI_HandleTypeDef* hspi, sx127x_cfg* modem);
void setCRC_ON(SPI_HandleTypeDef* hspi);
void setTimeout(SPI_HandleTypeDef* hspi);
uint8_t sx127x_Transmit(SPI_HandleTypeDef* hspi, sx127x_cfg* modem, uint8_t* data, uint8_t len);
void sendMode(SPI_HandleTypeDef* hspi, sx127x_cfg* modem);
void setBandwidth(SPI_HandleTypeDef* hspi, sx127x_cfg* modem);
void setPayloadLength(SPI_HandleTypeDef* hspi, sx127x_cfg* modem);
void setSyncWord(SPI_HandleTypeDef* hspi, sx127x_cfg* modem);
uint8_t available(SPI_HandleTypeDef* hspi, sx127x_cfg* modem);
uint8_t sx127x_Recive(SPI_HandleTypeDef* hspi, sx127x_cfg* modem);
void setAGC(SPI_HandleTypeDef* hspi);
void setSleep(SPI_HandleTypeDef* hspi);
void gotoMode(SPI_HandleTypeDef* hspi, uint8_t mode);
void setLoRaMode(SPI_HandleTypeDef* hspi);

#endif // MYLIB_SX127x_H_