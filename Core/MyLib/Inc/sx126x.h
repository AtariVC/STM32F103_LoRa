#ifndef MYLIB_SX126x_H_
#define MYLIB_SX126x_H_
#include "main.h"

//  0x80        0x00
//    |           |
//  Opcode      PacketType

/************ Opcodes ********************/
#define OPCODE_WRITE_BUFFER             0x0e // Запись данных в буффер
#define OPCODE_SET_STANDY               0x80 // Установить состояние готовности
#define OPCODE_SET_TX                   0x83 // Устанавливает устройство в режим transmit
#define OPCODE_SET_RF_FREQUENCY         0x86 // Установка частоты RF
#define OPCODE_SET_PACKET_TYPE          0x8a // Устанавливает модуль в режим LoRa или в FSK (Frequency Shift Keying)
                                             // частотная манипуляция - скачкообразно изменяется частота в зависимости от символа

#define OPCODE_SET_TX_PARAMS            0x8e // Устанавливает мощность и время линейного изменения 1 - power 2 - RampTime
                                     // 1) - 17 (0xEF) to +14 (0x0E) dBm by step of 1 dB if low power PA is selected
                                     // 2) - 9 (0xF7) to +22 (0x16) dBm by step of 1 dB if high power PA is selected
#define OPCODE_SET_BUFFER_BASE_ADDR     0x8f // Устанавливает где будут хранится данные 1 - TX base address, 2 - RX base address
#define OPCODE_SET_PACKET_PARAMS        0x8c // Определяет формат кадра,
#define OPCODE_SET_MODULATION_PARAMS    0x8b // Параметры модуляции
#define OPCODE_WRITE_REGISTER           0x0d // Запись в регистр
#define OPCODE_CLEAR_IRQ_STATUS         0x02 //
#define SetDIO2AsRfSwitchCtrl           0x9D // Настройте радио для управления ВЧ-переключателем с DIO2.
#define OPCODE_GET_DEVICE_ERRORS        0x17 // Возвращает ошибку, которая произошла в устройстве
#define OPCODE_SET_DIO_IRQ_PARAMS       0x08
#define OPCODE_GET_STATUS               0xc0
#define OPCODE_GET_IRQ_STATUS           0x12 // Статус отправки данных
/************ PacketType ********************/
#define SX126X_STDBY_RC                 0x00 // частота кристала 13МГц
#define SX126X_STDBY_XOSC               0x01 // 32 МГц
#define SX126X_PACKET_TYPE_GFSK         0x00 // FSK
#define SX126X_PACKET_TYPE_LORA         0x01 // LoRa

/********************************************/
#define SX126X_RAMP_10U                 0x00
#define SX126X_RAMP_20U                 0x01
#define SX126X_RAMP_40U                 0x02
#define SX126X_RAMP_80U                 0x03
#define SX126X_RAMP_200U                0x04
#define SX126X_RAMP_800U                0x05
#define SX126X_RAMP_1700U               0x06
#define SX126X_RAMP_3400U               0x07

#define SX126X_BW_7                     0x00  // 7.81 kHz real
#define SX126X_BW_10                    0x08  // 10.42 kHz real
#define SX126X_BW_15                    0x01  // 15.63 kHz real
#define SX126X_BW_20                    0x09  // 20.83 kHz real
#define SX126X_BW_31                    0x02  // 31.25 kHz real
#define SX126X_BW_41                    0x0A  // 41.67 kHz real
#define SX126X_BW_62                    0x03  // 62.50 kHz real
#define SX126X_BW_125                   0x04  // 125 kHz real
#define SX126X_BW_250                   0x05  // 250 kHz real
#define SX126X_BW_500                   0x06  // 500 kHz real

#define SX126X_CR_4_5                   0x01
#define SX126X_CR_4_6                   0x02
#define SX126X_CR_4_7                   0x03
#define SX126X_CR_4_8                   0x04

#define SX126X_1_BYTE                   0x00
#define SX126X_2_BYTE                   0x02
#define SX126X_1_BYTE_INV               0x04
#define SX126X_2_BYTE_INV               0x06

 #define SF5                            0x05
 #define SF6                            0x06
 #define SF7                            0x07
 #define SF8                            0x08
 #define SF9                            0x09
 #define SF10                           0x0A
 #define SF11                           0x0B
 #define SF12                           0x0C

#define SX126X_HEADER_TYPE_VARIABLE_LENGTH            0
#define SX126X_HEADER_TYPE_FIXED_LENGTH               1

#define SX126X_CRC_OFF                                0
#define SX126X_CRC_ON                                 1

#define SX126X_STANDARD_IQ                            0
#define SX126X_INVERTED_IQ                            1



typedef struct sx126x_cfg{
    uint8_t StdbyConfig; // SX126X_STDBY_RC
    uint8_t PacketType; // SX126X_PACKET_TYPE_GFSK / SX126X_PACKET_TYPE_LORA
    uint32_t RfFreq; // задаем частоту передатчика
    uint8_t Power; // 1) - 17 (0xEF) to +14 (0x0E) dBm by step of 1 dB if low power PA is selected
                   // 2) - 9 (0xF7) to +22 (0x16) dBm by step of 1 dB if high power PA is selected
    uint8_t RampTime; // SX126X_RAMP_800U
    uint8_t TX_Base_Address; // 0
    uint8_t RX_Base_Address; // 128
    uint8_t spredingFactor; // фактор распространения SF8
    uint8_t BandWidth; // SX126X_BW_62
    uint8_t crcRate; // SX126X_CR_4_7 ??
    uint8_t LowDataRateOptimize; // 0x00
    uint16_t SyncWord; // 0x1424
    uint16_t PreambleLen; // 0x000C
    uint8_t HeaderType; // 0x00 or 0x01
    uint8_t crcType; // SX126X_CRC_OFF
    uint8_t InvertIq; // 0x00
    uint8_t IrqStatus;
    // uint8_t CS_pin[2]; // GPIOC, GPIO_PIN_5
}sx126x_cfg;

void sx126x_Init(SPI_HandleTypeDef* hspi, sx126x_cfg* modem);
void sx126x_SendData(SPI_HandleTypeDef* hspi, sx126x_cfg* modem, uint8_t* data, uint16_t data_len);
void SetStandby(SPI_HandleTypeDef* hspi, sx126x_cfg* modem);
void SetPacketType(SPI_HandleTypeDef* hspi, sx126x_cfg* modem);
void SetRfFrequency(SPI_HandleTypeDef* hspi, sx126x_cfg* modem);
void SetTxParams(SPI_HandleTypeDef* hspi, sx126x_cfg* modem);
void SetBufferBaseAddress(SPI_HandleTypeDef* hspi, sx126x_cfg* modem);
void SetModulationParams(SPI_HandleTypeDef* hspi, sx126x_cfg* modem);
void WriteReg(SPI_HandleTypeDef* hspi, uint8_t* data, uint16_t len);
void SetPacketParams(SPI_HandleTypeDef* hspi, uint16_t preamble_len, uint8_t header_type,
                            uint8_t payload_len, uint8_t crc_type, uint8_t invert_iq);
void WriteBuffer(SPI_HandleTypeDef* hspi, uint8_t* data,  uint16_t len);
void SetTx(SPI_HandleTypeDef* hspi, uint16_t timeout_ms);
void ClearIrqStatus(SPI_HandleTypeDef* hspi, uint16_t param);
void sendOpcode(SPI_HandleTypeDef* hspi, uint8_t opCode, uint8_t* pData, uint16_t param_len);
void spiRead(SPI_HandleTypeDef* hspi, uint8_t regAddr, uint8_t* pData);
uint16_t GetDeviceErrors(SPI_HandleTypeDef* hspi);
void getAnswer(SPI_HandleTypeDef* hspi, uint8_t opCode, uint8_t* buf, uint16_t len);
void SetDioIrqParams(SPI_HandleTypeDef* hspi, uint16_t irq_mask, uint16_t dio1_mask, uint16_t dio2_mask, uint16_t dio3_mask);
void SetSyncWord(SPI_HandleTypeDef* hspi, sx126x_cfg* modem);
#endif // MYLIB_SX126x_H_