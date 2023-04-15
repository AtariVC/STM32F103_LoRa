#include "sx126x.h"
#include "main.h"
#include "gpio_ctrl.h"

// sx126x_config lora;


void sx126x_Init(SPI_HandleTypeDef* hspi, sx126x_cfg* modem){
    SetStandby(hspi, modem);
    SetPacketType(hspi, modem);
    SetRfFrequency(hspi, modem);
    SetTxParams(hspi, modem);
    SetModulationParams(hspi, modem);
    GetDeviceErrors(hspi);
    uint8_t extra_tx_params[3] = {0x08, 0x89, 0x01};
    SetDioIrqParams(hspi, 0x3F, 1 << 1, 0, 0);
    WriteReg(hspi, extra_tx_params, 3);
    SetSyncWord(hspi, modem);
    SetBufferBaseAddress(hspi, modem);
}
void sx126x_SendData(SPI_HandleTypeDef* hspi, sx126x_cfg* modem, uint8_t* data, uint16_t data_len){
    SetPacketParams(hspi, modem->PreambleLen, modem->HeaderType, data_len, modem->crcType, modem->InvertIq);
    WriteBuffer(hspi, data, data_len);
    SetTx(hspi, 1);
    // while(!modem->IrqStatus){
    //     GetIrqStatus(hspi, modem);
    // }
    // ClearIrqStatus(hspi, 0x01);
    uint8_t ans[1] ={0};
    getAnswer(hspi, OPCODE_GET_STATUS, ans, 1);
}
// Режим состояния готовности
void SetStandby(SPI_HandleTypeDef* hspi, sx126x_cfg* modem){
    sendOpcode(hspi, OPCODE_SET_STANDY, &modem->StdbyConfig, 1);
}
// Устанавливаеv модуль в режим LoRa или в FSK
void SetPacketType(SPI_HandleTypeDef* hspi, sx126x_cfg* modem){
    sendOpcode(hspi, OPCODE_SET_PACKET_TYPE, &modem->PacketType, 1);
}
// Частота передатчика
void SetRfFrequency(SPI_HandleTypeDef* hspi, sx126x_cfg* modem){
    uint32_t RF_Freq = ((uint32_t)(modem->RfFreq / 32000000.0) * (1 << 25) );  // 0x36000000, частота кристала 32MHz
    uint8_t data[4] = {(RF_Freq >> 24)  & 0xFF, (RF_Freq >> 16) & 0xFF, (RF_Freq >> 8) & 0xFF, RF_Freq & 0xFF};  // [0x36, 0x00, 0, 0]
    sendOpcode(hspi, OPCODE_SET_RF_FREQUENCY, data, 4);
}
// Выходная мощность и время линейного изменения
void SetTxParams(SPI_HandleTypeDef* hspi, sx126x_cfg* modem){
    uint8_t TxParams[2] = {modem->Power, modem->RampTime};
    sendOpcode(hspi, OPCODE_SET_TX_PARAMS, TxParams, 2);
}
// Устанавливает место хранения буффера
void SetBufferBaseAddress(SPI_HandleTypeDef* hspi, sx126x_cfg* modem){
    uint8_t BaseAdress[2] = {modem->TX_Base_Address, modem->RX_Base_Address};
    sendOpcode(hspi, OPCODE_SET_BUFFER_BASE_ADDR, BaseAdress, 2);
}
// Параметры модуляции
void SetModulationParams(SPI_HandleTypeDef* hspi, sx126x_cfg* modem){
    uint8_t ModulationParams[4] = {modem->spredingFactor, modem->BandWidth, modem->crcRate, modem->LowDataRateOptimize};
    sendOpcode(hspi, OPCODE_SET_MODULATION_PARAMS, ModulationParams, 4);
}
// Слово синхронизации
void SetSyncWord(SPI_HandleTypeDef* hspi, sx126x_cfg* modem){
    uint8_t data[4] = {0x07, 0x40, modem->SyncWord >> 8, modem->SyncWord & 0xFF};
    WriteReg(hspi, data, 4);
}
// Получить статус отправки данных
void GetIrqStatus(SPI_HandleTypeDef* hspi, sx126x_cfg* modem){
    uint8_t buf[3] = {0};
    getAnswer(hspi, OPCODE_GET_IRQ_STATUS, buf, 3);
    for(uint8_t i = 0; i < 8; i++)
        *((uint8_t *)(&(modem->IrqStatus)) + i) = (buf[2] >> i) & 0x01;
    *((uint8_t *)(&(modem->IrqStatus)) + 8) = (buf[1] >> 0) & 0x01;
    *((uint8_t *)(&(modem->IrqStatus)) + 9) = (buf[1] >> 1) & 0x01;
}
// Запись в регистр
void WriteReg(SPI_HandleTypeDef* hspi, uint8_t* data, uint16_t len){
    sendOpcode(hspi, OPCODE_WRITE_REGISTER, data, len);
}
// Определяет формат кадра
void SetPacketParams(SPI_HandleTypeDef* hspi, uint16_t preamble_len, uint8_t header_type,
                            uint8_t payload_len, uint8_t crc_type, uint8_t invert_iq){
    uint8_t PacketParams[6] = {(uint8_t)(preamble_len >> 8), (uint8_t)(preamble_len & 0xFF), header_type,
                       payload_len, crc_type, invert_iq};
    sendOpcode(hspi, OPCODE_SET_PACKET_PARAMS, PacketParams, 6);
}
// Запись в буфер
void WriteBuffer(SPI_HandleTypeDef* hspi, uint8_t* data,  uint16_t len){
    sendOpcode(hspi, OPCODE_WRITE_BUFFER, data, len);
}
// Усановка в режим передатчика
void SetTx(SPI_HandleTypeDef* hspi, uint16_t timeout_ms){
    uint32_t timeout = (uint32_t)(timeout_ms / 0.015625);
    uint8_t data[3] = {(timeout >> 16) & 0xFF, (timeout >> 8) & 0xFF, timeout & 0xFF};
    sendOpcode(hspi, OPCODE_SET_TX, data, 3);
}
// Ставим в 1 бит прерывания
void ClearIrqStatus(SPI_HandleTypeDef* hspi, uint16_t param){
    uint8_t data[2] = {(param >> 8) & 0xFF, param & 0xFF};
    sendOpcode(hspi, OPCODE_CLEAR_IRQ_STATUS, data, 2);
}
// Вернуть ошибку устройства
uint16_t GetDeviceErrors(SPI_HandleTypeDef* hspi){
    uint8_t buf[3] = {0};
    getAnswer(hspi, OPCODE_GET_DEVICE_ERRORS, buf, 3);
    return (uint16_t)((buf[1] << 8) | buf[2]);
}
// Получить ответ от устройства
void getAnswer(SPI_HandleTypeDef* hspi, uint8_t opCode, uint8_t* buf, uint16_t len){
    uint32_t Timeout = 2;
    HAL_SPI_Transmit(hspi, &opCode, 1, Timeout);
    HAL_SPI_Receive(hspi, buf, len, Timeout);
}

void SetDioIrqParams(SPI_HandleTypeDef* hspi, uint16_t irq_mask, uint16_t dio1_mask, uint16_t dio2_mask, uint16_t dio3_mask){
    uint8_t data[8] = {irq_mask >> 8, irq_mask & 0xFF, dio1_mask >> 8, dio1_mask & 0xFF, dio2_mask >> 8, dio2_mask & 0xFF, dio3_mask >> 8, dio3_mask & 0xFF};
    sendOpcode(hspi, OPCODE_SET_DIO_IRQ_PARAMS, data, 8);
}
// Передача опкода и данных
void sendOpcode(SPI_HandleTypeDef* hspi, uint8_t opCode, uint8_t* pData, uint16_t param_len){
    uint32_t Timeout = 2;
    HAL_SPI_Transmit(hspi, &opCode, 1, Timeout);
    HAL_SPI_Transmit(hspi, pData, param_len, Timeout);
}
/// ???????????????????????????????
void spiRead(SPI_HandleTypeDef* hspi, uint8_t regAddr, uint8_t* pData){
	HAL_StatusTypeDef ret;
	uint8_t address = regAddr | 0x80;
	uint8_t senddata[2] = {address, 0};
	uint8_t receivedata[2] = {0};

	ret = HAL_SPI_Receive(hspi, pData, 2, 20);
	if(ret != HAL_OK){
		return ret;
	}
	return ret;
}
/// ???????????????????????????????