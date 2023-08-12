#include "sx126x.h"
#include "main.h"
#include "gpio_ctrl.h"

// sx126x_config lora;


void sx126x_Init(SPI_HandleTypeDef* hspi, sx126x_cfg* modem){
    SetStandby(hspi, modem);    // 1
    SetPacketType(hspi, modem);
    Calibrate(hspi, modem, 0xFF);   //2
    SetRfFrequency(hspi, modem);    //3
    SetPaConfig(hspi, 0x02, 0x03, 0x00, 0x01);  //4
    SetTxParams(hspi, modem);   //5
    SetModulationParams(hspi, modem);   //6
    SetPacketParams(hspi, modem->PreambleLen, modem->HeaderType, modem->payloadLength, modem->crcType, modem->InvertIq); //7
    // GetDeviceErrors(hspi);
    // uint8_t extra_tx_params[3] = {0x08, 0x89, 0x01};
    // SetDioIrqParams(hspi, 0x3F, 1 << 1, 0, 0);
    // WriteReg(hspi, extra_tx_params, 3);
    SetSyncWord(hspi, modem);   //8
    // SetBufferBaseAddress(hspi, modem);
}
void sx126x_SendData(SPI_HandleTypeDef* hspi, sx126x_cfg* modem, uint8_t* data, uint16_t data_len){
    uint8_t txBaseAddress = 0;
    uint8_t rxBaseAddress = txBaseAddress + 0xFF;
    setBufferBaseAddress(hspi, txBaseAddress, rxBaseAddress); //1
    fixLoRaBw500(hspi, modem->BandWidth); //2
    WriteBuffer(hspi, data, data_len, &modem->bufferIndex); //3
    SetPacketParams(hspi, modem->PreambleLen, modem->HeaderType, modem->payloadLength, modem->crcType, modem->InvertIq); //4
    SetTx(hspi, 1); //5
    // while(!modem->IrqStatus){
    //     GetIrqStatus(hspi, modem);
    // }
    // ClearIrqStatus(hspi, 0x01);
    // uint8_t ans[1] ={0};
    // getAnswer(hspi, OPCODE_GET_STATUS, ans, 1);
}
// Режим состояния готовности
void SetStandby(SPI_HandleTypeDef* hspi, sx126x_cfg* modem){
    sendOpcode(hspi, OPCODE_SET_STANDY, &modem->StdbyConfig, 1);
}
void fixLoRaBw500(SPI_HandleTypeDef* hspi, uint8_t bw){
    uint8_t packetType;
    getPacketType(hspi, &packetType);
    uint8_t value;
    ReadReg(hspi, SX126X_REG_TX_MODULATION, &value, 1);
    if ((packetType == SX126X_PACKET_TYPE_LORA) && (bw == SX126X_BW_500)) value &= 0xFB;
    else value |= 0x04;
    WriteReg(hspi, SX126X_REG_TX_MODULATION, &value, 1);
}
void getPacketType(SPI_HandleTypeDef* hspi, uint8_t* packetType){
    uint8_t buf[2];
    getAnswer(hspi, OPCODE_SET_PACKET_TYPE, buf, 2);
    *packetType = buf[1];
}
// Устанавливаеv модуль в режим LoRa или в FSK
void SetPacketType(SPI_HandleTypeDef* hspi, sx126x_cfg* modem){
    sendOpcode(hspi, OPCODE_SET_PACKET_TYPE, &modem->PacketType, 1);
}
// Частота передатчика
void SetRfFrequency(SPI_HandleTypeDef* hspi, sx126x_cfg* modem){
    uint8_t calFreq[2];
    if (modem->RfFreq < 828000000){     // 779 - 787 Mhz
        calFreq[0] = SX126X_CAL_IMG_779;
        calFreq[1] = SX126X_CAL_IMG_787;
    }
    else if (modem->RfFreq < 877000000) {   // 863 - 870 Mhz
        calFreq[0] = SX126X_CAL_IMG_863;
        calFreq[1] = SX126X_CAL_IMG_870;
    }
    else if (modem->RfFreq < 1100000000) {  // 902 - 928 Mhz
        calFreq[0] = SX126X_CAL_IMG_902;
        calFreq[1] = SX126X_CAL_IMG_928;
    }
    uint32_t rfFreq = ((uint64_t)modem->RfFreq << 25 / 32000000);  // RfFreq = Frequency * 2^25 / 32000000, частота кристала 32MHz
    uint8_t data[4] = {(rfFreq >> 24)  & 0xFF, (rfFreq >> 16) & 0xFF, (rfFreq >> 8) & 0xFF, rfFreq & 0xFF};  // [0x36, 0x00, 0, 0]
    CalibrateImage(hspi, calFreq[0], calFreq[1]);
    sendOpcode(hspi, OPCODE_SET_RF_FREQUENCY, data, 4);
}
 void setBufferBaseAddress(SPI_HandleTypeDef* hspi, uint8_t txBaseAddress, uint8_t rxBaseAddress){
    uint8_t buf[2];
    buf[0] = txBaseAddress;
    buf[1] = rxBaseAddress;
    sendOpcode(hspi, OPCODE_SET_BUFER_BASE_ADDRESS, buf, 2);
 }
// Усилитель мощности
void SetPaConfig(SPI_HandleTypeDef* hspi, uint8_t paDutyCycle, uint8_t hpMax, uint8_t deviceSel, uint8_t paLut){
    uint8_t buf[4];
    buf[0] = paDutyCycle;
    buf[1] = hpMax;
    buf[2] = deviceSel;
    buf[3] = paLut;
    sendOpcode(hspi, OPCODE_CALIBRATE, buf, 4);
}
// Каллибровка
void Calibrate(SPI_HandleTypeDef* hspi, sx126x_cfg* modem, uint8_t calibParam){
    SetStandby(hspi, modem);
    sendOpcode(hspi, OPCODE_CALIBRATE, &calibParam, 1);
}
// Каллибровка изображения
void CalibrateImage(SPI_HandleTypeDef* hspi, uint8_t freq1, uint8_t freq2){
    uint8_t buf[2];
    buf[0] = freq1;
    buf[1] = freq2;
    sendOpcode(hspi, OPCODE_CALIBRATE_IMAGE, buf, 2);
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
    uint8_t ModulationParams[8] = {modem->spredingFactor, modem->BandWidth, modem->crcRate, modem->LowDataRateOptimize, 0x00, 0x00, 0x00, 0x00};
    sendOpcode(hspi, OPCODE_SET_MODULATION_PARAMS, ModulationParams, 8);
}
// Слово синхронизации
void SetSyncWord(SPI_HandleTypeDef* hspi, sx126x_cfg* modem){
    uint16_t adress = 0x0740;
    uint8_t data[2] = {modem->SyncWord >> 8, modem->SyncWord & 0xFF};
    WriteReg(hspi, adress, data, 2);
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
void WriteReg(SPI_HandleTypeDef* hspi, uint16_t adress, uint8_t* data, uint16_t len){
    uint8_t buf[len + 2];
    buf[0] = adress >> 8;
    buf[1] = adress & 0xFF;
    for(uint8_t i; i < len; i++) buf[i + 2] = data[i];
    sendOpcode(hspi, OPCODE_WRITE_REGISTER, buf, len);
}
// Чтение с регистра
void ReadReg(SPI_HandleTypeDef* hspi, uint16_t adress, uint8_t* data, uint16_t len){
    uint8_t addr[2];
    addr[0] = adress >> 8;
    addr[1] = adress & 0xFF;
    sendOpcodeAddressReadReg(hspi, OPCODE_READ_REGISTER, data, len, addr);
}
// Определяет формат кадра
void SetPacketParams(SPI_HandleTypeDef* hspi, uint16_t preamble_len, uint8_t header_type,
                            uint8_t payload_len, uint8_t crc_type, uint8_t invert_iq){
    uint8_t buf[9] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    buf[0] = preamble_len >> 8;
    buf[1] = preamble_len & 0xFF;
    buf[2] = header_type;
    buf[3] = payload_len;
    buf[4] = crc_type;
    buf[5] = invert_iq;
    sendOpcode(hspi, OPCODE_SET_PACKET_PARAMS, buf, 8);
}
// Запись в буфер
void WriteBuffer(SPI_HandleTypeDef* hspi, uint8_t* data,  uint16_t len, uint8_t* offset){
    uint8_t buf[len +1];
    buf[0] = *offset;
    for(uint8_t i = 0; i < len; i++) buf[i + 1] = data[i];
    sendOpcode(hspi, OPCODE_WRITE_BUFFER, buf, len + 1);
    (*offset) += len;
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
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 0);
    HAL_SPI_Transmit(hspi, &opCode, 1, Timeout);
    HAL_SPI_Receive(hspi, buf, len, Timeout);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 1);
}

void SetDioIrqParams(SPI_HandleTypeDef* hspi, uint16_t irq_mask, uint16_t dio1_mask, uint16_t dio2_mask, uint16_t dio3_mask){

    uint8_t data[8] = {irq_mask >> 8, irq_mask & 0xFF, dio1_mask >> 8, dio1_mask & 0xFF, dio2_mask >> 8, dio2_mask & 0xFF, dio3_mask >> 8, dio3_mask & 0xFF};
    sendOpcode(hspi, OPCODE_SET_DIO_IRQ_PARAMS, data, 8);
}
// Передача опкода и данных
void sendOpcode(SPI_HandleTypeDef* hspi, uint8_t opCode, uint8_t* pData, uint16_t param_len){
    uint32_t Timeout = 2;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 0);
    HAL_SPI_Transmit(hspi, &opCode, 1, Timeout);
    HAL_SPI_Transmit(hspi, pData, param_len, Timeout);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 1);
}
// Передача опкода и адресса для чтения из регистра
void sendOpcodeAddressReadReg(SPI_HandleTypeDef* hspi, uint8_t opCode, uint8_t* pData, uint16_t param_len, uint8_t* address){
    uint32_t Timeout = 2;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 0);
    HAL_SPI_Transmit(hspi, &opCode, 1, Timeout);
    HAL_SPI_Transmit(hspi, address, 2, Timeout);
    HAL_SPI_Receive(hspi, pData, param_len, Timeout);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 1);
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
/// ???????????????????????????????WWEEE