#include "sx127x.h"
#include "main.h"
#include "stdint.h"

uint8_t sx127x_Init(SPI_HandleTypeDef* hspi, sx127x_cfg* modem){
    uint8_t version;
    readReg(hspi, SX127X_REG_VERSION, &version, 1);
    if (version != 0x12){
        return 0;
    }
    gotoMode(hspi, SX127X_MODE_SLEEP);
    // Включение режима LoRa
    setLoRaMode(hspi);
    setFrequency(hspi, modem);
    // Мощность передатчика
    setPower(hspi, modem, PA_OUTPUT_PA_BOOST_PIN);
    setOCP(hspi, 100);
    // Установка начального адриса FIFO
    writeReg(hspi, SX127X_REG_FIFO_TX_BASE_ADDR, &modem->TX_Base_Address, 1);
    writeReg(hspi, SX127X_REG_FIFO_RX_BASE_ADDR, &modem->RX_Base_Address, 1);
    // Усиление низкого уровня шума
    setLNA(hspi);
    // Автоматическая регулировка усиления
    setAGC(hspi);
    setSpreadingFactor(hspi, modem);
    setBandwidth_CRC(hspi, modem);
    setTimeout(hspi);
    setCRC_ON(hspi);
    setPreamble(hspi, modem);
    // setPayloadLength(hspi, modem);
    // setSyncWord(hspi, modem);
    gotoMode(hspi, SX127X_MODE_STDBY);
    return 1;
}

uint8_t sx127x_Transmit(SPI_HandleTypeDef* hspi, sx127x_cfg* modem, uint8_t* data, uint8_t len){
    uint8_t irq = 0xFF;
    uint8_t read;
    int timeout = 500;

    gotoMode(hspi, SX127X_MODE_STDBY);
    readReg(hspi, SX127X_REG_FIFO_TX_BASE_ADDR, &read, 1);
    writeReg(hspi, SX127X_REG_FIFO_ADDR_PTR, &read, 1);
    writeReg(hspi, SX127X_REG_PAYLOAD_LENGTH, &len, 1);
    writeReg(hspi, SX127X_REG_FIFO, &data, len);
    gotoMode(hspi, SX127X_MODE_TX);
    writeReg(hspi, SX127X_REG_FIFO, &modem->bufferIndex, 1);
    while(1){
		readReg(hspi, SX127X_REG_IRQ_FLAGS, &read, 1);
		if((read & 0x08)!=0){
			writeReg(hspi, SX127X_REG_IRQ_FLAGS, &irq, 1);
			gotoMode(hspi, SX127X_MODE_STDBY);
			return 1;
		}
		else{
			if(--timeout==0){
				gotoMode(hspi, SX127X_MODE_STDBY);
				return 0;
			}
		}
	}

    // setStandby(hspi, modem);
}

uint8_t sx127x_Receive(SPI_HandleTypeDef* hspi, sx127x_cfg* modem, uint8_t* data, uint8_t len){
	uint8_t read;
	uint8_t number_of_bytes;
	uint8_t min = 0;
    uint8_t irq = 0xFF;

	for(int i=0; i<len; i++)
		data[i]=0;

	gotoMode(hspi, SX127X_MODE_STDBY);
	readReg(hspi, SX127X_REG_IRQ_FLAGS, &read, 1);
	if((read & 0x40) != 0){
		writeReg(hspi, SX127X_REG_IRQ_FLAGS, &irq, 1);
		readReg(hspi, SX127X_REG_RX_NB_BYTES, &number_of_bytes, 1);
		readReg(hspi, SX127X_REG_FIFO_RX_CURRENT_ADDR, &read, 1);
		writeReg(hspi, SX127X_REG_FIFO_ADDR_PTR, &read, 1);
		min = len >= number_of_bytes ? number_of_bytes : len;
		for(int i=0; i<min; i++)
			readReg(hspi, SX127X_REG_FIFO_TX_BASE_ADDR, &data[i], 1);
	}
	gotoMode(hspi, SX127X_MODE_RX_CONTINUOUS);
    return min;
}

void setLoRaMode(SPI_HandleTypeDef* hspi){
    uint8_t    data;
	uint8_t    read;
    readReg(hspi, SX127X_REG_OP_MODE, &read, 1);
	data = read | 0x80;
	writeReg(hspi, SX127X_REG_OP_MODE, &data, 1);
}

void gotoMode(SPI_HandleTypeDef* hspi, uint8_t mode){
    uint8_t read;
	uint8_t data;
    readReg(hspi, SX127X_REG_OP_MODE, &read, 1);
	data = read;

	if(mode == SX127X_MODE_SLEEP){
		data = (read & 0xF8) | 0x00;
	}else if (mode == SX127X_MODE_STDBY){
		data = (read & 0xF8) | 0x01;
	}else if (mode == SX127X_MODE_TX){
		data = (read & 0xF8) | 0x03;
	}else if (mode == SX127X_MODE_RX_CONTINUOUS){
		data = (read & 0xF8) | 0x05;
	}else if (mode == SX127X_MODE_RX_SINGLE){
		data = (read & 0xF8) | 0x06;
	}
	writeReg(hspi, SX127X_REG_OP_MODE, &data, 1);
}

void setAGC(SPI_HandleTypeDef* hspi){
    uint8_t AGC = 0x04;
    writeReg(hspi, SX127X_REG_MODEM_CONFIG_3, &AGC, 1);
}
uint8_t available(SPI_HandleTypeDef* hspi, sx127x_cfg* modem){
    uint8_t bytes;
    readReg(hspi, SX127X_REG_RX_NB_BYTES, &bytes, 1);
    return (bytes - modem->bufferIndex);
}

uint8_t parserPacket(SPI_HandleTypeDef* hspi, sx127x_cfg* modem, uint8_t size){
    uint8_t lengh_packet = 0;
    readReg(hspi, SX127X_REG_IRQ_FLAGS, &modem->IrqStatus, 1);

}


void setSyncWord(SPI_HandleTypeDef* hspi, sx127x_cfg* modem){
    writeReg(hspi, SX127X_REG_SYNC_WORD, &modem->SyncWord, 1);
}

void LoRa_begin(SPI_HandleTypeDef* hspi){
    uint8_t version = 0x00;
    uint8_t read;
    // while (version != 0x12 && version != 0x22) {
    //     readReg(hspi, SX127X_REG_VERSION, &read, 1);
    //     version = read;
    // }
    uint8_t data = 0x80;
    writeReg(hspi, SX127X_REG_OP_MODE, &data, 1);
    data |= SX127X_MODE_STDBY;
    writeReg(hspi, SX127X_REG_OP_MODE, &data, 1);
}

void setBandwidth(SPI_HandleTypeDef* hspi, sx127x_cfg* modem){
    writeReg(hspi, SX127X_REG_MODEM_CONFIG_1, &modem->BandWidth, 1);
}

void setSleep(SPI_HandleTypeDef* hspi){
    uint8_t data = SX127X_LONG_RANGE_MODE | SX127X_MODE_SLEEP;
    writeReg(hspi, SX127X_REG_OP_MODE, &data, 1);
}

void sendMode(SPI_HandleTypeDef* hspi, sx127x_cfg* modem){
    uint8_t data;
    uint8_t read;
    setPayloadLengt(hspi, modem);
    readReg(hspi, SX127X_REG_OP_MODE, &read, 1);
    data = read;
    data |= SX127X_MODE_TX | 0x80;
    writeReg(hspi, SX127X_REG_OP_MODE, &data, 1);
}

void setFrequency(SPI_HandleTypeDef* hspi, sx127x_cfg* modem){
    uint8_t  data;
	uint32_t F;

    F = ((uint64_t) modem->RfFreq << 19) / 32000000;

    // write Msb:
	data = (uint8_t) (F >> 16);
	writeReg(hspi, SX127X_REG_FRF_MSB, &data, 1);

	// write Mid:
	data = (uint8_t) (F >> 8);
	writeReg(hspi, SX127X_REG_FRF_MID, &data, 1);

	// write Lsb:
	data = (uint8_t) (F >> 0);
	writeReg(hspi, SX127X_REG_FRF_LSB, &data, 1);
}

void setPower(SPI_HandleTypeDef* hspi, sx127x_cfg* modem, uint8_t outputPin){
    // if(PA_OUTPUT_RFO_PIN == outputPin){
    //     // RFO
    //     if(modem->Power < 0){
    //         modem->Power = 0;
    //     }else if(modem->Power > 14){
    //         modem->Power = 14;
    //     }
    //     modem->Power |= 0x70;
    //     writeReg(hspi, SX127X_REG_PA_CONFIG, modem->Power, 1);
    // }else{
    //     // PA BOOST
    //     if(modem->Power > 17){
    //         if(modem->Power > 20){
    //             modem->Power = 20;
    //         }
    //         modem->Power -= 3;
    //         uint8_t data = 0x87;
    //         // High Power +20 dBm
    //         writeReg(hspi, SX127X_REG_PA_DAC, &data, 1);
    //         // Защита по току
    //         setOCP(hspi, 140);
    //     }else{
    //         if(modem->Power < 2){
    //             modem->Power = 2;
    //         }
    //         // +17dBm
    //         uint8_t data = 0x84;
    //         writeReg(hspi, SX127X_REG_PA_DAC, &data, 1);
    //         setOCP(hspi, 100);
    //     }
    //     uint8_t data = SX127X_TX_POWER_PA_BOOST | (modem->Power - 2);
    //     writeReg(hspi, SX127X_REG_PA_DAC, &data, 1);
    // }
    writeReg(hspi, SX127X_REG_PA_CONFIG, &modem->Power, 1);
}

// set over current protection:
void setOCP(SPI_HandleTypeDef* hspi, uint8_t current){
    uint8_t	OcpTrim = 0;

	if(current<45)
		current = 45;
	if(current>240)
		current = 240;

	if(current <= 120)
		OcpTrim = (current - 45)/5;
	else if(current <= 240)
		OcpTrim = (current + 30)/10;

    // OcpTrim = OcpTrim + (1 << 5);
    OcpTrim = 0x20 | (0xF1 & OcpTrim);
	writeReg(hspi, SX127X_REG_OCP, &OcpTrim, 1);
    // // enable or disable +20 dBm option on PA_BOOST pin
    // uint8_t pa_boost = 0x04;
    // writeReg(hspi, SX127X_REG_PA_DAC, &pa_boost, 1);
    // // set PA config
    // uint8_t pa_con =0xCF;
    // writeReg(hspi, SX127X_REG_PA_CONFIG, &pa_con, 1);
    // // set gain and boost LNA config
    // uint8_t lna =0x03;
    // writeReg(hspi, SX127X_REG_LNA, &lna, 1);
    // // enable or disable AGC
    // uint8_t agc1 =0xD4;
    // writeReg(hspi, SX127X_REG_MODEM_CONFIG_3, &agc1, 1);
    HAL_Delay(1);
}

// set LNA gain:
void setLNA(SPI_HandleTypeDef* hspi){
    uint8_t lna;
    readReg(hspi, SX127X_REG_LNA, &lna, 1);
    lna |= 0x20;
	writeReg(hspi, SX127X_REG_LNA, &lna, 1);
}

void setSpreadingFactor(SPI_HandleTypeDef* hspi, sx127x_cfg* modem){
    uint8_t	data;
	uint8_t	read;
    uint8_t SF;

	if(modem->spredingFactor > 12)
		SF = 12;
	if(modem->spredingFactor < 7)
		SF = 7;

	readReg(hspi, SX127X_REG_MODEM_CONFIG_2, &read, 1);

	data = (SF << 4) + (read & 0x0F);
	writeReg(hspi, SX127X_REG_MODEM_CONFIG_2, &data, 1);

    uint8_t optimize = 0x03;
    writeReg(hspi, SX127X_REG_DETECTION_OPTIMIZE, &optimize, 1);
    uint8_t threshold = 0x0A;
    writeReg(hspi, SX127X_REG_DETECTION_THRESHOLD, &threshold, 1);

}

void setPayloadLength(SPI_HandleTypeDef* hspi, sx127x_cfg* modem){
    writeReg(hspi, SX127X_REG_PAYLOAD_LENGTH, &modem->payloadLength, 1);
}

void setCRC_ON(SPI_HandleTypeDef* hspi){
    uint8_t read, data;

	readReg(hspi, SX127X_REG_MODEM_CONFIG_2, &read, 1);

	data = read | 0x07;
	writeReg(hspi, SX127X_REG_MODEM_CONFIG_2, &data, 1);
}

void setTimeout(SPI_HandleTypeDef* hspi){
    uint8_t data = 0xFF;
    writeReg(hspi, SX127X_REG_SYMB_TIMEOUT, &data, 1);
}

// set bandwidth, coding rate and expilicit mode:
	// 8 bit RegModemConfig --> | X | X | X | X | X | X | X | X |
	//       bits represent --> |   bandwidth   |     CR    |I/E|

void setBandwidth_CRC(SPI_HandleTypeDef* hspi, sx127x_cfg* modem){
    uint8_t data = 0;
    data = (modem->BandWidth << 4) | (modem->crcRate << 1) | (0 << 0);
    writeReg(hspi, SX127X_REG_MODEM_CONFIG_1, &data, 1);
}

void setPreamble(SPI_HandleTypeDef* hspi, sx127x_cfg* modem){
    uint8_t msb = modem->Preamble >> 8;
    uint8_t lsb = modem->Preamble >> 0;
    writeReg(hspi, SX127X_REG_PREAMBLE_MSB, &msb, 1);
    writeReg(hspi, SX127X_REG_PREAMBLE_LSB, &lsb, 1);
}

void setStandby(SPI_HandleTypeDef* hspi){
    uint8_t data = SX127X_LONG_RANGE_MODE | SX127X_MODE_STDBY;
    writeReg(hspi, SX127X_REG_OP_MODE, &data, 1);
}

void readReg(SPI_HandleTypeDef* hspi, uint8_t address, uint8_t* pData, uint8_t len){
    address &= 0x7F;
    uint32_t Timeout = 2;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 0);
    HAL_SPI_Transmit(hspi, &address, 1, Timeout);
    HAL_SPI_Receive(hspi, pData, len, Timeout);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 1);
}

void writeReg(SPI_HandleTypeDef* hspi, uint8_t address, uint8_t* pData, uint8_t len){
    uint32_t Timeout = 2;
    address |= 0x80;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 0);
    HAL_SPI_Transmit(hspi, &address, 1, Timeout);
    HAL_SPI_Transmit(hspi, pData, len, Timeout);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 1);
}
