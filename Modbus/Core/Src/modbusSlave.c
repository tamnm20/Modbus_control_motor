/*
 * modbusSlave.c
 *
 *  Created on: Oct 27, 2022
 *      Author: controllerstech.com
 */

#include "modbusSlave.h"
#include "string.h"
UART_HandleTypeDef huart1;

uint16_t Holding_Registers_Database[50]={
		0000,  0000,  0001,  3333,  4444,  5555,  6666,  7777,  8888,  9999,   // 0-9   40001-40010
		12345, 15432, 15535, 10234, 19876, 13579, 10293, 19827, 13456, 14567,  // 10-19 40011-40020
		21345, 22345, 24567, 25678, 26789, 24680, 20394, 29384, 26937, 27654,  // 20-29 40021-40030
		31245, 31456, 34567, 35678, 36789, 37890, 30948, 34958, 35867, 36092,  // 30-39 40031-40040
		45678, 46789, 47890, 41235, 42356, 43567, 40596, 49586, 48765, 41029,  // 40-49 40041-40050
};

static const uint16_t Input_Registers_Database[50]={
		0000,  1111,  2222,  3333,  4444,  5555,  6666,  7777,  8888,  9999,   // 0-9   30001-30010
		12345, 15432, 15535, 10234, 19876, 13579, 10293, 19827, 13456, 14567,  // 10-19 30011-30020
		21345, 22345, 24567, 25678, 26789, 24680, 20394, 29384, 26937, 27654,  // 20-29 30021-30030
		31245, 31456, 34567, 35678, 36789, 37890, 30948, 34958, 35867, 36092,  // 30-39 30031-30040
		45678, 46789, 47890, 41235, 42356, 43567, 40596, 49586, 48765, 41029,  // 40-49 30041-30050
};

uint8_t Coils_Database[25]={
		0b00000000, 0b00000000, 0b10101010, 0b01010101, 0b11001100,    // 0-39    1-40
		0b10100011, 0b01100110, 0b10101111, 0b01100000, 0b10111100,    // 40-79   41-80
		0b11001100, 0b01101100, 0b01010011, 0b11111111, 0b00000000,    // 80-119  81-120
		0b01010101, 0b00111100, 0b00001111, 0b11110000, 0b10001111,    // 120-159 121-160
		0b01010100, 0b10011001, 0b11111000, 0b00001101, 0b00101010,    // 160-199 161-200
};

static const uint8_t Inputs_Database[25]={
		0b01001001, 0b10011100, 0b10101010, 0b01010101, 0b11001100,    // 0-39    10001-10040
		0b10100011, 0b01100110, 0b10101111, 0b01100000, 0b10111100,    // 40-79   10041-10080
		0b11001100, 0b01101100, 0b01010011, 0b11111111, 0b00000000,    // 80-119  10081-10120
		0b01010101, 0b00111100, 0b00001111, 0b11110000, 0b10001111,    // 120-159 10121-10160
		0b01010100, 0b10011001, 0b11111000, 0b00001101, 0b00101010,    // 160-199 10161-10200
};

void sendData (uint8_t *data, int size)
{
	// we will calculate the CRC in this function itself
	uint16_t crc = crc16(data, size);
	data[size] = crc&0xFF;   // CRC LOW
	data[size+1] = (crc>>8)&0xFF;  // CRC HIGH

	HAL_UART_Transmit(&huart1, data, size+2, 1000);
}

void modbusException (uint8_t exceptioncode)
{
	//| SLAVE_ID | FUNCTION_CODE | Exception code | CRC     |
	//| 1 BYTE   |  1 BYTE       |    1 BYTE      | 2 BYTES |

	TxData[0] = RxData[0];       // slave ID
	TxData[1] = RxData[1]|0x80;  // adding 1 to the MSB of the function code
	TxData[2] = exceptioncode;   // Load the Exception code
	sendData(TxData, 3);         // send Data... CRC will be calculated in the function
}


uint8_t readHoldingRegs (void)
{
	uint16_t startAddr = ((RxData[2]<<8)|RxData[3]);  // start Register Address

	uint16_t numRegs = ((RxData[4]<<8)|RxData[5]);   // number to registers master has requested
	if ((numRegs<1)||(numRegs>125))  // maximum no. of Registers as per the PDF
	{
		modbusException (ILLEGAL_DATA_VALUE);  // send an exception
		return 0;
	}

	uint16_t endAddr = startAddr+numRegs-1;  // end Register
	if (endAddr>49)  // end Register can not be more than 49 as we only have record of 50 Registers in total
	{
		modbusException(ILLEGAL_DATA_ADDRESS);   // send an exception
		return 0;
	}

	// Prepare TxData buffer

	//| SLAVE_ID | FUNCTION_CODE | BYTE COUNT | DATA      | CRC     |
	//| 1 BYTE   |  1 BYTE       |  1 BYTE    | N*2 BYTES | 2 BYTES |

	TxData[0] = SLAVE_ID;  // slave ID
	TxData[1] = RxData[1];  // function code
	TxData[2] = numRegs*2;  // Byte count
	int indx = 3;  // we need to keep track of how many bytes has been stored in TxData Buffer

	for (int i=0; i<numRegs; i++)   // Load the actual data into TxData buffer
	{
		TxData[indx++] = (Holding_Registers_Database[startAddr]>>8)&0xFF;  // extract the higher byte
		TxData[indx++] = (Holding_Registers_Database[startAddr])&0xFF;   // extract the lower byte
		startAddr++;  // increment the register address
	}

	sendData(TxData, indx);  // send data... CRC will be calculated in the function itself
	return 1;   // success
}

uint8_t readInputRegs (void)
{
	uint16_t startAddr = ((RxData[2]<<8)|RxData[3]);  // start Register Address

	uint16_t numRegs = ((RxData[4]<<8)|RxData[5]);   // number to registers master has requested
	if ((numRegs<1)||(numRegs>125))  // maximum no. of Registers as per the PDF
	{
		modbusException (ILLEGAL_DATA_VALUE);  // send an exception
		return 0;
	}

	uint16_t endAddr = startAddr+numRegs-1;  // end Register
	if (endAddr>49)  // end Register can not be more than 49 as we only have record of 50 Registers in total
	{
		modbusException(ILLEGAL_DATA_ADDRESS);   // send an exception
		return 0;
	}

	// Prepare TxData buffer

	//| SLAVE_ID | FUNCTION_CODE | BYTE COUNT | DATA      | CRC     |
	//| 1 BYTE   |  1 BYTE       |  1 BYTE    | N*2 BYTES | 2 BYTES |

	TxData[0] = SLAVE_ID;  // slave ID
	TxData[1] = RxData[1];  // function code
	TxData[2] = numRegs*2;  // Byte count
	int indx = 3;  // we need to keep track of how many bytes has been stored in TxData Buffer

	for (int i=0; i<numRegs; i++)   // Load the actual data into TxData buffer
	{
		TxData[indx++] = (Input_Registers_Database[startAddr]>>8)&0xFF;  // extract the higher byte
		TxData[indx++] = (Input_Registers_Database[startAddr])&0xFF;   // extract the lower byte
		startAddr++;  // increment the register address
	}

	sendData(TxData, indx);  // send data... CRC will be calculated in the function itself
	return 1;   // success
}

uint8_t readCoils (void)
{
	uint16_t startAddr = ((RxData[2]<<8)|RxData[3]);  // start Coil Address

	uint16_t numCoils = ((RxData[4]<<8)|RxData[5]);   // number to coils master has requested
	if ((numCoils<1)||(numCoils>2000))  // maximum no. of coils as per the PDF
	{
		modbusException (ILLEGAL_DATA_VALUE);  // send an exception
		return 0;
	}

	uint16_t endAddr = startAddr+numCoils-1;  // Last coils address
	if (endAddr>199)  // end coil can not be more than 199 as we only have record of 200 (0-199) coils in total
	{
		modbusException(ILLEGAL_DATA_ADDRESS);   // send an exception
		return 0;
	}


	//reset TxData buffer
	memset (TxData, '\0', sizeof(TxData));

	// Prepare TxData buffer

	//| SLAVE_ID | FUNCTION_CODE | BYTE COUNT | DATA      | CRC     |
	//| 1 BYTE   |  1 BYTE       |  1 BYTE    | N*2 BYTES | 2 BYTES |

	TxData[0] = SLAVE_ID;  // slave ID
	TxData[1] = RxData[1];  // function code
	TxData[2] = (numCoils/8) + ((numCoils%8)>0 ? 1:0);  // Byte count
	int indx = 3;  // we need to keep track of how many bytes has been stored in TxData Buffer

	/* The approach is simple. We will read 1 bit at a time and store them in the Txdata buffer.
	 * First find the offset in the first byte we read from, for eg- if the start coil is 13,
	 * we will read from database[1] with an offset of 5. This bit will be stored in the TxData[0] at 0th position.
	 * Then we will keep shifting the database[1] to the right and read the bits.
	 * Once the bitposition has crossed the value 7, we will increment the startbyte
	 * When the indxposition exceeds 7, we increment the indx variable, so to copy into the next byte of the TxData
	 * This keeps going until the number of coils required have been copied
	 */
	int startByte = startAddr/8;  // which byte we have to start extracting the data from
	uint16_t bitPosition = startAddr%8;  // The shift position in the first byte
	int indxPosition = 0;  // The shift position in the current indx of the TxData buffer

	// Load the actual data into TxData buffer
	for (int i=0; i<numCoils; i++)
	{
		TxData[indx] |= ((Coils_Database[startByte] >> bitPosition) &0x01) << indxPosition;
		indxPosition++; bitPosition++;
		if (indxPosition>7)  // if the indxposition exceeds 7, we have to copy the data into the next byte position
		{
			indxPosition = 0;
			indx++;
		}
		if (bitPosition>7)  // if the bitposition exceeds 7, we have to increment the startbyte
		{
			bitPosition=0;
			startByte++;
		}
	}

	if (numCoils%8 != 0)indx++;  // increment the indx variable, only if the numcoils is not a multiple of 8
	sendData(TxData, indx);  // send data... CRC will be calculated in the function itself
	return 1;   // success
}

uint8_t readInputs (void)
{
	uint16_t startAddr = ((RxData[2]<<8)|RxData[3]);  // start Register Address

	uint16_t numCoils = ((RxData[4]<<8)|RxData[5]);   // number to coils master has requested
	if ((numCoils<1)||(numCoils>2000))  // maximum no. of coils as per the PDF
	{
		modbusException (ILLEGAL_DATA_VALUE);  // send an exception
		return 0;
	}

	uint16_t endAddr = startAddr+numCoils-1;  // Last coils address
	if (endAddr>199)  // end coil can not be more than 199 as we only have record of 200 (0-199) coils in total
	{
		modbusException(ILLEGAL_DATA_ADDRESS);   // send an exception
		return 0;
	}


	//reset TxData buffer
	memset (TxData, '\0', sizeof(TxData));

	// Prepare TxData buffer

	//| SLAVE_ID | FUNCTION_CODE | BYTE COUNT | DATA      | CRC     |
	//| 1 BYTE   |  1 BYTE       |  1 BYTE    | N*2 BYTES | 2 BYTES |

	TxData[0] = SLAVE_ID;  // slave ID
	TxData[1] = RxData[1];  // function code
	TxData[2] = (numCoils/8) + ((numCoils%8)>0 ? 1:0);  // Byte count
	int indx = 3;  // we need to keep track of how many bytes has been stored in TxData Buffer

	/* The approach is simple. We will read 1 bit at a time and store them in the Txdata buffer.
	 * First find the offset in the first byte we read from, for eg- if the start coil is 13,
	 * we will read from database[1] with an offset of 5. This bit will be stored in the TxData[0] at 0th position.
	 * Then we will keep shifting the database[1] to the right and read the bits.
	 * Once the bitposition has crossed the value 7, we will increment the startbyte
	 * When the indxposition exceeds 7, we increment the indx variable, so to copy into the next byte of the TxData
	 * This keeps going until the number of coils required have been copied
	 */
	int startByte = startAddr/8;  // which byte we have to start extracting the data from
	uint16_t bitPosition = startAddr%8;  // The shift position in the first byte
	int indxPosition = 0;  // The shift position in the current indx of the TxData buffer

	// Load the actual data into TxData buffer
	for (int i=0; i<numCoils; i++)
	{
		TxData[indx] |= ((Inputs_Database[startByte] >> bitPosition) &0x01) << indxPosition;
		indxPosition++; bitPosition++;
		if (indxPosition>7)  // if the indxposition exceeds 7, we have to copy the data into the next byte position
		{
			indxPosition = 0;
			indx++;
		}
		if (bitPosition>7)  // if the bitposition exceeds 7, we have to increment the startbyte
		{
			bitPosition=0;
			startByte++;
		}
	}

	if (numCoils%8 != 0)indx++;  // increment the indx variable, only if the numcoils is not a multiple of 8
	sendData(TxData, indx);  // send data... CRC will be calculated in the function itself
	return 1;   // success
}

uint8_t writeHoldingRegs (void)
{
	uint16_t startAddr = ((RxData[2]<<8)|RxData[3]);  // start Register Address

	uint16_t numRegs = ((RxData[4]<<8)|RxData[5]);   // number to registers master has requested
	if ((numRegs<1)||(numRegs>123))  // maximum no. of Registers as per the PDF
	{
		modbusException (ILLEGAL_DATA_VALUE);  // send an exception
		return 0;
	}

	uint16_t endAddr = startAddr+numRegs-1;  // end Register
	if (endAddr>49)  // end Register can not be more than 49 as we only have record of 50 Registers in total
	{
		modbusException(ILLEGAL_DATA_ADDRESS);   // send an exception
		return 0;
	}

	/* start saving 16 bit data
	 * Data starts from RxData[7] and we need to combine 2 bytes together
	 * 16 bit Data = firstByte<<8|secondByte
	 */
	int indx = 7;  // we need to keep track of index in RxData
//	for (int i=0; i<numRegs; i++)
//	{
//		Holding_Registers_Database[startAddr++] = (RxData[indx++]<<8)|RxData[indx++];
//	}
	for (int i = 0; i < numRegs; i++) {
	    uint8_t high = RxData[indx++];
	    uint8_t low  = RxData[indx++];
	    Holding_Registers_Database[startAddr++] = (high << 8) | low;
	}
	// Prepare Response

	//| SLAVE_ID | FUNCTION_CODE | Start Addr | num of Regs    | CRC     |
	//| 1 BYTE   |  1 BYTE       |  2 BYTE    | 2 BYTES      | 2 BYTES |

	TxData[0] = SLAVE_ID;    // slave ID
	TxData[1] = RxData[1];   // function code
	TxData[2] = RxData[2];   // Start Addr HIGH Byte
	TxData[3] = RxData[3];   // Start Addr LOW Byte
	TxData[4] = RxData[4];   // num of Regs HIGH Byte
	TxData[5] = RxData[5];   // num of Regs LOW Byte

	sendData(TxData, 6);  // send data... CRC will be calculated in the function itself
	return 1;   // success
}

uint8_t writeSingleReg (void)
{
	uint16_t startAddr = ((RxData[2]<<8)|RxData[3]);  // start Register Address

	if (startAddr>49)  // The Register Address can not be more than 49 as we only have record of 50 Registers in total
	{
		modbusException(ILLEGAL_DATA_ADDRESS);   // send an exception
		return 0;
	}

	/* Save the 16 bit data
	 * Data is the combination of 2 bytes, RxData[4] and RxData[5]
	 */

	Holding_Registers_Database[startAddr] = (RxData[4]<<8)|RxData[5];

	// Prepare Response

	//| SLAVE_ID | FUNCTION_CODE | Start Addr | Data     | CRC     |
	//| 1 BYTE   |  1 BYTE       |  2 BYTE    | 2 BYTES  | 2 BYTES |

	TxData[0] = SLAVE_ID;    // slave ID
	TxData[1] = RxData[1];   // function code
	TxData[2] = RxData[2];   // Start Addr HIGH Byte
	TxData[3] = RxData[3];   // Start Addr LOW Byte
	TxData[4] = RxData[4];   // Reg Data HIGH Byte
	TxData[5] = RxData[5];   // Reg Data LOW  Byte

	sendData(TxData, 6);  // send data... CRC will be calculated in the function itself
	return 1;   // success
}
__attribute__((used))
uint8_t writeCoil (void)
{
	uint16_t coilAddr = ((RxData[2]<<8) | RxData[3]);  // Coil address

	if (coilAddr > 199)  // Giới hạn 200 coil (0-199)
	{
		modbusException(ILLEGAL_DATA_ADDRESS);
		return 0;
	}

	// Dữ liệu ghi: 0xFF00 = ON, 0x0000 = OFF
	uint16_t coilValue = ((RxData[4]<<8) | RxData[5]);
	uint8_t bitVal = 0;

	if (coilValue == 0xFF00) bitVal = 1;
	else if (coilValue == 0x0000) bitVal = 0;
	else
	{
		modbusException(ILLEGAL_DATA_VALUE);
		return 0;
	}

	// Cập nhật vào Coils_Database
	uint16_t byteIndex = coilAddr / 8;
	uint16_t bitIndex  = coilAddr % 8;

	if (bitVal)
		Coils_Database[byteIndex] |=  (1 << bitIndex);
	else
		Coils_Database[byteIndex] &= ~(1 << bitIndex);

	// Gửi phản hồi (phải echo lại địa chỉ và giá trị như request)
	// | SLAVE_ID | FUNC_CODE | ADDR_H | ADDR_L | VAL_H | VAL_L | CRC |
	TxData[0] = SLAVE_ID;
	TxData[1] = RxData[1];
	TxData[2] = RxData[2];
	TxData[3] = RxData[3];
	TxData[4] = RxData[4];
	TxData[5] = RxData[5];

	sendData(TxData, 6); // CRC sẽ tính trong sendData
	return 1;
}
uint8_t writeCoils (void)
{
	uint16_t startAddr = ((RxData[2]<<8) | RxData[3]);   // địa chỉ coil bắt đầu
	uint16_t numCoils  = ((RxData[4]<<8) | RxData[5]);   // số lượng coil cần ghi
	//uint8_t  byteCount = RxData[6];                      // số byte dữ liệu tiếp theo

	if ((numCoils < 1) || (numCoils > 1968))  // Giới hạn Modbus
	{
		modbusException(ILLEGAL_DATA_VALUE);
		return 0;
	}

	uint16_t endAddr = startAddr + numCoils - 1;
	if (endAddr > 199)
	{
		modbusException(ILLEGAL_DATA_ADDRESS);
		return 0;
	}

	// Bắt đầu đọc dữ liệu coil từ RxData[7]
	int dataIndex = 7;
	int coilIndex = startAddr;
	for (int i = 0; i < numCoils; i++)
	{
		uint8_t bitVal = (RxData[dataIndex] >> (i % 8)) & 0x01;
		uint16_t byteIndex = coilIndex / 8;
		uint16_t bitIndex  = coilIndex % 8;

		if (bitVal)
			Coils_Database[byteIndex] |=  (1 << bitIndex);
		else
			Coils_Database[byteIndex] &= ~(1 << bitIndex);

		coilIndex++;
		if ((i % 8) == 7) dataIndex++; // chuyển sang byte kế tiếp
	}

	// Gửi phản hồi (echo lại địa chỉ bắt đầu và số lượng coil)
	// | SLAVE_ID | FUNC_CODE | ADDR_H | ADDR_L | NUM_H | NUM_L | CRC |
	TxData[0] = SLAVE_ID;
	TxData[1] = RxData[1];
	TxData[2] = RxData[2];
	TxData[3] = RxData[3];
	TxData[4] = RxData[4];
	TxData[5] = RxData[5];

	sendData(TxData, 6);  // CRC tính trong hàm
	return 1;
}
