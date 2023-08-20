#ifndef MODBUSRTU_MASTER_H
#define MODBUSRTU_MASTER_H

#include "PIC16F877A_UART.h"

#define TRISALARM TRISB1
#define ALARM RB1

// Maximum length of data block
#define LEN 4
#define MAX 0xFFFF
#define CRC16_POLYNOMIAL  0xA001

// ModbusRTU frame data
uint8_t ModbusFrame[LEN + 5];
__bit send = 0;

// Error frame structure
typedef struct ErrorFrame{
    uint8_t Function_error;
    uint8_t CRC_error;
}ErrorFrame;

typedef struct Memory{
    uint8_t last_slave;
    uint8_t last_function;
}Memory;

void mobusRTUmasterInit(void);
void Master_SendCommand(uint8_t sla_add, uint8_t function);
void SendErrorFrame(ErrorFrame Eframe,uint8_t function);
uint16_t CRCcheck(uint8_t *buf, uint8_t len);
ErrorFrame ReceiveData_Check(uint8_t *str);
__bit Master_DataHandling(uint8_t *buf);

#endif