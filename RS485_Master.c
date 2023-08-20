#include "config.h"
#include "Master_ModbusRTU.h"
#include "LCD16x2.h"
#include "Compressor_motor.h"
#include "PIC16F877A_timer0.h"
#include <stdio.h>

uint8_t count_motor = 0;
uint16_t count_modbus = 0;
uint8_t rcv_buffer[LEN + 5];
float targetTemp = 30;
float ftemp = 0;
float fhumid = 0;
char str1[17];
char str2[17];

// Functions prototype
float HCH1000_freqToHumidity(uint16_t freq);
float TC72_toFloat(signed int temp);
void displayData(void);

// Interrupt handling service 
void __interrupt() ISR(void)
{
    // Timer 0 timer interrupt
    if(TMR0IF){
        TMR0 = 6;
        // Calculate the PID value of the compressor motor every 100 millisecond
        if(count_motor++ == 100)
        {
            count_motor = 0;
            SpeedControl(100, targetTemp, ftemp);
        }
        // Send the command to get the data from slave device every 1 second
        if(count_modbus++ == 1000)
        {
            count_modbus = 0;
            Master_SendCommand(0x31, 0x33);
        }
        // Clear flag bit
        TMR0IF = 0;
    }
    // UART interrupt
    if(RCIF){
        // Error handling
        if (UARTrcvString((char*)rcv_buffer, LEN + 4))  
        {
            if(Master_DataHandling(rcv_buffer))
            {
                ftemp = TC72_toFloat((rcv_buffer[3] << 8) + rcv_buffer[2]);
                fhumid = HCH1000_freqToHumidity((rcv_buffer[5] << 8) + rcv_buffer[4]);
                displayData();
            }
            else
            {
                LCD_Clear();
                LCD_Set_Cursor(1,1);
                LCD_Write_String("Error! Check");
            }
        }
    }
}

void main(void){
    // Timer 0 timer mode initialization
    TMR0 = 6;
    timer0TimerInit(TIMER0_DIV_16);
    // Initiate the modbusRTU module
    mobusRTUmasterInit();
    // Initiate the controller module for the compressor motor
    compressorMotor_Init();
    // LCD initiate
    LCD_Init();
    while(1)
    {
    }
    return;
}

// Function to convert the HCH-1000 frequency to Celsius degree temperature
float HCH1000_freqToHumidity(uint16_t freq)
{
    return 565.1 - 0.076725 * freq;
}

// Function to convert the sensor output to real temperature value
float TC72_toFloat(signed int temp)
{
    float result = (float)(temp >> 8);      //Discard LSByte (Only holds fraction flag bits)
    char count = temp & 0x00C0;          
    if(count) 
    {
        count = count >> 6; 
        result += (count * 0.25);
    }
    return result;
}

// Function for LCD display
void displayData()
{
    LCD_Clear();
    sprintf(str1,"Temp: %.2f C", ftemp);
    sprintf(str2,"Humid: %.2f %%", fhumid);
    LCD_Set_Cursor(1,1);
    LCD_Write_String(str1);
    LCD_Set_Cursor(2,1);
    LCD_Write_String(str2);
}