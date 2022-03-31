/*
 * sigGen.c
 *
 *  Created on: Mar 23, 2022
 *      Author: Nicholas
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "tm4c123gh6pm.h"
#include "clock.h"
#include "gpio.h"
#include "spi1.h"
#include "wait.h"
#include "uart0.h"
#include "cmd.h"

// Enums
typedef enum _DAC
{
    DAC_A = 1,
    DAC_B = 2
} DAC;

// Pins
#define RED_LED PORTF,1
#define BLUE_LED PORTF,2
#define GREEN_LED PORTF,3

#define SPI_LDAC PORTD,2

// MCP2844
#define OUTPUT_SELECT 32768

/*  ==================== *
 *  O U T P U T  C A L B *
 *  ==================== */
// DAC Calibration Values
#define DAC_SLOPE_A 0.0005
#define DAC_OFFSET_A 0.0013

#define DAC_SLOPE_B 0.0005
#define DAC_OFFSET_B 0.0018

#define DAC_MAX_RVALUE 4095
#define DAC_MIN_RVALUE 0

// OUTPUT Calibration Values
#define OUT_SLOPE_A 4.5054
#define OUT_OFFSET_A -4.606

#define OUT_SLOPE_B 4.5
#define OUT_OFFSET_B -4.6


// ||||| D E B U G   D E F I N E |||||
#define DEBUG

void initHw()
{
    initSystemClockTo40Mhz();

    enablePort(PORTF);
    _delay_cycles(3);

	// Spi1 for communicating with SPI DAC
	// Uses pins D0-D1 and D3 (SPI RX ununsed)
    initSpi1(USE_SSI_FSS); // Port D in enabled in here
    setSpi1BaudRate(10e6, 40e6);
    setSpi1Mode(0,0);

	// UART for debugging and extra info
    initUart0();
    setUart0BaudRate(115200, 40e6);

	// LDAC pin for latching the SPI DAC 
    selectPinPushPullOutput(SPI_LDAC);
    setPinValue(SPI_LDAC, 1);

	// LEDs for debugging and extra info
    selectPinPushPullOutput(RED_LED);
    selectPinPushPullOutput(GREEN_LED);
    selectPinPushPullOutput(BLUE_LED);

}

// ==============================
// DAC UTILITIES
// ==============================

// Takes values from SPI buffer and latches it
void latchDAC()
{
    setPinValue(SPI_LDAC, 0);
    _delay_cycles(2); // 250 ns wait
    setPinValue(SPI_LDAC, 1);
}

// Takes in specified DAC and float voltage...
// ...converts to 12b value to send to DAC
bool selectDACVoltage(DAC select, float voltage)
{
    uint16_t r_value = 0;
    bool wrote2Spi = false;

    switch(select)
    {
    case DAC_A:
        // handles lower edge cases
        if(voltage >= 0 && voltage <= DAC_OFFSET_A)
            voltage = DAC_OFFSET_A;

        r_value = (voltage - DAC_OFFSET_A) / DAC_SLOPE_A;
		if( r_value < DAC_MAX_RVALUE )
		{
		    writeSpi1Data(0x3000 | (r_value & 0x0FFF) );
		    latchDAC();
		    wrote2Spi = true;
		}
		else
		    putsUart0("ERROR: R-Value Requested out of range...\n");
        break;
    case DAC_B:
        if(voltage >= 0 && voltage <= DAC_OFFSET_B)
			voltage = DAC_OFFSET_B;

        r_value = (voltage - DAC_OFFSET_B) / DAC_SLOPE_B;
		if( r_value < DAC_MAX_RVALUE )
		{
		    writeSpi1Data(0xB000 | (r_value & 0x0FFF) );
		    latchDAC();
		    wrote2Spi = true;
		}
		else
			putsUart0("ERROR: R-Value Requested out of range...\n");
        break;
    default:
		putsUart0("ERROR: Invalid DAC Selection for DAC Voltage.\n");
        break;
    }
	
#ifdef DEBUG
    char buffer[100];
    sprintf(buffer, "R-Value for DAC %X: %u\n", select + 9, r_value);
    putsUart0(buffer);
#endif

    return wrote2Spi;
}


bool selectOutputVoltage(DAC select, float voltage)
{
    float dacVoltage = 0;
	bool wrote2Spi = false;
    switch(select)
    {
    case DAC_A:
		if( voltage >= 0 && voltage <= OUT_OFFSET_A )
			voltage = OUT_OFFSET_A;
		dacVoltage = (dacVoltage - OUT_OFFSET_A) / OUT_SLOPE_A;
		wrote2Spi = selectDACVoltage(DAC_A, dacVoltage);
        break;
    case DAC_B:
		if( voltage >= 0 && voltage <= OUT_OFFSET_B )
			voltage = OUT_OFFSET_B;
		dacVoltage = (dacVoltage - OUT_OFFSET_B) / OUT_SLOPE_B;
		wrote2Spi = selectDACVoltage(DAC_B, dacVoltage);
        break;
    default:
		putsUart0("ERROR: Invalid DAC Selection for Output Voltage.\n");
        break;
    }

#ifdef DEBUG
    char buffer[100];
    sprintf(buffer, "Selected Output Voltage for DAC V: %f\tOUT V: %f\n", voltage, dacVoltage);
    putsUart0(buffer);
#endif

	return wrote2Spi;
}
 /* ======================================= *
  *             SHELL PROCESSING            *
  *  ====================================== */

int main(void)
{
    initHw();

    // For filling list of commands and executing them all together
    /*instruction inst_arr[MAX_INSTRUCTIONS];
    int8_t inst_index = 0;
    bool inst_max = false;*/

    // Start Up Light
    setPinValue(GREEN_LED, 1);
    waitMicrosecond(100000);
    setPinValue(GREEN_LED, 0);
    waitMicrosecond(100000);

    // Command Line Processing Info
    USER_DATA data;
    char buffer[MAX_CHARS + 1];

    // Start of Shell
    while( 1 )
    {

        putcUart0('>');
        setPinValue(BLUE_LED, 1);
        // Fills up buffer in data and waits for max characters or RETURN
        getsUart0(&data);
        setPinValue(BLUE_LED, 0);

        // Separates fields into Numeric, Upper Alpha, Lower Alpha, and Floats
        parseFields(&data);

#ifdef DEBUG
        // DEBUG info for USER_DATA
        uint8_t i;
        putcUart0('\n');
        for (i = 0; i < data.fieldCount; i++)
        {
            putcUart0(data.fieldType[i]);
            putcUart0('\t');
            putsUart0(&data.buffer[ data.fieldPosition[i] ]);
            putcUart0('\n');
        }
#endif


        /* ================== *
         *   SHELL COMMANDS   *
         * ================== */

        DAC dac;
        float voltage;

        /*  ======================= *
         *  ||||||||| D C ||||||||| *
         *  ======================= */
        if( isCommand(&data, "dc", 2) )
        {
            dac = (DAC)getFieldInteger(&data, 1);
            voltage = getFieldFloat(&data, 2);
			if(voltage == -1)
				voltage = (float)getFieldInteger(&data, 2);

            sprintf(buffer, "Float: %f\n", voltage);
            putsUart0(buffer);

			if( (dac <= 2 && voltage != -1) && selectOutputVoltage(dac, voltage) )
				putsUart0("Successfully wrote to DAC.");
			else
				putsUart0("ERROR: Could not write to DAC.");

        }

        /*  ======================== *
         *  ||||||| T E S T |||||||| *
         *  ======================== */
        else if( isCommand(&data, "test", 1) )
        {
            if( strcomp(getFieldString(&data, 1), "DAC") )
            {
                putsUart0("Testing DAC Voltages...");

                // TEST DAC VOLTAGES
                setPinValue(RED_LED, 1);
                writeSpi1Data(0x3FFF);
                writeSpi1Data(0xBFFF);
                latchDAC();
                waitMicrosecond(2000000);

                setPinValue(BLUE_LED, 1);
                writeSpi1Data(0x3800);
                writeSpi1Data(0xB800);
                latchDAC();
                waitMicrosecond(2000000);

                setPinValue(GREEN_LED, 1);
                writeSpi1Data(0x3000);
                writeSpi1Data(0xB000);
                latchDAC();
                waitMicrosecond(2000000);

                setPinValue(RED_LED, 0);
                setPinValue(BLUE_LED, 0);
                setPinValue(GREEN_LED, 0);
            }
            else
            {
                putsUart0("ERROR: Invalid argument for 'test'.");
            }
        }

        /*  =============================== *
         *  ||||||||| R E S E T ||||||||||| *
         *  =============================== */
        else if( isCommand(&data, "reset", 0) )
        {
            NVIC_APINT_R = NVIC_APINT_VECTKEY | NVIC_APINT_SYSRESETREQ;
        }

        /*  ============================= *
         *  ||||||||| H E L P ||||||||||| *
         *  ============================= */
        else if( isCommand(&data, "help", 0) )
        {
            putsUart0("Possible Commands:\n");
            putsUart0("dc OUT, VOLTAGE\n");
            putsUart0("cycles N\n");
            putsUart0("sine OUT, FREQ, AMP, [OFS]\n");
        }
        else
        {
            putsUart0("ERROR: Command not found. Try 'help' for options.\n");
        }

        data_flush(&data);
        putcUart0('\n');
    }
}
