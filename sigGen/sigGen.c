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
#include <math.h>
#include "tm4c123gh6pm.h"
#include "clock.h"
#include "gpio.h"
#include "spi1.h"
#include "wait.h"
#include "uart0.h"
#include "cmd.h" // Command line handling
#include "timer.h" // timer services

// Enums
typedef enum _DAC
{
    DAC_A = 1,
    DAC_B = 2
} DAC;

typedef enum _WAVE
{
	  SINE = 1,
	SQUARE = 2,
	   SAW = 3,
	   TRI = 4
} WAVE;

// Pins
#define RED_LED PORTF,1
#define BLUE_LED PORTF,2
#define GREEN_LED PORTF,3

#define SPI_LDAC PORTD,2

// MCP2844
#define OUTPUT_SELECT 32768

/*  ========================== *
 *   C A L B I B R A T I O N   *
 *  ========================== */

// DAC Calibration Values
#define DAC_SLOPE_A 0.000501
#define DAC_OFFSET_A 0.002917

#define DAC_SLOPE_B 0.0005
#define DAC_OFFSET_B 0.000583

#define DAC_MAX_RVALUE 4095
#define DAC_MIN_RVALUE 0

// OUTPUT Calibration Values
#define OUT_SLOPE_A -4.546829268
#define OUT_OFFSET_A 4.489140488

#define OUT_SLOPE_B -4.55864062
#define OUT_OFFSET_B 4.506753756

#define PRECISION_VALUE 4294967296 // 2^32


// ||||| D E B U G   D E F I N E |||||
#define DEBUG

void initHw()
{
    initSystemClockTo40Mhz();

    enablePort(PORTF);

	// Spi1 for communicating with SPI DAC
	// Uses pins D0-D1 and D3 (SPI RX ununsed)
    initSpi1(USE_SSI_FSS); // Port D in enabled in here
    setSpi1BaudRate(20e6, 40e6);
    setSpi1Mode(0,0);

	// UART for debugging and extra info
    initUart0();
    setUart0BaudRate(115200, 40e6);
	
	initTimer();

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
    _delay_cycles(4); // 25 ns wait each, total 100 ns wait
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

#define X5_A -0.000149548
#define X4_A -0.000278675
#define X3_A 0.002751298
#define X2_A 0
#define X1_A -0.197731281
#define X0_A 1.002577993

uint16_t output2RValue(DAC select, float voltage)
{
	float dacVoltage = 0;
	uint16_t r_value = 0;
	
	switch(select)
	{
	case DAC_A:
		dacVoltage = pow(voltage, 5) * X5_A + pow(voltage, 4) * X4_A + pow(voltage, 3) * X3_A + pow(voltage, 2) * X2_A + voltage * X1_A + X0_A;
		
		if(dacVoltage >= 0 && dacVoltage <= DAC_OFFSET_A)
            dacVoltage = DAC_OFFSET_A;
		
		r_value = (dacVoltage - DAC_OFFSET_A) / DAC_SLOPE_A;
		break;
	case DAC_B:
		dacVoltage = (voltage - OUT_OFFSET_B) / OUT_SLOPE_B;
		if(dacVoltage >= 0 && dacVoltage <= DAC_OFFSET_B)
            dacVoltage = DAC_OFFSET_B;
		r_value = (dacVoltage - DAC_OFFSET_B) / DAC_SLOPE_B;
		break;
	default:
		break;
	}
	
	return r_value;
}

bool selectOutputVoltage(DAC select, float voltage)
{
	float dacVoltage;
    uint16_t r_value = 0;
	bool wrote2Spi = false;
    switch(select)
    {
    case DAC_A:
		r_value = output2RValue(DAC_A, voltage);
		writeSpi1Data( 0x3000 | (r_value & 0x0FFF) );
		latchDAC();
		wrote2Spi = true;
        break;
		
    case DAC_B:
		r_value = output2RValue(DAC_B, voltage);
		writeSpi1Data( 0xB000 | (r_value & 0x0FFF) );
		latchDAC();
		wrote2Spi = true;
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
  *              LUT PROCESSING             *
  * ======================================= */

#define LUT_SIZE 2048
uint16_t lut_i = 0;
uint16_t lutA[LUT_SIZE] = {0};

void calculateWave(WAVE type, uint16_t addr[], DAC select, float amp, float ofs)
{
	ofs = 0;
	amp = 1;
	uint16_t i;
	float y;
	// gain should be bits/voltage * amp voltage I want
	switch(select)
	{
	case DAC_A:
		for(i = 0; i < LUT_SIZE; i++)
		{
			y = ( 2 * M_PI )*((float)i/(float)LUT_SIZE);
			lutA[i] = output2RValue(DAC_A, ofs + (amp * sin(y)));
			//phase += changePhase;
		}
		break;
	case DAC_B:
		break;
	}
	
#ifdef DEBUG
	char buffer[100];
	for(i = 0; i < LUT_SIZE; i++)
	{
	    sprintf(buffer, "%u\n", lutA[i] );
	    putsUart0(buffer);
	}
#endif
	
}

void tickIsr()
{
	if(lut_i == LUT_SIZE)
		lut_i = 0;
	
	writeSpi1Data( 0x3000 | lutA[lut_i++] );
	latchDAC();
    TIMER4_ICR_R = TIMER_ICR_TATOCINT;
}


 /* ======================================= *
  *           SHELL PROCESSING              *
  * ======================================= */

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
	
	
	uint32_t phaseAccum = 0;
	
	DAC dac;
    float voltage = -1, freq = -1, amp = -1, ofs = -1;
	

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

        /*  ======================= *
         *  ||||||||| D C ||||||||| *
         *  ======================= */
        if( isCommand(&data, "dc", 2) )
        {
            dac = (DAC)getFieldInteger(&data, 1);
            voltage = getFieldFloat(&data, 2);
			
#ifdef DEBUG
            sprintf(buffer, "Float: %f\n", voltage);
            putsUart0(buffer);
#endif

			if( (dac <= 2) && selectOutputVoltage(dac, voltage) )
				putsUart0("Successfully wrote to DAC.");
			else
				putsUart0("ERROR: Could not write DC Voltage to DAC.");

        }
		
		/*  ======================= *
         *  |||||||| D A C |||||||| *
         *  ======================= */
        else if( isCommand(&data, "dac", 2) )
        {
            dac = (DAC)getFieldInteger(&data, 1);
            voltage = getFieldFloat(&data, 2);
			if(voltage == -1)
				voltage = (float)getFieldInteger(&data, 2);
			
#ifdef DEBUG
            sprintf(buffer, "Float: %f\n", voltage);
            putsUart0(buffer);
#endif

			if( (dac <= 2 && voltage != -1) && selectDACVoltage(dac, voltage) )
				putsUart0("Successfully wrote to DAC.");
			else
				putsUart0("ERROR: Could not write DC Voltage to DAC.");

        }
		
		/*  ======================= *
         *  ||||||| S I N E ||||||| * 
         *  ======================= */
		else if( isCommand(&data, "sine", 4) )
		{
			dac = (DAC)getFieldInteger(&data, 1);
			freq = getFieldFloat(&data, 2);
			amp = getFieldFloat(&data, 3);
			ofs = getFieldFloat(&data, 4);
			ofs = 0;
			if( dac <= 2 && freq != -1 && amp != -1)
			{
				calculateWave(SINE, lutA, dac, amp, ofs);
				putsUart0("Successfully calculated Sine wave.");
				TIMER4_CTL_R |= TIMER_CTL_TAEN;
			}
			else if( strcomp(getFieldString(&data, 1), "stop") )
			{
				TIMER4_CTL_R &= ~TIMER_CTL_TAEN;
				putsUart0("ERROR: Could not calculate Sine wave.");
			}
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
                waitMicrosecond(4000000);

                setPinValue(BLUE_LED, 1);
                writeSpi1Data(0x3F00);
                writeSpi1Data(0xBF00);
                latchDAC();
                waitMicrosecond(4000000);

                setPinValue(BLUE_LED, 0);
                writeSpi1Data(0x3E00);
                writeSpi1Data(0xBE00);
                latchDAC();
                waitMicrosecond(4000000);
				
				setPinValue(BLUE_LED, 1);
                writeSpi1Data(0x3D00);
                writeSpi1Data(0xBD00);
                latchDAC();
                waitMicrosecond(4000000);
				
				setPinValue(BLUE_LED, 0);
                writeSpi1Data(0x3C00);
                writeSpi1Data(0xBC00);
                latchDAC();
                waitMicrosecond(4000000);
				
				setPinValue(BLUE_LED, 1);
                writeSpi1Data(0x3B00);
                writeSpi1Data(0xBB00);
                latchDAC();
                waitMicrosecond(4000000);
				
				setPinValue(BLUE_LED, 0);
                writeSpi1Data(0x3A00);
                writeSpi1Data(0xBA00);
                latchDAC();
                waitMicrosecond(4000000);
				
				setPinValue(BLUE_LED, 1);
                writeSpi1Data(0x3900);
                writeSpi1Data(0xB900);
                latchDAC();
                waitMicrosecond(4000000);
				
				setPinValue(BLUE_LED, 0);
                writeSpi1Data(0x3800);
                writeSpi1Data(0xB800);
                latchDAC();
                waitMicrosecond(4000000);
				
				setPinValue(BLUE_LED, 1);
                writeSpi1Data(0x3700);
                writeSpi1Data(0xB700);
                latchDAC();
                waitMicrosecond(4000000);
				
				setPinValue(BLUE_LED, 0);
                writeSpi1Data(0x3600);
                writeSpi1Data(0xB600);
                latchDAC();
                waitMicrosecond(4000000);
				
				setPinValue(BLUE_LED, 1);
                writeSpi1Data(0x3500);
                writeSpi1Data(0xB500);
                latchDAC();
                waitMicrosecond(4000000);
				
				setPinValue(BLUE_LED, 0);
                writeSpi1Data(0x3400);
                writeSpi1Data(0xB400);
                latchDAC();
                waitMicrosecond(4000000);
				
				setPinValue(BLUE_LED, 1);
                writeSpi1Data(0x3300);
                writeSpi1Data(0xB300);
                latchDAC();
                waitMicrosecond(4000000);
				
				setPinValue(BLUE_LED, 0);
                writeSpi1Data(0x3200);
                writeSpi1Data(0xB200);
                latchDAC();
                waitMicrosecond(4000000);
				
				setPinValue(BLUE_LED, 1);
                writeSpi1Data(0x3100);
                writeSpi1Data(0xB100);
                latchDAC();
                waitMicrosecond(4000000);
				
				setPinValue(BLUE_LED, 0);
                writeSpi1Data(0x3000);
                writeSpi1Data(0xB000);
                latchDAC();
                waitMicrosecond(4000000);
				
				
				setPinValue(BLUE_LED, 0);
				setPinValue(RED_LED, 0);
            }
			else if( strcomp(getFieldString(&data, 1), "value") )
			{
				// 0x7AE
				writeSpi1Data(0x37E1);
                writeSpi1Data(0xB7F3);
				latchDAC();
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
