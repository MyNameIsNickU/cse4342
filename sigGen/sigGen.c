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
    DAC_B = 2,
    DAC_INVALID = 3
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

#define X5_B
#define X4_B
#define X3_B
#define X2_B
#define X1_B
#define X0_B

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
	float dacVoltage = -1;
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

#define LUT_SIZE (uint32_t)2048
uint32_t lut_i_A = 0;
uint32_t lut_i_B = 0;
uint32_t currentCycles_A = 0;
uint32_t currentCycles_B = 0;
int32_t maxCycles_A = -1;
int32_t maxCycles_B = -1;
uint32_t phaseAccum_A = 0;
uint32_t phaseAccum_B = 0;
uint16_t lutA[LUT_SIZE] = {0};
uint16_t lutB[LUT_SIZE] = {0};
bool outA_EN = false;
bool outB_EN = false;

void calculateWave(WAVE type, DAC select, float amp, float ofs)
{
	//ofs = 0;
	//amp = 1;
	uint16_t i;
	float y;
	// gain should be bits/voltage * amp voltage I want
	switch(type)
	{
	case SINE:
		for(i = 0; i < LUT_SIZE; i++)
		{
			y = ( 2 * M_PI )*((float)i/(float)LUT_SIZE);
			if(select == DAC_A)
				lutA[i] = output2RValue(select, ofs + (amp * sin(y)));
			else if(select == DAC_B)
				lutB[i] = output2RValue(select, ofs + (amp * sin(y)));
		}
		break;
	case SQUARE:
		for(i = 0; i < LUT_SIZE; i++)
		{
			if(select == DAC_A)
			{
				if( i / (LUT_SIZE/2) == 0 )
					lutA[i] = output2RValue(select, ofs + amp);
				else if( i / (LUT_SIZE/2) == 1 )
					lutA[i] = output2RValue(select, ofs - amp);
			}
			else if(select == DAC_B)
			{
				if( i / (LUT_SIZE/2) == 0 )
					lutB[i] = output2RValue(select, ofs + amp);
				else if( i / (LUT_SIZE/2) == 1 )
					lutB[i] = output2RValue(select, ofs - amp);
			}
		}
		break;
	case SAW:
	// start at (ofs - amp) end at (ofs + amp)
		for(i = 0; i < LUT_SIZE; i++)
		{
			// y = b + mx | m = 2*amp, x = i/LUT_SIZE-1
			y = (ofs-amp) + (2.0*amp)*(float)i/((float)LUT_SIZE-1.0);
			
			if(select == DAC_A)	
				lutA[i] = output2RValue(select, y);
			
			else if(select == DAC_B)
				lutB[i] = output2RValue(select, y);
		}
		break;
	case TRI:
	//TODO: WRONG
		for(i = 0; i < LUT_SIZE; i++)
		{
			if(select == DAC_A)
			{
				if( i / (LUT_SIZE/2) == 0)
				{
					y = (ofs-amp) + (2.0*amp) * (float)i / (((float)LUT_SIZE-1.0)/2.0);
					lutA[i] = output2RValue(select, y);
				}
				else if( i / (LUT_SIZE/2) == 1)
				{
					y = (ofs+amp) - (2.0*amp) * (((float)i) - ((float)LUT_SIZE/2)) / (((float)LUT_SIZE-1.0)/2.0);
					lutA[i] = output2RValue(select, y);
				}
			}
			else if(select == DAC_B)
			{
				if( i / (LUT_SIZE/2) == 0)
				{
					y = (ofs-amp) + (2.0*amp) * (float)i / (((float)LUT_SIZE-1.0)/2.0);
					lutB[i] = output2RValue(select, y);
				}
				else if( i / (LUT_SIZE/2) == 1)
				{
					y = (ofs+amp) - (2.0*amp) * (((float)i) - ((float)LUT_SIZE/2)) / (((float)LUT_SIZE-1.0)/2.0);
					lutB[i] = output2RValue(select, y);
				}
			}
		}
		break;
	default:
		putsUart0("ERROR: Invalid waveform type.\n");
	}
	
	if(select == DAC_A)
		outA_EN = true;
	else if(select == DAC_B)
		outB_EN = true;
	
#ifdef DEBUG
	char buffer[100];
	for(i = 0; i < LUT_SIZE; i++)
	{
	    sprintf(buffer, "%u\t%u\n", lutA[i], lutB[i] );
	    putsUart0(buffer);
	}
#endif
	
}

void tickIsr()
{
	// for looping the wave
	if((lut_i_A >> 20) >= LUT_SIZE)
	{
		lut_i_A -= (LUT_SIZE << 20);
		currentCycles_A++;
	}
	
	if( (lut_i_B >> 20) >= LUT_SIZE)
	{
		lut_i_B -= (LUT_SIZE << 20);
		currentCycles_B++;
	}
	
	// if cycles hit the set limit, stop
	// ignore if the maxCycles value set to -1
	if(currentCycles_A == maxCycles_A && maxCycles_A != -1)
		outA_EN = false;
	if(currentCycles_B == maxCycles_B && maxCycles_B != -1)
		outB_EN = false;
	
	// writing each value to SPI
	if(outA_EN)
	{
		writeSpi1Data( 0x3000 | lutA[lut_i_A >> 20] );
		latchDAC();
		lut_i_A += phaseAccum_A;
	}
	
	if(outB_EN)
	{
		writeSpi1Data( 0xB000 | lutB[lut_i_B >> 20] );
		latchDAC();
		lut_i_B += phaseAccum_B;
	}
	
    TIMER4_ICR_R = TIMER_ICR_TATOCINT;
}

uint32_t float2uint(float input)
{
	uint8_t i;
	uint32_t returnValue = 0;
	
	uint16_t intValue = (uint32_t)input / 1;
	returnValue = intValue << 20;
	input -= intValue;
	float calcValue;
	
	for(i = 20; i > 0; i--)
	{
	    calcValue = input - (float)1/(2 << (20-i));
		if( calcValue > 0 )
		{
		    returnValue |= (1 << i-1);
		    input = calcValue;
		}

	}
	
	return returnValue;
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
	
	
	DAC dac = DAC_INVALID;
    float voltage = 0, freq = 0, amp = 1, ofs = 0;
	float freq_ref = (((float)40e6 / (float)TIMER4_TAILR_R)) * (1.0 / (float)LUT_SIZE);
	int32_t testValue = 0;

	/* calculateWave(SINE, DAC_A, 1, 0);
	freq = 20000;
	phaseAccum_A = float2uint( freq/freq_ref );
	TIMER4_CTL_R |= TIMER_CTL_TAEN; */
	//while(1);
	
	putsUart0("|Signal Generator START|\n");
	putsUart0("------------------------\n\n");
    // Start of Shell
    while( 1 )
    {
		ofs = 0;
		amp = 1;
		freq = 0;
		voltage = 0;

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
            sprintf(buffer, "DAC: %u\tVoltage: %f\n", dac, voltage);
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
         *  ||||| C Y C L E S ||||| *
         *  ======================= */
        else if( isCommand(&data, "cycles", 1) )
        {
			if(data.fieldType[1] == 'n')
			{
				maxCycles_A = getFieldInteger(&data, 1);
				maxCycles_B = getFieldInteger(&data, 1);
			}
			else if(data.fieldType[1] == 'a' && strcomp(getFieldString(&data, 1), "continuous") )
			{
				maxCycles_A = -1;
				maxCycles_B = -1;
			}
			else
			{
				putsUart0("ERROR: Invalid command for 'cycles'.");
			}

        }
		
		/*  ======================= *
         *  ||||||| S I N E ||||||| * 
         *  ======================= */
		else if( isCommand(&data, "sine", 3) )
		{
			dac = (DAC)getFieldInteger(&data, 1);
			freq = getFieldFloat(&data, 2);
			amp = getFieldFloat(&data, 3);
			
			if( isCommand(&data, "sine", 4) )
				ofs = getFieldFloat(&data, 4);
			
#ifdef DEBUG
			sprintf(buffer, "DAC: %u\tFreq: %f\tAmp: %f\tOFS: %f", dac, freq, amp, ofs);
			putsUart0(buffer);
#endif
			if( dac <= 2 )
			{
				calculateWave(SINE, dac, amp, ofs);
				putsUart0("Successfully calculated Sine wave.");
				if(dac == DAC_A)
					phaseAccum_A = float2uint(freq / freq_ref);
				else if(dac == DAC_B)
					phaseAccum_B = float2uint(freq / freq_ref);
				TIMER4_CTL_R |= TIMER_CTL_TAEN;
			}
			else if( strcomp(getFieldString(&data, 1), "stop") )
			{
				TIMER4_CTL_R &= ~TIMER_CTL_TAEN;
			}
			else
				putsUart0("ERROR: invalid argument for 'sine'.");
		}
		
		
		/*  ======================= *
         *  ||||| S Q U A R E ||||| * 
         *  ======================= */
		else if( isCommand(&data, "square", 3) )
		{
			dac = (DAC)getFieldInteger(&data, 1);
			freq = getFieldFloat(&data, 2);
			amp = getFieldFloat(&data, 3);
			
			if( isCommand(&data, "square", 4) )
				ofs = getFieldFloat(&data, 4);
			
#ifdef DEBUG
			sprintf(buffer, "DAC: %u\tFreq: %f\tAmp: %f\tOFS: %f", dac, freq, amp, ofs);
			putsUart0(buffer);
#endif
			if( dac <= 2 )
			{
				calculateWave(SQUARE, dac, amp, ofs);
				putsUart0("Successfully calculated Square wave.");
				if(dac == DAC_A)
					phaseAccum_A = float2uint(freq / freq_ref);
				else if(dac == DAC_B)
					phaseAccum_B = float2uint(freq / freq_ref);
				TIMER4_CTL_R |= TIMER_CTL_TAEN;
			}
			else if( strcomp(getFieldString(&data, 1), "stop") )
			{
				TIMER4_CTL_R &= ~TIMER_CTL_TAEN;
			}
			else
				putsUart0("ERROR: invalid argument for 'square'.");
		}
		
		/*  ======================= *
         *  |||||||| S A W |||||||| * 
         *  ======================= */
		else if( isCommand(&data, "sawtooth", 3) )
		{
			dac = (DAC)getFieldInteger(&data, 1);
			freq = getFieldFloat(&data, 2);
			amp = getFieldFloat(&data, 3);
			
			if( isCommand(&data, "sawtooth", 4) )
				ofs = getFieldFloat(&data, 4);
			
#ifdef DEBUG
			sprintf(buffer, "DAC: %u\tFreq: %f\tAmp: %f\tOFS: %f", dac, freq, amp, ofs);
			putsUart0(buffer);
#endif
			if( dac <= 2 )
			{
				calculateWave(SAW, dac, amp, ofs);
				putsUart0("Successfully calculated Sawtooth wave.");
				if(dac == DAC_A)
					phaseAccum_A = float2uint(freq / freq_ref);
				else if(dac == DAC_B)
					phaseAccum_B = float2uint(freq / freq_ref);
				TIMER4_CTL_R |= TIMER_CTL_TAEN;
			}
			else if( strcomp(getFieldString(&data, 1), "stop") )
			{
				TIMER4_CTL_R &= ~TIMER_CTL_TAEN;
			}
			else
				putsUart0("ERROR: invalid argument for 'sawtooth'.");
		}
		
		/*  ======================= *
         *  |||||||| T R I |||||||| * 
         *  ======================= */
		else if( isCommand(&data, "triangle", 3) )
		{
			dac = (DAC)getFieldInteger(&data, 1);
			freq = getFieldFloat(&data, 2);
			amp = getFieldFloat(&data, 3);
			
			if( isCommand(&data, "triangle", 4) )
				ofs = getFieldFloat(&data, 4);
			
#ifdef DEBUG
			sprintf(buffer, "DAC: %u\tFreq: %f\tAmp: %f\tOFS: %f", dac, freq, amp, ofs);
			putsUart0(buffer);
#endif
			if( dac <= 2 )
			{
				calculateWave(TRI, dac, amp, ofs);
				putsUart0("Successfully calculated Triangle wave.");
				if(dac == DAC_A)
					phaseAccum_A = float2uint(freq / freq_ref);
				else if(dac == DAC_B)
					phaseAccum_B = float2uint(freq / freq_ref);
				TIMER4_CTL_R |= TIMER_CTL_TAEN;
			}
			else if( strcomp(getFieldString(&data, 1), "stop") )
			{
				TIMER4_CTL_R &= ~TIMER_CTL_TAEN;
			}
			else
				putsUart0("ERROR: invalid argument for 'triangle'.");
		}

        /*  ======================== *
         *  ||||||| T E S T |||||||| *
         *  ======================== */
        else if( isCommand(&data, "test", 1) )
        {
            if( strcomp(getFieldString(&data, 1), "DAC") )
            {
                putsUart0("Testing DAC Voltages...");
				
				testValue = 0xFFF;
				setPinValue(RED_LED, 1);
				setPinValue(BLUE_LED, 1);
				writeSpi1Data(0x3000 | testValue);
				writeSpi1Data(0xB000 | testValue);
				latchDAC();
				waitMicrosecond(4000000);
				
				for(testValue = 0xF00; testValue >= 0; testValue -= 0x100)
				{
					setPinValue(RED_LED, !getPinValue(RED_LED) );
					setPinValue(BLUE_LED, !getPinValue(BLUE_LED) );
					writeSpi1Data(0x3000 | testValue);
					writeSpi1Data(0xB000 | testValue);
					latchDAC();
					waitMicrosecond(4000000);
				}
				
				setPinValue(RED_LED, 0);
				setPinValue(BLUE_LED, 0);

                /* // TEST DAC VOLTAGES
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
				setPinValue(RED_LED, 0); */
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
			putsUart0("square OUT, FREQ, AMP, [OFS]\n");
			putsUart0("sawtooth OUT, FREQ, AMP, [OFS]\n");
			putsUart0("UNTESTED: triangle OUT, FREQ, AMP, [OFS]\n");
        }
        else
        {
            putsUart0("ERROR: Command not found. Try 'help' for options.\n");
        }

        data_flush(&data);
        putcUart0('\n');
    }
}
