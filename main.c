/*
 * INFO:
 * This code demonstrates how to use the Curiosity Nano's AVR128DB DAC and ADC in an interrupt based configuration.
 * The code samples a signal with the ADC and conditions the sample for digital signal processing before demonstrating 
 * some simple processing and use of a circular buffer. The conditioning is then reveresed so that the
 * processed sample can output on the DAC. 
 * 
 * The sampling period of the implementation can be adjusted and experimented with and 
 * signal processing code can be added in the interrupt service routine (ISR).
 * 
 * PINOUT
 *  - PortD pin 1 (PD1): ADC in (with possibilities for other pins)
 *  - PortD pin 6 (PD6): DAC out
 * 
 * Notes on the implementation:
 *  - The ADC and DAC are both using 10-bit resolution.
 *  - The system clock is configured to run at the 24MHz, which is the maximum speed achievable.
 *  - The ADC clock is running at 24MHz/12=2MHz, which is its maximum possible speed (see datasheet).
 *  - Due to small deviations between chosen and achieved sampling period a small compensation function is defined. 
 *    Details can be found in the peripherals.c file.
 * 
 * Created: 17/06/2024
 * Author : Carl Richard Steen Fosse
 */ 

/* Definition of main clock frequency for timing calculations in the program.*/
#define F_CPU 24000000UL 

//-------------------User defined parameters-------------------
#define SAMPLING_PERIOD 60 //us 
#define CIRCULAR_BUFFER_SIZE 408
#define NUM_SAMPLES_DELAY 407
#define A_FLOAT (float) 0.9
//-------------------------------------------------------------
#define A (int16_t)(A_FLOAT*(1<<6))

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "peripherals.h"
#include <math.h>
#include <stdio.h>

/* Type definition for the circular buffer*/
typedef struct {
    uint16_t * const buffer;
    unsigned writeIndex;
} circular_buffer_t;

/* Function prototypes */
void circular_buffer_write(circular_buffer_t *c, uint16_t data);
uint16_t circular_buffer_read(circular_buffer_t *c, unsigned Xn);

 /**
 * @brief ISR for sampling and D/A conversion, with an example of signal processing.
 * The ISR is triggered when the TCA0 counter overflows(using vector TCA0_OVF_vect).
 */
ISR(TCA0_OVF_vect) {
   
    //Variable for storing new samples from the ADC.
    static uint16_t inputSample = 0;
    //Variable for storing the processed samples to the DAC.
    static uint16_t outputSample = 0;   
    
    //Declaring a circular buffer with a data space based on CIRCULAR_BUFFER_SIZE
    static uint16_t bufferDataSpace[CIRCULAR_BUFFER_SIZE];
    static circular_buffer_t c = {
        .buffer = bufferDataSpace,
        .writeIndex = 0
    };
    
    //Various variables for calculations
    static int16_t x = 0;
    static int16_t y = 0;
    static int16_t temp = 0;
    
    
    //Sampling with the ADC
    inputSample = ADC0_read();
    
    /*-------------Signal processing code below-------------*/
    
    //We subtract (2^10)/2 so that we operate with ints and values around 0.
    x = (int16_t) inputSample-512;
    
    //We right shift in order to use the lower 6 for fractional number formats.
    x = x << 6;
    
    //Multiplying x with (1-A) to prevent overflow
    x = ((int32_t)x*(1-A) >> 6);
    
    //We multiply x with our coefficient and store it in the buffer
    temp = ((int32_t)x*A >> 6) + (int32_t)(circular_buffer_read(&c, NUM_SAMPLES_DELAY)*A)>>6;
    
    circular_buffer_write(&c, temp);
    
    //We extract a delayed sample and store it in y
    
    y = circular_buffer_read(&c, NUM_SAMPLES_DELAY);
    
    //Shifting y back down to 10 bit
    
    //Converting to positive unsigned int values
    outputSample = (int16_t) y + 512;
  
    /* Signal processing stop*/
    
    //Using the DAC to convert and output the output sample after processing.
	DAC0_set_val(outputSample);
    
    //Resetting the TCA0 interrupt flag
	TCA0.SINGLE.INTFLAGS = TCA_SINGLE_OVF_bm;
}

/**
 * @brief Writes data to the next free space in the circular buffer c and
 * increments the writeIndex.
 * 
 * @param c A circular_buffer_t. Must be declared before use.
 * @param data The data to be written. Must be the same size as the 
 * data type used for circular_buffer_t.  
 */
void circular_buffer_write(circular_buffer_t *c, uint16_t data) {
    c -> buffer[c->writeIndex] = data;
    c->writeIndex = (c->writeIndex+1)%CIRCULAR_BUFFER_SIZE;
}

/**
 * @brief Reads data from a given position in the circular buffer relative to the current 
 * write index.
 * 
 * @param c A circular_buffer_t. Must be declared before use.
 * @param Xn The function will return the data at writeIndex-Xn. 
 * Xn should be between 0 and CIRCULAR_BUFFER_SIZE-1.
 */
uint16_t circular_buffer_read(circular_buffer_t *c, unsigned Xn) {
    return c->buffer[(c->writeIndex-Xn+CIRCULAR_BUFFER_SIZE-1)%CIRCULAR_BUFFER_SIZE];
}

int main(void)
{
    /* Run initialization functions */
    
    /* Configuring the system clock to run at 24MHz */
	CLK_configuration(); 
    /* Initializing reference voltages for DAC and ADC*/
	VREF_init(); 
    /* Initializing DAC */
	DAC0_init();
    /* Initializing ADC */
	ADC0_init();
    /* Initializing the TCA0 timer for regular interrupts at SAMPLING_PERIOD */
	TCA0_init(SAMPLING_PERIOD);
    
    /* Enabling global interrupts */
	sei();
    
    /* Setting PIN B3, LED0 on the board, as output. */
    PORTB.DIRSET = PIN3_bm;
    
    while (1) 
    {
        /* Blinking LED0 to verify that the MCU is running*/
		PORTB.OUT ^= PIN3_bm;
        _delay_ms(500);
    }
}