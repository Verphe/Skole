/*
 * main.c
 *
 * Created: 7/5/2022 9:48:26 AM
 * Author: Microchip Technology
 *
 * Info: Dette prosjektet demonstrerer KissFFT-biblioteket (et C-bibliotek for beregning av FFT) på AVR128DB48. Det er basert på et eksempel fra Microchip. 
 * Testen gjennomføres ved at det tas FFT av en sinus lagret i minne. Effekten av hver frekvenskomponent i FFT blir så beregnet før dataen sendes over UART.
 * Det er deretter mulig å visualisere dataen i Data Visualizer i MPLAB X.rosjektet er konfigurert til å bruke fixed-point beregninger på 16 bit. 
 * Se README og GitHub (https://github.com/microchip-pic-avr-examples/avr64ea48-digital-filters-studio/tree/master/fft) 
 * for detaljert info om demonstrasjonen. 
 *
 * Adjusted for AVR128DB48: March, 2025
 * Editor: Carl Richard Steen Fosse
 */ 

#include <xc.h>
#include "filter/kiss_fftr.h"
#include "filter/sine.h"
#include "peripherals/clock/clkctrl.h"
#include "peripherals/usart/usart3.h"
#include "peripherals/data_streamer/data_streamer.h"
#include <avr/cpufunc.h>
#include <math.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "peripherals/i2c/i2c.h"
#include "display/ssd1306.h"
 
#define NFFT 256

kiss_fft_scalar cpx_in[NFFT]; 
volatile uint16_t samples_amt = 0;
volatile bool buffer_full = false;
 
void ADC0_init(void);
void ADC0_start(void);

void ADC0_init(void)
{
    /* PIN D0*/
    /* Disable signal input buffer*/
    PORTD.PIN0CTRL &= ~PORT_ISC_gm;
    PORTD.PIN0CTRL |= PORT_ISC_INPUT_DISABLE_gc;
    
    /* Disable pull-up resistor */
    PORTD.PIN0CTRL &= PORT_PULLUPEN_bm;
    
    ADC0.CTRLC = ADC_PRESC_DIV256_gc;
    
    ADC0.CTRLA = ADC_ENABLE_bm /* ADC ENABLE: Enabled*/
                | ADC_RESSEL_10BIT_gc; /* 10-bit mode */
    
    /* Select ADC channel */
    ADC0.MUXPOS = ADC_MUXPOS_AIN0_gc;
    
    VREF.ADC0REF |= VREF_REFSEL_VDD_gc;
    
    ADC0.INTCTRL = ADC_RESRDY_bm;
}

void ADC0_start(void)
{
    samples_amt = 0;
    buffer_full = false;
    ADC0.COMMAND = ADC_STCONV_bm;
}

ISR(ADC0_RESRDY_vect)
{
    //Sett interruptflag
    ADC0.INTFLAGS = ADC_RESRDY_bm;
    
    //Les av ADC
    uint16_t result = ADC0.RES;
    
    //Sampling
    if(samples_amt < NFFT)
    {
        cpx_in[samples_amt++] = result;
    }
    
    //Full buffer
    if(samples_amt >= NFFT)
    {
        buffer_full = true;
        //Skru av ADC-teller-pin
        PORTD.OUTCLR = PIN3_bm;

        //Skru av interruptflag
        ADC0.INTCTRL &= ~ADC_RESRDY_bm;
    }
    else
    {
        //Start convertering
        ADC0.COMMAND = ADC_STCONV_bm;
    }
}
 
 int main(void)
 {
    clkctrl_init();
    USART3_Initialize();
    ADC0_init();
    
    //Setting up display
    uint8_t retVal = 0;
	retVal = SSD1306_Init (SSD1306_ADDR);  
     
    //Configuring pin for time performance measurement
    PORTD.DIRSET = PIN3_bm;
    PORTD.DIRSET = PIN4_bm;
    PORTB.DIRSET = PIN3_bm;
    volatile int out = (NFFT/2)+1;

    kiss_fftr_cfg cfg = kiss_fftr_alloc(NFFT, 0, NULL, NULL);  //kiss_fft_alloc( nfft ,is_inverse_fft ,0,0 );
    kiss_fft_cpx cpx_out[out];

    volatile int16_t watch_real=4;
    volatile int16_t watch_imag = 35;
    volatile uint32_t pwr;
    volatile uint16_t cnt=0;
    

    sei();
    PORTD.OUTSET = PIN3_bm;
    ADC0_start();
    
    while(1) {
        if(buffer_full){
            PORTB.DIRTGL |= PIN3_bm;
            SSD1306_ClearScreen();
            PORTD.OUTSET = PIN4_bm; // Make PD4 output logic high
            kiss_fftr(cfg, cpx_in , cpx_out);      // The actual FFT operation
            PORTD.OUTCLR = PIN4_bm; // Make PD4 output logic low

            for(int n=out; n>0; n--)
            {
                //putting cpx_out.r  into watchable variables
                cnt = n;
                watch_real = cpx_out[n].r;
                watch_imag = cpx_out[n].i;

                //Calculating the power spectrum
                pwr = 20*log10(cpx_out[n].r * cpx_out[n].r + cpx_out[n].i * cpx_out[n].i);

                //Writing four variables to USART, so they can be read by MPLAB DV:
                // 0. Start token
                // 1. watch_real (int16_t)
                // 2. watch_imag (int16_t)
                // 3. cnt (uint16_t))
                // 4. pwr (uint_32_t))
                // .. and the end token
                variableWrite_SendFrame(watch_real, watch_imag, cnt, pwr);
                SSD1306_DrawLine (out-n, out-n+1, 0, pwr);
            }
            SSD1306_UpdateScreen (SSD1306_ADDR);
            //SSD1306_ClearScreen();
            
            ADC0_start();
            ADC0.INTCTRL |= ADC_RESRDY_bm;
        }
    }
    kiss_fft_cleanup();                         // Tidy up after you.
 }
