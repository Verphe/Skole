/*
 * INFO:
 * Denne koden implementerer utgangspunktet for et demoduleringssystem. I sin nåværende form tar systemet inn punktprøver med ADCen, filtrerer disse gjennom et 1 ordens IIR filter 
 * og tar så absoluttverdien av resultatet før dette sendes ut igjen på DACen. Resultatet er også tilgjengelig i hovedløkka og skrives der til terminalen med UART.
 *  * 
 * Det er også lagt inn funksjonalitet som sjekker om avbruddsrutninen kjører for tregt ved å se på verdiene til telleren underveis i rutinen. Om telleren
 * rekker en hel periode før avbruddsrutinen er ferdig er prossesen for treg og systemet går inn i et feilmodus. Merk at dette ikke er en idéell løsning.
 *  
 * PINOUT
 *  - PortD pinne 1 (PD1): ADC inn (med mulighet for å bruke andre pinner)
 *  - PortD pinne 6 (PD6): DAC ut
 *  - PortB pinne 3 (PB3): LED0 ut (LEDen på kortet)
 * 
 * Notes on the implementation:
 *  - ADC har en oppløsning på 10 bit.
 *  - Systemklokka er 24MHz, som er den maksimale hastigheten.
 *  - ADC-klokka er 2MHz, som er dens maksimale hastighet (see datablad for AVR128DB48).
 *  - Detaljer rundt periferienhetimplentasjoner finner man i usart.h/.c og peripherals.h/.c.
 *  - Filen config.h inneholder sentrale definisjoner som blir brukt i hele prosjektet.
 * 
 * Created: 17/06/2024
 * Author : Carl Richard Steen Fosse
 */ 

/* Global configurations for the project */
#include "config.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>

#include "usart.h"
#include "peripherals.h"

//-------------------Brukerdefinerte parametere-------------------
#define SAMPLING_PERIOD 1000 //us 
#define NUM_SAMPLES_DELAY 1

//Båndpassfilterparametre
#define N 1 //For 2^N
#define CIRCULAR_BUFFER_SIZE (1 << N) //Gir bufferstørrelse 2^N

/* Midlingsfilterparametre*/
#define N_mid 6
#define CIRCULAR_BUFFER_MID_SIZE (1 << N_mid)

/* Skaleringsfaktor for å unngå numerisk overflyt*/
#define A_FLOAT (float) 0.8
#define A (int16_t)(A_FLOAT*(1<<6))

/* Definisjon av filterorden*/
#define FILTER_ORDER 2

/* Filterkoeffsienter. Må være samme antall som orden. Her konfigurert som
 * høypass-filter.
 */
static const int16_t BP[FILTER_ORDER] = {(float)(-0.596)*(1<<6),(float)(-0.903)*(1<<6)}; 
static const int16_t BP0[FILTER_ORDER] = {(float)(0.596)*(1<<6),(float)(-0.903)*(1<<6)}; 

//-------------------------------------------------------------

/* Globale variabel for å holde på absluttverdien av filtrert data*/
volatile int16_t bp_filt_abs = 0;
volatile int16_t bp_filt_abs0 = 0;
static uint16_t output_sample = 0;
volatile bool result_ready = false;
volatile bool error_mode = false;

/* Typedefinisjon for ringbufferen*/
typedef struct {
    int16_t * const buffer;
    unsigned writeIndex;
    unsigned size;
} circular_buffer_t;

/* Funksjonsprototyper */
void circular_buffer_write(circular_buffer_t *c, int16_t data);
int16_t circular_buffer_read(circular_buffer_t *c, unsigned Xn);
int16_t bp_filter(int16_t sample);
int16_t bp_filter0(int16_t sample);
int16_t mean_filter(int16_t filtered_sample);
int16_t mean_filter0(int16_t filtered_sample);

/**
 * @brief ISR for sampling og D/A-konvertering, med et eksempel på signalbehandling.
 * ISRen utløses etter TCA0-telleren maksverdien sin (overflow med avbruddsvektor TCA0_OVF_vect).
 */
ISR(TCA0_OVF_vect) {
    // Tilbakestiller TCA0 avbruddsflagget
    TCA0.SINGLE.INTFLAGS = TCA_SINGLE_OVF_bm;

    
    // Variabel for å lagre nye punktprøver fra ADCen.
    static volatile uint16_t input_sample = 0;
    

    
    //Variabel til bruk under signalbehandlingen.
    static int16_t normalized_sample = 0;
    
    //Variabel for lagring av filterresultater.
    static int16_t bp_filt_result = 0;
    static int16_t bp_filt_result0 = 0;
    
    //Variabel for lagring av midlingsfilterresultater.
    static int16_t mean_filt_abs = 0;
    static int16_t mean_filt_abs0 = 0;
    //Punktprøving med ADCen
    input_sample = ADC0_read();
   
    /*--Signalbehandling starter--*/  
    
    /* Justerer punktprøvet verdi til å variere rundt 0 */
    normalized_sample = (int16_t)input_sample - 512;
    
    /* Skalerer for økt nøyaktighet  */
    normalized_sample = normalized_sample << 6;
    /* Skalerer for å hindre numerisk overflyt*/
    normalized_sample = normalized_sample - ((int32_t)A*normalized_sample>>6);
    
    /* Høypassfiltrering av punktprøven */
    bp_filt_result = bp_filter(normalized_sample); 
    bp_filt_result0 = bp_filter0(normalized_sample); 

    /* Energiestimat med absoluttverdien av punktprøven. Se math.h*/
    bp_filt_abs = abs(bp_filt_result);
    bp_filt_abs0 = abs(bp_filt_result0);
    
    /* Middelverdifiltrering av energiestimatene*/
    mean_filt_abs = mean_filter(bp_filt_abs);
    mean_filt_abs0 = mean_filter0(bp_filt_abs0);
    
    result_ready = true;
    
    /* Justerer filtrert resultat til å kunne sendes ut på DACen*/
    if (mean_filt_abs < mean_filt_abs0){
        output_sample = 1023;
    }
    else if (mean_filt_abs > mean_filt_abs0){
        output_sample = 300;
    }
        
    /*--Signalbehandling slutter--*/         
    DAC0_set_val(output_sample);
    
    /* Sjekker om ISRen er for treg ved å se om telleren har gått én runde. 
     * Hvis dette er tilfellet deaktiverer vi telleren. Systemet går så i feilmodus.
     */
    if (TCA0.SINGLE.INTFLAGS & TCA_SINGLE_OVF_bm) {
        error_mode = true;
        TCA0.SINGLE.CTRLA = ~TCA_SINGLE_ENABLE_bm; 
    }
}

/**
 * @brief Et førsteordens rekursivt høypassfilter.
 * 
 * @param sample En 16-bits verdi som skal filtreres.
 */
int16_t bp_filter(int16_t sample) {
    //Deklarerer en ringbuffer med dataområde gitt av CIRCULAR_BUFFER_SIZE
    static int16_t bp_buffer_data_space[CIRCULAR_BUFFER_SIZE];
    static circular_buffer_t bp_filt_buff = {
        .buffer = bp_buffer_data_space,
        .writeIndex = 0,
        .size = CIRCULAR_BUFFER_SIZE
    };
    
    static int16_t delayed_sample = 0;
    static int16_t delayed_sample2 = 0;
    static int16_t result = 0;
    
    delayed_sample = circular_buffer_read(&bp_filt_buff, 0);
    delayed_sample2 = circular_buffer_read(&bp_filt_buff, NUM_SAMPLES_DELAY);
    
    result = sample + ((int32_t)BP[0]*delayed_sample>>6) + ((int32_t)BP[1]*delayed_sample2>>6);
    
    circular_buffer_write(&bp_filt_buff, result);
    
    return result;
}

int16_t bp_filter0(int16_t sample) {
    //Deklarerer en ringbuffer med dataområde gitt av CIRCULAR_BUFFER_SIZE
    static int16_t bp_buffer_data_space0[CIRCULAR_BUFFER_SIZE];
    static circular_buffer_t bp_filt_buff0 = {
        .buffer = bp_buffer_data_space0,
        .writeIndex = 0,
        .size = CIRCULAR_BUFFER_SIZE
    };
    
    static int16_t delayed_sample = 0;
    static int16_t delayed_sample2 = 0;
    static int16_t result0 = 0;
    
    delayed_sample = circular_buffer_read(&bp_filt_buff0, 0);
    delayed_sample2 = circular_buffer_read(&bp_filt_buff0, NUM_SAMPLES_DELAY);
    
    result0 = sample + ((int32_t)BP0[0]*delayed_sample>>6) + ((int32_t)BP0[1]*delayed_sample2>>6);
    
    circular_buffer_write(&bp_filt_buff0, result0);
    
    return result0;
}

int16_t mean_filter(int16_t filtered_sample){
    static int16_t mean_buffer_data_space[CIRCULAR_BUFFER_MID_SIZE];
    static circular_buffer_t mean_filt_buff = {
        .buffer = mean_buffer_data_space,
        .writeIndex = 0,
        .size = CIRCULAR_BUFFER_MID_SIZE
    };
    
    static int32_t sum = 0;
    //static int32_t mean_sum = 0;
    
    int16_t old_value = circular_buffer_read(&mean_filt_buff, -1);
    sum -= old_value;
    circular_buffer_write(&mean_filt_buff, filtered_sample);
    sum += filtered_sample;
    
    //mean_sum = sum / CIRCULAR_BUFFER_MID_SIZE;
    
    return (sum >> N_mid);
}

int16_t mean_filter0(int16_t filtered_sample){
  static int16_t mean_buffer_data_space0[CIRCULAR_BUFFER_MID_SIZE];
    static circular_buffer_t mean_filt_buff0 = {
        .buffer = mean_buffer_data_space0,
        .writeIndex = 0,
        .size = CIRCULAR_BUFFER_MID_SIZE
    };
    
    static int32_t sum0 = 0;
    //static int32_t mean_sum0 = 0;
    
    int16_t old_value = circular_buffer_read(&mean_filt_buff0, -1);
    sum0 -= old_value;
    circular_buffer_write(&mean_filt_buff0, filtered_sample);
    sum0 += filtered_sample;
    //mean_sum0 = sum / CIRCULAR_BUFFER_MID_SIZE;
    
    //return mean_sum0;  
    return(sum0 >> N_mid);
}

/**
 * @brief Skriver data til neste ledige plass i ringbufferet c og
 * øker writeIndex.
 * 
 * @param c En circular_buffer_t. Må deklareres før bruk.
 * @param data Dataene som skal skrives. Må være samme størrelse som 
 * datatypen brukt for circular_buffer_t.  
 */
void circular_buffer_write(circular_buffer_t *c, int16_t data) {
    c -> buffer[c->writeIndex] = data;
    c->writeIndex = (c->writeIndex+1)%(c->size);
}

/**
 * @brief Leser data fra en gitt posisjon i ringbufferen relativt til nåværende 
 * writeIndex.
 * 
 * @param c En circular_buffer_t. Må deklareres før bruk.
 * @param Xn Funksjonen vil returnere dataene ved writeIndex-Xn. 
 * Xn bør være mellom 0 og CIRCULAR_BUFFER_SIZE-1.
 */
int16_t circular_buffer_read(circular_buffer_t *c, unsigned Xn) {
    return c->buffer[(c->writeIndex-Xn+(c->size)-1)%(c->size)];
}

int main(void)
{
    /* Run initialization functions */
    
    /* Konfigurerer systemklokka til å kjøre på 24MHz */
	CLK_configuration(); 
    /* Initialiserer referansespenning for DAC and ADC.
     * VDD brukes for begge.
     */
	VREF_init(); 
    /* Initialiserer DAC */
	DAC0_init();
    /* Initialiserer ADC */
	ADC0_init();
    /* Initialiserer TCA0 telleren med regulære avbrudd hver SAMPLING_PERIOD */
	TCA0_init(SAMPLING_PERIOD);
    
    /* Initialiserer UART med en baudrate på 115200 bits/s */
   USART3_init(20);
    
    /* Aktiverer globale avbrudd */
	sei();
    
    /* Setter PIN B3, LED0 på AVR-kortet, som utgang. */
    PORTB.DIRSET = PIN3_bm;

    while (1) 
    {
        if(result_ready) {
            printf("%d\r\n", output_sample);
        }
                
        /* Blinker LEDen for å vise at koden kjører. */
        PORTB.OUT ^= PIN3_bm;
        if(error_mode) {
            /* Rask eller ingen blink betyr at noe er galt*/
            _delay_ms(100);
            //printf("-- Error mode! -- \n");
        } else {
            _delay_ms(500);
        }
            
    }
}
