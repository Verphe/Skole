/*
 * INFO:
 * Denne koden demonsterer en bruk av ADCen og UASRTen til AVR128DB48 for å lage et enkelt, presist termometer. 
 * Spenning fra temperatursensoren blir punktprøvet med ADCen i en avbruddsbasert konfigurasjon og nye punktprøver blir akkumulert
 * i en ringbuffer med størrelse 2^N. Summen gjøres tilgjengelige for hovedløkka.
 * 
 * I hovedløkka brukes finner man middelverdien av de summerte punktprøvene, som lagres som et flyttall og skrives til UART.
 * 
 * Det er også lagt inn funksjonalitet som sjekker om avbruddsrutninen kjører for tregt ved å se på verdiene til telleren underveis i rutinen. Om telleren
 * rekker en hel periode før avbruddsrutinen er ferdig er prossesen for treg og systemet går inn i et feilmodus. Merk at dette ikke er en idéell løsning.
 *  
 * PINOUT
 *  - PortD pinne 1 (PD1): ADC inn (med mulighet for å bruke andre pinner)
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

#include "usart.h"
#include "peripherals.h"

//-------------------Brukerdefinerte parametere-------------------
#define SAMPLING_PERIOD 1333 //us 
#define N 12 //For 2^N
#define CIRCULAR_BUFFER_SIZE (1 << N) //Gir bufferstørrelse 2^N
#define ANALOG_AMPLIFICATION 1001 //Forsterking

//-------------------------------------------------------------

volatile int32_t new_sample_sum = 0;
volatile bool new_sample_sum_ready = false;

volatile bool error_mode = false;

/* Typedefinisjon for ringbufferen*/
typedef struct {
    int16_t * const buffer;
    unsigned writeIndex;
} circular_buffer_t;

/* Funksjonsprototyper */
void circular_buffer_write(circular_buffer_t *c, int16_t data);
int16_t circular_buffer_read(circular_buffer_t *c, unsigned Xn);
int32_t accumulator(int16_t sample);
int32_t accumulator_fast(int16_t sample);

/**
 * @brief ISR for sampling og D/A-konvertering, med et eksempel på signalbehandling.
 * ISRen utløses etter TCA0-telleren maksverdien sin (overflow med avbruddsvektor TCA0_OVF_vect).
 */
ISR(TCA0_OVF_vect) {
    // Tilbakestiller TCA0 avbruddsflagget
    TCA0.SINGLE.INTFLAGS = TCA_SINGLE_OVF_bm;
    
    // Variabel for å lagre nye punktprøver fra ADCen.
    static uint16_t input_sample = 0;
        
    static int32_t accumulated_samples = 0;
    
    // Punktprøving med ADCen
    input_sample = ADC0_read();
        
    accumulated_samples = accumulator((int16_t) input_sample);
    
    new_sample_sum = accumulated_samples;
    new_sample_sum_ready = true;
    
    /* Sjekker om ISRen er for treg ved å se om telleren har gått én runde. 
     * Hvis dette er tilfellet deaktiverer vi telleren. Systemet går så i feilmodus.
     */
    if (TCA0.SINGLE.INTFLAGS & TCA_SINGLE_OVF_bm) {
        error_mode = true;
        TCA0.SINGLE.CTRLA = ~TCA_SINGLE_ENABLE_bm; 
    }
}

/**
 * @brief Legger punktprøven sample i en buffer og akkumulerer innholdet.
 * Kan brukes til middelverdiberegning.
 * 
 * @param sample En punktprøve fra ADCen med typen int16_t.
 * @return Middelverdien som en int16_t.
 */
int32_t accumulator(int16_t sample) {
    //Deklarerer en ringbuffer med dataområde gitt av CIRCULAR_BUFFER_SIZE
    static int16_t bufferDataSpace[CIRCULAR_BUFFER_SIZE];
    static circular_buffer_t c = {
        .buffer = bufferDataSpace,
        .writeIndex = 0
    };
    
    circular_buffer_write(&c, sample);
    
    int32_t sum = 0;
    
    for(uint8_t n = 0; n < CIRCULAR_BUFFER_SIZE; n++) {
        sum += (int32_t) c.buffer[n];
    }
    
    return sum;
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
    c->writeIndex = (c->writeIndex+1)%CIRCULAR_BUFFER_SIZE;
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
    return c->buffer[(c->writeIndex-Xn+CIRCULAR_BUFFER_SIZE)%CIRCULAR_BUFFER_SIZE];
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
    
    /* Initialiserer UART med en baudrate på 9600 bits/s */
    USART3_init(9600);
    
    /* Aktiverer globale avbrudd */
	sei();
    
    /* Setter PIN B3, LED0 på AVR-kortet, som utgang. */
    PORTB.DIRSET = PIN3_bm;
    
    static int32_t current_sample_sum = 0;
    
    static double current_temp = 0;
    
    unsigned long uptime = 0;
    
    printf("--Starter termometer-- \n");
    
    while (1) 
    {
        /* Om vi har en ny akkumulert verdi og ikke er i feilmodus
         * printer vi ut verdien på terminalen. NB: skjer bare hvert 500ms.
         */
        if(new_sample_sum_ready && !error_mode) {
            current_sample_sum = new_sample_sum;
            
            /* Utregnign av middelverdien akkumulerte punktprøver med referansespenning definert i config.h.
             * Faktoren 10 er gitt av bitbredden til ADCen.
             */
            current_temp = (double) current_sample_sum*VOLTAGE_REFERENCE_VDD/(pow(2,N+10)*ANALOG_AMPLIFICATION);
            
            printf("Målings#: %lu, Spenning float: %.3f mV\n", uptime, current_temp);

            uptime++;
            new_sample_sum_ready = false;
        }
        
        /* Blinker LEDen for å vise at koden kjører. */
        PORTB.OUT ^= PIN3_bm;
        if(error_mode) {
            /* Rask eller ingen blink betyr at noe er galt*/
            _delay_ms(100);
            printf("-- Error mode! -- \n");
        } else {        
            /* Blinker hvert halve sekund ved normal operasjon*/
            _delay_ms(500);
        }
    }
}
