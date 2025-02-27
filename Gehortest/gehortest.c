/*
 * File:   main.c
 * Author: crfosse
 * 
 * Dette programmet gjennomfÃ¸rer et frekvenssveip ved hjelp av direkte digital syntese (DDS) pÃ¥ DACen til AVR128DB48. Sveipet gjennomfÃ¸res ved hjelp av telleren TCA0 og er avbruddsstyrt for Ã¥ fÃ¥ en jevn
 * punktprÃ¸vingsfrekvens. Prinsippene for DDS og frekvenssveip, samt noen praktiske detaljer, er beskrevet det det tekniske notatet [1]. Sveipet bruker en cosinus-oppslagstabell pÃ¥ 2^13 punkter, lagret i "cosine_table.h".
 * Noen detaljer:
 * - Utgangspinne for DAC: pinne PD6.
 * - Prossessorfrekvens: 24MHz.
 * - PunktprÃ¸vingsfrekvens: 16384 Hz.
 *
 * Kilder:
 * [1] - L. Lundheim: Generering av frekvenssveip, internt notat, NTNU, 2025
 * Created on 6. februar 2025, 14:42
 */

/* Faste konstanter - disse trenger man i utgangspunktet ikke Ã¥ justere*/
#define F_CPU 24000000UL

#include <avr/io.h>
#include <avr/cpufunc.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include "cosine_table.h"
#include "extraTest.h"

/* Parametere for frekvenssveip. Justering av disse endrer oppfÃ¸rselen til sveipet. */


const uint32_t M_SAMPLES = 8192; /* Antall punktprÃ¸ver i oppslagstabellen */
const uint32_t F_SAMPLE = 16384; /* Hz - Samplingsfrekvens */

volatile int menuVariable = 1;
volatile uint32_t guessedFrequency = 0;
volatile uint32_t rand_seed = 0;

volatile uint16_t F_0;
volatile uint16_t F_1;
volatile uint16_t F_RAND;
volatile int T_SWEEP;


/* Beregning av initialverdi for d_k og konstanten dd_k. */
//#define K_ZERO (float)M_SAMPLES*F_0/(float)F_SAMPLE*((uint32_t)1<<16) /* Ligning 16 i [1] */
//#define DELTA_DELTA_K (float)M_SAMPLES*(F_1-F_0)/((float)T_SWEEP*F_SAMPLE*F_SAMPLE)*((uint32_t)1<<16) /* Ligning 19 i [1]*/

/**
 * @brief Initialiserer hovedklokkka (F_CPU) til maksimal hastighet: 24MHz. 
 * Det er mulig Ã¥ endre til andre klokkefrekvenser i funksjonen.
 */

void inputFromUser();
void run_sweep();
void DAC0_init(void);
void TCA0_init();
void CLK_configuration(void);
void statusCase();
void printResults();
void randomSeedGenerator();

void CLK_configuration(void)
{
	/* Setter OSCHF-klokka til 24 MHz */
	ccp_write_io ((uint8_t *) & CLKCTRL.OSCHFCTRLA, CLKCTRL_FRQSEL_24M_gc 
                                                   | CLKCTRL_AUTOTUNE_bm);
}

 /**
 * @brief Initialiserer DACen. Den bruker pinne PD6.
 */
 void DAC0_init(void)
 {
     /* Konfigurerer pinne PD6*/
	 /* Deaktiverer digital input buffer */
	 PORTD.PIN6CTRL &= ~PORT_ISC_gm;
	 PORTD.PIN6CTRL |= PORT_ISC_INPUT_DISABLE_gc;
     
	 /* Deaktiverer opptrekksmotstand */
	 PORTD.PIN6CTRL &= ~PORT_PULLUPEN_bm;
     
     VREF.DAC0REF |= VREF_REFSEL_VDD_gc;

	 /* Aktiverer DACen, utgangsbuffer */
	 DAC0.CTRLA = DAC_ENABLE_bm | DAC_OUTEN_bm;
 }

  /**
 * @brief Setter Ã¸nsket verdi pÃ¥ DACen.
 *
 * @Param val 10 bits verdi som skal konverters.
 */
 void DAC0_set_val(uint16_t val)
 {
	 /* Lagrer de to LSbene i DAC0.DATAL */
	 DAC0.DATAL = (val & (0x03)) << 6;
	 /* Lagrer de Ã¥tte MSbene i DAC0.DATAH */
	 DAC0.DATAH = val >> 2;
 }
 
 /**
 * @brief Initialiserer telleren TCA0. Telleren aktiveres ikke.
 */
void TCA0_init() {
    /* Konfigrerer tellerperioden (hvor mange klokkesykluser den skal telle).
    *  Formel: F_CPU/(F_CNT*SKALERING)
    */
	TCA0.SINGLE.PER = (uint16_t)(F_CPU/(F_SAMPLE*1));
    
    /* Aktiverer avbrudd fra telleroverflyt */
	TCA0.SINGLE.INTCTRL = TCA_SINGLE_OVF_bm;
	
    /* Setter telleren i 16-bits modus med en klokkefrekvens pÃ¥ F_FPU/1. */
  	TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV1_gc; 

}

 /**
 * @brief Setter i gang et sveip ved Ã¥ aktivere telleren.
 */
void run_sweep() 
{
    /* Tenner LED0 for Ã¥ markere start pÃ¥ sveip */
    PORTB.OUT &= ~PIN3_bm;

    /* Aktiverer telleren for Ã¥ starte sveip*/
    TCA0.SINGLE.CTRLA = TCA_SINGLE_ENABLE_bm; 
}

 /**
  * @brief Avbruddsrutine for telleren. Denne hÃ¥ndterer frekvenssveipet.
  */
 ISR(TCA0_OVF_vect) {
    
    /* Definerer variabler for Ã¥ holde punktprÃ¸ver*/
    static int16_t curr_sample = 0; /* Verdier med fortegn fra cosinustabellen */
    static uint16_t dac_val = 0; /* Verdier uten fortegn som skal skrives til DACen*/
    /* Definerer variabler for Ã¥ hÃ¥ndtere sveipet og frekvensgenrerering. 
    *  Se notatet [1] for detaljer.*/
    
    static uint16_t k = 0;
    
    static uint32_t d_k;
    static uint32_t d_k_mono;
    static uint32_t dd_k;
    static int initialised = 0;
    
    if(!initialised)
    {        
        d_k = (float)M_SAMPLES*F_0/(float)F_SAMPLE*((uint32_t)1<<16);
        d_k_mono = (float)M_SAMPLES*F_RAND/(float)F_SAMPLE*((uint32_t)1<<16);
        dd_k = (float)M_SAMPLES*(F_1-F_0)/((float)T_SWEEP*F_SAMPLE*F_SAMPLE)*((uint32_t)1<<16);
        initialised = 1;
    }
    /* Kjører sweep eller tone basert på menucase*/
    if(menuVariable == 2){
        k = (k+(d_k_mono>>16)) % M_SAMPLES;
    } 
    else
    {
        d_k = (d_k + dd_k)  % ((uint32_t)F_1 << (15)); 
        guessedFrequency = d_k; //Oppdaterer frekvensen globalt
        k = (k+(d_k>>16)) % M_SAMPLES;
    }
            
    curr_sample = sineLookupTable[k];

    /* Konverterer til positiv 10-bits verdi*/
    dac_val = (curr_sample >> 6)+512; 
    DAC0_set_val(dac_val);
    
    /* NÃ¥r d_k er 0 er et sveip gjennomfÃ¸rt. */
    if(d_k==0) 
    {
        /* Slukker LED0 */
        PORTB.OUT = PIN3_bm;

        /* SlÃ¥r av telleren */
        TCA0.SINGLE.CTRLA &= ~TCA_SINGLE_ENABLE_bm; 

        /* Setter d_k til initialverdi */
        d_k = (float)M_SAMPLES*F_0/(float)F_SAMPLE*((uint32_t)1<<16);
        
        //Gå til neste handling i systemet
        menuVariable++;
    }
    
    /* Tilbakestiller avbruddsflagget for TCA0 */
	TCA0.SINGLE.INTFLAGS = TCA_SINGLE_OVF_bm;
}
 
ISR(PORTB_PORT_vect)
{
    if(PORTB.INTFLAGS & PIN2_bm)
    {
            
        //Hopper videre i switch case
        menuVariable++;
        
        //Skrur av teller
        TCA0.SINGLE.CTRLA &= ~TCA_SINGLE_ENABLE_bm;
        
        _delay_ms(2000);
        
        //Aktiverer knappens interruptflag
        PORTB.INTFLAGS = PIN2_bm;

    }
}

void statusCase()
{
    switch(menuVariable)
    {
        //Startup. Venter pa at brukeren skal starte
        case 0:
            randomSeedGenerator();
            break;
        //Input fra bruker
        case 1:
            cli();
            inputFromUser();
            sei();
            break;
        //Spill av den tilfeldige frekvensen
        case 2:
            run_sweep();
            break;
        //Spill av frekvenssveip
        case 3:
            run_sweep();
            break;
        //Skriver ut resultater
        case 4:
            cli();
            printResults();
            break;
    }
} 

void inputFromUser()
{
    printf("\r\nSett en startsfrekvens: ");
    F_0 = USART3_read_int();
    while(USART3_read() != '\n'); // Tømmer bufferen”
    printf("%d\r\n", F_0);

    printf("\r\nSett en sluttfrekvens: ");
    F_1 = USART3_read_int();
    while(USART3_read() != '\n'); // Tømmer bufferen”

    
    printf("\r\nSett varighet på frekvenssveipet: ");
    T_SWEEP = USART3_read_int();
    while(USART3_read() != '\n'); // Tømmer bufferen”

    //Utregning av random variabel
    srand(rand_seed);
    F_RAND = F_0 + rand() % (F_1 - F_0);
    
    menuVariable++;
}

void printResults()
{
    uint32_t multiplexer = (float)F_SAMPLE*(guessedFrequency>>16)/(float)M_SAMPLES;
    printf("You pressed at %d\r\n", multiplexer);
    menuVariable++;
    int differenceInGuess = abs(F_RAND - multiplexer);
    if(differenceInGuess <= 100 )
    {
        printf("You won!\r\n");
    }
    else 
    {
        printf("You lost! You were %d away.", differenceInGuess);
    }
}

void randomSeedGenerator()
{
    rand_seed++;
    if((rand_seed) >= pow(2,31)){
        rand_seed = 0;
    }
}
 

 
int main(void) 
{   
    /* Setter hovedklokka til 24MHz for Ã¥ fÃ¥ hÃ¸y samplingsfrekvens. */
    CLK_configuration();

    /* Initialiserer DACen*/
    DAC0_init();

    /* Intitaliserer telleren. Merk: den aktiveres ikke.*/
    TCA0_init();
    
    USART3_init();
    
    /* Setter pinne PB2, BTN0 pÃ¥ kortet, som inngang. */
    PORTB.DIR &= ~ PIN2_bm;
    PORTB.PIN2CTRL |= PORT_PULLUPEN_bm | PORT_ISC_FALLING_gc;    

    /* Setter pinne PB3, LED0 pÃ¥ kortet, som utgang. */
    PORTB.DIRSET = PIN3_bm;
    PORTB.OUT = PIN3_bm;
    
    sei();

    while (1) 
    {
        statusCase();
    }
}
