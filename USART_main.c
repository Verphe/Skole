/*
 * File:   main.c
 * Author: hverp
 *
 * Created on January 16, 2025, 9:01 PM
 */

#define F_CPU 4000000UL //Standardklokkefrekvens
#define USART_BAUD_RATE(BAUD_RATE) ((float)(F_CPU * 64 / (16 * (float)BAUD_RATE)) + 0.5) //BAUD_RATE eq. fra usart-guide
#define COMMON_BAUD_RATE 9600

#include <avr/io.h>
#include <util/delay.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

//static const char tx_char = '3';
char tx_char = 0;
char rx_char = 0;

//Initialiserer alle funksjoner før de lages
void USART3_init(unsigned long baud);
void USART3_sendChar(char c);
uint8_t USART3_read();
static int USART3_printChar(char c, FILE *stream);
//static int USART3_printChar(char c, FILE *stream);
void USART2_init(unsigned long baud);
void USART2_sendChar(char c);
uint8_t USART2_read();
void flush();
void timeout();
void BFR();


static FILE USART_stream = FDEV_SETUP_STREAM(USART3_printChar, NULL, _FDEV_SETUP_WRITE);


int main(void)
{
    PORTB.DIRSET = PIN3_bm;
    PORTB.OUT |= PIN3_bm;
    
    stdout = &USART_stream;
    
    USART2_init(COMMON_BAUD_RATE);
    USART3_init(COMMON_BAUD_RATE);
    //rx_char = 0;
    //printf("Cool dude\r\n");
    srand(time(NULL)); 
    int j = 0;
    
    while (j < 1000)
    {
        ///*
        flush();
        int i = rand() % 2;
        tx_char = i + '0';
        USART3_sendChar(tx_char);
        _delay_ms(2);
        rx_char = USART2_read();
        
        
        if (rx_char == tx_char) {
            PORTB.OUT &= ~PIN3_bm;
            //printf("%c \r\n",tx_char);
            //printf("\n TX OK!\n\n\r");
        } else {
            PORTB.OUT |= PIN3_bm;
            //printf("%c. \r\n",tx_char);
            //flush();
            //printf("TX BAD!\n\n\r");
            //_delay_ms(10);
        }
         //*/
        printf("Sent: %c, Received: %c\n\r", tx_char, rx_char);
        BFR();
        rx_char = 0;
        _delay_ms(10);
        j++;
    }
}

void BFR()
{
    static uint16_t charNum = 0;
    static uint16_t bitError = 0;
    static double bitErrorRatio = 0;

    uint8_t temp1 = tx_char ^ rx_char;

    for(int i = 0; i < 8; i++){
        if((temp1 / 128) >= 1){
            bitError++;
        }
        temp1 = temp1 << 1;
    }

    charNum++;
    bitErrorRatio = (float)bitError / (float)(8 * charNum);
    printf("BER: %f\r\n\n\n", bitErrorRatio);
    printf("------------------\r\n");
}

static int USART3_printChar(char c, FILE *stream)
{
 USART3_sendChar(c);
 return 0;
}


void USART2_init(unsigned long baud)
{
    PORTMUX.USARTROUTEA |= PORTMUX_USART2_ALT1_gc;
    PORTF.DIR &= ~PIN5_bm; //Set 1 to input
    PORTF.DIR |= PIN4_bm; //Set 0 to output

    USART2.BAUD = (uint16_t)USART_BAUD_RATE(baud); //Setter baud-rate
    USART2.CTRLB |= USART_TXEN_bm; //Setter transmitteren på 
    USART2.CTRLB |= USART_RXEN_bm; //Setter receiveren på 
}

void USART2_sendChar(char c)
{
 while (!(USART2.STATUS & USART_DREIF_bm)) //Dersom usart er klar og data registeret er tomt:
 {
     ;
 }
 USART2.TXDATAL = c; //Skriv ut data
}

uint8_t USART2_read()
{
 while (!(USART2.STATUS & USART_RXCIF_bm))
 {
  timeout();
 }
 return USART2.RXDATAL;
}

void USART3_init(unsigned long baud)
{
    PORTB.DIR &= ~PIN1_bm; //Set 1 to input
    PORTB.DIR |= PIN0_bm; //Set 0 to output

    USART3.BAUD = (uint16_t)USART_BAUD_RATE(baud); //Setter baud-rate
    USART3.CTRLB |= USART_TXEN_bm; //Setter transmitteren på 
    USART3.CTRLB |= USART_RXEN_bm; //Setter receiveren på
}

void USART3_sendChar(char c)
{
 while (!(USART3.STATUS & USART_DREIF_bm)) //Dersom usart er klar og data registeret er tomt:
 {
 ;
 }
 USART3.TXDATAL = c; //Skriv ut data
}


uint8_t USART3_read()
{
 while (!(USART3.STATUS & USART_RXCIF_bm))
 {   
 ;
 }
 return USART3.RXDATAL;
}

void flush() {
    // Tøm RX-bufferet ved å lese alle data som fortsatt er i RXCIF
    while ((USART2.STATUS & USART_RXCIF_bm)) {
        volatile uint8_t flushVar = USART2.RXDATAL; // Les og forkast data
    }
}
    
void timeout(){
    int16_t timeout_time = 1000; 
    while (!(USART2.STATUS & USART_RXCIF_bm) && timeout_time--) {
        _delay_ms(1);
        if(timeout_time == 900){
            printf("Timed out!");
        }
    }
    if(timeout_time == 0){
        printf("Resetting USART2!");
        USART2_init(COMMON_BAUD_RATE);
        return;
    }
}
