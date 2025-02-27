#define F_CPU 24000000UL
#define USART_BAUD_RATE(BAUD_RATE) ((float)(F_CPU * 64 / (16 * (float)BAUD_RATE)) + 0.5)

#include "extraTest.h"  

static int USART3_printChar(char c, FILE *stream)
{
 USART3_sendChar(c);
 return 0;

}
static FILE USART_stream = FDEV_SETUP_STREAM(USART3_printChar, NULL, _FDEV_SETUP_WRITE);

void USART3_init(void)
{
PORTB.DIRSET = PIN0_bm; //TX-pinne som inngang, endret fra PORTC.DIR |= PIN0_bm;
PORTB.DIRCLR = PIN1_bm; //RX-pin som utgang, endret fra PORTC.DIR &= ~PIN1_bm;

USART3.BAUD = (uint16_t)USART_BAUD_RATE(9600);

USART3.CTRLB |= USART_TXEN_bm;
USART3.CTRLB |= USART_RXEN_bm;
stdout = &USART_stream;

//USART3.CTRLA |= USART_RXCIE_bm;
}

void USART3_sendChar(char c)
{
    while (!(USART3.STATUS & USART_DREIF_bm)) //vent på at dataregisteret til USART3 tømmes
    {
    ;
    }
    USART3.TXDATAL = c; // Send c
}

void USART3_sendString(char *str)
    {
    for(size_t i = 0; i < strlen(str); i++) //sender et og et tegn med USART3_sendChar()
    {
        USART3_sendChar(str[i]);
    }
}

char USART3_read(void) {
    while (!(USART3.STATUS & USART_RXCIF_bm));  // Vent på data
    return USART3.RXDATAL;
}

int USART3_read_int(void) {
    char buffer[10];  // Buffer for å holde inndata
    uint8_t i = 0;
    
    while (1) {
        char c = USART3_read();  // Les ett tegn om gangen
        if (c == '\n' || c == '\r') {  // Avslutt hvis Enter trykkes
            buffer[i] = '\0';
            break;
        }
        if (i < sizeof(buffer) - 1) {
            buffer[i++] = c;
        }
    }
    
    return atoi(buffer);  // Konverter tekst til int
}


