/* 
 * File:   extraTest.h
 * Author: hverp
 *
 * Created on 21 February 2025, 14:19
 */

#ifndef EXTRATEST_H
#define	EXTRATEST_H

#ifdef	__cplusplus
extern "C" {
#endif

#include <avr/io.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>    

void USART3_init(void);

void USART3_sendChar(char c);

void USART3_sendString(char *str);

char USART3_read(void);

int USART3_read_int(void);

#ifdef	__cplusplus
}
#endif

#endif	/* EXTRATEST_H */

