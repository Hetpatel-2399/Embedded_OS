/*
 * morse.h
 *
 *  Created on: Apr 23, 2021
 *      Author: ENDUSER
 */

#ifndef INC_MORSE_H_
#define INC_MORSE_H_


typedef enum{
	OFF = 0,
	ON = 1
}MORSELED;

extern char *letters[];
extern char *numbers[];
extern unsigned int dot_duration;


extern void ledMorseCode(MORSELED on);
void flash_morse_code(char *morse_code);
void flash_dot_or_dash(char dot_or_dash);

#endif /* INC_MORSE_H_ */
