/*
 * morse.c
 *
 *  Created on: Apr 23, 2021
 *      Author: ENDUSER
 */

#include "morse.h"
#include "cmsis_os.h"

#define NULL ((char *)0)

char *letters[] = {
  // The letters A-Z in Morse code
  ".-", "-...", "-.-.", "-..", ".", "..-.", "--.", "....", "..",
  ".---", "-.-", ".-..", "--", "-.", "---", ".--.", "--.-", ".-.",
  "...", "-", "..-", "...-", ".--", "-..-", "-.--", "--.."
};

char *numbers[] = {
  // The numbers 0-9 in Morse code
  "-----", ".----", "..---", "...--", "....-", ".....", "-....",
  "--...", "---..", "----."
};

unsigned int dot_duration = 200;
/**
  *  Flashes the Morse code for the input letter or number
  *  @param morse_code pointer to the morse code
  */
void flash_morse_code(char *morse_code) {

  unsigned int i = 0;

  // Read the dots and dashes and flash accordingly
  while (morse_code[i] != NULL) {
    flash_dot_or_dash(morse_code[i]);
    i++;
  }

  // Space between two letters is equal to three dots
  osDelay(dot_duration * 3);
}

/**
  *  Flashes the dot or dash in the Morse code
  *  @param dot_or_dash character that is a dot or a dash
  */
void flash_dot_or_dash(char dot_or_dash) {

  // Make the LED shine
  ledMorseCode(ON);


  if (dot_or_dash == '.') { // If it is a dot
	  osDelay(dot_duration);
  }
  else { // Has to be a dash...equal to three dots
	  osDelay(dot_duration * 3);
  }

  // Turn the LED off
  ledMorseCode(OFF);

  // Give space between parts of the same letter...equal to one dot
  osDelay(dot_duration);
}

