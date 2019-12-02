#include "main.h"
#include "hal_LCD.h"
#include <stdbool.h>
#include <stdio.h>

void welcomeMsgCLI(void); /* welcome message on CLI */
void receiveMsgCLI(void);

void displayLCD(_Bool zone, unsigned char *temp, unsigned char *soil); /* display on LCD */

void uartDisplay(uint8_t *sendText, uint8_t length); /* send to UART */
void tempMoistureCLI(float temperature, float moisture);
void motorCLI(char* res);
#define MOTOR_CYCLES 20