#include "uart.h"

/* LCD display */
void displayLCD(_Bool zone, unsigned char *temp, unsigned char *soil)
{
    showChar('T', pos1);
    showChar('M', pos4);

    if (! zone) /* zone 1*/
    {
        showChar(((temp[0]/10)+'0'), pos2);
        showChar(((temp[0]%10)+'0'), pos3);
        showChar(((soil[0]/10)+'0'), pos5);
        showChar(((soil[0]%10)+'0'), pos6);
    }
    else /* zone 2*/
    {
        showChar(((temp[1]/10)+'0'), pos2);
        showChar(((temp[1]%10)+'0'), pos3);
        showChar(((soil[1]/10)+'0'), pos5);
        showChar(((soil[1]%10)+'0'), pos6);
    }
}


/* Tx to UART */
void uartDisplay(uint8_t *sendText, uint8_t length)
{
    EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 13); /* send carrier return*/
    while(EUSCI_A_UART_queryStatusFlags(EUSCI_A0_BASE, EUSCI_A_UART_BUSY)) {} /* wait for UART to be free - stop busy */
    EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 10); /* send new line*/
    while(EUSCI_A_UART_queryStatusFlags(EUSCI_A0_BASE, EUSCI_A_UART_BUSY)) {} /* wait for UART to be free - stop busy */

    int i;
    for (i = 0 ; i < length ; i++)
    {
        EUSCI_A_UART_transmitData(EUSCI_A0_BASE, sendText[i]); /* send message */
        while(EUSCI_A_UART_queryStatusFlags(EUSCI_A0_BASE, EUSCI_A_UART_BUSY)) {} /* wait for UART to be free - stop busy */
    }

    if ((sendText[0] != '>') && (sendText[0] != '#') && (sendText[0] != ' ')) /* if not enter key or welcome message, it was command, make new line */
    {
        EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 13); /* send carrier return*/
        while(EUSCI_A_UART_queryStatusFlags(EUSCI_A0_BASE, EUSCI_A_UART_BUSY)) {} /* wait for UART to be free - stop busy */
        EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 10); /* send new line*/
        while(EUSCI_A_UART_queryStatusFlags(EUSCI_A0_BASE, EUSCI_A_UART_BUSY)) {} /* wait for UART to be free - stop busy */
        EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 10); /* send new line*/
        while(EUSCI_A_UART_queryStatusFlags(EUSCI_A0_BASE, EUSCI_A_UART_BUSY)) {} /* wait for UART to be free - stop busy */
        EUSCI_A_UART_transmitData(EUSCI_A0_BASE, '>'); /* send new prompt */
        while(EUSCI_A_UART_queryStatusFlags(EUSCI_A0_BASE, EUSCI_A_UART_BUSY)) {} /* wait for UART to be free - stop busy */
        EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 32); /* send space*/
    }
}

/* CLI welcome message */
void welcomeMsgCLI(void)
{
    uint8_t cliWelcome[100];
    int cliIndex;
    for (cliIndex = 0 ; cliIndex < 100 ; cliIndex++)
        cliWelcome[cliIndex]= 0; /* initialize welcome message */

    strcpy((char*) cliWelcome, "#               ECE 298 - Group 7              #");
    uartDisplay(cliWelcome, strlen((char*) cliWelcome));
    strcpy((char*) cliWelcome, " Threshold for each motor (0-100) must be set immediately \r\n");
    uartDisplay(cliWelcome, strlen((char*) cliWelcome));
    strcpy((char*) cliWelcome, " use the format xx:xx:, in the order temperature, moisture \r\n");
    uartDisplay(cliWelcome, strlen((char*) cliWelcome));
    strcpy((char*) cliWelcome, " use the format xxxx: where x is a 1 or 0 to set enables for motors \r\n");
    uartDisplay(cliWelcome, strlen((char*) cliWelcome));
    strcpy((char*) cliWelcome, " use the format xxxx: where x is a 1 or 0 to set enables for motors \r\n");
}

void receiveMsgCLI(void)
{
    uint8_t cliReceived[100];
    int cliIndex;

    strcpy((char*) cliReceived, "received\n");
    uartDisplay(cliReceived, strlen((char*) cliReceived));
    // strcpy((char*) cliReceived, "received\n");
    // uartDisplay(cliReceived, strlen((char*) cliReceived));
}

void tempMoistureCLI(float temperature, float moisture)
{
    uint8_t cliMessage[100];
    int cliIndex;
    
    sprintf(cliMessage, "Temperature: %d, moisture: %d\n", (int)temperature, (int)moisture);
    uartDisplay(cliMessage, strlen((char*) cliMessage));
}

void motorCLI(char* res) {
    uint8_t cliMessage[100];
    int cliIndex;
    
    sprintf(cliMessage, "%d, %d, %d, %d\n", res[0], res[1], res[2], res[3]);
    uartDisplay(cliMessage, strlen((char*) cliMessage));
}

