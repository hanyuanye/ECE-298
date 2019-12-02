// #include "driverlib.h"
// #include <msp430fr4133.h>
// #include <msp430.h> 
// #include <stdio.h>
// #include <stdlib.h>

// #define one 0x60
// #define two 0xDB 
// #define three 0xF3
// #define four 0x67
// #define five 0xB7
// #define six 0xBF
// #define seven 0XE4
// #define eight 0xFF
// #define nine 0xF7

// #define GPIO_ADC_FUNCTION GPIO_PRIMARY_MODULE_FUNCTION

// #define VCC 3.3 // Measured voltage of Ardunio 5V line
// #define R_DIV 1000 // Measured resistance of 3.3k resistor

// /****************************/
// /*      STRUCT              */
// /****************************/

// /* This struct is used for collecting  *
//  * GPIO pins/ports into the same place */
// typedef struct {
//     uint32_t port;
//     uint32_t pin;
//     uint8_t  adc_input;
// } IO_Device;

// /***************************/
// /*   GPIO INIT PROTOTYPES  */
// /***************************/
// void gpio_init_sensor(IO_Device device);
// void gpio_init_io(IO_Device device);
// void gpio_init(void);

// /****************************/
// /*      PINS                */
// /****************************/

// /* Sensors */
// IO_Device moisture_1    = { GPIO_PORT_P1, GPIO_PIN6, ADC_INPUT_A6 };
// IO_Device moisture_2    = { GPIO_PORT_P1, GPIO_PIN4, ADC_INPUT_A4 };
// IO_Device temperature_1 = { GPIO_PORT_P1, GPIO_PIN3, ADC_INPUT_A3 };
// IO_Device temperature_2 = { GPIO_PORT_P1, GPIO_PIN5, ADC_INPUT_A5 };
// IO_Device photo_sensor  = { GPIO_PORT_P8, GPIO_PIN1, ADC_INPUT_A9 };

// /* MOTORS */
// IO_Device ventilation_motor_1  = { GPIO_PORT_P5, GPIO_PIN3, 0 };
// IO_Device ventilation_motor_2  = { GPIO_PORT_P5, GPIO_PIN0, 0 };

// IO_Device irrigation_motor_A_1 = { GPIO_PORT_P2, GPIO_PIN7, 0 };
// IO_Device irrigation_motor_A_2 = { GPIO_PORT_P8, GPIO_PIN0, 0 };

// IO_Device irrigation_motor_B_1 = { GPIO_PORT_P5, GPIO_PIN1, 0 };
// IO_Device irrigation_motor_B_2 = { GPIO_PORT_P2, GPIO_PIN5, 0 };

// IO_Device pwm                  = { GPIO_PORT_P1, GPIO_PIN7, 0 };

// /*******************************/
// /*   GPIO INIT IMPLEMENTATION  */
// /*******************************/

// /* To be used to initialize adc *
//  * IO Devices                   */
// void gpio_init_sensor(IO_Device device) {
//     // Pins run parallel on adc
//     GPIO_setAsPeripheralModuleFunctionInputPin(
//         device.port,
//         device.pin,
//         GPIO_ADC_FUNCTION);
    
//     gpio_init_io(device);
// }

// /* To be used to initalize non-adc *
//  * IO Devices                      */
// void gpio_init_io(IO_Device device) {
//     GPIO_setAsOutputPin(
//         device.port,
//         device.pin
//     );

//     GPIO_setOutputLowOnPin(device.port, device.pin);
// }

// void gpio_init(void){
//     gpio_init_sensor(moisture_1);
//     gpio_init_sensor(moisture_2);
//     gpio_init_sensor(temperature_1);
//     gpio_init_sensor(temperature_2);
//     gpio_init_sensor(photo_sensor);

//     gpio_init_io(ventilation_motor_1);
//     gpio_init_io(ventilation_motor_2);
//     gpio_init_io(irrigation_motor_A_1);
//     gpio_init_io(irrigation_motor_B_1);
//     gpio_init_io(irrigation_motor_A_2);
//     gpio_init_io(irrigation_motor_B_2);
//     gpio_init_io(pwm);
    
//     GPIO_setOutputHighOnPin(pwm.port, pwm.pin);
// }

// /****************************/
// /*     ADC TEMP GLOBAL      */
// /****************************/
// volatile float adc_result = 0;

// /****************************/
// /*     ADC PROTOTYPES       */
// /****************************/
// void initialize_adc(void);
// float read_adc(IO_Device device);

// /****************************/
// /*     ADC INTERRUPT        */
// /****************************/
// //ADC10 interrupt service routine
// #if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
// #pragma vector=ADC_VECTOR
// __interrupt
// #elif defined(__GNUC__)
// __attribute__((interrupt(ADC_VECTOR)))
// #endif
// void ADC_ISR (void)
// {
//     switch (__even_in_range(ADCIV,12)){
//         case 12:        //ADC10IFG0
//             adc_result = ADC_getResults(ADC_BASE);
//             __bic_SR_register_on_exit(CPUOFF);
//             break;
//         default: break;
//     }
// }

// /****************************/
// /*     ADC IMPLEMENTATION   */
// /****************************/
// void initialize_adc(void){
//     ADC_init(ADC_BASE, ADC_SAMPLEHOLDSOURCE_SC, ADC_CLOCKSOURCE_ADCOSC, ADC_CLOCKDIVIDER_1);
//     ADC_enable(ADC_BASE);
//     ADC_setupSamplingTimer(ADC_BASE, ADC_CYCLEHOLD_16_CYCLES, ADC_MULTIPLESAMPLESDISABLE);

//     ADC_clearInterrupt(ADC_BASE, ADC_COMPLETED_INTERRUPT);

//     //Enable Memory Buffer interrupt
//     ADC_enableInterrupt(ADC_BASE, ADC_COMPLETED_INTERRUPT);
// }

// float read_adc(IO_Device device) {
//     initialize_adc();
//     ADC_configureMemory(ADC_BASE, device.adc_input, ADC_VREFPOS_AVCC, ADC_VREFNEG_AVSS);
//     //Delay between conversions
//     __delay_cycles(5000);

//     //Enable and Start the conversion
//     //in Single-Channel, Single Conversion Mode
//     ADC_startConversion(ADC_BASE, ADC_SINGLECHANNEL);

//     //LPM0, ADC10_ISR will force exit
//     __bis_SR_register(CPUOFF + GIE);

//     return adc_result;
// }

// /***************************/
// /*      LCD FUNCTIONS      */
// /***************************/

// int charLUT[10] = {0xFC, 0x60, 0xDB, 0xF3, 0x67, 0xB7, 0xBF, 0xE4, 0xFF, 0xF7};
// void clear_memory(void)
// {
//     LCD_E_clearAllMemory(LCD_E_BASE);
// }

// void configureCOMSEG (void)
// {
//     // L0, L1, L2, L3: COM pins
//     // L0 = COM0, L1 = COM1, L2 = COM2, L3 = COM3
//     LCD_E_setPinAsCOM(LCD_E_BASE, LCD_E_SEGMENT_LINE_0, LCD_E_MEMORY_COM0);
//     LCD_E_setPinAsCOM(LCD_E_BASE, LCD_E_SEGMENT_LINE_1, LCD_E_MEMORY_COM1);
//     LCD_E_setPinAsCOM(LCD_E_BASE, LCD_E_SEGMENT_LINE_2, LCD_E_MEMORY_COM2);
//     LCD_E_setPinAsCOM(LCD_E_BASE, LCD_E_SEGMENT_LINE_3, LCD_E_MEMORY_COM3);

// }

// void hold(int n){
//     //Hold value
//     int delay;
//     for (delay = 0; delay <n; delay++)
//         __delay_cycles(1000); 
// }

// void lcd_reset(void){
//     //Turn on LCD
//     LCD_E_on(LCD_E_BASE);
//     hold(2000);
//     //Turn off LCD
//     LCD_E_off (LCD_E_BASE);
    
//     // Clear LCD memory
//     clear_memory();
//     // Configure COMs and SEGs
//     configureCOMSEG();
// }

// void lcd_config_display_mois(uint8_t mask1, uint8_t mask2, uint8_t mask3, uint8_t mask4, uint8_t mask5, uint8_t mask6,
//                                 uint8_t mask7, uint8_t mask8, uint8_t mask9, uint8_t mask10){
//     // Moisture
//     LCD_E_setMemory(LCD_E_BASE, LCD_E_MEMORY_BLINKINGMEMORY_4, mask1);
//     LCD_E_setMemory(LCD_E_BASE, LCD_E_MEMORY_BLINKINGMEMORY_5, mask2);
//     LCD_E_setMemory(LCD_E_BASE, LCD_E_MEMORY_BLINKINGMEMORY_6, mask3);
//     LCD_E_setMemory(LCD_E_BASE, LCD_E_MEMORY_BLINKINGMEMORY_7, mask4);
    
//     LCD_E_setMemory(LCD_E_BASE, LCD_E_MEMORY_BLINKINGMEMORY_8, mask5);
//     LCD_E_setMemory(LCD_E_BASE, LCD_E_MEMORY_BLINKINGMEMORY_10, mask6);
//     LCD_E_setMemory(LCD_E_BASE, LCD_E_MEMORY_BLINKINGMEMORY_11, mask7);
//     LCD_E_setMemory(LCD_E_BASE, LCD_E_MEMORY_BLINKINGMEMORY_2, mask8);
    
//     LCD_E_setMemory(LCD_E_BASE, LCD_E_MEMORY_BLINKINGMEMORY_18, mask9);
//     LCD_E_setMemory(LCD_E_BASE, LCD_E_MEMORY_BLINKINGMEMORY_19, mask10);
// }

// void lcd_config_display_temp(uint8_t mask1, uint8_t mask2, uint8_t mask3, uint8_t mask4, uint8_t mask5, uint8_t mask6,
//                                 uint8_t mask7, uint8_t mask8, uint8_t mask9, uint8_t mask10){
//     // Temperature
//     LCD_E_setMemory(LCD_E_BASE, LCD_E_MEMORY_BLINKINGMEMORY_4, mask1);
//     LCD_E_setMemory(LCD_E_BASE, LCD_E_MEMORY_BLINKINGMEMORY_5, mask2);
//     LCD_E_setMemory(LCD_E_BASE, LCD_E_MEMORY_BLINKINGMEMORY_6, mask3);
//     LCD_E_setMemory(LCD_E_BASE, LCD_E_MEMORY_BLINKINGMEMORY_7, mask4);
    
//     LCD_E_setMemory(LCD_E_BASE, LCD_E_MEMORY_BLINKINGMEMORY_8, mask5);
//     LCD_E_setMemory(LCD_E_BASE, LCD_E_MEMORY_BLINKINGMEMORY_10, mask6);
//     LCD_E_setMemory(LCD_E_BASE, LCD_E_MEMORY_BLINKINGMEMORY_11, mask7);
//     LCD_E_setMemory(LCD_E_BASE, LCD_E_MEMORY_BLINKINGMEMORY_2, mask8);
    
//     LCD_E_setMemory(LCD_E_BASE, LCD_E_MEMORY_BLINKINGMEMORY_3, mask9);
//     LCD_E_setMemory(LCD_E_BASE, LCD_E_MEMORY_BLINKINGMEMORY_18, mask10);
// }

// int parse_int_to_hex(int n){
//     return charLUT[n];
// }

// void print_lcd(float temp, float moisture, int zone) {
//     int zone_num = zone ? two : one;
//     lcd_reset();
//     {
//         int tens    = (int)temp / 100;
//         int ones    = ((int)temp / 10) % 10;
//         int decimal = (int)temp % 10;
//         lcd_config_display_temp(0x80, 0x50, zone_num, 0x04, parse_int_to_hex(tens), parse_int_to_hex(ones),
//                             0x01, parse_int_to_hex(decimal), 0x04, 0x9C);
//     }

//     lcd_reset();    
//     __delay_cycles(1000);
    
//     {
//         int tens    = (int)moisture/ 100;
//         int ones    = ((int)moisture / 10) % 10;
//         int decimal = (int)moisture% 10;
//         lcd_config_display_mois(0x6C, 0xA0, zone_num, 0x04, parse_int_to_hex(tens), parse_int_to_hex(ones),
//                     0x01, parse_int_to_hex(decimal), 0x27, 0xAA);
//     }
//     __delay_cycles(1000);
// }

// /******************************/
// /*   UART PROTOCOL ENABLE     */
// /******************************/

// EUSCI_A_UART_initParam param = {0};

// volatile char rcv_buf[1000];
// volatile int rcv_idx = 0;

// void init_uart() {
//     memset(rcv_buf, 0, 1000);
//     /* UART: It configures P1.0 and P1.1 to be connected internally to the
//      * eSCSI module, which is a serial communications module, and places it
//      * in UART mode. This let's you communicate with the PC via a software
//      * COM port over the USB cable. You can use a console program, like PuTTY,
//      * to type to your LaunchPad. The code in this sample just echos back
//      * whatever character was received.
//      */

//     //Configure UART pins, which maps them to a COM port over the USB cable
//     //Set P1.0 and P1.1 as Secondary Module Function Input.
//     GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1, GPIO_PIN1, GPIO_PRIMARY_MODULE_FUNCTION);
//     GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P1, GPIO_PIN0, GPIO_PRIMARY_MODULE_FUNCTION);

//     /*
//      * UART Configuration Parameter. These are the configuration parameters to
//      * make the eUSCI A UART module to operate with a 9600 baud rate. These
//      * values were calculated using the online calculator that TI provides at:
//      * http://software-dl.ti.com/msp430/msp430_public_sw/mcu/msp430/MSP430BaudRateConverter/index.html
//      */

//     //SMCLK = 1MHz, Baudrate = 9600
//     //UCBRx = 6, UCBRFx = 8, UCBRSx = 17, UCOS16 = 1
//     EUSCI_A_UART_initParam param = {0};
//         param.selectClockSource = EUSCI_A_UART_CLOCKSOURCE_SMCLK;
//         param.clockPrescalar    = 6;
//         param.firstModReg       = 8;
//         param.secondModReg      = 17;
//         param.parity            = EUSCI_A_UART_NO_PARITY;
//         param.msborLsbFirst     = EUSCI_A_UART_LSB_FIRST;
//         param.numberofStopBits  = EUSCI_A_UART_ONE_STOP_BIT;
//         param.uartMode          = EUSCI_A_UART_MODE;
//         param.overSampling      = 1;

//     if(STATUS_FAIL == EUSCI_A_UART_init(EUSCI_A0_BASE, &param))
//     {
//         return;
//     }

//     EUSCI_A_UART_enable(EUSCI_A0_BASE);

//     EUSCI_A_UART_clearInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);

//     // Enable EUSCI_A0 RX interrupt
//     EUSCI_A_UART_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);   
    
// }

// int rcv_validated(int inputs) {
//     int counter = 0;
//     int i;
//     for (i = 0; i < rcv_idx; i++) {
//         if (rcv_buf[i] == ':') {
//             counter++;
//         }
//     }
    
//     return counter >= inputs;
// }


// void displayConsole(uint8_t *message, uint8_t length) {
//     EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 13);
//     EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 10);
//     uint8_t i = 0;
//     while(i < length) {
//         EUSCI_A_UART_transmitData(EUSCI_A0_BASE, message[i]);
//         i++;
//     }

//     EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 13);
//     EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 10);
// }

// /* EUSCI A0 UART ISR - Echoes data back to PC host */
// #pragma vector=USCI_A0_VECTOR
// __interrupt
// void EUSCIA0_ISR(void)
// {
//     uint8_t RxStatus = EUSCI_A_UART_getInterruptStatus(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG);
//     uint8_t TxStatus = EUSCI_A_UART_getInterruptStatus(EUSCI_A0_BASE, EUSCI_A_UART_TRANSMIT_INTERRUPT_FLAG);

//     EUSCI_A_UART_clearInterrupt(EUSCI_A0_BASE, RxStatus);

//     EUSCI_A_UART_clearInterrupt(EUSCI_A0_BASE, TxStatus);

//     if(RxStatus) {
//         uint8_t input = EUSCI_A_UART_receiveData(EUSCI_A0_BASE);
//         if(input == '\r') {
//             uint8_t msg[256];
//             sprintf(msg, "ok");
//             displayConsole(msg, strlen(msg));
//         } else {
//             EUSCI_A_UART_transmitData(EUSCI_A0_BASE, input);
//         }
//     }


// }

// /******************************/
// /*       MAIN INIT            */
// /******************************/
// void init_MSP430() {
//   // Hold watchdog
//     WDT_A_hold(WDT_A_BASE);

//     //Port select XT1
//     GPIO_setAsPeripheralModuleFunctionInputPin(
//         GPIO_PORT_P4,
//         GPIO_PIN1 + GPIO_PIN2,
//         GPIO_PRIMARY_MODULE_FUNCTION
//         );

//     //Set external frequency for XT1
//     CS_setExternalClockSource(32768);

//     //Select XT1 as the clock source for ACLK with no frequency divider
//     CS_initClockSignal(CS_ACLK, CS_XT1CLK_SELECT, CS_CLOCK_DIVIDER_1);

//     //Start XT1 with no time out
//     // CS_turnOnXT1(CS_XT1_DRIVE_0);
    
//         // Initializes the XT1 crystal oscillator
//     CS_turnOnXT1LF(CS_XT1_DRIVE_1);
//     // Set SMCLK = DCO with frequency divider of 1
//     CS_initClockSignal(CS_SMCLK, CS_DCOCLKDIV_SELECT, CS_CLOCK_DIVIDER_1);
//     // Set MCLK = DCO with frequency divider of 1
//     CS_initClockSignal(CS_MCLK, CS_DCOCLKDIV_SELECT, CS_CLOCK_DIVIDER_1);

//     /*
//      * Disable the GPIO power-on default high-impedance mode to activate
//      * previously configured port settings
//      */
//     PMM_unlockLPM5();

//     // L0~L26 & L36~L39 pins selected
//     LCD_E_setPinAsLCDFunctionEx(LCD_E_BASE, LCD_E_SEGMENT_LINE_0, LCD_E_SEGMENT_LINE_26);
//     LCD_E_setPinAsLCDFunctionEx(LCD_E_BASE, LCD_E_SEGMENT_LINE_36, LCD_E_SEGMENT_LINE_39);

//     LCD_E_initParam initParams = LCD_E_INIT_PARAM;
//     initParams.clockDivider = LCD_E_CLOCKDIVIDER_8;
//     initParams.muxRate = LCD_E_4_MUX;
//     initParams.segments = LCD_E_SEGMENTS_ENABLED;

//     // Init LCD as 4-mux mode
//     LCD_E_init(LCD_E_BASE, &initParams);

//     // LCD Operation - Mode 3, internal 3.08v, charge pump 256Hz
//     LCD_E_setVLCDSource(LCD_E_BASE, LCD_E_INTERNAL_REFERENCE_VOLTAGE, LCD_E_EXTERNAL_SUPPLY_VOLTAGE);
//     LCD_E_setVLCDVoltage(LCD_E_BASE, LCD_E_REFERENCE_VOLTAGE_3_08V);

//     LCD_E_enableChargePump(LCD_E_BASE);
//     LCD_E_setChargePumpFreq(LCD_E_BASE, LCD_E_CHARGEPUMP_FREQ_16);
    
//     // Actual logic here:
    
//     // Let's enable internal sensors for testing purposes without breadboard
//     PMM_enableInternalReference();
//     while (PMM_REFGEN_NOTREADY == PMM_getVariableReferenceVoltageStatus());
    
//     // Initialize our necessary pins and modules!
//     gpio_init();
    
//     P1REN = BIT2;
//     P1OUT |= BIT2;

//     init_uart();
// }

// /******************************/
// /*   MAIN LOOP PROTOTYPES     */
// /******************************/

// float LIGHT_THRESHOLD = 3.0;
// float MOISTURE_THRESHOLD = 500; // Scale is 0 to 760
// float TEMPERATURE_THRESHOLD = 23.6;

// /* Index 0: Moisture 1    *
//  * Index 1: Moisture 2    *
//  * Index 2: Temperature 1 *
//  * Index 3: Temperature 2 *
//  * Index 4: Light Sensor  */
// float sensors_data[5];

// int irrigation_1_enabled = 0;
// int irrigation_2_enabled = 0;

// int ventilation_1_enabled = 0;
// int ventilation_2_enabled = 0;

// void load_sensor_data();

// int ventilation_enabled(float temperature, float light_data);
// int irrigation_enabled(float moisture, float light_data);

// // Value of 0 means Zone 1
// // Value of 1 means Zone 2
// int print_zone = 0;

// int ventilation_output(IO_Device motor,
//                       int enabled, 
//                       float temperature, 
//                       float light_data);

// int irrigation_output(IO_Device motor_A, 
//                       IO_Device motor_B,
//                       int enabled,
//                       float moisture, 
//                       float light_data);

// /******************************/
// /*   MAIN LOOP IMPLEMENTATION */
// /******************************/

// int get_value(char* rcv_buf, int start, int end) {
//     char new_str[end - start];
//     strcpy(new_str, rcv_buf + start);
//     return (int)atoi(new_str);
// }

// void load_values(char* rcv_buf) {
//     int start = 0;
//     int end = start;
//     while (rcv_buf[end] != ':') {
//         end++;
//     }
    
//     MOISTURE_THRESHOLD = (float)get_value(rcv_buf, start, end);
//     end++;
    
//     start = end;
//     while (rcv_buf[end] != ':') {
//         end++;
//     }
    
//     TEMPERATURE_THRESHOLD = (float)get_value(rcv_buf, start, end);
    
//     memset(rcv_buf, 0, 1000);
// }

// void load_motors(char* rcv_buf, int** res) {
//     memcpy(*res, rcv_buf, 4);
// }

// int ventilation_enabled(float temperature, float light_data) {
//     return temperature > TEMPERATURE_THRESHOLD * 10 && 
//           light_data >= LIGHT_THRESHOLD;
// }

// int irrigation_enabled(float moisture, float light_data) {
//     return moisture < MOISTURE_THRESHOLD &&
//           light_data < LIGHT_THRESHOLD;
// }

// void load_sensor_data() {
//     sensors_data[0] = read_adc(temperature_1);
//     sensors_data[1] = read_adc(temperature_2);
//     sensors_data[2] = read_adc(moisture_1);
//     sensors_data[3] = read_adc(moisture_2);
    
//     float lightV = read_adc(photo_sensor) * VCC / 1023.0;
//     // sensors_data[4] = R_DIV * (VCC / lightV - 1.0);
//     sensors_data[4] = lightV;
// }

// int ventilation_output(IO_Device motor,
//                       int enabled, 
//                       float temperature, 
//                       float light_data) {

//     int ventilation_on = ventilation_enabled(temperature, light_data);
//     if (ventilation_on/* && enabled*/) {
//         GPIO_setOutputHighOnPin(motor.port, motor.pin);
//     }
//     else {
//         GPIO_setOutputLowOnPin(motor.port, motor.pin);
//     }
    
//     return enabled;
// }

// int irrigation_output(IO_Device motor_A, 
//                       IO_Device motor_B,
//                       int enabled,
//                       float moisture, 
//                       float light_data) {
//     int irrigation_on = irrigation_enabled(moisture, light_data);
    
//     if (irrigation_on /*&& enabled*/) {
//         GPIO_setOutputHighOnPin(motor_B.port, motor_B.pin);
//         GPIO_setOutputLowOnPin(motor_A.port, motor_A.pin);
//     }

//     if (!irrigation_on) {
//         GPIO_setOutputLowOnPin(motor_A.port, motor_A.pin);
//         GPIO_setOutputLowOnPin(motor_B.port, motor_B.pin);
//     }

//     return enabled;
// }

// void main(void){

//     init_MSP430();

//     while(rcv_idx == 0);    
//     GPIO_setOutputHighOnPin(irrigation_motor_B_2.port, irrigation_motor_B_2.pin);    
//     GPIO_setOutputHighOnPin(irrigation_motor_A_2.port, irrigation_motor_A_2.pin);
//     load_values(rcv_buf);
    
//     int* motor_enables = (int*)malloc(4);
//     memset(motor_enables, 1, 4);
    
//     while(1){
//         if (rcv_validated(1)) {
//             load_motors(rcv_buf, &motor_enables);
//         }
    
//         load_sensor_data();
        
//         ventilation_1_enabled = ventilation_output(ventilation_motor_1, motor_enables[0], sensors_data[0], sensors_data[4]);
//         ventilation_2_enabled = ventilation_output(ventilation_motor_2, motor_enables[1], sensors_data[1], sensors_data[4]);

//         irrigation_1_enabled = irrigation_output(irrigation_motor_A_1,
//                                                  irrigation_motor_A_2, 
//                                                  motor_enables[2], 
//                                                  sensors_data[2], 
//                                                  sensors_data[4]);

//         irrigation_2_enabled = irrigation_output(irrigation_motor_B_1,
//                                                  irrigation_motor_B_2, 
//                                                  motor_enables[3], 
//                                                  sensors_data[3], 
//                                                  sensors_data[4]);

        
//         int toggled = (P1IN & BIT2) != BIT2;
 
//         float temp     = toggled ? sensors_data[1] : sensors_data[0];       
//         float moisture = toggled ? sensors_data[3] : sensors_data[2];
        
//         print_lcd(temp, moisture, toggled);
//     }
    
//      // Enter LPM3.5
//     PMM_turnOffRegulator();
//     __bis_SR_register(LPM4_bits | GIE);
// }


