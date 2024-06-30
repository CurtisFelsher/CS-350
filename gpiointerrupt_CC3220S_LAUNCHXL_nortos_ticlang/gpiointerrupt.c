/*
 * Copyright (c) 2015-2020, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdint.h>       // Standard integer types
#include <stddef.h>       // Standard definitions
#include <string.h>       // String operations
#include <stdio.h>        // Standard input/output
#include <stdbool.h>      // Boolean type
#include <ti/drivers/GPIO.h>    // GPIO driver
#include <ti/drivers/Timer.h>   // Timer driver
#include <ti/drivers/I2C.h>     // I2C driver
#include <ti/drivers/UART2.h>   // UART2 driver
#include "ti_drivers_config.h"  // Driver configuration

// Simplify UART output
#define DISPLAY(x) UART2_write(uart, output, x, NULL)
#define gPeriod 100 // Global period - milliseconds
#define taskNumber 3 // Tasks number

// Global variables
char output[64];              // UART output buffer
UART2_Handle uart;            // UART handle
volatile bool TimerFlag = false;   // Timer interrupt flag
volatile bool Button0Flag = false; // Button 0 interrupt flag
volatile bool Button1Flag = false; // Button 1 interrupt flag
Timer_Handle timer0;          // Timer handle
I2C_Handle i2c;               // I2C handle

int setpoint = 25; // Initial temp
int heat = 0;      // Heat status (0 = off, 1 = on)
int seconds = 0;   // Time in seconds since board was reset
int temp = 0;      // Current temp in Celsius

// Task structure to manage tasks
struct task {
    void (*func)();  // Pointer to function
    int timePassed;  // Time passed since last execution
    int period;      // Execution period in milliseconds
};

// Function to check button presses and adjust setpoint
void checkButtons() {
    if (Button0Flag) {       // If button 0 was pressed
        Button0Flag = false; // Clear flag
        setpoint++;          // Increase setpoint
    }
    if (Button1Flag) {       // If button 1 was pressed
        Button1Flag = false; // Clear flag
        setpoint--;          // Decrease setpoint
    }
}

// Function to read temperature from TMP006 sensor
int16_t readTemp(void) {
    uint8_t txBuffer[1];      // Buffer to hold address
    uint8_t rxBuffer[2];      // Buffer to hold received data
    I2C_Transaction i2cTransaction; // I2C transaction structure

    txBuffer[0] = 0x00;       // Point to the temp register
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 2;  // Read two bytes from the sensor
    i2cTransaction.targetAddress = 0x41; // TMP006 I2C address

    // Perform I2C transfer
    if (I2C_transfer(i2c, &i2cTransaction)) {
        // Combine the bytes into a 16-bit value
        int16_t rawTemp = (rxBuffer[0] << 8) | rxBuffer[1];

        // Shift value to get the 14-bit temperature data
        rawTemp >>= 2; // TMP006 provides temperature in 14-bit format

        // Proper sign extension for 14-bit data
        if (rawTemp & 0x2000) { // If the sign bit (bit 13) is set
            rawTemp |= 0xC000;  // Sign-extend to 16 bits
        }

        // Convert to Celsius
        float temperatureCelsius = rawTemp * 0.03125;

        // Convert to integer for display purposes
        return (int16_t)(temperatureCelsius + 0.5);   // Round to nearest integer
    } else {
        return -1;
    }
}

// Function to read the temp and control the heater
void readTemperature() {
    temp = readTemp();           // Read the current temp
    if (temp > setpoint) {       // If temperature is above setpoint
        GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF); // Turn off LED
        heat = 0;                // Turn off heater
    } else {                     // If temperature is below setpoint
        GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);  // Turn on LED
        heat = 1;                // Turn on heater
    }
}

// Function to update the LED status and UART output
void updateLEDandUART() {
    seconds++;                   // Increment seconds counter
    // Send data to UART
    DISPLAY(snprintf(output, 64, "<%02d,%02d,%d,%04d>\n\r", temp, setpoint, heat, seconds));
}

// Array of tasks to be performed periodically
struct task tasks[taskNumber] = {
    {checkButtons, 0, 200},      // Check buttons 200ms
    {readTemperature, 0, 500},   // Read temp 500ms
    {updateLEDandUART, 0, 1000}  // Update LED and UART 1000ms
};

// Timer callback function to set the TimerFlag
void timerCallback(Timer_Handle myHandle, int_fast16_t status) {
    TimerFlag = true;            // Set the TimerFlag
}

// Function to initialize the timer
void initTimer(void) {
    Timer_Params params;         // Timer parameters
    Timer_init();                // Initialize timer driver
    Timer_Params_init(&params);  // Initialize timer parameters
    params.period = gPeriod * 1000; // Set period in microseconds
    params.periodUnits = Timer_PERIOD_US; // Set period units
    params.timerMode = Timer_CONTINUOUS_CALLBACK; // Set timer mode
    params.timerCallback = timerCallback; // Set timer callback

    timer0 = Timer_open(CONFIG_TIMER_0, &params); // Open timer
    if (timer0 == NULL) {       // Check if timer opened successfully
        while (1) {}
    }
    if (Timer_start(timer0) == Timer_STATUS_ERROR) { // Start timer
        while (1) {}
    }
}

// Function to initialize UART
void initUART(void) {
    UART2_Params params;         // UART parameters
    UART2_Params_init(&params);  // Initialize UART parameters
    params.writeMode = UART2_Mode_BLOCKING; // Set write mode
    params.readMode = UART2_Mode_BLOCKING;  // Set read mode
    params.readReturnMode = UART2_ReadReturnMode_FULL; // Set read return mode
    params.baudRate = 115200;    // Set baud rate

    uart = UART2_open(CONFIG_UART2_0, &params); // Open UART
    if (uart == NULL) {
        while (1) {}
    }
}

// Function to initialize the I2C
void initI2C(void) {
    I2C_Params i2cParams;        // I2C parameters
    I2C_init();                  // Initialize I2C driver
    I2C_Params_init(&i2cParams); // Initialize I2C parameters
    i2cParams.bitRate = I2C_400kHz; // Set I2C bit rate

    i2c = I2C_open(CONFIG_I2C_0, &i2cParams); // Open I2C
    if (i2c == NULL) {
        while (1);
    }

    uint8_t txBuffer[1];         // Buffer to hold register address
    uint8_t rxBuffer[2];         // Buffer to hold received data
    I2C_Transaction i2cTransaction; // I2C transaction structure

    // List of possible sensors
    static const struct {
        uint8_t address;        // I2C address of the sensor
        uint8_t resultReg;      // Register to read from
        char *id;               // Sensor ID
    } sensors[3] = {
        {0x48, 0x0000, "11X"},
        {0x49, 0x0000, "116"},
        {0x41, 0x0001, "006"}
    };

    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 1;

    bool found = false;         // Flag to indicate if sensor is found
    for (int i = 0; i < 3; ++i) { // Loop through possible sensors
        i2cTransaction.targetAddress = sensors[i].address; // Set target address
        txBuffer[0] = sensors[i].resultReg; // Set register address

        if (I2C_transfer(i2c, &i2cTransaction)) { // Perform I2C transfer
            found = true;        // Sensor found
            break;               // Exit loop
        }
    }

    if (!found) {                // If no sensor found
        while (1);               // Enter infinite loop
    }
}

// GPIO callback function for button 0
void gpioButtonFxn0(uint_least8_t index) {
    Button0Flag = true; // Set flag on button press
}

// GPIO callback function for button 1
void gpioButtonFxn1(uint_least8_t index) {
    Button1Flag = true; // Set flag on button press
}

// Main thread function
void *mainThread(void *arg0) {
    initUART();        // Initialize UART
    initI2C();         // Initialize I2C
    initTimer();       // Initialize Timer

    GPIO_init();       // Initialize GPIO
    // Set configuration for LED
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    // Set configuration for button 0
    GPIO_setConfig(CONFIG_GPIO_BUTTON_0, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);
    // Set callback for button 0
    GPIO_setCallback(CONFIG_GPIO_BUTTON_0, gpioButtonFxn0);
    // Enable interrupt for button 0
    GPIO_enableInt(CONFIG_GPIO_BUTTON_0);

    // If button 1 is different from button 0
    if (CONFIG_GPIO_BUTTON_0 != CONFIG_GPIO_BUTTON_1) {
        // Set configuration for button 1
        GPIO_setConfig(CONFIG_GPIO_BUTTON_1, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);
        // Set callback for button 1
        GPIO_setCallback(CONFIG_GPIO_BUTTON_1, gpioButtonFxn1);
        // Enable interrupt for button 1
        GPIO_enableInt(CONFIG_GPIO_BUTTON_1);
    }

    // Main loop
    while (1) {
        if (TimerFlag) { // Check if timer interrupt occurred
            TimerFlag = false; // Clear the timer flag

            // Loop through tasks
            for (int i = 0; i < taskNumber; i++) {
                tasks[i].timePassed += gPeriod; // Update time passed
                // Check if it's time to execute the task
                while (tasks[i].timePassed >= tasks[i].period) {
                    tasks[i].timePassed -= tasks[i].period; // Update time passed
                    tasks[i].func(); // Execute the task
                }
            }
        }
    }
}
