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

/*
 *  ======== gpiointerrupt.c ========
 */
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>

// TI's driver header files.
#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/UART.h>
#include <ti/drivers/Timer.h>

// TI's driver configuration header.
#include "ti_drivers_config.h"

#define DISPLAY(x) UART_write(uart, &output, x);
#define PLUS_BTN CONFIG_GPIO_BUTTON_0
#define MINUS_BTN CONFIG_GPIO_BUTTON_1
#define HEAT_LED CONFIG_GPIO_LED_0
#define AMBER_LED 10
#define GREEN_LED 11
#define NUM_TASKS 4

// Constant Values
// Largest common time period for tasks
const unsigned long g_task_time_step = 100;

// Function declarations for state machine tick functions (defined after main).
int TickFunc_ButtonCheck(int state);
int TickFunc_UpdateLED(int state);
int TickFunc_Report(int state);
int TickFunc_TempCheck(int state);

// General purpose task structure, to be used in an array.
typedef struct task
{
    int state;
    unsigned long period;
    unsigned long elapsedTime;
    int (*TickFunc)(int);
} task;

// States for each state machine.
enum Update_LED_States
{
    UL_Start,
    UL_LED_On,
    UL_LED_Off
} Update_LED_States;

enum Btn_Check_States
{
    BC_Start,
    BC_Wait,
    BC_Check_Btn
} Btn_Check_State;

enum Temp_Check_States
{
    TC_Start,
    TC_Wait,
    TC_Read_Temp
} Temp_Check_State;

enum Report_States
{
    R_Start,
    R_Wait,
    R_Send
} Report_State;

// Program Global Variables
// Flags for the set temperature +/- buttons.
unsigned char g_minus_press = 0;
unsigned char g_plus_press = 0;

// Environmental and set temperature values, in Celsius.
int16_t g_env_temp = 0;
int16_t g_set_temp = 21;

// Indicates whether heat is on.
char g_heat = 0x0;

// Amount of time since startup in hundreds of milliseconds.
unsigned short g_uptime = 0;

// Array of tasks for task scheduler, used in TimerISR().
task tasks[NUM_TASKS];

// TimerFlag is set to 1 when Timer_Callback is called (timer elapsed).
volatile unsigned char TimerFlag = 0;

// I2C Global Variables
static const struct
{
    uint8_t address;
    uint8_t resultReg;
    char *id;
}
sensors[3] =
{
    { 0x48, 0x0000, "11X" },
    { 0x49, 0x0000, "116" },
    { 0x41, 0x0001, "006" }
};

uint8_t txBuffer[1];
uint8_t rxBuffer[2];

I2C_Transaction i2cTransaction;

// UART Global Variables
char output[64];
int bytesToSend;

// Driver Handles
UART_Handle uart;
I2C_Handle i2c;
Timer_Handle timer0;

// Callback functions
/*
 *  ======== timerCallback ========
 *  Callback function for the Timer interrupt.
 *  Sets the flag indicating the timer interrupt has occurred.
 */
void timerCallback(Timer_Handle myHandle, int_fast16_t status) {
    TimerFlag = 1;
}

/*
 *  ======== plusButtonCallback ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_0.
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 *  Sets the flag for global variable plus_btn
 */
void plusButtonCallback(uint_least8_t index)
{
    g_plus_press = 1;
}

/*
 *  ======== minusButtonCallback ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_1.
 *  This may not be used for all boards.
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 *  *  Sets the flag for global variable minus_btn
 */
void minusButtonCallback(uint_least8_t index)
{
    g_minus_press = 1;
}

/*
 *  ======== TimerISR ========
 * Interrupt Service Routine, set off whenever
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 *  Sets the flag for global variable plus_btn
 */
void TimerISR()
{
    unsigned char i;
    for (i = 0; i < NUM_TASKS; ++i)
    {
        if (tasks[i].elapsedTime >= tasks[i].period)
        {
            tasks[i].state = tasks[i].TickFunc(tasks[i].state);
            tasks[i].elapsedTime = 0;
        }
        tasks[i].elapsedTime += g_task_time_step;
    }
}

// Make sure you call initUART() before calling this function.
void initI2C(void)
{
    int8_t i, found;
    I2C_Params i2cParams;
    DISPLAY(snprintf(output, 64, "Initializing I2C Driver - "))

    // Initialize the driver.
    I2C_init();

    // Configure the driver.
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;

    // Open the driver.
    i2c = I2C_open(CONFIG_I2C_0, &i2cParams);
    if (i2c == NULL)
    {
      DISPLAY(snprintf(output, 64, "Failed\n\r"))
      while (1);
    }
    DISPLAY(snprintf(output, 32, "Passed\n\r"))

    // Boards were shipped with different sensors.
    // Welcome to the world of embedded systems.
    // Try to determine which sensor we have.
    // Scan through the possible sensor addresses.
    /* Common I2C transaction setup */
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 0;
    found = false;

    for (i = 0; i < 3; ++i)
    {
      i2cTransaction.slaveAddress = sensors[i].address;
      txBuffer[0] = sensors[i].resultReg;

      DISPLAY(snprintf(output, 64, "Is this %s? ", sensors[i].id))

      if (I2C_transfer(i2c, &i2cTransaction))
      {
        DISPLAY(snprintf(output, 64, "Found\n\r"))
        found = true;
        break;
      }

      DISPLAY(snprintf(output, 64, "No\n\r"))
    }

    if (found)
    {
      DISPLAY(snprintf(output, 64, "Detected TMP%s I2C address: %x\n\r",
              sensors[i].id, i2cTransaction.slaveAddress))
    }
    else
    {
      DISPLAY(snprintf(output, 64,
              "Temperature sensor not found, contact professor\n\r"))
    }
}

// Read temperature of I2C sensor (TMP006).
int16_t readTemp(void)
{
    int16_t temperature = 0;
    i2cTransaction.readCount = 2;

    if (I2C_transfer(i2c, &i2cTransaction))
    {
      /*
      * Extract degrees C from the received data;
      * see TMP sensor datasheet.
      */
      temperature = (rxBuffer[0] << 8) | (rxBuffer[1]);
      temperature *= 0.0078125;

      /*
      * If the MSB is set '1', then we have a 2's complement
      * negative value which needs to be sign extended.
      */
      if (rxBuffer[0] & 0x80)
      {
        temperature |= 0xF000;
      }
    }
    else
    {
      DISPLAY(snprintf(output, 64,
        "Error reading temperature sensor (%d)\n\r", i2cTransaction.status))
      DISPLAY(snprintf(output, 64,
        "Power cycle your board by unplugging USB and plugging back in.\n\r"))
    }

    return temperature;
}

void initTimer(void)
{
    Timer_Params params;

    // Initialize the driver.
    Timer_init();

    // Configure the driver.
    Timer_Params_init(&params);
    params.period = 100000;
    params.periodUnits = Timer_PERIOD_US;
    params.timerMode = Timer_CONTINUOUS_CALLBACK;
    params.timerCallback = timerCallback;

    // Open the driver
    timer0 = Timer_open(CONFIG_TIMER_0, &params);

    if (timer0 == NULL)
    {
        /* Failed to initialize timer! */
        while (1) {}
    }

    if (Timer_start(timer0) == Timer_STATUS_ERROR)
    {
        /* Failed to start timer! */
        while (1) {}
    }
}

void initUART(void)
{
    UART_Params uartParams;
    // Initialize the driver.
    UART_init();

    // Configure the driver.
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.baudRate = 115200;

    // Open the driver.
    uart = UART_open(CONFIG_UART_0, &uartParams);

    if (uart == NULL)
    {
        /* UART_open() failed */
        while (1);
    }
}

/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    // Initializing hardware.
    initTimer();
    initUART();
    initI2C();
    GPIO_init();

    // Disabling unnecessary LEDs. Configure and turn off.
    //GPIO_setConfig(AMBER_LED, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    //GPIO_write(AMBER_LED, CONFIG_GPIO_LED_OFF);
    //GPIO_setConfig(GREEN_LED, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    //GPIO_write(GREEN_LED, CONFIG_GPIO_LED_OFF);

    /* Configuring Heat LED (CONFIG_GPIO_LED_0), temperature+ button
     * (CONFIG_GPIO_BTN_0), and temperature- button (CONFIG_GPIO_BTN_1),
     * then setting callbacks and enabling their interrupts.
     */
    // Heat LED setup.
    GPIO_setConfig(HEAT_LED, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);

    // Temperature+ button setup. Configure, set callback, enable.
    GPIO_setConfig(PLUS_BTN, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);
    GPIO_setCallback(PLUS_BTN, plusButtonCallback);
    GPIO_enableInt(PLUS_BTN);

    // Sanity checking buttons.
    if (PLUS_BTN != MINUS_BTN)
    {
        // Temperature- button setup. Configure, set callback, enable.
        GPIO_setConfig(MINUS_BTN, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);
        GPIO_setCallback(MINUS_BTN, minusButtonCallback);
        GPIO_enableInt(MINUS_BTN);
    }

    // Setting up starting tasks in global task array.
    // Task for the button check state machine.
    tasks[0].state = BC_Start;
    tasks[0].period = 200;
    tasks[0].elapsedTime = 0;
    tasks[0].TickFunc = &TickFunc_ButtonCheck;

    // Task for the temperature check state machine.
    tasks[1].state = TC_Start;
    tasks[1].period = 500;
    tasks[1].elapsedTime = 0;
    tasks[1].TickFunc = &TickFunc_TempCheck;

    // Task for the LED controller state machine.
    tasks[2].state = UL_Start;
    tasks[2].period = 1000;
    tasks[2].elapsedTime = 0;
    tasks[2].TickFunc = &TickFunc_UpdateLED;

    // Task for the reporting state machine.
    tasks[3].state = R_Start;
    tasks[3].period = 1000;
    tasks[3].elapsedTime = 0;
    tasks[3].TickFunc = &TickFunc_Report;

    /*
     * Set number of bytes in report message. Set here so it is only set once
     * and isn't in the state machine to cause mysterious behavior for writes
     * to the UART elsewhere.
     */
    bytesToSend = 16;
    // Loop forever, reset timer flag and call timer routine when flag gets set.
    while (1)
    {
        while (!TimerFlag) {}
        TimerFlag = 0;
        TimerISR();
    }
}

int TickFunc_ButtonCheck(int state)
{
    // SM Transitions
    switch (state)
    {
    case BC_Start:
        // No point in checking buttons on startup, straight to wait.
        state = BC_Wait;
        break;

    case BC_Wait:
        if (g_minus_press || g_plus_press)
        {
            state = BC_Check_Btn;
        }
        else
        {
            state = BC_Wait;
        }
        break;

    case BC_Check_Btn:
        state = BC_Wait;
        break;

    default:
        // restart if somehow the state is bad
        state = BC_Start;
    }

    // SM Actions
    switch (state)
    {
    case BC_Check_Btn:
        /*
         * Branch controls designed to catch the possibility a user pressed
         * both buttons, where no action is taken but to reset button flags.
         */
        if (g_minus_press)
        {
            g_minus_press = 0;
            g_set_temp -= 1;
        }
        else if (g_plus_press)
        {
            g_plus_press = 0;
            g_set_temp += 1;
        }
        else
        {
            g_minus_press = 0;
            g_plus_press = 0;
        }
        break;

    default:
        break;
    }
    return state;
}


int TickFunc_TempCheck(int state)
{
    // SM transitions
    switch (state)
    {
    case TC_Start:
        // Temperature ready on startup, go ahead and read it.
        state = TC_Read_Temp;
        break;

    case TC_Wait:
        state = TC_Read_Temp;
        break;

    case TC_Read_Temp:
        state = TC_Wait;
        break;

    default:
        state = TC_Start;
    }

    // SM actions.
    switch (state)
    {
    case TC_Read_Temp:
        g_env_temp = readTemp();
        if (g_env_temp >= g_set_temp)
        {
            g_heat = 0x0;
        }
        else
        {
            g_heat = 0x1;
        }
        break;

    default:
        break;
    }
    return state;
}

int TickFunc_UpdateLED(int state)
{
    // State transitions
    switch (state)
    {
    case UL_Start:
        state = UL_LED_Off;
        break;

    case UL_LED_On:
        if (!g_heat)
        {
            state = UL_LED_Off;
        }
        else
        {
            state = UL_LED_On;
        }
        break;

    case UL_LED_Off:
        if (g_heat)
        {
            state = UL_LED_On;
        }
        else
        {
            state = UL_LED_Off;
        }
        break;

    default:
        state = UL_Start;
    }

    //State actions
    switch (state)
    {
    case UL_LED_On:
        GPIO_write(HEAT_LED, CONFIG_GPIO_LED_ON);
        break;

    case UL_LED_Off:
        GPIO_write(HEAT_LED, CONFIG_GPIO_LED_OFF);
        break;

    default:
        break;
    }
    return state;
}


int TickFunc_Report(int state)
{
    //State transitions
    switch (state)
    {
    case R_Start:
        // Go ahead and send out an initial report.
        state = R_Send;
        break;

    case R_Wait:
        state = R_Send;
        break;

    case R_Send:
        state = R_Wait;
        break;

    default:
        state = R_Start;

    }

    // State actions
    switch (state)
    {
    case R_Send:
       g_uptime += 1;
       snprintf(output, 64, "<%02d,%02d,%d,%04d>\n\r",
                g_env_temp, g_set_temp, g_heat, g_uptime);
       UART_write(uart, &output, bytesToSend);

    default:
       break;
    }

    return state;
}
