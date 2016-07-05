/* mbed Microcontroller Library
 * Copyright (c) 2006-2013 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#include <string.h>
#include "mbed.h"
#include "BLE.h"
#include "MPU6050.h"

#include "UARTService.h"

#define NEED_CONSOLE_OUTPUT 1 /* Set this if you need debug messages on the console;
                               * it will have an impact on code-size and power consumption. */

#if NEED_CONSOLE_OUTPUT
#define DEBUG(STR) { if (uart) uart->write(STR, strlen(STR)); }
#else
#define DEBUG(...) /* nothing */
#endif /* #if NEED_CONSOLE_OUTPUT */

BLEDevice  ble;
DigitalOut led1(LED1);
UARTService *uart;

MPU6050 mpu(I2C_SDA0, I2C_SCL0);
static const char DEVICENAME[] = "Ashimoto";
char *accelSend;

uint8_t accelPayload[sizeof(float)*10] = {0,};

void updateValue(void){
    float   acData[3];
    float   at = 0.0f;
    
    //Create a timer called acTimer
    //Read value from accelerometer
    //Read the value of the timer after a value was recieved
    //Reset the timer
    Timer acTimer;  
    acTimer.start();
    mpu.getAccelero(acData);
    acTimer.stop();
    at = acTimer.read_ms();
    acTimer.reset();
    
    //Copy the accelerometer data to an array called accelPayload
    memcpy(accelPayload+sizeof(float)*0, &acData[0], sizeof(acData[0]));
    memcpy(accelPayload+sizeof(float)*1, &acData[1], sizeof(acData[1]));
    memcpy(accelPayload+sizeof(float)*2, &acData[2], sizeof(acData[2]));
    memcpy(accelPayload+sizeof(float)*3, &at, sizeof(at));
    
    sprintf(accelSend, "%d %d %d %d\n", accelPayload[0], accelPayload[1], accelPayload[2], accelPayload[3]);
}
    

void disconnectionCallback(const Gap::DisconnectionCallbackParams_t *params)
{
    DEBUG("Disconnected!\n\r");
    DEBUG("Restarting the advertising process\n\r");
    ble.startAdvertising();
}

void periodicCallback(void)
{
    led1 = !led1;
    updateValue();
    DEBUG(accelSend);
    DEBUG(" it worked\n")
}

int main(void)
{
    led1 = 1;
    accelSend = (char *)malloc(100);
    
    //updateValue();
    Ticker ticker;
    ticker.attach(periodicCallback, 1);

    DEBUG("Initialising the nRF51822\n\r");
    ble.init();
    ble.onDisconnection(disconnectionCallback);
    
    uart = new UARTService(ble);

    /* setup device name */
    ble.setDeviceName((const uint8_t *)DEVICENAME);

    /* setup advertising */
    ble.accumulateAdvertisingPayload(GapAdvertisingData::BREDR_NOT_SUPPORTED);
    ble.setAdvertisingType(GapAdvertisingParams::ADV_CONNECTABLE_UNDIRECTED);
    ble.accumulateAdvertisingPayload(GapAdvertisingData::SHORTENED_LOCAL_NAME,
                                     (const uint8_t *)"BLE UART", sizeof("BLE UART") - 1);
    ble.accumulateAdvertisingPayload(GapAdvertisingData::COMPLETE_LIST_128BIT_SERVICE_IDS,
                                     (const uint8_t *)UARTServiceUUID_reversed, sizeof(UARTServiceUUID_reversed));

    ble.setAdvertisingInterval(160); /* 100ms; in multiples of 0.625ms. */
    ble.startAdvertising();

    while (true) {
        ble.waitForEvent();
    }
    
    
}
