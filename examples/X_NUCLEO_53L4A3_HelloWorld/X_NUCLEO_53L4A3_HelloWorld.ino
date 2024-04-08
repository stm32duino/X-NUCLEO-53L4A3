/**
 ******************************************************************************
 * @file    X_NUCLEO_53L4A3_HelloWorld.ino
 * @author  STMicroelectronics
 * @version V1.0.0
 * @date    01 April 2024
 * @brief   Arduino test application for the STMicrolectronics X-NUCLEO-53L4A3
 *          proximity sensor expansion board based on FlightSense.
 *          This application makes use of C++ classes obtained from the C
 *          components' drivers.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2024 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include <vl53l4ed_class.h>


#define DEV_I2C Wire
#define SerialPort Serial

#ifndef LED_BUILTIN
  #define LED_BUILTIN 13
#endif
#define LedPin LED_BUILTIN

/* Please uncomment the line below if you want also to use the satellites */
//#define SATELLITES_MOUNTED

#define XSHUT_TOP 3

#ifdef SATELLITES_MOUNTED
  #define XSHUT_LEFT 6
  #define XSHUT_RIGHT 4
#endif

// Components.
VL53L4ED sensor_vl53l4ed_top(&DEV_I2C, XSHUT_TOP);
#ifdef SATELLITES_MOUNTED
  VL53L4ED sensor_vl53l4ed_left(&DEV_I2C, XSHUT_LEFT);
  VL53L4ED sensor_vl53l4ed_right(&DEV_I2C, XSHUT_RIGHT);
#endif

/* Setup ---------------------------------------------------------------------*/

void setup()
{
  // Led.
  pinMode(LedPin, OUTPUT);

  // Initialize serial for output.
  SerialPort.begin(115200);
  SerialPort.println("Starting...");

  // Initialize I2C bus.
  DEV_I2C.begin();

  // Configure VL53L4ED top component.
  sensor_vl53l4ed_top.begin();

  // Switch off VL53L4ED top component.
  sensor_vl53l4ed_top.VL53L4ED_Off();

#ifdef SATELLITES_MOUNTED
  // Configure (if present) VL53L4ED left component.
  sensor_vl53l4ed_left.begin();

  //Switch off (if present) VL53L4ED left component.
  sensor_vl53l4ed_left.VL53L4ED_Off();

  // Configure (if present) VL53L4ED right component.
  sensor_vl53l4ed_right.begin();

  // Switch off (if present) VL53L4ED right component.
  sensor_vl53l4ed_right.VL53L4ED_Off();
#endif

  //Initialize all the sensors
  sensor_vl53l4ed_top.InitSensor(0x10);
#ifdef SATELLITES_MOUNTED
  sensor_vl53l4ed_left.InitSensor(0x12);
  sensor_vl53l4ed_right.InitSensor(0x14);
#endif

  // Start Measurements
  sensor_vl53l4ed_top.VL53L4ED_SetRangeTiming(200, 0);
  sensor_vl53l4ed_top.VL53L4ED_StartRanging();
#ifdef SATELLITES_MOUNTED
  sensor_vl53l4ed_left.VL53L4ED_SetRangeTiming(200, 0);
  sensor_vl53l4ed_left.VL53L4ED_StartRanging();
  sensor_vl53l4ed_right.VL53L4ED_SetRangeTiming(200, 0);
  sensor_vl53l4ed_right.VL53L4ED_StartRanging();
#endif
}

void loop()
{
  VL53L4ED_ResultsData_t results;
  uint8_t NewDataReady = 0;
  char report[128];
  uint8_t status;

  do {
    status = sensor_vl53l4ed_top.VL53L4ED_CheckForDataReady(&NewDataReady);
  } while (!NewDataReady);

  //Led on
  digitalWrite(LedPin, HIGH);

  if ((!status) && (NewDataReady != 0)) {
    // (Mandatory) Clear HW interrupt to restart measurements
    sensor_vl53l4ed_top.VL53L4ED_ClearInterrupt();

    // Read measured distance. RangeStatus = 0 means valid data
    sensor_vl53l4ed_top.VL53L4ED_GetResult(&results);
    snprintf(report, sizeof(report), "VL53L4ED Top: Status = %3u, Distance = %5u mm, Signal = %6u kcps/spad\r\n",
             results.range_status,
             results.distance_mm,
             results.signal_per_spad_kcps);
    SerialPort.print(report);
  }

  digitalWrite(LedPin, LOW);

#ifdef SATELLITES_MOUNTED
  NewDataReady = 0;

  do {
    status = sensor_vl53l4ed_left.VL53L4ED_CheckForDataReady(&NewDataReady);
  } while (!NewDataReady);

  //Led on
  digitalWrite(LedPin, HIGH);

  if ((!status) && (NewDataReady != 0)) {
    // (Mandatory) Clear HW interrupt to restart measurements
    sensor_vl53l4ed_left.VL53L4ED_ClearInterrupt();

    // Read measured distance. RangeStatus = 0 means valid data
    sensor_vl53l4ed_left.VL53L4ED_GetResult(&results);
    snprintf(report, sizeof(report), "VL53L4ED Left: Status = %3u, Distance = %5u mm, Signal = %6u kcps/spad\r\n",
             results.range_status,
             results.distance_mm,
             results.signal_per_spad_kcps);
    SerialPort.print(report);
  }

  digitalWrite(LedPin, LOW);

  NewDataReady = 0;

  do {
    status = sensor_vl53l4ed_right.VL53L4ED_CheckForDataReady(&NewDataReady);
  } while (!NewDataReady);

  //Led on
  digitalWrite(LedPin, HIGH);

  if ((!status) && (NewDataReady != 0)) {
    // (Mandatory) Clear HW interrupt to restart measurements
    sensor_vl53l4ed_right.VL53L4ED_ClearInterrupt();

    // Read measured distance. RangeStatus = 0 means valid data
    sensor_vl53l4ed_right.VL53L4ED_GetResult(&results);
    snprintf(report, sizeof(report), "VL53L4ED Right: Status = %3u, Distance = %5u mm, Signal = %6u kcps/spad\r\n",
             results.range_status,
             results.distance_mm,
             results.signal_per_spad_kcps);
    SerialPort.print(report);
  }

  digitalWrite(LedPin, LOW);
#endif
}
