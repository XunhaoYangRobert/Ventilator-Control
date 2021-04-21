#include <Wire.h>
#include <pwm_lib.h>

using namespace arduino_due::pwm_lib;

// IMPORTANT: must connect each pin with corresponding valve/sensor!!!
#define VALVE 6 // PWM, PIN 6
#define PRESSURE_SENSOR A1

#define PWM_PERIOD 100000 // 1kHz

int pressure; // Pressure sensor
int counter;

// PWM variables
pwm<pwm_pin::PWML7_PC24> pin6; // PIN 6

/**
 * Set up function for the system.
 */
void setup() {
  SerialUSB.begin(115200);
  SerialUSB.println("BOOT OK\n"); // Work

  // Set the valve PWM drivers
  pin6.start(PWM_PERIOD, 0);

  SerialUSB.println("pressure");
}

void loop() {
  if (counter == 0) {
    SerialUSB.println("System starting...");
  } else if (counter == 3) {
    SerialUSB.println("pressure");
  } else if (counter > 3) {
    pressure = analogRead(PRESSURE_SENSOR);
    SerialUSB.println(pressure);

    pin6.set_duty((int)(PWM_PERIOD * 0.55));
  }

  counter++;
  delay(500);
}
