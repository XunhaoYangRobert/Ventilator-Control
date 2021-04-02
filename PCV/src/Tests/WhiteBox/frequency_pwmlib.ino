// Setting up the arduino IDE 
// first open up this file: C:\Users\jerome\AppData\Local\Arduino15\packages\arduino\hardware\samd\1.8.6

// Then add -std=gnu++11 to compiler.cpp.flags=

// example of how to do custom PWM frequency

// The code below uses the PWM_lib:

#include "pwm_lib.h"
#include "tc_lib.h"

using namespace arduino_due::pwm_lib;

#define PWM_PERIOD_PIN_6 20000 // 5kHz, in hundredth of usecs (1e-8 secs)
#define PWM_DUTY_PIN_6 10000 // 50% duty cycle

// defining pwm object using pin 6, pin PC24 mapped to digital pin 6 on the DUE
// this object uses PWM channel 7
pwm<pwm_pin::PWML7_PC24> pwm_pin6;

void setup () {
Serial.begin(9600);
pwm_pin6.start(PWM_PERIOD_PIN_6, PWM_DUTY_PIN_6);

	Serial.println("================================================");
Serial.println("========= pwm_lib - basic_test on digital pin 6============");
Serial.println("================================================");

}

