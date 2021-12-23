#include <Wire.h>
#include <pwm_lib.h>

using namespace arduino_due::pwm_lib;

// IMPORTANT: must connect each pin with corresponding valve/sensor!!!
#define INVALVE 6 // PWM, PIN 6
#define OUTVALVE 7 // PWM, PIN 7
#define PRESSURE_SENSOR A1

#define PWM_PERIOD 100000 // 1kHz
#define MAXIMUM_FLOW 10000

// User settings
int PEEP = 450; // In rawadc

// Inner circuit variables
int pressure; // Pressure sensor in inner circuit

// PID variables
int totalError;
int previousError;
float kp = 50;
float ki;
float kd = 5;

// PWM variables
pwm<pwm_pin::PWML7_PC24> pwmInValve; // PIN 6, inhalation valve
pwm<pwm_pin::PWML6_PC23> pwmOutValve; // PIN 7, exhalation valve

// Test variable
boolean startExhale = false;

/**
 * Given the target pressure value, this function compute the target flow to
 * to gain, which will be furtherly transformed into PWM signal by the effector.
 * 
 * @param targetPressure current targeting pressure (in adc)
 * @return target flow
 */
int PIDController(int targetPressure) {
  int targetFlow;
  int error;

  error = targetPressure - pressure;
  totalError += error;
  targetFlow = (int)(0 + kp * error + ki * totalError + kd * (error - previousError));
  previousError = error;

  return targetFlow;
}

/**
 * This function regulates INVALVE and OUTVALVE based on the targetFlow.
 * 
 * @param targetFlow current targeting flow
 */
void Effector(int targetFlow) {
  if (targetFlow == 0) {
    pwmInValve.set_duty(0);
    pwmOutValve.set_duty(0);
  } else if (targetFlow > 0) {
    if (targetFlow > MAXIMUM_FLOW) {
      targetFlow = MAXIMUM_FLOW;
    }

    float percentOpen = targetFlow * 1.0 / MAXIMUM_FLOW;
    pwmInValve.set_duty((int)(PWM_PERIOD * (0.6 + 0.3 * percentOpen)));
    pwmOutValve.set_duty(0);
  } else {
    targetFlow = -targetFlow;
    if (targetFlow > MAXIMUM_FLOW) {
      targetFlow = MAXIMUM_FLOW;
    }

    float percentOpen = targetFlow * 1.0 / MAXIMUM_FLOW;
    pwmOutValve.set_duty((int)(PWM_PERIOD * (0.6 + 0.3 * percentOpen)));
    pwmInValve.set_duty(0);
  }
}

/**
 * This function regulates the behavior of the ventilator during a single
 * exhalation period (of the main loop).
 */
void exhalation() {
  int trigger = 50; // Determine whether to naturally breath out or to involve PID
  
  if (pressure - PEEP >= trigger) {
    breathOutEffector();
  } else {
    int targetFlow = PIDController(PEEP);
    Effector(targetFlow);
  }
}

/**
 * This function would open the OUTVALVE and open the INVALVE to a practical
 * minimum to enable a natural breathing out.
 */
void breathOutEffector() {
  pwmOutValve.set_duty(PWM_PERIOD);
  // TODO: limit inhale valve to practical minimum
  pwmInValve.set_duty(0);
}

/**
 * Set up function for the system.
 */
void setup() {
  SerialUSB.begin(115200);
  SerialUSB.println("BOOT OK\n"); // Work

  // Set the valve PWM drivers
  pwmInValve.start(PWM_PERIOD, 0);
  pwmOutValve.start(PWM_PERIOD, 0);

  // Set the ADC resolution to 4096
  analogReadResolution(12); 
}

/**
 * Main loop for inner circuit.
 */
void loop() {// 50 Hz
  pressure = analogRead(PRESSURE_SENSOR);
  SerialUSB.println(pressure);

  if (pressure > 700) {
    startExhale = true;
  }
  if (startExhale == true) {
    exhalation();
  } else {
    pwmInValve.set_duty(PWM_PERIOD);
    pwmOutValve.set_duty(0);
  }

  delay(20);
}
