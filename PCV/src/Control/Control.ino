#include <Wire.h>
#include <pwm_lib.h>

using namespace arduino_due::pwm_lib;

// IMPORTANT: must connect each pin with corresponding valve/sensor!!!
#define INVALVE 6 // PWM, PIN 6
#define OUTVALVE 7 // PWM, PIN 7
#define AIRVALVE 8 // PWM, PIN 8
#define O2VALVE 9 // PWM, PIN 9
#define PRESSURE_SENSOR A1

#define INHALATION 0
#define EXHALATION 1

#define PWM_PERIOD 100000 // 1kHz
#define MAXIMUM_FLOW 10000

// User settings
float targetPressure;
float PEEP;
float Tinhale; // Duration of a single inhalation in millisecond
float Texhale; // Duration of a single exhalation in millisecond
float FiO2; // Must be greater than room air oxygen concentration 0.21

// System variables
unsigned long referenceTime; // Reference system start time
unsigned long startOfBreath; // Keep track of the start time of each breath

// Inner circuit variables
boolean mode; // Used to identify which states the ventilator is in
int inFlow; // Flow sensor in inhalation tube
int outFlow; // Flow sensor in exhalation tube
int inO2; // O2 sensor in inhalation tube
int outO2; // O2 sensor in exhalation tube
int pressure; // Pressure sensor in inner circuit

// Outer circuit variables
int outerPressure; // Pressure sensor in outer circuit

// PID variables
int totalError;
int previousError;
float kp = 50;
float ki;
float kd = 5;

// PWM variables
pwm<pwm_pin::PWML7_PC24> pwmInValve; // PIN 6, inhalation valve
pwm<pwm_pin::PWML6_PC23> pwmOutValve; // PIN 7, exhalation valve
pwm<pwm_pin::PWML5_PC22> pwmAirValve; // PIN 8, air valve
pwm<pwm_pin::PWML4_PC21> pwmO2Valve; // PIN 9, O2 valve

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
 * inhalation period (of the main loop).
 */
void inhalation() {
  // Call transition if a transition from exhalation just happened
  if (mode == EXHALATION) {
    transition();
  }

  int targetFlow = PIDController(targetPressure);
  Effector(targetFlow);
}

/**
 * This function regulates the behavior of the ventilator during a single
 * exhalation period (of the main loop).
 */
void exhalation() {
  // Call transition if a trasition from inhalation just happened
  if (mode == INHALATION) {
    transition();
  }

  int trigger = 50; // Determine whether to naturally breath out or to involve PID
  if (pressure - PEEP >= trigger) {
    breathOutEffector();
  } else {
    int targetFlow = PIDController(PEEP);
    Effector(targetFlow);

    // TODO: add patient-triggered breath logic
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
 * This function is called between every inhalation and exhalation. It resets
 * global variables for the PID control, and between the interval when
 * inhalation ends and exhalation starts, it calculates compliance and RSBI.
 */
void transition() {
  totalError = 0;
  previousError = 0;
  mode = !mode; // Switch mode between inhalation and exhalation

  // TODO: Compliance tracking and RSBI
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
  pwmAirValve.start(PWM_PERIOD, 0);
  pwmO2Valve.start(PWM_PERIOD, 0);

  // Set referenceTime as when the program starts
  referenceTime = millis();
  startOfBreath = 0;

  // Set the ADC resolution to 4096
  analogReadResolution(12); 
}

/**
 * Background loop.
 */
void loop() {
  ;
}

/**
 * Main loop for inner circuit.
 */
void loop1(20) {// 50 Hz
  // TODO: sensor value sampling
  pressure = analogRead(PRESSURE_SENSOR);

  // Calculate current time in this cycle
  unsigned long systemTime = millis() - referenceTime;
  unsigned long cycleTime = systemTime - startOfBreath;
  if (cycleTime < Tinhale) {
    inhalation();
  } else if (cycleTime < Tinhale + Texhale) {
    exhalation();
  } else if (cycleTime >= Tinhale + Texhale) {
    startOfBreath = systemTime; // Reset startOfBreath to start next breath
  }
}

/**
 * Main loop for outer circuit.
 */
void loop2(20) { // 50 Hz
  // (Air Flow)/(Oxygen Flow) = (1 - FiO2)/(FiO2 - 0.21)
  float airValvePercentOpen;
  float O2ValvePercentOpen;
  if (FiO2 < 0.605) { // Air Flow greater than Oxygen Flow
    pwmAirValve.set_duty(PWM_PERIOD); // Completely open air valve
    O2ValvePercentOpen = (FiO2 - 0.21)/(1 - FiO2);
    pwmO2Valve.set_duty((int)(PWM_PERIOD * (0.6 + 0.3 * O2ValvePercentOpen)));
  } else { // Oxygen Flow is greater than or equal to Air Flow
    pwmO2Valve.set_duty(PWM_PERIOD); // Completely open O2 valve
    airValvePercentOpen = (1 - FiO2)/(FiO2 - 0.21);
    pwmAirValve.set_duty((int)(PWM_PERIOD * (0.6 + 0.3 * airValvePercentOpen)));
  } 
}
