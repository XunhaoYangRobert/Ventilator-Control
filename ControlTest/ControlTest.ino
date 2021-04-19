#include <Wire.h>
#include <pwm_lib.h>

using namespace arduino_due::pwm_lib;

// IMPORTANT: must connect each pin with corresponding valve!!!
#define INVALVE 6 // PWM, PIN 6
#define OUTVALVE 7 // PWM, PIN 7
#define AIRVALVE 8 // PWM, PIN 8
#define O2VALVE 9 // PWM, PIN 9

#define INHALATION 0
#define EXHALATION 1

#define PWM_PERIOD 100000 // 1kHz

// System variables
unsigned long referenceTime; // Reference system start time
unsigned long startOfBreath; // Keep track of the start time of each breath

// Inner circuit variables
float PEEP;
float targetPressure = 512; // In rawadc
float Tinhale; // Duration of a single inhalation
float Texhale; // Duration of a single exhalation
boolean mode; // Used to identify which states the ventilator is in
int inFlow; // Flow sensor in inhalation tube
int outFlow; // Flow sensor in exhalation tube
int inO2; // O2 sensor in inhalation tube
int outO2; // O2 sensor in exhalation tube
int pressure; // Pressure sensor in inner circuit

// Outer circuit variables
float FiO2 = 0.21; // Must be greater than room air oxygen concentration 0.21
int outerPressure; // Pressure sensor in outer circuit

// PID variables
int totalError;
int previousError;
float kp = 10;
float ki = 10;
float kd;

// PWM variables
pwm<pwm_pin::PWML7_PC24> pwmInValve; // PIN 6
pwm<pwm_pin::PWML6_PC23> pwmOutValve; // PIN 7

/**
 * Function to compute PSI for 150PAAB5.
 * @param rawadc raw analog signal
 * @return corresponding PSI
 */
float adc2PSI150PA(int rawadc) {
  // // Conversion equation from the data sheet
  // float PSI = (rawadc * 3.3 / 1023.0 - 0.33) * 150 / 2.64;

  // Result from linear regression
  float PSI = 0.1737 * rawadc - 32.31021;
  return PSI;

  // TODO: recalibrate pressure sensor after connecting everything
}

/**
 * Inversion function of adc2PSI150PA.
 * @param PSI
 * @return corresponding adc   
 */
int PSI2adc150PA(float PSI) {
  // Result from linear regression
  int adc = (int)((PSI + 32.31021) / 0.1737);
  return adc;

  // TODO: recalibrate pressure sensor after connnecting everything
}

/**
 * Given the target pressure value, this function compute the target flow to
 * to gain, which will be furtherly transformed into PWM signal by the effector.
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

  /*******DEBUG******/
  SerialUSB.print("current error: ");
  SerialUSB.println(error);
  SerialUSB.print("current target flow: ");
  SerialUSB.println(targetFlow);

  return targetFlow;
}

/**
 * This function regulates INVALVE and OUTVALVE based on the targetFlow and
 * mode.
 * @param targetFlow current targeting flow
 */
void Effector(int targetFlow) {
  if (targetFlow = 0) {
    pwmInValve.set_duty(0);
    pwmOutValve.set_duty(0);
  } else if (targetFlow > 0) {
    int PWMSignal = targetFlow;
    if (PWMSignal > 256) {
      PWMSignal = 256;
    }

    float percentOpen = PWMSignal * 1.0 / 256;
    pwmInValve.set_duty((int)(PWM_PERIOD * (0.6 + 0.3 * percentOpen)));
    pwmOutValve.set_duty(0);
  } else {
    int PWMSignal = -targetFlow;

    if (PWMSignal > 256) {
      PWMSignal = 256;
    }

    float percentOpen = PWMSignal * 1.0 / 256;
    pwmInValve.set_duty((int)(PWM_PERIOD * (0.6 + 0.3 * percentOpen)));
    pwmOutValve.set_duty(0);
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

  // TODO: set this trigger value
  int trigger = 1024; // Determine whether to naturally breath out or to involve PID
  if (pressure - PEEP >= trigger) {
    breathOutEffector();
  } else {
    int targetFlow = PIDController(targetPressure);
    Effector(targetFlow);
  }
}

/**
 * This function would open the OUTVALVE and open the INVALVE to a practical
 * minimum to enable a natural breathing out.
 */
void breathOutEffector() {
  // TODO: pass PWM signal to exhale valve for completely open it and limit
  // inhale valve to practical minimum
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

  // Set referenceTime as when the program starts
  referenceTime = millis();
  startOfBreath = 0;
}

/**
 * Main loop for inner circuit.
 */
void loop() {// 50 Hz
  // TODO:sensor value sampling
  pressure = analogRead(A1);
  SerialUSB.println("------------------------------");
  SerialUSB.print("current pressure: ");
  SerialUSB.println(pressure);

  // Calculate current time in this cycle
  unsigned long systemTime = millis() - referenceTime;
  unsigned long cycleTime = systemTime - startOfBreath;
//  if (cycleTime < Tinhale) {
//    inhalation();
//  } else if (cycleTime < Texhale) {
//    exhalation();
//  } else if (cycleTime > Texhale) {
//    startOfBreath = systemTime; // Reset startOfBreath to start next breath
//  }
  inhalation(); //DEBUG: testing PID during inhalation

  delay(1000);
}

///**
// * Main loop for outer circuit.
// */
//void loop2(1) { // 1kHz
//  int oxygenFlow = 0; // TODO: set a standard HIGH ENOUGH value as airFlow is calculated based on this
//  int airFlow;
//
//  airFlow = (int)((1 - FiO2) / (FiO2 - 0.21) * oxygenFlow);
//
//  // TODO: Send corresponding PWM signals to AIRVALVE and O2VALVE
//}
