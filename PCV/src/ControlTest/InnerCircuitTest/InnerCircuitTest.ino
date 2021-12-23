#include <Wire.h>
#include <pwm_lib.h>

using namespace arduino_due::pwm_lib;

// IMPORTANT: must connect each pin with corresponding valve/sensor!!!
#define INVALVE 6 // PWM, PIN 6
#define OUTVALVE 7 // PWM, PIN 7
#define AIRVALVE 8 // PWM, PIN 8
#define O2VALVE 9 // PWM, PIN 9
#define PRESSURE_SENSOR A1

// Flow sensors setup
#define AFLAADD 0x49
#define HF_CAL 8333
#define HF_OFFSET 1638
#define INFLOW_SENSOR 0
#define OUTFLOW_SENSOR 1

#define INHALATION 0
#define EXHALATION 1

#define PWM_PERIOD 100000 // 1kHz
#define MAXIMUM_FLOW 10000

// User settings
double targetPressure;
double PEEP;
int Tinhale; // Duration of a single inhalation in millisecond
int Texhale; // Duration of a single exhalation in millisecond

// System variables
unsigned long referenceTime; // Reference system start time
unsigned long startOfBreath; // Keep track of the start time of each breath

// Inner circuit variables
boolean mode; // Used to identify which states the ventilator is in
int pressure; // Pressure sensor in inner circuit
int inFlow; // Flow sensor in inhalation tube
int outFlow; // Flow sensor in exhalation tube

// PID variables
int error;
int totalError;
int previousError;
double kp = 50;
double ki;
double kd = 5;
int tolerance = 20; // Acceptable tolerance of target value

// PWM variables
pwm<pwm_pin::PWML7_PC24> pwmInValve; // PIN 6, inhalation valve
pwm<pwm_pin::PWML6_PC23> pwmOutValve; // PIN 7, exhalation valve
pwm<pwm_pin::PWML5_PC22> pwmAirValve; // PIN 8, air valve
pwm<pwm_pin::PWML4_PC21> pwmO2Valve; // PIN 9, O2 valve
// Datatype for air flow
union flowbytes{
  uint16_t raw;
  uint8_t bytes[2];
  struct{uint8_t byte0,byte1;};
};

/**
 * Transformation function for pressure sensor HSCDANN005PGAA5, from analog count to cmH2O
 * 
 * @param adc analog count
 * @return corresponding cmH2O
 */
double adc2cmH2O(int adc) {
  return 70.307 * (0.00152625 * adc - 0.625); // with PWM resolution of 4096
}

/**
 * Transformation function for pressure sensor HSCDANN005PGAA5, from cmH2O to analog count
 * 
 * @param cmH2O pressure
 * @return corresponding analog count
 */
int cmH2O2adc(double cmH2O) {
  return 655.2 * (cmH2O / 70.307) + 409.5; // with PWM resolution of 4096
}

/**
 * Flow sensor reader.
 * 
 * @param flowSensor INFLOW_SENSOR or OUTFLOW_SENSOR
 * @return digital count
 */
int32_t flowRead(int flowSensor) {
  flowbytes flow;
  int i = 2;

  if (flowSensor == INFLOW_SENSOR) {
    Wire.requestFrom(AFLAADD, 2);

    while(Wire.available()) { // slave may send less than requested 
      if(i) {
        i--;
        flow.bytes[i]= Wire.read();
      } else {
        char c = Wire.read(); // receive a byte as character
      }
    }
  } else if (flowSensor == OUTFLOW_SENSOR) {
    Wire1.requestFrom(AFLAADD, 2);

    while(Wire1.available()) { // slave may send less than requested
      if(i) {
        i--;
        flow.bytes[i]= Wire1.read();
      } else {
        char c = Wire1.read(); // receive a byte as character
      }
    }
  }

  return flow.raw;
}

/**
 * Transformation function for air flow sensor, from digital count to SLPM
 * 
 * @param raw digital count
 * @return corresponding SLPM
 */
double rawFlow2SLPM (int16_t raw) {
  double fullScaleRange = 100;
  return fullScaleRange * ((raw / 16384.0) - 0.1) / 0.8;
}

/**
 * Given the target pressure value, this function compute the target flow to
 * to gain, which will be furtherly transformed into PWM signal by the effector.
 * 
 * @param targetPressure current targeting pressure (in adc)
 * @return target flow
 */
int PIDController(int targetPressure) {
  int targetFlow;

  error = targetPressure - pressure;
  totalError += error;
  targetFlow = (int)(0 + kp * error + ki * totalError + kd * (error - previousError));
  previousError = error;
  if (abs(error) < tolerance) {
    targetFlow = 0;
  }
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

    double percentOpen = targetFlow * 1.0 / MAXIMUM_FLOW;
    pwmInValve.set_duty((int)(PWM_PERIOD * (0.6 + 0.3 * percentOpen)));
    pwmOutValve.set_duty(0);
  } else {
    targetFlow = -targetFlow;
    if (targetFlow > MAXIMUM_FLOW) {
      targetFlow = MAXIMUM_FLOW;
    }

    double percentOpen = targetFlow * 1.0 / MAXIMUM_FLOW;
    pwmOutValve.set_duty((int)(PWM_PERIOD * (0.6 + 0.3 * percentOpen)));
    pwmInValve.set_duty(0);
  }
}

/**
 * This function would open the INVALVE completely and close the OUTVALVE to allow
 * breathing in.
 */
void breathInEffector() {
  pwmInValve.set_duty(PWM_PERIOD);
  pwmOutValve.set_duty(0);
}

/**
 * This function would open the OUTVALVE and open the INVALVE to a practical
 * minimum to enable a natural breathing-out.
 */
void breathOutEffector() {
  pwmOutValve.set_duty(PWM_PERIOD);
  // TODO: limit inhale valve to practical minimum
  pwmInValve.set_duty(0);
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

  int trigger = 40; // Determine whether to completely open invalve or to involve PID
  if (targetPressure - pressure >= trigger) {
    breathInEffector();
  } else {
    int targetFlow = PIDController(targetPressure);
  Effector(targetFlow);
  }
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

  int trigger = 40; // Determine whether to naturally breath out or to involve PID
  if (pressure - PEEP >= trigger) {
    breathOutEffector();
  } else {
    int targetFlow = PIDController(PEEP);
    Effector(targetFlow);

    // TODO: add patient-triggered breath logic
  }
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
  SerialUSB.println("BOOT OK\n");

  // I2C setup for flow sensor
  Wire.begin();
  Wire1.begin();

  // Set the valve PWM drivers
  pwmInValve.start(PWM_PERIOD, 0);
  pwmOutValve.start(PWM_PERIOD, 0);

  // Set referenceTime as when the program starts
  referenceTime = millis();
  startOfBreath = 0;

  // Set the ADC resolution to 4096
  analogReadResolution(12); 

  // For testing purposes
  targetPressure = 800;
  PEEP = 500;
  Tinhale = 2000;
  Texhale = 2000;
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
void loop_control_inner(100) { // 10 Hz
  // TODO: sensors value sampling
  pwmAirValve.set_duty(PWM_PERIOD);
  pwmO2Valve.set_duty(0);
  pressure = analogRead(PRESSURE_SENSOR);
  inFlow = flowRead(INFLOW_SENSOR);
  outFlow = flowRead(OUTFLOW_SENSOR);

  // For test use
  SerialUSB.println(adc2cmH2O(pressure));
  
  // Calculate current time in this cycle
  unsigned long systemTime = millis() - referenceTime;
  unsigned long cycleTime = systemTime - startOfBreath;

  // Control System
  if (cycleTime < Tinhale) {
    inhalation();
  } else if (cycleTime < Tinhale + Texhale) {
    exhalation();
  } else if (cycleTime >= Tinhale + Texhale) {
    startOfBreath = systemTime; // Reset startOfBreath to start next breath
  }
}
