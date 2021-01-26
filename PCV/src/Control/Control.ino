#define VALVES
#define MODE
#define ADCP

#include <Wire.h>

#ifdef VALVES
// These value mappings start 3=closest to bottom of PCB 0=closest to top of PCB
#define AIRVALVE 6 /* PWM, PIN 6 */
#define O2VALVE 7 /* PWM, PIN 7 */
#define INVALVE2 8 /* PWM, PIN 8 */
#define OUTVALVE3 9 /* PWM, PIN 9 */

byte V;
#endif // VALVES

#ifdef MODE
#define INHALATION 0
#define EXHALATION 1
#endif // MODE

// Sensor values
float inFlowSense;
float outFlowSense;
float inO2Sense;
float outO2Sense;
float pressureSense;

// PID values
float totalError = 0;
float previousError = 0;
float kp = 0;
float ki = 0;
float kd = 0;

// Outer circuit values
float FiO2 = 0.21; // Must be greater than room air oxygen concentration 0.21

// Reference system start time
unsigned long referenceTime;
// PEEP value set
unsigned int PEEP;
// targetPressure value for PID
float targetPressure;
// length of one time of inhalation
float Tinhale;
// length of one time of exhalation
float Texhale;
// used for call transition function before inhale and exhale
// 0 stand for inhale 
boolean mode = EXHALATION;

/*
 * Function to compute PSI for 150PAAB5.
 * @param rawadc raw signal
 * @return corresponding PSI
 */
float Pressure2PSI150PA(int rawadc) {
    float PSI = (rawadc-140.0);// 102.4);
    PSI = PSI*150/819.2;
    return PSI;
}

/**
 * Given the target pressure value, this function compute the target flow to
 * to gain, which will be furtherly transformed into PWM signal by the effector.
 *
 * @param targetPressure current targeting pressure
 * @return target flow
 */
float PIDController(float targetPressure) {
    float targetFlow;
    float error;

    // TODO: link sensor to this variable
    float detectedPressure = 0;

    error = targetPressure - sensorPressure;
    totalError += error;
    targetFlow = 0 + kp * error + ki * totalError + kd * (error - previousError);
    previousError = error;

    return targetFlow;
}

/*
 * This function regulates INVALVE and OUTVALVE based on the targetFlow and
 * mode.
 * @param targetFlow current targeting flow
 */
void Effector(float targetFlow) {
    if (targetFlow >= 0) {
        // TODO: change target flow into corresponding PWM signal
        int PWMSignal = (int)targetFlow;

        // TODO: Pass PWMSignal to inhale valve and close exhale valve
    } else {
        // TODO: change target flow into corresponding PWM signal
        int PWMSignal = (int)(-targetFlow);

        // TODO: Pass PWMSignal to exhale valve and close inhale valve
    }
    
}

void inhalation(){
  // call for transition function every firs time call inhalation in a period
  if (mode == EXHALATION) {
    transition();
    mode = INHALATION;
  }
  
  float targetFlow = PIDController(targetPressure);
  Effector(targetFlow);
}

void exhalation(){
  // call for transition function every firs time call exhalation in a period
  if (mode == INHALATION) {
    transition();
    mode = EXHALATION;
  }
  
  float trigger; //trigger using pressure difference between current and target to change state for exhale
  if (pressureSense - PEEP >= trigger) {
    breathOutEffector();
  }
  else {
    float targetFlow = PIDController(targetPressure);
    Effector(targetFlow);
  }
}

void breathOutEffector() {
  //TODO: pass PWM signal to exhale valve for completely open it and limit inhale valve to practical minimum
}

/*
 * This function is called between every inhalation and exhalation. It resets
 * global variables for the PID control, and between the interval when 
 * inhalation ends and exhalation starts, it calculates compliance and RSBI.
 */
void transition(){
    totalError = 0;
    previousError = 0;
    
    // TODO: Compliance tracking and RSBI
}

/*
 * Set up function for the system.
 */
void setup() {
    SerialUSB.begin(115200);

    #ifndef SERIAL1
    // Needed for not missing out on messages but hangs code if no serial
    // monitor is listening.
    while(!SerialUSB) {
        ;
    }
    #endif // SERIAL1
    SerialUSB.println("BOOT OK\n"); // Work

    #ifdef VALVES
    // Set the valve PWM drivers to output
    pinMode(VALVE0, OUTPUT);
    pinMode(VALVE2, OUTPUT);
    pinMode(VALVE2, OUTPUT);
    pinMode(VALVE3, OUTPUT);
    V = 10;
    #endif // VALVES
    // setting time reference for starting program
    referenceTime = millis();
}

/*
// Test program
void loop() {
    //============ READ ONBOARD ANALOGUE PRESSURE SENSOR INPUT ================
    #ifdef ADCP
    int pressureV = analogRead(A1);
    SerialUSB.print("U4 Pressure value: ");
    SerialUSB.println(pressureV);
    pressureV = analogRead(A2);
    SerialUSB.print("U5 Pressure value: ");
    SerialUSB.println(pressureV);
    pressureV = analogRead(A3);
    SerialUSB.print("U6 Pressure value (PSI): ");
    SerialUSB.println(pressureV);
    pressureV = analogRead(A4);
    SerialUSB.print("U7 Pressure value: ");
    SerialUSB.println(pressureV);
    pressureV = analogRead(A5);
    SerialUSB.print("U8 Pressure value: ");
    SerialUSB.println(pressureV);
    #endif // ADCP

    //================== TEST VALVE DRIVERS ==============================
    #ifdef VALVES
    // Sweep PWM ratios for each valve driver.
    SerialUSB.println("Solenoid Valve Tests");
    SerialUSB.print("Valve 0: "); SerialUSB.print(0);SerialUSB.println(" Open");
    // Test A, 50/50
    analogWrite(VALVE0,V); // easy to probe with scope.
    SerialUSB.print("Valve 1: ");SerialUSB.print(V);SerialUSB.println(" Open");
    analogWrite(VALVE1,V);    // Saftest for driving the system
    // Test C, 100/0
    SerialUSB.print("Valve 2: ");SerialUSB.print(V);SerialUSB.println(" Open");
    analogWrite(VALVE2,V);
    // Dummy
    SerialUSB.print("Valve 3: ");SerialUSB.print(V);SerialUSB.println(" Open");
    analogWrite(VALVE3,V);    // Test last if required
    V+=50;
    #endif // VALVES

    delay(500); // every 1/2 second...
}
*/

/*
 * Main loop for inner circuit.
 */
void loop1() {
  // TODO:sensor value sampling
  unsigned long currentTime = millis();
  currentTime = currentTime - referenceTime();
  // cycleTime: use for calculating inhale/exhale 
  unsigned long cycleTime = currentTime;
  //TODO: set this shit correct
  if (cycleTime < Tinhale) {
    inhalation();
  }
  else if (cycleTime < Texhale) {
    exhalation();
  }
  else if (cycleTime > Texhale) {
    //TODO: cycleTime reset
  }
  
}

/*
 * Main loop for outer circuit.
 */
void loop2() {
    float oxygenFlow = 0; // TODO set a standard HIGH ENOUGH value
    float airFlow;

    airFlow = (1 - FiO2) / (FiO2 - 0.21) * oxygenFlow;

    // TODO: send corresponding PWM signals to AIRVALVE and O2VALVE
}
