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
#endif // VALVES

#ifdef MODE
#define INHALATION 0
#define EXHALATION 1
#endif // MODE

// Sensor values
int inFlowSense;
int outFlowSense;
int inO2Sense;
int outO2Sense;
int pressureSense;

// PID values
int totalError = 0;
int previousError = 0;
float kp = 0.0;
float ki = 0.0;
float kd = 0.0;

// Outer circuit values
float FiO2 = 0.21; // Must be greater than room air oxygen concentration 0.21

// System values
unsigned long referenceTime; // Reference system start time
unsigned long startOfBreath; // Keep track of the start time of the breath
float PEEP;
float targetPressure;
float Tinhale; // Duration of a single inhalation
float Texhale; // Duration of a single exhalation
boolean mode = EXHALATION; // Used to identify which states the ventilator is in

/*
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

/*
 * Inversion function of adc2PSI150PA.
 * @param PSI
 * @return corresponding adc
 */
int PSI2adc150PA(float PSI) {
    // Result from linear regression
    int adc = (int)((PSI + 32.31021) / 0.1737);
    return adc;

    // TODO: recalibrate pressure sensor after connnecting everything

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

    error = targetPressure - pressureSense;
    totalError += error;
    targetFlow = (int)(0 + kp * error + ki * totalError + kd * (error - previousError));
    previousError = error;

    return targetFlow;
}

/*
 * This function regulates INVALVE and OUTVALVE based on the targetFlow and
 * mode.
 * @param targetFlow current targeting flow
 */
void Effector(int targetFlow) {
    if (targetFlow >= 0) {
        // TODO: change target flow into corresponding PWM signal
        int PWMSignal = targetFlow;

        // TODO: Pass PWMSignal to inhale valve and close exhale valve
    } else {
        // TODO: change target flow into corresponding PWM signal
        int PWMSignal = (-targetFlow);

        // TODO: Pass PWMSignal to exhale valve and close inhale valve
    }

}

/*
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

/*
 * This function regulates the behavior of the ventilator during a single
 * exhalation period (of the main loop).
 */
void exhalation(){
    // Call transition if a trasition from inhalation just happened
    if (mode == INHALATION) {
        transition();
    }

    // TODO: set this trigger value
    int trigger = 1024; // Determine whether to naturally breath out or to involve PID
    if (pressureSense - PEEP >= trigger) {
        breathOutEffector();
    } else {
        int targetFlow = PIDController(targetPressure);
        Effector(targetFlow);
    }
}

/*
 * This function would open the OUTVALVE and open the INVALVE to a practical
 * minimum to enable a natural breathing out.
 */
void breathOutEffector() {
    // TODO: pass PWM signal to exhale valve for completely open it and limit
    // inhale valve to practical minimum
}

/*
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
    #endif // VALVES

    // Set referenceTime as when the program starts
    referenceTime = millis();
    startOfBreath = 0;
}

/*
 * Main loop for inner circuit.
 */
void loop1() {
    // TODO:sensor value sampling

    // Calculate current time in this cycle
    unsigned long systemTime = millis() - referenceTime();
    unsigned long cycleTime = systemTime - StartOfBreath;
    if (cycleTime < Tinhale) {
        inhalation();
    } else if (cycleTime < Texhale) {
        exhalation();
    } else if (cycleTime > Texhale) {
        StartOfBreath = systemTime; // Reset StartOfBreath to start next breath
    }
}

/*
 * Main loop for outer circuit.
 */
void loop2() {
    int oxygenFlow = 0; // TODO: set a standard HIGH ENOUGH value in 0-254
    int airFlow;

    airFlow = (int)((1 - FiO2) / (FiO2 - 0.21) * oxygenFlow);

    // TODO: Send corresponding PWM signals to AIRVALVE and O2VALVE
}
