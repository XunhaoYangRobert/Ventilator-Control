#include <DueTimer.h>

// These value mappings start 3=closest to bottom of PCB 0=closest to top of PCB
#define AIRVALVE 6 /* PWM, PIN 6 */
#define O2VALVE 7 /* PWM, PIN 7 */
#define INVALVE 8 /* PWM, PIN 8 */
#define OUTVALVE 9 /*PWM, PIN 9 */

// Values for valve's PWM property
#define PWMRESOLUTION 100 // Resolution for PWM control signal
#define PWMFREQUENCY 1.2 // kHZ

// PWM control variables
int dutyCycle[4] = {20, 40, 60, 80}; // For air valve, O2 valve, inhalation valve and exhalation valve successively
int PWMcounter = 0;
bool digitalSignalOn = false;

/*
 * This function controls underlying logic of the PWM signals sent to each valves.
 */
void valveHandler() {
    if (PWMcounter == PWMRESOLUTION) {
        PWMcounter = 0; // Reset counter after a period
    }

    // Air valve handler
    (PWMcounter < dutyCycle[0]) ? digitalSignalOn = true : digitalSignalOn = false;
    digitalWrite(AIRVALVE, digitalSignalOn);

    // O2 valve handler
    (PWMcounter < dutyCycle[1]) ? digitalSignalOn = true : digitalSignalOn = false;
    digitalWrite(O2VALVE, digitalSignalOn);

    // Inhalation valve handler
    (PWMcounter < dutyCycle[2]) ? digitalSignalOn = true : digitalSignalOn = false;
    digitalWrite(INVALVE, digitalSignalOn);

    // Exhalation valve handler
    (PWMcounter < dutyCycle[3]) ? digitalSignalOn = true : digitalSignalOn = false;
    digitalWrite(OUTVALVE, digitalSignalOn);

    PWMcounter++;
}

void setup() {
    // Setting up PWM handler for valves
    pinMode(AIRVALVE, OUTPUT);
    pinMode(O2VALVE, OUTPUT);
    pinMode(INVALVE, OUTPUT);
    pinMode(OUTVALVE, OUTPUT);

    Timer3.attachInterrupt(valveHandler).start((1.0 / PWMFREQUENCY) * 100 / PWMRESOLUTION);
}
