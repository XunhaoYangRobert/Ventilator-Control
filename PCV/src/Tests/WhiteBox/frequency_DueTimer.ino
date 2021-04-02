#include <DueTimer.h>

// These value mappings start 3=closest to bottom of PCB 0=closest to top of PCB
#define AIRVALVE 6 /* PWM, PIN 6 */
#define O2VALVE 7 /* PWM, PIN 7 */
#define INVALVE 8 /* PWM, PIN 8 */
#define OUTVALVE 9 /*PWM, PIN 9 */

// Values for valve's PWM property
#define PWMRESOLUTION 255 /* Resolution for PWM control signal */
#define PWMFREQUENCY 1.2 /* kHZ */

// PWM control variables
int dutyCycle[4] = {50, 100, 150, 200}; /* For air valve, O2 valve, inhalation valve and exhalation valve successively */
int PWMcounter = 0;
bool digitalSignalOn = false;

//Main loop variables
int markspace = 10;

/* 
 * More efficient IO operation
 */
inline void digitalWriteDirect(int pin, boolean val) {
  if(val) g_APinDescription[pin].pPort -> PIO_SODR = g_APinDescription[pin].ulPin;
  else    g_APinDescription[pin].pPort -> PIO_CODR = g_APinDescription[pin].ulPin;
}

/*
 * This function controls underlying logic of the PWM signals sent to each valves.
 */
void valveHandler() {
    if (PWMcounter >= PWMRESOLUTION) {
        digitalWriteDirect(AIRVALVE, true);
        digitalWriteDirect(O2VALVE, true);
        digitalWriteDirect(INVALVE, true);
        digitalWriteDirect(OUTVALVE, true);
        PWMcounter = 0; // Reset counter after a period
    }

    // Valve handlers
    if (PWMcounter == dutyCycle[0])
      digitalWriteDirect(AIRVALVE, false);

    if (PWMcounter == dutyCycle[1])
      digitalWriteDirect(O2VALVE, false);

    if (PWMcounter == dutyCycle[2])
      digitalWriteDirect(INVALVE, false);

    if (PWMcounter == dutyCycle[3])
      digitalWriteDirect(OUTVALVE, false);

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

void loop(){
    markspace += 20;
    if(markspace>=PWMRESOLUTION)
      markspace = 10;

    dutyCycle[0] = markspace;
    dutyCycle[1] = markspace;
    dutyCycle[2] = markspace;
    dutyCycle[3] = markspace;

    delay(500);
}
