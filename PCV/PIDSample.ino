#define ADCP /*OK*/
#define VALVES /*OK*/

#include <Wire.h>

#ifdef VALVES
#define VALVE0 6/*PWM, PIN 6*/
#define VALVE1 7/*PWM, PIN 7*/
#define VALVE2 8/*PWM, PIN 8*/
#define VALVE3 9/*PWM, PIN 9*/

byte V;
#endif//VALVES

// PID values
float totalError = 0;
float previousError = 0;
float kp = 0;
float ki = 0;
float kd = 0;

//function to compute PSI for 150PAAB5
float Pressure2PSI150PA(int rawadc)
{
    float PSI = (rawadc-140.0);//102.4);
    PSI = PSI*150/819.2;
    return PSI;
}

void setup()
{

    SerialUSB.begin(115200);

#ifndef SERIAL1
    while(!SerialUSB)
    {
        ;//Needed for not missing out on messages but hangs code if no serial monitor is listenting.
    }
#endif //SERIAL1
    SerialUSB.println("BOOT OK\n"); //Work


#ifdef VALVES
    //Set the valve PWM drivers to output
    pinMode(VALVE0, OUTPUT);
    pinMode(VALVE2, OUTPUT);
    pinMode(VALVE2, OUTPUT);
    pinMode(VALVE3, OUTPUT);
    V = 10;
#endif//VALVES
}

//main program loop
void loop()
{

    //========================== READ ONBOARD ANALOGUE PRESSURE SENSOR INPUT ====================
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
#endif //ADCP

    //================== TEST VALVE DRIVERS ==============================
#ifdef VALVES
    //Sweep PWM ratios for each valve driver.
    SerialUSB.println("Solenoid Valve Tests");
    SerialUSB.print("Valve 0: "); SerialUSB.print(0);SerialUSB.println(" Open");
    //Test A, 50/50
    analogWrite(VALVE0,V); //easy to probe with scope.
    SerialUSB.print("Valve 1: ");SerialUSB.print(V);SerialUSB.println(" Open");
    analogWrite(VALVE1,V);    //Saftest for driving the system
    //Test C, 100/0
    SerialUSB.print("Valve 2: ");SerialUSB.print(V);SerialUSB.println(" Open");
    analogWrite(VALVE2,V);
    //Dummy
    SerialUSB.print("Valve 3: ");SerialUSB.print(V);SerialUSB.println(" Open");
    analogWrite(VALVE3,V);    //Test last if required
    V+=50;
#endif//VALVES

    delay(500);//every 1/2 second...
}

float PIDController(float targetPressure, int mode) {
    float targetFlow = 0;
    float error;

    // TODO: link sensor to this variable
    float sensor Pressure = 0;

    error = targetPressure - sensorPressure;
    totalError += error;
    targetFlow = 0 + kp*error + ki*totalError + kd*(error - previousError);
    /*
       if (Valve_PWM > 255)
       Valve_PWM = 255;
       else if (Valve_PWM < 0)
       Valve_PWM = 0;
     */
    previousError = error;

    return targetFlow;

}

//TODO: clear total error
void clear(){}

//TODO: safe value, exhalation basic flow
int Effector(){}
