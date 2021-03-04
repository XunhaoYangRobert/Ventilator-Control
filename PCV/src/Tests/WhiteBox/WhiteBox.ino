#define VENTILATORLED 3

#define VALVEVDDIN 61/*A7 NOT WORKING, NOT SURE...*/

#define MODEMTXPIN 13 /*PWM is 13 on the DUE mapping*/
#define MODEMRXPIN 2 /*PIOA0 is mapped to Arduino pin 2, or PB25,Peripheral B)*/

#define BUZZERPIN 5 /*PWM5 on DUE buzzes the buzzer*/

#define AIRFLOW /*OK*/
//#define ADCO2 /*OK NOTE: Patch was required for gnd, which was floating. PCB has been updated.*/
//#define DIGIPOT /*OK NOTE: PCB design corrected to tie A0 pin to gnd. Device operates without this fix, however behaviour is not specified.*/
//#define ADCP /*OK*/
//#define SPIISR /*OK*/
//#define VALVE12VSENSE /*Failed. This is a mystery*/
//#define SERIAL1 /*OK*/
//#define MODEM /*OK*/
//#define PMIC /*OK*/
//#define BUZZER /*OK*/
#define VALVES /*OK NOTE: 12Vgnd and 5vgnd must be connected either at PSU or the PCB can be patched. Not sure best way.*/

#include <Wire.h>
// #include <ADS1115.h>//O2 Sensors: For the ADS1115 you can use the library from i2cdevlib.com. see: https://github.com/jrowberg/i2cdevlib/tree/master/Arduino
#include <DS3904.h>
#include <PWM.h>
#include <DueTimer.h> // Change PWM frequency
#ifdef BUZZER
#include "NewToneLib.h"
#endif

#ifdef PMIC
//Due Pin maps for PMIC Status lines
#define PG 47
#define BATTSTAT1 46
#define BATTSTAT2 48

//Status codes for BATTSTAT1 << 1 + BATTSTAT2 (note logic values reverse of switch status)
//                    Bstat1   Bstat2
#define DECODEBATTSTAT(x,y) (x << 1) | y
#define BATTPRECHARGE   (0<<1) | 0
#define BATTFASTCHARGE  (0<<1) | 1
#define BATTCHARGEDONE  (1<<1) | 0
#define BATTCHARGEFAULT (1<<1) | 1

#endif //PMIC

//Setup ADS1115
#ifdef ADCO2 //NOTE: Verifide this code works on the vanilla DUE with 2k pullups, but fails on our board with 1.5k pullups. requirement is 4.7k, but we need 10k for the digipot.
ADS1115 adc0(ADS1115_DEFAULT_ADDRESS);//ADS1115_ADDRESS_ADDR_SCL);//ADS1115_ADDRESS_ADDR_SDA);//ADS1115_ADDRESS_ADDR_VDD);//
#endif

//Constants for honeywell airflow sensor
#ifdef AIRFLOW
#define AFLAADD 0x49
#define HF_CAL 8333
#define HF_OFFSET 1638
#define SENSOR_MAX_FLOW 1667 // ml/sec - for 100 L/m sensors

//Use to pack flow reading.
union flowbytes{
  uint16_t raw;
  uint8_t bytes[2];
  struct{uint8_t byte0,byte1;};
};


int32_t airFlowFromBytes(flowbytes* flow)
{

  if(flow->raw & 0xC000)
    SerialUSB.print("Very suspicious data! Oh no!\n");


  int32_t cal = (HF_CAL * (uint32_t) (flow->raw - HF_OFFSET)) >> 16;
  if(cal < 0) cal = 0; // don't let it wrap 
  return(cal);
}

#endif //AIRFLOW


#if defined(DIGIPOT) || defined(MODEM)
DS3904 resistor(BASE_ADDRESS);
int r0ohm = 5000;//10000; //Kohm value
int r1ohm = 10000; //kohm value
int r2ohm = 2000; //Kohm value

#endif //DIGIPOT

//Define globals for Modem ISR to use
#ifdef MODEM
/**************************************************************************************/
/*                        Capture PWM frequency and duty cycle                        */
/*      Hook a jumper between pin 2 (TIOA0) and pin A7 (TIOA1)                        */
/* In this schema, TIOA0 is the interupt input I believe, which is also PWM2 / pin2   */
/**************************************************************************************/

volatile uint32_t CaptureCountA, CaptureCountB, TimerCount;
volatile boolean CaptureFlag;
#endif //MODEM

#ifdef VALVES
#define VALVE0 6/*PWM, PIN 6*/
#define VALVE1 7/*PWM, PIN 7*/
#define VALVE2 8/*PWM, PIN 8*/
#define VALVE3 9/*PWM, PIN 9*/

byte V;
#endif//VALVES

//function to compute PSI for 150PAAB5
float Pressure2PSI150PA(int rawadc)
{
  float PSI = (rawadc-140.0);//102.4);
  PSI = PSI*150/819.2;
  return PSI;
}

#ifdef BUZZER
MyTone t(false);
#endif

// Function to custom PWM frequency
bool ledOn = false;
void myHandler(){
  ledOn = !ledOn;

//if markspace < threshold
//  ledOn = 0
//else
//  ledOn = 1

  digitalWrite(6, ledOn); // Led on, off, on, off...
  digitalWrite(7, ledOn); // Led on, off, on, off...
  digitalWrite(8, ledOn); // Led on, off, on, off...
  digitalWrite(9, ledOn); // Led on, off, on, off...
}

void setup() 
{

  /* //setting up pwm function timer */
  /* InitTimersSafe(); */ 
  /* SetPinFrequencySafe(VALVE0, 1200); */
  /* SetPinFrequencySafe(VALVE1, 1200); */
  /* SetPinFrequencySafe(VALVE2, 1200); */
  /* SetPinFrequencySafe(VALVE3, 1200); */

  //Setup code
  pinMode(VENTILATORLED,OUTPUT);

  //Setup I2C
  Wire.begin();  // join i2c bus 0 (address optional for master)
  Wire1.begin(); // join i2c bus 1 (as above)

  SerialUSB.begin(115200);

  #ifndef SERIAL1
  while(!SerialUSB)
  {
    ;//Needed for not missing out on messages but hangs code if no serial monitor is listenting.
  }
  #endif //SERIAL1
  SerialUSB.println("BOOT OK\n"); //Work
  

  #ifdef ADCO2
  bool adc0con;
  adc0con = adc0.testConnection();
  //configure ADC for reading.
  SerialUSB.print("check ADC is connected...");
  SerialUSB.println(adc0con ? "ADS1115 connection successful" : "ADS1115 connection failed");

  // We're going to do single shot sampling
  adc0.setMode(ADS1115_MODE_CONTINUOUS);
  
  // Slow things down so that we can see that the "poll for conversion" code works
  adc0.setRate(ADS1115_RATE_8);
    
  // Set the gain (PGA) +/- 6.144V
  // Note that any analog input must be higher than –0.3V and less than VDD +0.3
  adc0.setGain(ADS1115_PGA_6P144);
  #endif //ADC02

  #if defined(DIGIPOT) || defined(MODEM)
  resistor.setValue(RESISTOR_0, HIGH_Z);
  // Setting values in ohms is an approximation only
  resistor.setOhmValue(RESISTOR_0, r0ohm);
  resistor.setOhmValue(RESISTOR_1, r1ohm);
  resistor.setOhmValue(RESISTOR_2, r2ohm);//0x7F); //also hase "setValue" which I guess is less useful?
  SerialUSB.print("Digipot Values set... r0: "); SerialUSB.println(r0ohm); 
  SerialUSB.print(" r1: "); SerialUSB.println(r1ohm);
  SerialUSB.print(" r2: "); SerialUSB.println(r2ohm);
  #endif //DIGIPOT

  
  #ifdef SPIISR
  //Set the SPI into slave interupt driven mode. Note this is not well supported, hence verbose setup.
  //SPI serial recieve
  PMC->PMC_PCER0 |= PMC_PCER0_PID24;    // SPI0 power ON
  PIOA->PIO_PDR = PIO_PDR_P25 | PIO_PDR_P26 | PIO_PDR_P27 | PIO_PDR_P28;
  PIOA->PIO_ABSR &= ~(PIO_PA25A_SPI0_MISO | PIO_PA26A_SPI0_MOSI | PIO_PA27A_SPI0_SPCK | PIO_PA28A_SPI0_NPCS0);
  // SPI Disable
  SPI0->SPI_CR = SPI_CR_SPIDIS;// SPI is in slave mode after software reset !!
  // Perform a SPI software reset twice, like SAM does.
  SPI0->SPI_CR = SPI_CR_SWRST;
  SPI0->SPI_CR = SPI_CR_SWRST;
  delay(10);
  REG_SPI0_MR = 0;   // Slave mode
  // Receive Data Register Full Interrupt
  SPI0->SPI_IER = SPI_IER_RDRF;
  NVIC_EnableIRQ(SPI0_IRQn);
  SPI0->SPI_TDR = SPI0->SPI_TDR | (uint16_t) 0xcafe;
  SPI0->SPI_CSR[0] = SPI_CSR_NCPHA|SPI_CSR_BITS_16_BIT; 
  REG_SPI0_CR = 1;   // Enable SPI
  SerialUSB.println("SPI0 set to loopback with CS=PA28\n");
  #endif //SPIISR

  #ifdef SERIAL1
  Serial.begin(9600);
  while(!Serial){};
  #endif //SERIAL1

  #ifdef MODEM
  //Set up the write (pretty simple...)
  pinMode(MODEMTXPIN, OUTPUT);  // sets the pin as output
  pinMode(MODEMRXPIN, INPUT);   // sets the pin as input.
  //setup the peripheral to be B...
  PIOB->PIO_ABSR |= 1<<25;
  //Setup the timer counter to read 
  /*************         Capture a PWM frequency and duty cycle          ****************/
  PMC->PMC_PCER0 |= PMC_PCER0_PID28;                      // Timer Counter 0 channel 1 IS TC1, TC1 power ON

  TC0->TC_CHANNEL[1].TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK1 // capture mode, MCK/2 = 42 MHz, clk on rising edge
                              | TC_CMR_ABETRG              // TIOA is used as the external trigger
                              | TC_CMR_LDRA_RISING         // load RA on rising edge of trigger input
                              | TC_CMR_LDRB_FALLING;       // load RB on falling edge of trigger input
                              
  // If you want to capture PWM data from TC1_Handler()
  TC0->TC_CHANNEL[1].TC_IER |= TC_IER_LDRAS | TC_IER_LDRBS; // Trigger interruption on Load RA and load RB
  
  TC0->TC_CHANNEL[1].TC_CCR = TC_CCR_SWTRG | TC_CCR_CLKEN; // Reset TC counter and enable

  NVIC_EnableIRQ(TC1_IRQn);                                // Enable TC1 interrupts
  #endif //MODEM

  #ifdef PMIC
  //Set the PMIC inputs to read
  pinMode(PG, INPUT);
  pinMode(BATTSTAT1, INPUT);
  pinMode(BATTSTAT2, INPUT);
  #endif //PMIC

  #ifdef VALVES
  //Set the valve PWM drivers to output
  pinMode(VALVE0, OUTPUT);
  pinMode(VALVE2, OUTPUT);
  pinMode(VALVE2, OUTPUT);
  pinMode(VALVE3, OUTPUT);
  V = 10;

  Timer3.attachInterrupt(myHandler);
  //Timer3.start(50000); // Calls every 50ms
  //Timer3.start(500); //call every 0.5ms or 1 KHz
  Timer3.start(416);      //1.2 KHz
  #endif//VALVES

  //Setup the buzzer
  #ifdef BUZZER
  pinMode(BUZZERPIN, OUTPUT); //pin declared as OUTPUT
  #endif

  digitalWrite(VENTILATORLED, LOW);    // turn the LED off by making the voltage LOW
}

//main program loop
void loop()
{
  //SerialUSB.println("Ventilator white box tests... \n");  //Serial Monitor test.
  // wait for a second
  //
  // put your main code here, to run repeatedly:
  //Peripheral Test, LED. Very useful to know that the code is actually running without JTAG capability
  digitalWrite(VENTILATORLED, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);//500);                       // wait for a second
  digitalWrite(VENTILATORLED, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);//500); 

  //=========================== READ O2 SENSOR ========================
  #ifdef ADCO2
  SerialUSB.println("Read O2 Sensor ADCs\n");
  adc0.setMultiplexer(ADS1115_MUX_P0_NG); //for testing this is set to single shot read from chl 0
  SerialUSB.print("O2 0: "); SerialUSB.print(adc0.getMilliVolts()); SerialUSB.print("mV\t\n");
  adc0.setMultiplexer(ADS1115_MUX_P1_NG); //for testing this is set to single shot read from chl 0
  SerialUSB.print("O2 1: "); SerialUSB.print(adc0.getMilliVolts()); SerialUSB.print("mV\t\n");
  SerialUSB.println("\n");

  #endif

  //========================== READ AIRFLOW SENSOR ====================
  #ifdef AIRFLOW
  flowbytes flow;
  int i = 2;

  //Test Flow sensor attached to JP1 on I2C bus 0
  Wire.requestFrom(AFLAADD, 2);    // request 2 bytes from the airflow device on JP1.

  while(Wire.available())    // slave may send less than requested
  { 
    if(i)
      {i--;flow.bytes[i]= Wire.read();}
    else
      char c = Wire.read(); // receive a byte as character (junk if we don't need this)
  }
  int32_t cal = airFlowFromBytes(&flow);  //convert to a normalized scale of airflow units

  SerialUSB.print("JP1 Calibrated Airflow read: ");
  SerialUSB.print(cal);
  SerialUSB.print('\n');

  //Test Flow sensor attached to JP2 on I2C bus 1
  flowbytes flow1;
  int ii = 2;
  
  Wire1.requestFrom(AFLAADD, 2);    // request 2 bytes from slave device #2

  while(Wire1.available())    // slave may send less than requested
  { 
    if(ii)
      {ii--;flow1.bytes[ii]= Wire1.read();}
    else
      char c = Wire1.read(); // receive a byte as character (junk if we don't need this)
    //Serial.print(c);         // print the character
    //i++;
  }
  cal = airFlowFromBytes(&flow1);

  SerialUSB.print("JP2 Calibrated Airflow read: ");
  SerialUSB.print(cal);
  SerialUSB.print('\n');

  #endif //AIRFLOW

  //========================== READ DIGIPOTENTIOMETER (SET IN INIT) ====================
  #ifdef DIGIPOT
  //byte value; you can also read the hex equivilent value with getValue. useful if you are just changing it around or whatever since Ohms is quantized anyway.
  int value;
 
  value = resistor.getOhmValue(RESISTOR_0);
  SerialUSB.print("Resistor 0 was set to : ");SerialUSB.print(r0ohm);SerialUSB.print("Ohm read : "); SerialUSB.println(value);
  value = resistor.getOhmValue(RESISTOR_1);
  SerialUSB.print("Resistor 1 was set to : ");SerialUSB.print(r1ohm);SerialUSB.print("Ohm read : "); SerialUSB.println(value);
  value = resistor.getOhmValue(RESISTOR_2);
  SerialUSB.print("Resistor 2 was set to : ");SerialUSB.print(r2ohm);SerialUSB.print("Ohm read : "); SerialUSB.println(value);
  
  #endif //DIGIPOT

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
  SerialUSB.println(Pressure2PSI150PA(pressureV));
  pressureV = analogRead(A4);
  SerialUSB.print("U7 Pressure value: ");
  SerialUSB.println(pressureV);
  pressureV = analogRead(A5);
  SerialUSB.print("U8 Pressure value: ");
  SerialUSB.println(pressureV);      
  #endif //ADCP

  //========================== TEST 12V SUPPLY SENSE ====================
  #ifdef VALVE12VSENSE
  int ValveVDDStat = analogRead(A0);//VALVEVDDIN);//A7);
  SerialUSB.print("Control Valve Power Status: ");
  SerialUSB.println(ValveVDDStat);  
  #endif //12VSENSE

  //========================== TEST SERIAL OUTPUT (REQUIRES A BOARD TO LISTEN ====================
  #ifdef SERIAL1
  SerialUSB.println("3v3 Serial Port Test...");
  Serial.write("Ventilator P1\n", 14);
  while (Serial.available() > 0) {
    char inByte = Serial.read();
    SerialUSB.write(inByte);
  }
  #endif //SERIAL1

  //========================== TEST CUSTOM MODEM (REQUIRES AN OSCILLOSCOPE) ====================
  #ifdef MODEM
  SerialUSB.println("Modem Carrier TX Test DEBUG OUTPUT DUTY 50/50");
  //Set a duty cycle of 50% on the PWM
  //This is not quite right honestly. The modem should be driven by a timer counter in wave generation mode
  //In that way, it would be far easier to use an interrupt based method to push out a FSK buffer.

  //TODO: This pin should be re-routed to a timer counter output multiplexed pin. Specifically, one multiplexed to TC0
  analogWrite(MODEMTXPIN,128); //Default is 1KHz wave. Shape is pretty horrible.
  
  //see if the input was able to read anything.
  //Before messing with timer interrupts etc, check if you can see anything on the input pin
  if(digitalRead(MODEMRXPIN))
  {
    SerialUSB.println("Pin High");
  }
  else
    SerialUSB.println("Pin Low");
/*  if(CaptureFlag != false)
  {
    CaptureFlag = false;
    SerialUSB.println("Modem Detected something input");
  }
  else
  {
    SerialUSB.println("Modem Did not detect anything input");
  }
  */
  #endif //MODEM

  //===================== TEST PMIC STATUS SIGNALS (CAN MODIFY BY POP/DEPOP BATTERY ================
  #ifdef PMIC

  SerialUSB.println("PMIC Power Supply Status report");
  
  if(digitalRead(PG))
    SerialUSB.println("Board power input: Fail. Reserve Battery Active");    
  else
    SerialUSB.println("Board power input: OK");

  SerialUSB.println("");
  SerialUSB.println("PMIC reserve battery status report");
  byte b1,b2;
  b1 = digitalRead(BATTSTAT1);
  b2 = digitalRead(BATTSTAT2);
  switch(DECODEBATTSTAT(b1,b2))
  {
    case BATTPRECHARGE:
      SerialUSB.println("LiPo battery: Pre-charge mode");
    break;
    case BATTFASTCHARGE:
      SerialUSB.println("LiPo battery: Fast charge mode");
    break;
    case BATTCHARGEDONE:
      SerialUSB.println("LiPo battery: Charge done");
    break;
    case BATTCHARGEFAULT:
      SerialUSB.println("LiPo battery: Failure warning");
    break;
    default :
      SerialUSB.println("PMIC Software Error. Impossible");
    break;
  }
  #endif //PMIC

  //================== TEST VALVE DRIVERS ==============================
  #ifdef VALVES

  //Sweep PWM ratios for each valve driver.
  SerialUSB.println("Solenoid Valve Tests");
  SerialUSB.print("Valve 0: "); SerialUSB.print(V);SerialUSB.println(" Open");
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
  V+=10;
  #endif//VALVES

  //================ RING THE BUZZER ===================================
  #ifdef BUZZER
  t.tone(BUZZERPIN, 1000, 500);//pin, frequency(Hz), duration(ms)
  #endif
}

#ifdef SPIISR
//========================== SPI LOOPBACK ISR, PARROT READ IN BYTES ====================
void SPI0_Handler()
{
  //Note these registers are globally available, TDR is output, RDR is input, both are 2 bytes.
  SPI0->SPI_TDR = SPI0->SPI_RDR;
}
#endif //SPIISR

#ifdef MODEM
//================== MODEM FREQUENCY DETECT (REQUIRES A FUNCTION GENERATOR PLUGED INTO MODEM ========
//See: https://forum.arduino.cc/index.php?topic=480228.0
// Note that you could either test status register by polling in loop()
void TC1_Handler() {

  //Registers A and B (RA and RB) are used as capture registers. They are loaded with
  //the counter value TC_CV when a programmable event occurs on the signal TIOA1.
  //TimerCount = TC0->TC_CHANNEL[1].TC_CV;            // save the timer counter register, for testing

  uint32_t status = TC0->TC_CHANNEL[1].TC_SR;       // Read & Save satus register -->Clear status register

  // If TC_SR_LOVRSRA is set, RA or RB have been loaded at least twice without any read
  // of the corresponding register since the last read of the Status Register,
  // We are losing some values,trigger of TC_Handler is not fast enough !!
  //if (status & TC_SR_LOVRS) abort();

  // TODO: calculate frequency and duty cycle from data below *****************
  if (status & TC_SR_LDRAS) {  // If ISR is fired by LDRAS then ....
    CaptureCountA = TC0->TC_CHANNEL[1].TC_RA;        // get data from capture register A for TC0 channel 1
  }
  else { /* if (status && TC_SR_LDRBS)*/  // If ISR is fired by LDRBS then ....
    CaptureCountB = TC0->TC_CHANNEL[1].TC_RB;         // get data from capture register B for TC0 channel 1

    CaptureFlag = 1;                      // set flag indicating a new capture value is present
  }

}
#endif
