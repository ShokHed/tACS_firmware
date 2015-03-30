//----------------------------------------------------------------------------
//              tACS Firmware
//
//Description:  Firmware for tACS device. This program runs on initialization,
//              verifys the test schedule data sent via PC interface,
//              initialzes harwdware timer and handles ISR, sets DAC value to
//              value to correponds to frequency, selects switch channel that
//              corresponds to correct waveform, selects digipot value for 
//              correct output attenuation, takes ADC reading of voltage
//              across output test resistor, sends output reading with
//              timestamp to PC interface.
//
//Developers:   Matson Ovstedal, Tyler Bannon
//
//Version:      1.0
//----------------------------------------------------------------------------
#include <SPI.h>
#include <avr/io.h>
#include <avr/interrupt.h>
int BluetoothData;
boolean running = false;
const int DACloadPin = 9; //Pin that controls SR latch to write to 2nd row shift registers, if held low registers are transparent
const int DACclockPin = 7; //Serial writing clock pin
const int DACserialDat = 8; //Serial writing line
const int dcPin = A4; //Pin that controls the DC and pulsed DC output waveform
const int negdcPin = A5; //Pin that controls the negative DC output waveform
const int squarePin = A3; //Pin that controls the square wave output waveform
const int sinePin = A2; //Pin that controls the sine wave output waveform
const int digiPotSync = 10; //sync pin for digi pot SPI communication
const int adcPin = A0;
const byte enableDigiPotMSB = 0x18; //B00011000
const byte enableDigiPotLSB = 0x02; //B00000010
const byte digiPotCommand = 0x04; //B00000100
float vPrime; 
int voltage;
int headRes; //sets digi pot resistance 0=0% of max, 100=100% of max. Max output occurs at 0,minimum output occurs at 100
float resPrime;
int resistance;
String outputWave; //string to be displayed on serial monitor
//Command variables
byte frequency; //SET VALUE HERE, only first 12 bits will be loaded, (valid: 0-4095)
byte waveForm;
byte amplitude;
byte runTime;
int tick; //Global 1 second 'tick' variable incremented by harware timer 1
unsigned long start_time;
unsigned long last_time;
word value;
word trunkTime;
String string1;
String string2;
int i;
// Define various ADC prescaler
const unsigned char PS_16 = (1 << ADPS2);  //Prescaler set value for max speed
const unsigned char PS_32 = (1 << ADPS2) | (1 << ADPS0);
const unsigned char PS_64 = (1 << ADPS2) | (1 << ADPS1);
const unsigned char PS_128 = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); //Prescaler used to clear
//New sending protocol
uint8_t bufSize;
uint8_t buf [4];

// --------------------------------------------------- //
// Initial setup
// Pin assignment and pin tasking assigned below
// Serial communication started and speed declared
void setup() {
  bufSize = 4;
  frequency = 1;     //****Frequency here (limits:1-251)****
  pinMode(DACloadPin, OUTPUT); //SR latch for 2nd row of shift registers
  pinMode(DACclockPin, OUTPUT); //Clock for serial writing
  pinMode(DACserialDat, OUTPUT); //Serial writing pin
  pinMode(dcPin, OUTPUT); //DC and pulsed DC writing pin
  pinMode(negdcPin, OUTPUT); //Negative DC writing pin
  pinMode(squarePin, OUTPUT); //Square wave writing pin
  pinMode(sinePin, OUTPUT); //Sinewave writing pin
  pinMode(digiPotSync, OUTPUT); //digi pot Syncing pin
  Serial.begin(115200);
  
    //Messages removed for testing
//  Serial.println("Hello BT!");           // Send init 
  SPI.begin();
  SPI.setBitOrder(MSBFIRST); //given on digipot datasheet
  SPI.setDataMode(SPI_MODE1); //see digipot datasheet
  digitalWrite(DACloadPin,HIGH); //guarantees load pin is initialized inactive
  digitalWrite(dcPin,LOW); //open all waveform switches to start
  digitalWrite(negdcPin,LOW);
  digitalWrite(squarePin,LOW);
  digitalWrite(sinePin,LOW);
  digitalWrite(digiPotSync, HIGH); //disable SPI on digi pot
  i=0;
  // set up the ADC
  ADCSRA &= ~PS_128;  // remove bits set by Arduino library
  // you can choose a prescaler from above.
  // PS_16, PS_32, PS_64 or PS_128
  ADCSRA |= PS_16;    // set our own prescaler to 16
  enableWiperProgram();
  
  //Welcome message commented for testing
//  Serial.println();
//  Serial.println();
//  Serial.println("*******************************************");
//  Serial.println("* Welcome to SHOKhed serial data protocol *");
//  Serial.println("*******************************************");
  //testing
  
  //Timer harware initialization:
  tick = 0;          //initialize tick to 0
  cli();             // disable global interrupts
  TCCR1A = 0;        // set entire TCCR1A register to 0
  TCCR1B = 0;        // same for TCCR1B
  // set compare match register to desired timer count:
  OCR1A = 15624;  //15642 timer overflows means interrupt every 1 second
  // turn on CTC mode:
  TCCR1B |= (1 << WGM12);
  // Set CS10 and CS12 bits for 1024 prescaler:
  TCCR1B |= (1 << CS10);  //16MHz with 1024 prescaler means 'tick' every 6.4E-5s
  TCCR1B |= (1 << CS12);
  // enable timer compare interrupt:
  TIMSK1 |= (1 << OCIE1A);
  sei(); //enable global interrupts
}

// ------------------------------------------------- //
//Main loop checks for data in serial buffer,
//takes output reading and sends to PC interface,
//checks run time.
void loop() 
{
  if(Serial.available() > 3){
    running = false;
    readInput();
  }
  
  //---------------------------------------------
  //Test and measure loop begins here 
  if(running){
    measure();
  }
}

void measure(){
//    word dataPoints [4];        // [0]=1st voltage, [1]=1st deltaT, [2]=2nd voltage, [3]=2nd deltaT 
//    byte numPoints = sizeof(dataPoints)/2;
//    byte count = 0;
    int ADC1=0;
    int deltaT=0;
  
    //First data point
    last_time = micros();        //Take last_time for trunk time = start-last
    ADC1 = analogRead(A0);      //Take ADC reading of output current    
    Serial.print('$');    //Send header
    Serial.write(lowByte(ADC1));
    Serial.write(highByte(ADC1));
    Serial.write(lowByte(deltaT));
    Serial.write(highByte(deltaT));
    
     //Messages removed for testing
//    Serial.println(string2);     //Print data point
  
  //Not first data point
  while(running) {
    
    //1st try
//    while(count <= numPoints){
//      start_time = micros();                          //Record for deltaT
//      dataPoints[count] = word(analogRead(A0));       //Cast ADC value as word in array
//      count++;                                        //Incriment counter
//      dataPoints[count] = word(start_time-last_time); //Cast deltaT as word in array
//      last_time = start_time;
//      count++;                                        //Watch for array index error
//    }
    //1st try
    
    start_time=micros();
    ADC1 = analogRead(A0);
    deltaT = int(start_time-last_time);
    last_time=start_time;
    
  
  Serial.print('$');    //Send header
  Serial.write(lowByte(ADC1));
  Serial.write(highByte(ADC1));
  Serial.write(lowByte(deltaT));
  Serial.write(highByte(deltaT));
//  for(byte i=0; i<numPoints; i++){
//  Serial.write(lowByte( dataPoints[i] ));
//  Serial.write(highByte( dataPoints[i] ));
//  }
//count = 0;
  if(tick/30 > runTime || Serial.available()){ //Check to see if runtime exceeded or new treatment arrived
  endCycle();
  }
    //New code end
 }
}

//Timer interrupt, increments tick by 1 every 1 second
ISR(TIMER1_COMPA_vect)
{
    tick++;
}

// ------------------------------------------------- //
//Opens output switch, turns off timer, sends
//message to PCinterface
void endCycle(){
  running = false;  //disallow
  disableInts();    //Turn timer off until new treatment schedule is loaded
  waveSwitch(0);    //Open all switches to stop treatment current output
  tick = 0;         //Set tick to 0
  
//  if(Serial.available()){  //New input interrupts treatment
//    readInput();
//  }
  
  //else{  //Treatment time elapsed
  
  //Messages removed for testing
    Serial.print("Treatment time of ");
    Serial.print(tick/30);
    Serial.println(" minutes has elapsed.");
    Serial.println("User may input new treatment specifications at any time.");
    Serial.println("*");
  //}
  
}

//Function sends command vector back to PC for verification
void sendInput(){
    //Messages removed for testing
//  Serial.print(waveForm);
//  Serial.print(headRes);
//  Serial.print(frequency);
//  Serial.print(runTime); 
}

// ------------------------------------------------- //
//Disables output while reading new input parameters,
//validates all treatment current parameters, if valid
//parameters are received values are set and values are
//resent to PC interface.
void readInput(){ 
  //Close all switches while setting output parameters
  waveSwitch(0);
  byte inBytes[]={0,0,0,0};  //Temp holder for new output parameters
  //Reads in new output parameters
  for(int i=0; i < 4; i++){
    inBytes[i] = Serial.read();
  }
  //Set gobal run time parameters
  waveForm = inBytes[0];
  headRes = inBytes[1];
  frequency = inBytes[2];
  runTime = inBytes[3];
  //Change waveform to new selected value
  if(inBytes[0]+inBytes[1]+inBytes[2]+inBytes[3] != 0){
    potRes();
    freqAdjust();
    waveSwitch(waveForm);  //Selects new output type, output signal begins when switch closes 
    //Set tick to 0 to begin time keeping for current test schedule
    running = true;
    tick = 0;
    //sei();  //enable global interrupts for tick increment  
    enableInts();   //Enable timer now that treatment has begun
    sendInput();    //Send treatment specifications back to PC for user verification
  }
  
  else{
      //Messages removed for testing
//    Serial.println("Treatment stopped by user");
  }
}

// ------------------------------------------------- //
//
void potRes()
{
  //digi-pot resistance setting code here
  // ****to be written****
  resPrime = (float)(100-headRes)*(1023.0/100.0)+0.5;
  resistance = (int)resPrime;
  digitalWrite(digiPotSync, LOW);
  byte shiftedRes = (resistance >> 8);
  byte MSbyte = (digiPotCommand | shiftedRes);
  byte LSbyte = (resistance & 0xFF);
  SPI.transfer(MSbyte);
  SPI.transfer(LSbyte);
  digitalWrite(digiPotSync, HIGH);
}

void enableWiperProgram()
{
  digitalWrite(digiPotSync, LOW);
  SPI.transfer(enableDigiPotMSB);
  SPI.transfer(enableDigiPotLSB);
    
    //Messages removed for testing
//  Serial.print("Enable data-word is: ");
//  //BLU.print("Enable data-word is: ");
//  Serial.print(enableDigiPotMSB, BIN);
//  //BLU.print(enableDigiPotMSB, BIN);
//  Serial.print("    ");
//  //BLU.print("    ");
//  Serial.println(enableDigiPotLSB, BIN);
  //BLU.println(enableDigiPotLSB, BIN);
  digitalWrite(digiPotSync, HIGH);
}

// ------------------------------------------------- //
//Sets the state of the Wave Switch
//Picks waveform at output. 0 = open circuit,
//1 = DC, 2 = negative DC, 3 = square wave, 
//4 = sine wave
void waveSwitch(byte localWave)
{
digitalWrite(dcPin,LOW); //open all waveform switches to start
digitalWrite(negdcPin,LOW);
digitalWrite(squarePin,LOW);
digitalWrite(sinePin,LOW);
switch (localWave) 
{
  case 0:{break;} //Case 0: leave all switches closed, no output
  case 1: //C=1: DC/Pulsed DC waveform
  {
    digitalWrite(dcPin,HIGH);//close switch
    outputWave = "DC"; //to print to console
    break;
  }
  case 2: //C=2: negative DC waveform
  {
    digitalWrite(negdcPin,HIGH);//close switch
    outputWave = "Negative DC"; //to print to console
    break;
  }  
  case 3: //C=3: square wave waveform
  {
    digitalWrite(squarePin,HIGH);//close switch
    outputWave = "Square Wave"; //to print to console
    break;
  }
  case 4: //C=4: sine wave waveform
  {
    digitalWrite(sinePin,HIGH);//close switch
    outputWave = "Sine Wave"; //to print to console
    break;
  }
}
}
// ----------------------------------------------- //
// Normalizes frequency float and sets DAC voltage
// Sets frequency at output
void freqAdjust()
{
    //Messages removed for testing
//  Serial.print("frequency received = ");
//  Serial.println(frequency);
  
  vPrime = (float)(217-frequency)*(4095.0/216);
  voltage = (int)vPrime;
  //voltage = 2000;
  digitalWrite(DACloadPin,HIGH);
  shiftOut(DACserialDat , DACclockPin, MSBFIRST , highByte(voltage) );
  shiftOut(DACserialDat , DACclockPin, MSBFIRST , lowByte(voltage) );
  digitalWrite(DACloadPin,LOW);
  //delay(1);
  digitalWrite(DACloadPin,HIGH); //digitalWrite(loadPin,HIGH)
  
    //Messages removed for testing
// Serial.print("Frequency bits sent = ");
// Serial.println(voltage);
// Serial.print("Sent as highByte = ");
// Serial.println(highByte(voltage), DEC);
// Serial.print(", and lowByte = ");
// Serial.print(lowByte(voltage), DEC);
}

//This is used to disable the timer when waiting for new command vector
void disableInts(){
  TCCR1B = 0;
}

//This function is used to enable the timer after is has been disabled 
void enableInts(){
  // turn on CTC mode:
  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << CS10);  //16MHz with 1024 prescaler means 'tick' every 6.4E-5s
  TCCR1B |= (1 << CS12);
}
