/* Master 74HC165 Test 20180303
  Author: Larry Bush zathras3@verizon.net
  Acme Switch and Signal Co
  03 March 2018

    Connects two Arduinos together using RS485 hardware.
    The Master Arduino polls the Remote Arduino using
    character "p".  The Remote sends back the number of
    bytes (usually 8) and then sends the bytes to the Master.
    These bytes are stored in an array and then output to a
    bank of 74HV595 shift registers and the Serial Monitor.

    Works with RS485 remote Version 7.
    This version runs continuously.
    RS485 data rate is 38400 bps.
    Includes Test-Run-Stop switch.
    Seems to work well.
    Pin D09 reserved for RS485 direction control if needed.

    Serial Monitor data rate is 38400 bps.

    This Version implements setup and test for 74HC165 inputs
    at the Master Arduino.  The test reads a single byte from
    a 74HC165 8-bit Parallel-to-Serial Shift Register and
    writes it out to the eight 74HC595 8-bit Serial-to-Parallel
    Shift Registers.
*/

/*******************************************************/
/*  INCLUDE LIBRARIES                                  */
/*******************************************************/
#include <SoftwareSerial.h>

/************************************************************/
/*  INPUT SHIFT REGISTER PIN ASSIGNMENTS                    */
/*  Connect the following pins between the Arduino and the  */
/*  74HC165                                                 */
/*  Connect pins A-H to 5V or GND or switches or whatever   */
/************************************************************/
// Connect Pin D07 to SER_OUT (serial data out)
const byte data_pin = 7;
// Connect Pin D06 to SH/LD (shift or active low load)
const byte shld_pin = 6;
// Connect Pin D05 to CLK (the clock that times the shifting)
// clock on low-to-high transition when CE is low and SH/LD is high
const byte clk_pin = 5;
// Connect Pin D04 to CE (clock enable, active low
const byte ce_pin = 4; //  also known as CLK INH)

/****************************************************************/
/*  OUTPUT SHIFT REGISTER PIN ASSIGNMENTS                       */
/*  Connect the following pins between your Arduino and the     */
/*  74HC595 Breakout Board                                      */
/*  Connect pins A-H to LEDs, pin /Reset to VCC, pin /CE to GND */
/****************************************************************/
const byte clockPin = 10; // Connect Pin D10 to CLOCK
const byte latchPin = 11;  // Connect Pin D11 to L_CLOCK
const byte dataPin = 12;  // Connect Pin D12 to SER_IN

/*************************************************************/
/*  LinkSprite RS485 Shield V2.1 PIN ASSIGNMENTS             */
/*  Uses pins D02 (TX), D03 (RX) and D09 (direction control) */
/*************************************************************/
const byte SSerialRX = 3;  // serial receive pin D03
const byte SSerialTX = 2;  // serial transmit pin D02
const byte SSerialDirection = 9;  //direction control

/*********************************************************/
/* DECLARE TEST-RUN-STOP SWITCH.  SPDT center-off-toggle */
/*********************************************************/
// run-stop switch on pin A0.  high for run, low for stop
const int runstop = A0;
// run-test switch on pin A1.  high for run, low for test
const int runtest = A1;

/*******************************************************/
/* DECLARE CONSTANTS                                   */
/*******************************************************/
// pulsewidth in microseconds to trigger the shift register latch
const int pulsewidth = 5;
const int numberofregisters = 8;  // size of outputdataArray
const int inputregisters = 2; //  size of inputdataArray
const byte pin13LED = 13;

/*******************************************************/
/* DECLARE OBJECTS                                     */
/*******************************************************/
SoftwareSerial RS485Serial(SSerialRX, SSerialTX); // RX, TX pins

/*******************************************************/
/* DECLARE ARRAYS                                      */
/*******************************************************/
byte outputdataArray[numberofregisters];
byte inputdataArray[inputregisters];

/*******************************************************/
/*  DECLARE GLOBAL VARIABLES                           */
/*******************************************************/
char charReceived;  //  from Remote Arduino
int numberofbytes;  // set by Remote Arduino
int pollcount = 0;  // count the number of times poll
int errorcount = 0; //  cumulative error count
unsigned long elapsed_time; // measures time per cycle

/******************************************************/
/*  SETUP - setup code runs once                      */
/******************************************************/
void setup()
{
  // start the serial port for the Serial Monitor
  Serial.begin(38400); // set the data rate
  delay(50); // allow interface to settle
  Serial.println("Program RS485 Master 03 March 2018");
  Serial.println("Starting Serial Monitor at 38400 bps.");

  // Start the software serial port for RS485
  RS485Serial.begin(38400);  // set the RS485 data rate
  delay(50);  //  allow interface to settle
  Serial.println("Starting RS485 Serial Port at 38400 bps.");

  pinMode(pin13LED, OUTPUT);  //  Use the on-board LED to show shift register activity

  // Initialize pins for 74HC595 shift register
  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, OUTPUT);

  //  Initialize each 74HC165 control pin to either output or input
  //  Initialize pins to control the 74HC165 shift register
  //  with each pin with the exception of the serial
  //  data we get back on the data_pin input line.
  pinMode(shld_pin, OUTPUT);
  pinMode(ce_pin, OUTPUT);
  pinMode(clk_pin, OUTPUT);
  pinMode(data_pin, INPUT);

  //  Set required initial states of these two 74HC165 pins
  //  according to the datasheet timing diagram
  digitalWrite(clk_pin, HIGH);
  digitalWrite(shld_pin, HIGH);

  // Initialize Test-Run-Stop switch
  pinMode(runstop, INPUT_PULLUP);
  pinMode(runtest, INPUT_PULLUP);

} // end setup

/*****************************************************/
/*  LOOP - runs continuously to infinity and beyond! */
/*****************************************************/
void loop()
{
  while (digitalRead(runstop) == 0) { // runstop switch = stop
  }

  while (digitalRead(runtest) == 0) { //  run test routines
    Serial.println();
    Serial.println("Starting local test routines");
    turn_off();  // set all the outputs High - turns LEDs off
    delay(1000);  // time for visual inspection
    turn_on(); // set all the outputs Low - turns LEDs on
    delay(2000);  // time for visual inspection
    turn_off();  // set all the outputs High - turns LEDs off
    delay(1000);  // time for visual inspection
    test();
    delay(5000);  // time for visual inspection
  }

  //  begin polling Remote Arduino with code = p
  digitalWrite(pin13LED, HIGH); // start activity
  elapsed_time = millis;
  // send poll character to remote Arduino
  RS485Serial.write("p");
  pollcount++;
  Serial.println("*****************************************");
  Serial.print("Polling remote Arduino, count = ");
  Serial.println(pollcount);

  // wait for Remote Arduino to respond
  while (RS485Serial.available() <= 0) {
  } //  end while

  //  read received byte from RS485
  numberofbytes = RS485Serial.read();
  Serial.println("Data received from Remote Arduino");
  Serial.print("Number of bytes to be received = ");
  Serial.println(numberofbytes);
  if (numberofbytes != numberofregisters)
  {
    Serial.println("BYTE COUNT ERROR");
    errorcount++;
    numberofbytes = 0;
    return;  // exit from loop - may not be needed
  } //  end if

  Serial.print("Error count = ");
  Serial.println(errorcount);

  //  Wait for buffer to fill
  while (RS485Serial.available() < numberofbytes) {
  } //  end while

  //  receive data and load array
  for (int i = 0; i < numberofbytes; i++) {
    outputdataArray[i] = RS485Serial.read();
  } //  end for loop

  write_data();  //  write data to shift registers
  display_output_array(); //  write data to Serial Monitor
  digitalWrite(pin13LED, LOW); // end shift activity

  read_data(); //  Read the input shift registers
  //  Send input data to Remote Arduino
  RS485Serial.write("q");
  RS485Serial.write(inputregisters);

  Serial.println("Sending data to remote Arduino");
  for (int i = 0; i < inputregisters; i++) {
    RS485Serial.write(inputdataArray[i]);  // send data to the Remote
  } // end FOR sending input data array to Remote

  for (int i = 0; i < inputregisters; i++) {
    Serial.print("Byte  ");
    Serial.print(i);
    Serial.print("     ABCDEFGH : ");
    print_byte(inputdataArray[i]);     // send to the Serial Monitor
  } // end FOR sending input data array to Serial Monitor

  Serial.print("Elapsed Time is  ");
  elapsed_time = millis;
  Serial.print(elapsed_time);
  Serial.print("  milliseconds");
  Serial.println();
  delay(10); //  wait before next poll
} // end loop

/*****************************************************************/
/* Function print_byte                                           */
/* This function prints all the 1's and 0's of a byte            */
/* Called from displayarray                                      */
/*****************************************************************/
void print_byte(byte val)
{
  byte i; //  local variable
  for (i = 0; i <= 7; i++) {
    // Magic bit shift, if you care look up the <<, >>, and & operators
    Serial.print(val >> i & 1, BIN);
  }
  Serial.print("\n"); // Go to the next line
}

/*****************************************************************/
/* Function test                                                 */
/* Reads single 74HC165 input shift register                     */
/* Outputs result to all 8 74HC595 output shift registers        */
/*****************************************************************/
void test() {
  byte testdata;
  Serial.println();
  Serial.println("Running local test");
  Serial.println("\nThe test data from the shift register is: ");
  //  read input shift registers
  //  Latch the state of the A-H data lines into the 74HC165
  //  by taking the SHLD pin low for 5 microseconds
  digitalWrite(shld_pin, LOW);
  delayMicroseconds(pulsewidth);
  digitalWrite(shld_pin, HIGH);

  //  read the input shift registers
  read_data();
  testdata = inputdataArray[0];

  Serial.print("Byte  ");
  Serial.print("0");
  Serial.print("     ABCDEFGH : ");
  print_byte(testdata);     // send to the Serial Monitor

  //  load the array
  for (int i = 0; i < numberofregisters; i++) {
    outputdataArray[i] = testdata;
  } //  end for loop

  write_data();  //  write the array to the shift registers
  display_output_array(); //  send the data to the Serial Monitor

  Serial.println();
} //  end test function

/*****************************************************************/
/* Function read_data                                            */
/* Reads 74HC165 input shift registers                           */
/*****************************************************************/
void read_data() {
  Serial.print("Reading input shift register");
  //  read input shift registers
  //  Latch the state of the A-H data lines into the 74HC165
  //  by taking the SHLD pin low for 5 microseconds
  digitalWrite(shld_pin, LOW);
  delayMicroseconds(pulsewidth);
  digitalWrite(shld_pin, HIGH);

  //  Read the shift registers
  digitalWrite(clk_pin, HIGH);
  digitalWrite(ce_pin, LOW); // Enable the clock

  // Get the A-H values
  for (int i = 0; i < inputregisters; i++) {
    inputdataArray[i] = shiftIn(data_pin, clk_pin, MSBFIRST);
  }
  digitalWrite(ce_pin, HIGH); // Disable the clock
  Serial.println();
} //  end read_data function

/******************************************************************/
/*  Function writedata writes outputdataArray to the specified    */
/*  number of registers                                           */
/*  Called from each test subroutine                              */
/******************************************************************/
void write_data() {
  byte i; //  local variable counter for shift register loop
  for (i = 0; i < numberofregisters; i++) {
    //  repeats for the specified number of shift registers
    //  shiftout send a byte to output shift register
    shiftOut(dataPin, clockPin, MSBFIRST, outputdataArray[i] );
  } //  end for loop

  //  latch the data with 5 microsecond pulse
  digitalWrite(latchPin, LOW);
  delayMicroseconds(pulsewidth);
  digitalWrite(latchPin, HIGH); //  latches data on shift registers
} //  end write_data function

/*****************************************************************/
/*  Function turn_off sets all bits in all registers HIGH        */
/*  Called from loop                                             */
/*****************************************************************/
void turn_off() {
  byte testdata = 0xFF; //0b11111111 sets all bits in register high
  Serial.println("Turning off all the outputs");
  Serial.println("When Darkness Falls!");

  //  load the array
  for (int b = 0; b < numberofregisters; b++) {
    outputdataArray[b] = testdata;
  }

  write_data();  // write the array to the registers
  display_output_array(); // send the data to Serial Monitor
} //  end turn_off function

/*****************************************************************/
/*  Function turn_on sets all bits in all registers LOW          */
/*  Called from loop                                             */
/*****************************************************************/
void turn_on() {
  byte testdata = 0x00; //0b00000000 sets all bits in register low
  Serial.println("Turning on all the outputs");
  Serial.println("Let There Be Light!");

  //  load the array
  for (int b = 0; b < numberofregisters; b++) {
    outputdataArray[b] = testdata;
  }

  write_data();  // write the array to the registers
  display_output_array(); // display the array to the Serial Monitor
} //  end turn_on function

/**************************************************************/
/*  Function displayoutputarray sends outputdataArray to the  */
/*  Serial Monitor                                            */
/*  Called from turnon and turnoff functions                  */
/**************************************************************/
void display_output_array() {
  Serial.println("\nThe data sent to the output shift registers is: ");
  for (int i = 0; i < numberofregisters; i++)
  {
    //  Print out the values being written to the shift register
    Serial.print("Byte  ");
    Serial.print(i);
    Serial.print("     ABCDEFGH : ");
    //  Calls the subroutine that prints ones and zeroes
    // Print every 1 and 0 that correlates with A through H
    print_byte(outputdataArray[i]); // send to Serial Monitor
  } //  end for loop
  Serial.println();
} // end display_output_array function
