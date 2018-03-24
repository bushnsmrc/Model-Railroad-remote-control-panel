/* RS485 Remote Version 7
    Author:  Larry Bush   zathras3@verizon.net
    Acme Switch and Signal Co.
    A Division of The Chesapeake System
    26 February 2018

    Used with Master Version HC165test_d
    When polled by the Master Arduino,
    loads 8 bytes of data from 74HC165 Serial
    In/Parallel Out Shift Registers into a
    data array and sends the array thru the RS485
    bus shield to the Master Arduino.

    RS485 shield uses pins 2 (TX), 3 (RX)

    Pin 13 LED is on while data is shifted.

    I am using the same Arduino pin assignments
    for both the Master and Remote.
    The initialization and setup include the cod
    for both, even if unused.

    Test routines for the Remote are TBD.

    This version of the remote only implements
    the 74HC165 shift register for input.

    The data direction pin on the RS485 shield
    is not used and should be jumpered to TX.
    But I am reserving pin D09 for this.

    Version 2 adds switch-case construct to prepare
    for future use of other commands from master
    Arduino.

    Version 3 adds receipt of data from the Master
    Arduino for output to 74HC595 shift registers
    for control of relays.

    Version 3a fixes handshaking for case "q".

    Version 4a cleans up some code

    Version 5 changes handling of output shift
    registers so no longer interleaved with sending
    data to Serial Monitor.

    Version 6 creates a function to display the
    input data array.

    Version 7 adds loopback test.
*/

/**********************************************/
/*  INCLUDE LIBRARIES                         */
/**********************************************/
#include <SoftwareSerial.h>

/************************************************************/
/*  INPUT SHIFT REGISTER PIN ASSIGNMENTS                    */
/*  Connect the following pins between the Arduino and the  */
/*  74HC165                                                 */
/*  Connect pins A-H to 5V or GND or switches or whatever   */
/************************************************************/
// Assign Pin D07 to SER_OUT (serial data out)
const byte data_pin = 7;
// Assign Pin D06 to SH/LD (shift or active low load)
const byte shld_pin = 6;
// Assign Pin D05 to CLK (the clock that times the shifting)
// clock on low-to-high transition when CE is low and SH/LD is high
const byte clk_pin = 5;
// Assign Pin D04 to CE (clock enable, active low
const byte ce_pin = 4; //  also known as CLK INH)

/************************************************************/
/*  OUTPUT SHIFT REGISTER PIN ASSIGNMENTS                   */
/*  Connect Arduino pins D10, D11, and D12 to the 74HC595   */
/*  Connect 74HC595 pins A-H to LEDs or                     */
/*  opto-isolated relays or whatever                        */
/************************************************************/
const byte clockPin = 10; // Assign Pin D10 to CLOCK
const byte latchPin = 11;  // Assign Pin D11 to L_CLOCK
const byte dataPin = 12;  // Assign Pin D12 to SER_IN

/*************************************************************/
/*  LINKSPRITE RS485 SHIELD V2.1 PIN ASSIGNMENTS             */
/*  Uses pins D02 (TX), D03 (RX) and D09 (direction control) */
/*************************************************************/
const byte SSerialRX = 3;   // serial receive pin
const byte SSerialTX = 2;   // serial transmit pin
const byte SSerialDirection = 9;    // direction control

/**************************************************/
/* TEST-RUN-STOP SWITCH.  SPDT center off toggle  */
/**************************************************/
// run-stop switch on pin A0.  high for run, low for stop
const int runstop = A0;
// run-test switch on pin A1.  high for run, low for test
const int runtest = A1;

/**********************************************/
/*  DECLARE CONSTANTS                         */
/**********************************************/

// pulsewidth in microseconds to trigger the shift register latch
const int pulsewidth = 5;
const int inputregisters = 8; //  size of inputdataArray
const int outputregisters = 2;  //  size of outputdataArray
const byte pin13LED = 13;
const byte testdata = 0b01010101;

/**********************************************/
/*  DECLARE OBJECTS                           */
/**********************************************/
SoftwareSerial RS485Serial(SSerialRX, SSerialTX); // RX, TX pins

/**********************************************/
/*  DECLARE ARRAYS                            */
/**********************************************/
byte testdataArray[inputregisters];
byte inputdataArray[inputregisters];
byte outputdataArray[outputregisters];

/**********************************************/
/*  DECLARE GLOBAL VARIABLES                  */
/**********************************************/
char charReceived;  // polling character from Master
int errorcount = 0; // cumulative error count
int bytecount;  // number of bytes of data sent from Master
byte toggle;    // store state of toggle switch
byte incoming;  // store the 8 bits loaded from the 74HC165

/**********************************************/
/*  SETUP routine runs once                   */
/**********************************************/
void setup() {
  // start the serial port to Serial Monitor
  Serial.begin(9600); // set the data rate
  delay(100); // allow interface to settle
  Serial.println("Starting Program RS485 Remote V7");

  pinMode(pin13LED, OUTPUT);
  digitalWrite(pin13LED, LOW); // shut LED 13 OFF

  //  turn off the outputs on the output shift register
  turnoff();

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

  // Initialize pins for 74HC595 shift register
  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, OUTPUT);

  // Initialize pins for test-run-stop switch
  pinMode(runtest, INPUT_PULLUP);
  pinMode(runstop, INPUT_PULLUP);

  // Load testdataArray
  testdataArray[0] = 0b00000001;
  testdataArray[1] = 0b00000011;
  testdataArray[2] = 0b00000101;
  testdataArray[3] = 0b00001001;
  testdataArray[4] = 0b00010001;
  testdataArray[5] = 0b00100001;
  testdataArray[6] = 0b01000001;
  testdataArray[7] = 0b10000001;

  // Start the software serial port
  RS485Serial.begin(38400);  // set the data rate
  Serial.println("Starting RS485 Port at 38400 bps");
  delay(50);  //  allow the interface to settle

} // End Setup

/**********************************************/
/*  LOOP routine runs to infinity and beyond  */
/**********************************************/
void loop()
{
  if (digitalRead(runtest) == 0) {
    Serial.println("Loopback Test Mode!");
  }
  // read shift registers and store input data in array
  // then if polled by Master Arduino,
  // send the array to the Master and Serial Monitor,
  // otherwise loop
  digitalWrite(pin13LED, HIGH); // show activity
  Serial.println("\nReading the input shift registers: ");
  //  read input shift registers
  //  Latch the state of the A-H data lines into the 74HC165
  //  by taking the SHLD pin low for 5 microseconds
  digitalWrite(shld_pin, LOW);
  delayMicroseconds(pulsewidth);
  digitalWrite(shld_pin, HIGH);

  for (int i = 0; i < inputregisters; i++)  //  reads each register in turn
  {
    //  repeats for the specified number of shift registers
    inputdataArray[i] = read_shift_regs(); // Read a 74HC165 shift reg
  }

  digitalWrite(pin13LED, LOW); // end shift register activity
  displayinputarray();  //  send data to Serial Monitor

  if (RS485Serial.available())  // check for command from Master
  {
    charReceived = RS485Serial.read();  // read the character from the Master Arduino
    Serial.println("Polled by Master Arduino");
    Serial.print("Character Received = ");
    Serial.println(charReceived);     // send to the Serial Monitor

    switch (charReceived) {

      case 'p': //  Master Arduino requesting data
        delay(10);  //  wait 10 msec before responding
        RS485Serial.write(inputregisters);  // send the byte count
        Serial.print("Number of bytes to send to Master Arduino = ");
        Serial.println(inputregisters); // send to the Serial Monitor
        Serial.println("Data sent to Master Arduino");

        // if runtest switch = test, then load first byte from Master into
        // inputdataArray so it gets sent back to Master
        if (digitalRead(runtest) == 0) {
          for (int i = 0; i < inputregisters; i++)
          {
            inputdataArray[i] = outputdataArray[0];
          }
        }

        for (int i = 0; i < inputregisters; i++)
        {
          RS485Serial.write(inputdataArray[i]);  // send data to the Master
        } // end FOR sending input data

        displayinputarray();     // send to the Serial Monitor
        Serial.println();
        break;  //  ends case=p

      case 'q': //  Master is sending data to Remote
        // wait for byte count from Master
        while (RS485Serial.available() <= 0) {
        } //  end while
        bytecount = RS485Serial.read();  // read the data count
        //  check byte count
        switch (bytecount) {
          case outputregisters: //  make sure bytecount is same as registers

            Serial.print("Number of bytes to read from Master Arduino = ");
            Serial.println(bytecount); // send to the Serial Monitor
            Serial.println("Receiving data from Master Arduino");

            //  Wait for buffer to fill
            while (RS485Serial.available() < bytecount) {
            } //  end while

            //  receive data and load array
            for (int i = 0; i < bytecount; i++) {
              outputdataArray[i] = RS485Serial.read();
            } //  end for
            delay(10);  //  wait 10 msec
            
            // send bytes to 74HC595 output shift registers
            digitalWrite(pin13LED, HIGH); // show shift register activity
            //  for (int j = 0; j < bytecount; j++)
            //  {
              //  shiftOut(dataPin, clockPin, MSBFIRST, outputdataArray[j]);
            //  } //  end for
            //  latch the output shift registers
            //  digitalWrite(latchPin, LOW);
            //  delayMicroseconds(pulsewidth);
            //  digitalWrite(latchPin, HIGH); // transfers register data to outputs
            writedata();  //  write outputdataArray to output shift registers
            digitalWrite(pin13LED, LOW); // end shift register activity
            displayoutputarray(); //  send data to Serial Monitor
            break;

          default:
            delay(10);
            Serial.println("ERROR.  Data count does not = 2");
            error();
            break;  //  end default case
        } //  end switch

      default:  //  all other characters
        delay(10);
        Serial.println("ERROR.  Char received does not = p or q");
        error();
        break;  //  end default case
    } //  end switch
  } // end IF
} // End Loop

/************************************************************************/
/*  Function read_shift_regs triggers the 74HC165 input shift registers */
/*  to load values from its A-H inputs and clocks and returns one byte  */
/*  Called from loop                                                    */
/************************************************************************/
byte read_shift_regs() {
  // Declare an 8 bit local variable to carry each bit value of A-H
  byte the_shifted = 0;

  // Required initial states of these two pins according to the datasheet
  //  timing diagram
  digitalWrite(clk_pin, HIGH);
  digitalWrite(ce_pin, LOW); // Enable the clock

  // Get the A-H values
  the_shifted = shiftIn(data_pin, clk_pin, MSBFIRST);
  digitalWrite(ce_pin, HIGH); // Disable the clock

  return the_shifted;
} //  end read_shift_regs function

/**********************************************/
/*  Function print_byte prints all the 1's and */
/*  0's of a byte                             */
/*  called from loop                          */
/**********************************************/
void print_byte(byte val)
{
  byte i; //  local variable
  for (byte i = 0; i <= 7; i++)
  {
    // Magic bit shift, if you care look up the <<, >>, and & operators
    Serial.print(val >> i & 1, BIN);
  }
  Serial.print("\n"); // Go to the next line
} // end print_byte function

/***********************************************/
/*  Function error tracks and prints the error */
/*  Count                                      */
/***********************************************/
void error()
{
  errorcount++;   //  increment global variable
  Serial.print("Error count = ");
  Serial.print(errorcount);
} //  end error function

/**********************************************************************/
/*  Function turnoff sets all bits in all output registers HIGH       */
/*  Called from loop                                                  */
/**********************************************************************/
void turnoff() {
  byte testdata = 0xFF; //0b11111111 turns off all bits in register
  Serial.println("Turning off all the outputs");
  Serial.println("When Darkness Falls!");

  //  load the array
  for (int i = 0; i < outputregisters; i++) {
    outputdataArray[i] = testdata;
  }

  writedata();  // write the array to the registers
  displayoutputarray(); // send the data to Serial Monitor

} //  end turnoff

/***********************************************************************/
/*  Function writedata writes outputdataArray to the specified number  */
/*  of output registers                                                */
/*  Called from turnoff subroutine and case = "q"                      */
/***********************************************************************/
void writedata() {
  byte i; //  local variable counter for shift register loop
  for (i = 0; i < outputregisters; i++) {
    //  repeats for the specified number of shift registers
    //  shiftout send a byte to output shift register
    shiftOut(dataPin, clockPin, MSBFIRST, outputdataArray[i] );
  } //  end

  //  latch the data
  digitalWrite(latchPin, LOW);
  delayMicroseconds(pulsewidth);
  digitalWrite(latchPin, HIGH); //  latches data on shift registers
} //  end writedata

/**********************************************************************/
/*  Function displayoutputarray sends outputdataArray to the Serial   */
/*  Monitor                                                           */
/*  Called from turnon and turnoff                                    */
/**********************************************************************/
void displayoutputarray() {
  Serial.println("\nThe data sent to the shift registers is: ");
  for (int i = 0; i < outputregisters; i++)
  {
    //  Print out the values being written to the shift register
    Serial.print("Byte  ");
    Serial.print(i);
    Serial.print("     ABCDEFGH : ");
    //  Calls the subroutine that prints ones and zeroes
    //  Print every 1 and 0 that correlates with A through H
    print_byte(outputdataArray[i]); // send to Serial Monitor
  }
} // end displayoutputarray

/**************************************************************************/
/*  Function displayinputarray sends inputdataArray to the Serial Monitor */
/*  Called from turnon and turnoff                                        */
/**************************************************************************/
void displayinputarray() {
  Serial.println("\nThe data received from the shift registers is: ");
  for (int i = 0; i < inputregisters; i++)
  {
    //  Print out the values being written to the shift register
    Serial.print("Byte  ");
    Serial.print(i);
    Serial.print("     ABCDEFGH : ");
    //  Calls the subroutine that prints ones and zeroes
    //  Print every 1 and 0 that correlates with A through H
    print_byte(inputdataArray[i]); // send to Serial Monitor
  }
} // end displayinputarray
