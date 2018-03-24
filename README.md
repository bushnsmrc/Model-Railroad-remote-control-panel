# Model-Railroad-remote-control-panel
Control Panel for Hell Yard based on Arduino

The North Shore Model Railroad Club (NSMRC.org) has a six-track hidden storage yard (Hell Yard) located about 30 feet from the operator's location.  It is complex, including the six storage tracks on a reversing loop and two each entrance and exit tracks.  With a dozen electrical track blocks, 14 switch machines, DCC reversers, and various occupancy sensors, there is a lot of wiring involved - multiple 25-pair phone cables.  Since I was already interested in experimenting with Arduino, I volunteered to integrate Arduinos, communications devices, I/O devices, and relays to make it all happen with a lot less wiring.  I also want to make the designs re-useable to the extent possible, because many other control panels (it is a very large layout) may benefit from the same treatment.

So far (March 2018), I have used shift registers (74HC165 and 74HC595) for I/O devices and RS485 for communications.  There is a master Arduino Uno at the control panel and a remote Arduino at the hidden yard.  There are 64 bits going from Hell Yard to the control panel and 16 bits going down to Hell Yard.  Switches, relays, switch machine position indicators, occupancy sensors, LEDs, DCC status, and RS485 have all been integrated and sort of work.  Switch machines controls have not been prototyped yet - my plan is to use a SX1509 I2C device to read the pushbuttons.

I also want to try using XBee series 1 devices to eliminate cabling entirely.

I have code for the master and remote Arduinos.  Still requires some work.  I have tried to make the labels and functions consistent between the two programs.
In operation, the master alternates between uploading data from the remote and downloading data to the remote.
- The master pings the remote and waits for a response.  The remote responds with the number of bytes followed by the bytes.  The master stores the bytes in an array that is then shifted into the bank of 74HC595 shift registers (which drive LEDs).
- The master reads the bank of 74HC165 shift registers and then pings the remote a second time (with a different character) and follows that with the number of bytes followed by the bytes.
- The master has a local test mode that reads one of the input shift registers and sneds the data to the output shift registers.
- The remote will have a loopback test mode the will take one byte sent from the master and send it back to the master.
