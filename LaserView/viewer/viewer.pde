/*
  Graph
 
 A simple example of communication from the Arduino board to the computer:
 the value of analog input 0 is sent out the serial port.  We call this "serial"
 communication because the connection appears to both the Arduino and the
 computer as a serial port, even though it may actually use
 a USB cable. Bytes are sent one after another (serially) from the Arduino
 to the computer.
 
 You can use the Arduino serial monitor to view the sent data, or it can
 be read by Processing, PD, Max/MSP, or any other program capable of reading 
 data from a serial port.  The Processing code below graphs the data received 
 so you can see the value of the analog input changing over time.
 
 The circuit:
 Any analog input sensor is attached to analog in pin 0.
  
 created 2006
 by David A. Mellis
 modified 14 Apr 2009
 by Tom Igoe and Scott Fitzgerald
 
 This example code is in the public domain.

 http://www.arduino.cc/en/Tutorial/Graph
 */
/*
void setup() {
  // initialize the serial communication:
  Serial.begin(9600);
}

void loop() {
  // send the value of analog input 0:
  Serial.println(analogRead(A0));
  // wait a bit for the analog-to-digital converter 
  // to stabilize after the last reading:
  delay(10);
}
*/
// Processing code for this example
 
 // Graphing sketch
 
 
 // This program takes ASCII-encoded strings
 // from the serial port at 9600 baud and graphs them. It expects values in the
 // range 0 to 1023, followed by a newline, or newline and carriage return
 
 // Created 20 Apr 2005
 // Updated 18 Jan 2008
 // by Tom Igoe
 // This example code is in the public domain.
 
 import processing.serial.*;
 
 Serial myPort;        // The serial port
 int xPos = 2;         // horizontal position of the graph
 float prevY = 0;
 float K = 0.125;
 
 void setup () {
 
 // set the window size:
 size(800, 600);        
 //textFont(loadFont("Arial-Black-48.vlw"));
 // List all the available serial ports
 println(Serial.list());
 // I know that the first port in the serial list on my mac
 // is always my  Arduino, so I open Serial.list()[0].
 // Open whatever port is the one you're using.
 myPort = new Serial(this, Serial.list()[1], 115200);
 // don't generate a serialEvent() unless you get a newline character:
 myPort.bufferUntil('\n');
 // set inital background:
 background(1);
 }
 void draw () {
 // everything happens in the serialEvent()
   fill(0xFFFFFFFF);
   text(K, 200, 60);
   if(keyPressed){
     if(key == 'p' || key == 'P'){
       myPort.write('p');
       fill(0x00000000);
       text(K, 200, 60);
       K += 0.01;
       fill(0xFFFFFFFF);
       text(K, 200, 60);
       delay(100);
     }
     else if(key == 'l' || key == 'L'){
       myPort.write('l');
       fill(0x00000000);
       text(K, 200, 60);
       K -= 0.01;
       fill(0xFFFFFFFF);
       text(K, 200, 60);
       delay(100);

     }
   }
   
 }
 
void serialEvent (Serial myPort) {
  // get the ASCII string:
  String inString = myPort.readStringUntil('\n');

  if (inString != null) {
    // trim off any whitespace:
    inString = trim(inString);
    String[] ang_range = split(inString,',');
    // convert to an int and map to the screen height:
    if(ang_range.length > 1){
      float angle = float(ang_range[0]); 
      float range = float(ang_range[1])/10; 
      if(angle == 0){
        background(1);
      }
      if(angle == 400){
        print("Ang_vel: ");
        println(range);
      }
      if(angle == 500){
        print("Lin_vel: ");
        println(range);
      }
 
      // draw the line:
      stroke(127,34,255);
      ellipse(range*cos(radians(angle)) + 300, range*sin(radians(angle)) + 300,5,5);
      stroke(0xFF0000);
      ellipse(300,300,5,5);

    }
  }
}
 

