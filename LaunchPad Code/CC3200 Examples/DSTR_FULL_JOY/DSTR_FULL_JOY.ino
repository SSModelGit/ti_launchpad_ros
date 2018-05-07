/*
Name: Shashank Swaminathan 
Date: 25 April 2018
Team Name: GRU
Purpose: Receive a UDP string via an in-built WiFi shield on the CC3200 LaunchPad; use the values to drive a DSTR robot with a three dimensional manipulator.
*/

// channel names
#define STEERING 0
#define THROTTLE 1
#define LEFT 1
#define CLAWOPEN 2
#define unused3 3
#define RIGHT 4
#define CLAWCLOSE 5
#define YAW 6
#define TELESCOPE 7
#define CLAWUP 8
#define CLAWDOWN 9
#define SPEEDSCALE 10
#define MODE 11
#define MAXBUF 255

// motor pins
#define IN1 RED_LED
#define IN2 31
#define IN3 GREEN_LED
#define IN4 YELLOW_LED

// servo pins (can be digital, does not require analog)
#define SLIDER 5
#define BASE 8
#define DROPPER 27
#define GRABBER 28

// #include <SPI.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Servo.h>

char ssid[] = "GRU"; //  your network SSID (name)
char pass[] = "iamgruuu";    // your network password (use for WPA, or use as key for WEP)

unsigned int localPort = 2391;      // local port to listen on
char packetBuffer[MAXBUF]; //buffer to hold incoming packet
int leftVal, rightVal;

Servo telescope, yaw, drop, claw;
unsigned long timer, lastTime;
int yawVal, telescopeVal, clawVertVal, grabVal;
// int yawIncrement = 10; int grabIncrement = 10; int interval = 5; // time between each update of servo cycle, in milliseconds
int yawIncrement = 10; int yawTracker = 0; // time between each update of servo cycle, in milliseconds

WiFiUDP Udp;

void setup() {
  //Set up motor pins
  pinMode(IN1, OUTPUT); digitalWrite(IN1, LOW);
  pinMode(IN2, OUTPUT); digitalWrite(IN2, LOW);
  pinMode(IN3, OUTPUT); digitalWrite(IN3, LOW);
  pinMode(IN4, OUTPUT); digitalWrite(IN4, LOW);

  // attach servo pins
  telescope.attach(SLIDER); telescopeVal = 69; telescope.write(telescopeVal);
  yaw.attach(BASE); yawVal = 90; yaw.write(yawVal);
  drop.attach(DROPPER); clawVertVal = 69; drop.write(clawVertVal);
  claw.attach(GRABBER); grabVal = 0; claw.write(grabVal);
  
  //Initialize serial and wait for port to open:
  Serial.begin(250000);

  Serial.print("Starting network...");
  WiFi.beginNetwork(ssid, pass);
  Serial.println("done.");
  
  printWifiStatus();

  Serial.println("\nStarting connection to server...");
  // if you get a connection, report back via serial:
  Udp.begin(localPort);

  // lastTime = millis();
}

void loop() {

  // read from que
  int packetSize = Udp.parsePacket();
  if (packetSize) {
    getPacket(packetSize);
    printPacketInfo();
    drive(packetBuffer[MODE]);
    driveArm();
  }
  // Behavior list:
  // Only when IN1 = RED_LED, IN2 = 31, IN3 = GREEN_LED, IN4 = YELLOW_LED
  // TANK BEHAVIOR
  // Left Joystick - up dims red LED, down does nothing
  // Right Joystick - up dims yellow LED, down dims green LED
  
  // JOY BEHAVIOR
  // Left Joystick - up dims yellow LED and red LED, down dims green LED
  //                 right dims green LED and red LED, left dims yellow LED
  // Right Joystick - nothing
  
  // CROSSPAD - UP telescopes out, DOWN telescopes in, LEFT rotates left, RIGHT rotates right
  // LB - Raises CLAW
  // RB - Lowers CLAW
  // LT - Opens CLAW
  // RT - Closes CLAW
}

void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

void getPacket(int pSize) {
  Serial.print("Received packet of size ");
  Serial.println(pSize);
  Serial.print("From "); 
  IPAddress remoteIp = Udp.remoteIP();
  IPAddress localIp = WiFi.localIP();
  Serial.print(remoteIp);
  Serial.print(", port ");
  Serial.print(Udp.remotePort());
  Serial.print(", to ");
  Serial.println(localIp);
  // read the packet into packetBufffer
  int len = Udp.read(packetBuffer, MAXBUF);
  if (len > 0) packetBuffer[len] = 0;
}

void printPacketInfo() {
  Serial.println("Contents:");
  Serial.println(packetBuffer);
  Serial.print(packetBuffer[0], DEC);
  Serial.print(" | ");
  Serial.print(packetBuffer[1], DEC);
  Serial.print(" | ");
  Serial.print(packetBuffer[2], DEC);
  Serial.print(" | ");
  Serial.print(packetBuffer[3], DEC);
  Serial.print(" | ");
  Serial.print(packetBuffer[4], DEC);
  Serial.print(" | ");
  Serial.print(packetBuffer[5], DEC);
  Serial.print(" | ");
  Serial.print(packetBuffer[6], DEC);
  Serial.print(" | ");
  Serial.print(packetBuffer[7], DEC);
  Serial.print(" | ");
  Serial.print(packetBuffer[8], DEC);
  Serial.print(" | ");
  Serial.print(packetBuffer[9], DEC);
  Serial.print(" | ");
  Serial.print(packetBuffer[10], DEC);
  Serial.print(" | ");
  Serial.println(packetBuffer[11], DEC);
  Serial.print("\n");
}

void driveArm() {
  // follows crane setup - rotation of base, horizontal telescoping of arm, dropping and lifting of the grabber, and grabbing motions at the end actuator
  
  // Drive the non-continuous servos - yawing of the base of the arm, grabbing of claw
  /* timer = millis();
  int compGrabVal = (int) packetBuffer[CLAWOPEN] - (int) packetBuffer[CLAWCLOSE];
  if ((timer - lastTime - interval) > 0) {
    yawVal = max(min(yawVal + (int) ((( (float) packetBuffer[YAW] / 127.0) - 1.0) * yawIncrement), 180), 0);
    grabVal = max(min(grabVal +  compGrabVal * grabIncrement, 180), 0);
    lastTime = timer;
    yaw.write(yawVal); // pressing up on the crosspad causes counterclockwise servo rotation, down causes clockwise
    claw.write(grabVal);
    Serial.print(yawVal); Serial.print(" | ");
  } */

  if((int) packetBuffer[YAW] != yawTracker) {
    if( (int) packetBuffer[YAW] > 129 || (int) packetBuffer[YAW] < 125) {
      yawVal = max(min(yawVal + (int) ((( (float) packetBuffer[YAW] / 127.0) - 1.0) * yawIncrement), 180), 0);
      yaw.write(yawVal);
    }
    yawTracker = (int) packetBuffer[YAW];
  }
  Serial.print(yawVal); Serial.print(" | ");

  // Drive the telescoping of the arm
  telescopeVal = (int) (138.0/255.0 * (float) packetBuffer[TELESCOPE]);
  telescope.write(telescopeVal);
  Serial.print(telescopeVal); Serial.print(" | ");

  // Drive the claw's dropping and lifting motion
  clawVertVal = max(min(69 + (35 * (int) packetBuffer[CLAWDOWN]) -  (35 * (int) packetBuffer[CLAWUP]), 180), 0); // currently set such that counterclockwise lowers the claw, and clockwise raises it
  drop.write(clawVertVal);
  Serial.print(clawVertVal); Serial.print(" | ");

  // Print the claw's grabbing servo value
  grabVal = 90 + (45 * (int) packetBuffer[CLAWOPEN]) -  (45 * (int) packetBuffer[CLAWCLOSE]);
  claw.write(grabVal);
  Serial.println(grabVal);
}

void drive(int opMode) {
  if (opMode == 2) {
    joyToTank(packetBuffer[STEERING], packetBuffer[THROTTLE]);
    rightWheelWrite(rightVal);
    leftWheelWrite(leftVal);
  }
  else {
    rightWheelWrite(packetBuffer[RIGHT]);
    leftWheelWrite(packetBuffer[LEFT]);
  }
}

void joyToTank(int steering, int throttle) {
  leftVal = min(max(throttle - steering + 128, 0), 255);
  rightVal = min(max(throttle - 127 + steering, 0), 255);
}

void rightWheelWrite(int rVal) {
  if(rVal >= 127) { // original was 0xbb
    motorWrite(IN1, rVal); //If byte 2 was 0xbb then this writes the speed from byte 3 to the pin for RED_LED
    analogWrite(IN2, 255);
  }
  
  if(rVal < 127) { // original was 0xaa
    motorWrite(IN2, rVal); //If byte 2 was 0xaa then this writes the speed from byte 3 to pin 31
    analogWrite(IN1, 255);
  }
  Serial.print("RIGHT WHEEL | ");
  Serial.println(rVal);
}

void leftWheelWrite(int lVal) {
  if(lVal >= 127) { //This is checking the hex value of byte 0 for the direction
    motorWrite(IN3, lVal); //If byte 0 was 0xbb then this writes the speed from byte 1 to the pin for GREEN_LED
    analogWrite(IN4, 255);             
  }
  
  if(lVal < 127) {
    motorWrite(IN4, lVal); //If byte 0 was 0xaa then this writes the speed from btye 1 to the pin for YELLOW_LED
    analogWrite(IN3, 255);
  }
  Serial.print("LEFT WHEEL | ");
  Serial.println(lVal);
}

void motorWrite(int motorPin, int output) {
  Serial.print(output);
  Serial.print(" | ");
  int adjustedOut;
  if (output >= 127) {
    adjustedOut = min(max(510 - 2 * output, 0), 255);
  }
  else {
    adjustedOut = min(max( (int) (2.03 * (float) output), 0), 255);
  }
  Serial.println(adjustedOut);
  analogWrite(motorPin, adjustedOut);
}

