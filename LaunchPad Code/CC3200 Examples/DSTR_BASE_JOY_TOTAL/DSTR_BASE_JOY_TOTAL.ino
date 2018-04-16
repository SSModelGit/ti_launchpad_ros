/*
  WiFi UDP Send and Receive String

 This sketch wait an UDP packet on localPort using a WiFi shield.

 created 30 December 2012
 by dlf (Metodo2 srl)
 mod by Shashank Swaminathan on 11 April 2018

 */

#define STEERING 0
#define THROTTLE 1
#define LEFT 1
#define unused2 2
#define unused3 3
#define RIGHT 4
#define unused5 5
#define unused6 6
#define unused7 7
#define unused8 8
#define unused9 9
#define SPEEDSCALE 10
#define MODE 11
#define MAXBUF 255

#define IN1 RED_LED
#define IN2 31
#define IN3 GREEN_LED
#define IN4 YELLOW_LED

// #include <SPI.h>
#include <WiFi.h>
#include <WiFiUdp.h>

int status = WL_IDLE_STATUS;
IPAddress ip(192, 168, 0, 118);
char ssid[] = "ssid"; //  your network SSID (name)
char pass[] = "WPApass";    // your network password (use for WPA, or use as key for WEP)

unsigned int localPort = 2391;      // local port to listen on
char packetBuffer[MAXBUF]; //buffer to hold incoming packet
int leftVal, rightVal;

WiFiUDP Udp;

void setup() {
  //Setup pins
  // pinMode(RED_LED, OUTPUT); digitalWrite(RED_LED, LOW);
  // pinMode(GREEN_LED, OUTPUT); digitalWrite(GREEN_LED, LOW);
  // pinMode(YELLOW_LED, OUTPUT); digitalWrite(YELLOW_LED, LOW);

  pinMode(IN1, OUTPUT); digitalWrite(IN1, LOW);
  pinMode(IN2, OUTPUT); digitalWrite(IN2, LOW);
  pinMode(IN3, OUTPUT); digitalWrite(IN3, LOW);
  pinMode(IN4, OUTPUT); digitalWrite(IN4, LOW);
  
  //Initialize serial and wait for port to open:
  Serial.begin(115200);

  WiFi.config(ip);
  // attempt to connect to Wifi network:
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);

    int i = 0;
    for(int j = 0; j < 6; j++) {
      switch(i) {
        case 1:
          digitalWrite(RED_LED, LOW);
          digitalWrite(YELLOW_LED, HIGH);
          digitalWrite(GREEN_LED, LOW);
          delay(150);
          i++;
          break;
        case 2:
          digitalWrite(RED_LED, LOW);
          digitalWrite(YELLOW_LED, LOW);
          digitalWrite(GREEN_LED, HIGH);
          delay(150);
          i++;
          break;
        default:
          digitalWrite(RED_LED, HIGH);
          digitalWrite(YELLOW_LED, LOW);
          digitalWrite(GREEN_LED, LOW);
          delay(150);
          i=1;
          break;
      }
    }
  }
  Serial.println("Connected to wifi");
  printWifiStatus();
  flash();

  Serial.println("\nStarting connection to server...");
  // if you get a connection, report back via serial:
  Udp.begin(localPort);
}

void loop() {

  // read from que
  int packetSize = Udp.parsePacket();
  if (packetSize) {
    getPacket(packetSize);
    printPacketInfo();
    drive(packetBuffer[MODE]);
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
  Serial.println(packetBuffer[7], DEC);
  Serial.print("\n");
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

void flash() {
  digitalWrite(RED_LED, HIGH);
  digitalWrite(GREEN_LED, HIGH);
  digitalWrite(YELLOW_LED, HIGH);
  delay(100);
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(YELLOW_LED, LOW);
  delay(100);
  digitalWrite(RED_LED, HIGH);
  digitalWrite(GREEN_LED, HIGH);
  digitalWrite(YELLOW_LED, HIGH);
  delay(100);
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(YELLOW_LED, LOW);
  delay(100);
  digitalWrite(RED_LED, HIGH);
  digitalWrite(GREEN_LED, HIGH);
  digitalWrite(YELLOW_LED, HIGH);
  delay(100);
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(YELLOW_LED, LOW);
  delay(100);
  digitalWrite(RED_LED, HIGH);
  digitalWrite(GREEN_LED, HIGH);
  digitalWrite(YELLOW_LED, HIGH);
  delay(100);
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(YELLOW_LED, LOW);
  delay(100);
  digitalWrite(RED_LED, HIGH);
  digitalWrite(GREEN_LED, HIGH);
  digitalWrite(YELLOW_LED, HIGH);
  delay(100);
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(YELLOW_LED, LOW);
  delay(100);
  digitalWrite(RED_LED, HIGH);
  digitalWrite(GREEN_LED, HIGH);
  digitalWrite(YELLOW_LED, HIGH);
  delay(100);
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(YELLOW_LED, LOW);
  delay(100);
  digitalWrite(RED_LED, HIGH);
  digitalWrite(GREEN_LED, HIGH);
  digitalWrite(YELLOW_LED, HIGH);
  delay(100);
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(YELLOW_LED, LOW);
  delay(100);
}

