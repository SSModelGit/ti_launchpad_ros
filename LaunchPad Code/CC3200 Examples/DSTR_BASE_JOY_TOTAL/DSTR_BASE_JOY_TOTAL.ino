/*
  WiFi UDP Send and Receive String

 This sketch wait an UDP packet on localPort using a WiFi shield.

 created 30 December 2012
 by dlf (Metodo2 srl)
 mod by Shashank Swaminathan on 11 April 2018

 */

#define STEERING 0
#define THROTTLE 1
#define unused2 2
#define unused3 3
#define unused4 4
#define unused5 5
#define unused6 6
#define unused7 7

// #include <SPI.h>
#include <WiFi.h>
#include <WiFiUdp.h>

int status = WL_IDLE_STATUS;
IPAddress ip(192, 168, 0, 118);
char ssid[] = "Maker"; //  your network SSID (name)
char pass[] = "3900BotClub";    // your network password (use for WPA, or use as key for WEP)

unsigned int localPort = 2391;      // local port to listen on
char packetBuffer[255]; //buffer to hold incoming packet
int leftVal, rightVal;

WiFiUDP Udp;

void setup() {
  //Setup pins
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(YELLOW_LED, OUTPUT);
  pinMode(31, OUTPUT);
  
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

  Serial.println("\nStarting new connection to server...");
  // if you get a connection, report back via serial:
  Udp.begin(localPort);
}

void loop() {

  // read from que
  int packetSize = Udp.parsePacket();
  if (packetSize) {
    getPacket(packetSize);
    printPacketInfo();
    Serial.println("PWM SET");
    joyToTank(packetBuffer[STEERING], packetBuffer[THROTTLE]);
    rightWheelWrite();
    leftWheelWrite();
  }
  // Behavior list:
  // Left Joystick - up dims red LED, down does nothing
  // Right Joystick - up dims yellow LED, down dims green LED
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
  int len = Udp.read(packetBuffer, 255);
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

void joyToTank(int steering, int throttle) {
  leftVal = min(max(throttle - steering + 128, 0), 255);
  rightVal = min(max(throttle - 127 + steering, 0), 255);
}

void rightWheelWrite() {
  if(rightVal >= 127) { // original was 0xbb
    Serial.println("BB");
    motorWrite(RED_LED, rightVal); //If byte 2 was 0xbb then this writes the speed from byte 3 to the pin for RED_LED
    analogWrite(31, 255);
  }
  
  if(rightVal < 127) { // original was 0xaa
    Serial.println("AA");
    motorWrite(31, rightVal); //If byte 2 was 0xaa then this writes the speed from byte 3 to pin 31
    analogWrite(RED_LED, 255);
  }
}

void leftWheelWrite() {
  if(leftVal >= 127) { //This is checking the hex value of byte 0 for the direction
    motorWrite(GREEN_LED, leftVal); //If byte 0 was 0xbb then this writes the speed from byte 1 to the pin for GREEN_LED
    analogWrite(YELLOW_LED, 255);             
  }
  
  if(leftVal < 127) {
    motorWrite(YELLOW_LED, leftVal); //If byte 0 was 0xaa then this writes the speed from btye 1 to the pin for YELLOW_LED
    analogWrite(GREEN_LED, 255);
  }
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

