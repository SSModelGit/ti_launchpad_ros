/*
  WiFi UDP Send and Receive String

 This sketch wait an UDP packet on localPort using a WiFi shield.

 created 30 December 2012
 by dlf (Metodo2 srl)
 mod by Shashank Swaminathan on 11 April 2018

 */

#define unused0 0
#define unused1 1
#define CLAWOPEN 2
#define unused3 3
#define unused4 4
#define CLAWCLOSE 5
#define YAW 6
#define TELESCOPE 7
#define CLAWUP 8
#define CLAWDOWN 9
#define SPEEDSCALE 10
#define MODE 11
#define MAXBUF 255

#define SLIDER RED_LED
#define BASE GREEN_LED
#define DROPPER YELLOW_LED
#define GRABBER 31

#include <WiFi.h>
#include <WiFiUdp.h>
#include <Servo.h>

int status = WL_IDLE_STATUS;
IPAddress ip(192, 168, 0, 119);
char ssid[] = "ssid"; //  your network SSID (name)
char pass[] = "WPApass";    // your network password (use for WPA, or use as key for WEP)

unsigned int localPort = 2391;      // local port to listen on
char packetBuffer[MAXBUF]; //buffer to hold incoming packet
int leftVal, rightVal;

Servo telescope, yaw, drop, claw;

WiFiUDP Udp;

void setup() {
  //Setup pins
  // pinMode(RED_LED, OUTPUT); digitalWrite(RED_LED, LOW);
  // pinMode(GREEN_LED, OUTPUT); digitalWrite(GREEN_LED, LOW);
  // pinMode(YELLOW_LED, OUTPUT); digitalWrite(YELLOW_LED, LOW);

  telescope.attach(SLIDER);
  yaw.attach(BASE);
  drop.attach(DROPPER);
  claw.attach(GRABBER);
  
  //Initialize serial and wait for port to open:
  Serial.begin(115200);

  WiFi.config(ip);
  // attempt to connect to Wifi network:
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);

    /* int i = 0;
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
    } */
    delay(900); // remove the LED feedback - its purpose is to only serve as a debugger
  }
  Serial.println("Connected to wifi");
  printWifiStatus();
  // flash();

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
    driveArm();
  }
  // Behavior list:
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
  
  // Drive the yawing of the base of the arm
  int yawVal = (int) (180.0/255.0 * (float) packetBuffer[YAW]);
  yaw.write(yawVal); // pressing the left of the crosspad makes the servo spin counterclockwise, the right does clockwise
  Serial.print(yawVal); Serial.print(" | ");

  // Drive the telescoping of the arm
  int telescopeVal = (int) (140.0/255.0 * (float) packetBuffer[TELESCOPE]);
  telescope.write(telescopeVal); // pressing up on the crosspad causes counterclockwise servo rotation, down causes clockwise
  Serial.print(telescopeVal); Serial.print(" | ");

  // Drive the claw's dropping and lifting motion
  int clawVertVal = 70 + (35 * (int) packetBuffer[CLAWDOWN]) -  (35 * (int) packetBuffer[CLAWUP]); // currently set such that counterclockwise lowers the claw, and clockwise raises it
  drop.write(clawVertVal);
  Serial.print(clawVertVal); Serial.print(" | ");

  // Drive the claw grabbing motion
  int grabVal = 70 + (10 * (int) packetBuffer[CLAWOPEN]) -  (10 * (int) packetBuffer[CLAWCLOSE]); // currently set such that counterclockwise servo rotation opens the claw, and clockwise closes it
  claw.write(grabVal);
  Serial.println(grabVal);
}

// meant for debugger purposes only
/* void flash() {
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
} */

