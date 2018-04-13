/*
  WiFi UDP Send and Receive String

 This sketch wait an UDP packet on localPort using a WiFi shield.

 created 30 December 2012
 by dlf (Metodo2 srl)
 mod by Shashank Swaminathan on 11 April 2018

 */


// #include <SPI.h>
#include <WiFi.h>
#include <WiFiUdp.h>

int status = WL_IDLE_STATUS;
IPAddress ip(192, 168, 0, 117);
char ssid[] = "yournetwork"; //  your network SSID (name)
char pass[] = "networkpwd";    // your network password (use for WPA, or use as key for WEP)

unsigned int localPort = 2391;      // local port to listen on
char packetBuffer[255]; //buffer to hold incoming packet
char  ReplyBuffer[] = "acknowledged";       // a string to send back

WiFiUDP Udp;

void setup() {
  //Initialize serial and wait for port to open:
  Serial.begin(115200);

  WiFi.config(ip);
  // attempt to connect to Wifi network:
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    delay(10000);
  }
  Serial.println("Connected to wifi");
  printWifiStatus();

  Serial.println("\nStarting connection to server...");
  // if you get a connection, report back via serial:
  Udp.begin(localPort);
}

void loop() {
  int packetSize = Udp.parsePacket(); // check if there is anything on the que
  if (packetSize) { // there is something on the que
    // read from que
    getPacket(packetSize);
  }
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

