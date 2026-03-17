#include <WiFi.h>
#include <WiFiUdp.h>

char ssid[] = "squid"; //  your network SSID (name)
char pass[] = "securepassword";  
unsigned int localPort = 2390;   
unsigned int remotePort = 2390;   
IPAddress remote_IP(192,168,4,16);

char packetBuffer[255]; //buffer to hold incoming packet
char outputBuffer[1024];

WiFiUDP wifiudp;

void readUDP(){
  int packetlength = wifiudp.parsePacket();
  if(packetlength){
  // Serial.println(packetlength);
    //  remote_IP = wifiudp.remoteIP();
    int len = wifiudp.read(packetBuffer, 255);
    if (len > 0) {
      packetBuffer[len] = '\0'; // Null-terminate the received data to treat as a string
    }
    Serial.print(packetBuffer); // Print the contents

  }
  
}


void setup() {
  Serial.begin(115200);

  WiFi.softAP(ssid, pass);
  wifiudp.begin(localPort);
  // Print IP Address
  // Serial.print("AP IP address: ");
  Serial.println(WiFi.softAPIP());
  // put your setup code here, to run once:


}




void loop() {
  // put your main code here, to run repeatedly:
  readUDP();
}
