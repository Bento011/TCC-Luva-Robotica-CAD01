#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <OSCBundle.h>


// you can find this written on the board of some Arduino Ethernets or shields
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED }; 
// NOTEAlternatively, you can assign a fixed IP to configure your Ethernet shield.
//       IPAddress  ip[] = { 192, 168, 0, 154 };
IPAddress ip(192, 168, 0, 154);
int serverPort = 8500; // Emotiv BCI out port
//Create UDP message object
String OSC_cmd;
float OSC_value;
bool flag_OSC;
EthernetUDP Udp;

void setup(){
  Serial.begin(9600); //9600 for a "normal" Arduino board (Uno for example). 115200 for a Teensy ++2 
  Serial.println("Emotiv BCI OSC test");

  // start the Ethernet connection:
  // NOTEAlternatively, you can assign a fixed IP to configure your Ethernet shield.
  //       Ethernet.begin(mac, ip);
  Ethernet.begin(mac, ip);
  if (Udp.begin(serverPort) == 0) {
    Serial.println("Failed to configure Ethernet using DHCP");
    // no point in carrying on, so do nothing forevermore:
    while(true);
  }
  
  // print your local IP address:
  Serial.print("Arduino IP address");
  for (byte thisByte = 0; thisByte < 4; thisByte++) {
    // print the value of each byte of the IP address:
    Serial.print(Ethernet.localIP()[thisByte], DEC);
    Serial.print(".");
  }

  Serial.println();

  Udp.begin(serverPort);

}

// Programa Central
void loop() {
  OSCMsgReceive();
  
}

void OSCMsgReceive() {
  // Serial.println("OSCMsgReceive");
  flag_OSC = false;
  while (flag_OSC == false){
    int size = Udp.parsePacket();  
    // Serial.println(size);
    if(size > 0) {    
      OSCBundle bundleIN;
      while(size--)
        bundleIN.fill(Udp.read());
      if(!bundleIN.hasError()){
          // Serial.println("foi");
          bundleIN.route("/com", processMC); // Mental_Commands
      }
    }
  }
}

void processMC(OSCMessage &msg, int addrOffset) {  
  Serial.println("processMC");
  if(msg.match("/neutral", addrOffset)) {
    OSC_cmd = "Neutral";
  } else if(msg.match("/push", addrOffset)) {
    OSC_cmd = "Push";
  } else if(msg.match("/pull", addrOffset)) {
    OSC_cmd = "Pull";
  } else if(msg.match("/left", addrOffset)) {
    OSC_cmd = "Left";
  } else if(msg.match("/right", addrOffset)) {
    OSC_cmd = "Right";
  } else if(msg.match("/lift", addrOffset)) {
    OSC_cmd = "lift";
  } else if(msg.match("/drop", addrOffset)) {
    OSC_cmd = "drop";
  } else if(msg.match("/rotateLeft", addrOffset)) {
    OSC_cmd = "rotateLeft";
  } else if(msg.match("/rotateRight", addrOffset)) {
    OSC_cmd = "rotateRight";
  } else if(msg.match("/rotateClockwise", addrOffset)) {
    OSC_cmd = "rotateClockwise";
  } else if(msg.match("/rotateCounterClockwise", addrOffset)) {
    OSC_cmd = "rotateCounterClockwise";
  } else if(msg.match("/rotateForwards", addrOffset)) {
    OSC_cmd = "rotateForwards";
  } else if(msg.match("/rotateReverse", addrOffset)) {
    OSC_cmd = "rotateReverse";
  } else if(msg.match("/disappear", addrOffset)) {
    OSC_cmd = "disappear";
  }

  if(msg.isFloat(0)) {
    OSC_value = msg.getFloat(0);
  }
  Serial.println(OSC_cmd + ": " + OSC_value);
  flag_OSC = true;
}