
#include "XL320.h"
#include <HalfDuplexHardwareSerial.h>

//alusta servo
XL320 jaw;

//muuttujia asennon lukua varten
int par1, par2, pos;  

// Viive pakettien välillä
int delayBetweenSendReceive = 500;

// Servon ID
int servoID = 1;

void setup() {

  //kytkentä: laita servon datapinni koekytkentälevylle, koekytkentälevyltä sitten 2 piuhaa arduinon pinneihin 0 ja 1, maapinni GND:hen ja VCC-pinni 5V:hen
  //huom! ota piuhat 0 ja 1 irti arduinosta kun lataat koodin arduinolle, ja laita vasta sitten kiinni kun koodi on täysin uploadattu, muuten tulee erroria synkronoinnin kanssa
  pinMode(0, INPUT_PULLUP);
  pinMode(1, INPUT_PULLUP);
  
  pinMode(LED_BUILTIN, OUTPUT);

  //Käytä HalfDuplexSerialia Serialin sijasta, toimii ohjelmistoversiona tri-state-bufferista
  HalfDuplexSerial.begin(1000000);
  HalfDuplexSerial.setTimeout(delayBetweenSendReceive);

  // alusta oma servo
  jaw.begin(HalfDuplexSerial);
  
  //aseta joint speed
  jaw.setJointSpeed(servoID, 1023);
  delay(delayBetweenSendReceive);

}

void loop() {

  //kun haluat lukea, laita kirjoitusosa kommenttiin
  //lukuosa alkaa tästä
  jaw.getJointPosition(servoID);
  
  byte buffer[256];
  XL320::Packet p = XL320::Packet(buffer, jaw.readPacket(buffer,256));
  if (p.isValid() && p.getParameterCount()>=3) {
    //lue servon nykyinen asento parametreista 1 ja 2 (0 on virhetietoa varten)
    par1 = p.getParameter(1);
    par2 = p.getParameter(2);
    pos = (par1)|(par2<<8);
    HalfDuplexSerial.println(pos);     //printtaa rivin esim. "⸮⸮⸮  %  -⸮⸮⸮⸮  U⸮⸮⸮⸮902", missä 902 on luettu asento ja alkuosasta ei tarvitse välittää, ota luettu asento talteen!
    digitalWrite(LED_BUILTIN, HIGH);   //jos data on ok, laita led L päälle merkiksi
  } else {
    digitalWrite(LED_BUILTIN, LOW);
  }
  delay(delayBetweenSendReceive);
  //lukuosa loppuu tähän

  //kun luet dataa, älä kirjoita samaan aikaan, ja sama toisin päin: muuten homma takkuaa ja pahasti
  //kun haluat kirjoittaa, ota nämä pois kommentista (ja aina kun kirjoitat asennon laita perään viivettä ennen uutta kirjoitusta!) ja laita lukuosa kommenttiin
  
  //kirjoitusosa alkaa tästä
  /*
  jaw.moveJoint(servoID, 900);     //900 esimerkkiasento, korvaa omalla halutulla asennolla
  delay(delayBetweenSendReceive);

  jaw.moveJoint(servoID, 990);     //990 esimerkkiasento, korvaa omalla halutulla asennolla
  delay(delayBetweenSendReceive);

  */
  //kirjoitusosa loppuu tähän
}
