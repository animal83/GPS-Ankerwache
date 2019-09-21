#include <Arduino.h>
#include <U8x8lib.h>
#include <TimerOne.h>
#include <SoftwareSerial.h>


#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif

U8X8_SSD1306_128X64_NONAME_SW_I2C u8x8(/* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE);

SoftwareSerial GPSSerial(14,15);

String UPDATE_10_sec= "$PMTK220,5000*1B\r\n";
String ONLY = "$PMTK314,0,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*2A\r\n";


#define pinALARM 8
#define pinENTER 7
#define pinUP 6
#define pinDOWN 5



uint8_t flag=0;  // Für ender des NMEA Stings
char c; // Einzelzeichen lesen des NMEA Sting übertragung 
String NMEAtmp=""; //Zeit weise übertragung 
String nmea[15]; // Unterteilter NEMA String in einzel Strings


uint8_t toleranz = 5;  // Tollerranz weter o bis 50
uint16_t distanz = 0; // Distanzwert zum Ankerpunkt
float distanzNow = 0; // Aktuellberchneter Abstand zum Nanker
int8_t menupunkt = 0; // Anzeige Menu Punkt  
bool norelooper = 0; // Display wir nur Autualiesiert wenn Enderungen eingetreten sind
bool buttonlock = 0; // keine 2 Taten aufeinaml

float latitude1 = 0; // Breitengrad Ankerpunkt
float longitude1 = 0; // Lengengrad Ankerpunkt
float latitude2 = 0; // Breitengrad Messpunkt
float longitude2 = 0; // Lengengrad Messpunkt


void readGPS(){
  
  if(GPSSerial.available()>0){
    c=GPSSerial.read();
    NMEAtmp.concat(c);
  }
 
  if(c=='\r'){
    flag=1;
  }
}

void SplitNMEAStr(String tempMsg){  // NMEA String NMEAtmp wir unterteilt in supstrings

  int pos = 0;
  int stringplace = 0;
  for (int i = 0; i < tempMsg.length(); i++) {
      if (tempMsg.substring(i, i + 1) == ",") {
        nmea[pos] = tempMsg.substring(stringplace, i);
        stringplace = i + 1;
        pos++;
      }
      if (i == tempMsg.length() - 1) {
        nmea[pos] = tempMsg.substring(stringplace, i);
      }
    }
}

String ConvertLat() {
  String posneg = "";
  if (nmea[4] == "S") {
    posneg = "-";
  }
  String latfirst;
  float latsecond;
  for (int i = 0; i < nmea[3].length(); i++) {
    if (nmea[3].substring(i, i + 1) == ".") {
      latfirst = nmea[3].substring(0, i - 2);
      latsecond = nmea[3].substring(i - 2).toFloat();
    }
  }
  latsecond = latsecond / 60;
  String CalcLat = "";

  char charVal[9];
  dtostrf(latsecond, 4, 6, charVal);
  for (int i = 0; i < sizeof(charVal); i++)
  {
    CalcLat += charVal[i];
  }
  latfirst += CalcLat.substring(1);
  latfirst = posneg += latfirst;
  return latfirst;
}

String ConvertLng() {
  String posneg = "";
  if (nmea[6] == "W") {
    posneg = "-";
  }

  String lngfirst;
  float lngsecond;
  for (int i = 0; i < nmea[5].length(); i++) {
    if (nmea[5].substring(i, i + 1) == ".") {
      lngfirst = nmea[5].substring(0, i - 2);
      //Serial.println(lngfirst);
      lngsecond = nmea[5].substring(i - 2).toFloat();
      //Serial.println(lngsecond);

    }
  }
  lngsecond = lngsecond / 60;
  String CalcLng = "";
  char charVal[9];
  dtostrf(lngsecond, 4, 6, charVal);
  for (int i = 0; i < sizeof(charVal); i++)
  {
    CalcLng += charVal[i];
  }
  lngfirst += CalcLng.substring(1);
  lngfirst = posneg += lngfirst;
  return lngfirst;
}

float distance_between (float lat1, float long1, float lat2, float long2){ // berechnet die Distanz zwischen 2 GPS punkten 
  // returns distance in meters between two positions, both specified
  // as signed decimal-degrees latitude and longitude. Uses great-circle
  // distance computation for hypothetical sphere of radius 6372795 meters.
  // Because Earth is no exact sphere, rounding errors may be up to 0.5%.
  // Courtesy of Maarten Lamers
  float delta = radians(long1 - long2);
  float sdlong = sin(delta);
  float cdlong = cos(delta);
  lat1 = radians(lat1);
  lat2 = radians(lat2);
  float slat1 = sin(lat1);
  float clat1 = cos(lat1);
  float slat2 = sin(lat2);
  float clat2 = cos(lat2);
  delta = (clat1 * slat2) - (slat1 * clat2 * cdlong);
  delta = sq(delta);
  delta += sq(clat2 * sdlong);
  delta = sqrt(delta);
  float denom = (slat1 * slat2) + (clat1 * clat2 * cdlong);
  delta = atan2(delta, denom);
  return delta * 6372795;
}

uint16_t setDistanz(uint16_t distanz) { // Distanz zum Anker Manuel eingeben werte 0 bis 999 in Metern
  u8x8.clearDisplay();
  u8x8.drawString(0, 0, "Distanz in M:");
  u8x8.setCursor(2, 1);
  u8x8.print(distanz);
  u8x8.drawString(0, 2, "Neue Distanz: ");
  uint16_t oldDistanz = distanz;
  while (ButtoneENTER() != 1) {
    if (oldDistanz != distanz) {
      u8x8.clearLine(3);
      u8x8.setCursor(2, 3);
      u8x8.print(distanz);
      oldDistanz = distanz;
    }
    delay(5);
    if ( ButtoneUP() == 1) {
      distanz++;
    }
    if ( ButtoneDOWN() == 1) {
      distanz--;
    }
    if (distanz > 999) {
      distanz = 0;
    }
  }
  u8x8.clearDisplay();
  return distanz;
}

void showDistanz() { // Ausgabe der Distanz auf dem Displaz hinder dem Menu punkt Distanz
  u8x8.setCursor(13, 2);
  u8x8.print(distanz);
}

uint8_t setToleranz(uint8_t toleranz) { // Toleranz in der Distanz Werte 0 bis 50 in Metern
  u8x8.clearDisplay();
  u8x8.drawString(0, 0, "Toleranz in M:");
  u8x8.setCursor(2, 1);
  u8x8.print(toleranz);
  u8x8.drawString(0, 2, "Neue Toleranz: ");
  uint8_t oldToleranz = toleranz;
  while (ButtoneENTER() != 1) {
    if (oldToleranz != toleranz) {
      u8x8.clearLine(3);
      u8x8.setCursor(2, 3);
      u8x8.print(toleranz);
      oldToleranz = toleranz;
    }
    delay(5);
    if ( ButtoneUP() == 1) {
      toleranz++;
    }
    if ( ButtoneDOWN() == 1) {
      toleranz--;
    }
    if (toleranz > 50) {
      toleranz = 50;
    }
  }
  u8x8.clearDisplay();
  return toleranz;
}

void showToleranz() { // Ausgabe der Toleranz auf dem Displaz hinder dem Menu punkt Toleranz
  u8x8.setCursor(13, 3);
  u8x8.print(toleranz);
}

void showPOS1isSet() { // Zeigt an das Die POS1 gesetzt ist duchr ein * am anfang der ersten Zeile 
  if (latitude1 != 0 and longitude1 != 0) {
    u8x8.drawString(0, 0, "*");
  } else {
    u8x8.drawString(0, 0, " ");
  }
}

void alarmTest() { // Spielt einen Alarm Test ab und zeit im OLDE "TestAlarm"
  u8x8.clearDisplay();
  u8x8.drawString(2, 3, "Alarm Test");

  for (uint16_t i = 10; i < 2500; i =  i + 25) {
    tone(pinALARM, i);
    delay(100);
    noTone(pinALARM);
  }

  u8x8.clearDisplay();
}

void info() { // InfoSeite 
  
  u8x8.clearDisplay();
  
  u8x8.drawString(0, 0, "SAT: ??");
  u8x8.drawString(8, 0, "+/-: ??m");
  
  // Angabe Breiten und Längengrad der aktuellen position
  u8x8.drawString(0, 1, "B:");
  u8x8.setCursor(5, 1);
  u8x8.print(nmea[3]);
  
  u8x8.drawString(0, 2, "L:");
  u8x8.setCursor(4, 2);
  u8x8.print(nmea[5]);

  // Ist der Punkt des Ankers gesetzt
  u8x8.drawString(0, 4, "POS1   :");
  if (latitude1 != 0 and longitude1 != 0) {
    u8x8.drawString(9, 4, "YES");
  } else {
    u8x8.drawString(9, 4, "NO ");
  }

  // Aktuelle Entfernung zum Ankerpunkt
  u8x8.drawString(0, 5, "ENTFER.:");
  if(latitude1 != 0 and longitude1 != 0){
    distanzNow =  distance_between(latitude1, longitude1, latitude2, longitude2);
    u8x8.setCursor(9, 5);
    u8x8.print(distanzNow);
  }else{
    u8x8.setCursor(9, 5);
    u8x8.print(0);
  }
  
  // Eingestellte Distanz
  u8x8.drawString(0, 6, "DISTANZ:");
  u8x8.setCursor(9, 6);
  u8x8.print(distanz);
  
  // Aktuell eingestellte Tollerranz
  u8x8.drawString(0, 7, "TOLERA.:");
  u8x8.setCursor(9, 7);
  u8x8.print(toleranz);

  // Warten biss Enterbutten gedrückt wird oder info nach 5sce neu aufrufen 
  long count = 0;
  while (ButtoneENTER() != 1 and count < 2000) {
    count++;
    delay(1);
    }
    if (count >= 1999){
      info();
    }
    
  u8x8.clearDisplay();
}

void showPOS1miss(){
  u8x8.clearDisplay();
  u8x8.drawString(0, 3, "POS1 ist nicht  "); 
  u8x8.drawString(0, 4, "   gesetzt!");
  delay(3000);
  u8x8.clearDisplay();
}

bool ButtoneENTER() {
  uint8_t i = 0;
  i = digitalRead(pinENTER);
  delay(5);
  if (i == 1) {
    return 1;
  } else {
    return 0;
  }
}

bool ButtoneUP() {
  bool i = 0;
  i = digitalRead(pinUP);
  delay(5);
  if (i == 1) {
    return 1;
  } else {
    return 0;
  }
}

bool ButtoneDOWN() {
  bool i = 0;
  i = digitalRead(pinDOWN);
  delay(5);
  if (i == 1) {
    return 1;
  } else {
    return 0;
  }
}

void mainmenu() {

  if (menupunkt > 7) {
    menupunkt = 0;
  }

  if (menupunkt < 0) {
    menupunkt = 7;
  }

  u8x8.drawString(2, 0, "POS 1");
  u8x8.drawString(2, 1, "POS 2");
  u8x8.drawString(2, 2, "DISTANZ");
  u8x8.drawString(2, 3, "TOLERANZ");
  u8x8.drawString(2, 4, "POS DEL");
  u8x8.drawString(2, 5, "ALARM TEST");
  u8x8.drawString(2, 6, "INFO");
  u8x8.drawString(2, 7, "XXXXXX");

  u8x8.setInverseFont(1);

  if (menupunkt == 0) {
    u8x8.drawString(2, 0, "POS 1");
  }
  if (menupunkt == 1) {
    u8x8.drawString(2, 1, "POS 2");
  }
  if (menupunkt == 2) {
    u8x8.drawString(2, 2, "DISTANZ");
  }
  if (menupunkt == 3) {
    u8x8.drawString(2, 3, "TOLERANZ");
  }
  if (menupunkt == 4) {
    u8x8.drawString(2, 4, "POS DEL");
  }
  if (menupunkt == 5) {
    u8x8.drawString(2, 5, "ALARM TEST");
  }
  if (menupunkt == 6) {
    u8x8.drawString(2, 6, "INFO");
  }
  if (menupunkt == 7) {
    u8x8.drawString(2, 7, "XXXXXX");
  }

  u8x8.setInverseFont(0);
  return menupunkt;

}

void setup() {
  Serial.begin(115200);
  delay(100);
  pinMode(pinUP, INPUT);
  pinMode(pinDOWN, INPUT);
  pinMode(pinENTER, INPUT);
  pinMode(pinALARM, OUTPUT);
  u8x8.begin();
  u8x8.setPowerSave(0);
  u8x8.setFont(u8x8_font_chroma48medium8_r);
  GPSSerial.begin(9600);
  delay(100);
  GPSSerial.print(UPDATE_10_sec);
  delay(100);
  GPSSerial.print(ONLY);
  delay(100);
  Timer1.initialize(1000);
  Timer1.attachInterrupt(readGPS);
}

// Hautprogram
void loop() {

  if (norelooper == 0) {
    mainmenu();
    showToleranz();
    showDistanz();
    showPOS1isSet();
    norelooper = 1;
    buttonlock = 0;
  }

  if (ButtoneUP() == 1 and buttonlock == 0) { // Menu Punkt RunterZehelen 
    buttonlock = 1;
    norelooper = 0;
    menupunkt--;
  }

  if (ButtoneDOWN() == 1 and buttonlock == 0) { // Menu Punkt Raufzählen 
    buttonlock = 1;
    norelooper = 0;
    menupunkt++;
  }

  if (ButtoneENTER() == 1 and menupunkt == 0 and buttonlock == 0) { // POS1
    if(latitude1 == 0 && longitude1 == 0){
      latitude1 = latitude2;
      longitude1 = longitude2;
      buttonlock = 1;
      norelooper = 0;
    }
  }
  
  if (ButtoneENTER() == 1 and menupunkt == 1 and buttonlock == 0) { // DISTANZ MIT POS2
    if (latitude1 != 0 and longitude1 != 0){
        distanz =  round(distance_between(latitude1, longitude1, latitude2, longitude2));
    }else{
      showPOS1miss();
    }
    buttonlock = 1;
    norelooper = 0;
  }
  
  if (ButtoneENTER() == 1 and menupunkt == 2 and buttonlock == 0) { //Distanz
      distanz = setDistanz(distanz);
      buttonlock = 1;
      norelooper = 0;
  }
  
  if (ButtoneENTER() == 1 and menupunkt == 3) { // Toleranz
      toleranz = setToleranz(toleranz);
      buttonlock = 1;
      norelooper = 0;
  }
  
  if (ButtoneENTER() == 1 and menupunkt == 4 and buttonlock == 0) { // POS DEL
      latitude1 = 0;
      longitude1 = 0;
      buttonlock = 1;
      norelooper = 0;
  }
  
  if (ButtoneENTER() == 1 and menupunkt == 5 and buttonlock == 0) { //Alarmtest
    alarmTest();
    buttonlock = 1;
    norelooper = 0;
  }
  
  if (ButtoneENTER() == 1 and menupunkt == 6 and buttonlock == 0) { //INFO
    info();
    buttonlock = 1;
    norelooper = 0;
  }
  
  if (ButtoneENTER() == 1 and menupunkt == 7 and buttonlock == 0) { //xxxxx
    buttonlock = 1;
    norelooper = 0;
  }

  if(flag==1){  // Abfrage der NMEA daten untübertragung in String 
    Timer1.stop();
    NMEAtmp.trim();
    SplitNMEAStr(NMEAtmp);
    nmea[3] = ConvertLat();
    nmea[5] = ConvertLng();
    latitude2 = nmea[3].toFloat();
    longitude2 = nmea[3].toFloat();
      Serial.print(nmea[2]);
      Serial.print("  ");
      Serial.print(nmea[3]);
      Serial.print("  ");
      Serial.println(nmea[5]);
    NMEAtmp="";
    flag=0;
    Timer1.restart();
  }

}
