#include <Arduino.h>
#include <U8x8lib.h>


#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif

U8X8_SSD1306_128X64_NONAME_SW_I2C u8x8(/* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE);
HardwareSerial &GPSSerial = Serial3;

const char GPS_UPDATE_5_SECONDS[] = "$PMTK220,5000*1B\r\n";
const char GPS_RMC_ONLY[] = "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*35\r\n";

constexpr uint8_t pinALARM = 9;
constexpr uint8_t pinENTER = 8;
constexpr uint8_t pinUP = 7;
constexpr uint8_t pinDOWN = 6;
constexpr size_t NMEA_BUFFER_SIZE = 96;
constexpr unsigned long GPS_FIX_TIMEOUT_MS = 15000;
constexpr unsigned long BUTTON_DEBOUNCE_MS = 25;

struct DebouncedButton {
  uint8_t pin;
  bool stableState;
  bool lastReading;
  unsigned long lastChange;
};

enum class AlarmMode : uint8_t {
  None,
  Test,
  Anchor
};

char nmeaBuffer[NMEA_BUFFER_SIZE];
size_t nmeaLength = 0;

DebouncedButton enterButton = {pinENTER, false, false, 0};
DebouncedButton upButton = {pinUP, false, false, 0};
DebouncedButton downButton = {pinDOWN, false, false, 0};


uint8_t toleranz = 5;    // Tollerranz weter o bis 50
uint16_t distance = 0;   // Distanzwert zum Ankerpunkt
float distanceNow = 0;   // Aktuellberchneter Abstand zum Nanker
int8_t menupunkt = 0;    // Anzeige Menu Punkt
bool menuDirty = true;
bool anchorSet = false;
bool gpsFixValid = false;
bool anchorAlarmSilenced = false;
unsigned long lastGpsFix = 0;

float latitude1 = 0;     // Breitengrad Ankerpunkt
float longitude1 = 0;    // Lengengrad Ankerpunkt
float latitude2 = 0;     // Breitengrad Messpunkt
float longitude2 = 0;    // Lengengrad Messpunkt

AlarmMode alarmMode = AlarmMode::None;
uint16_t alarmFrequency = 10;
int8_t alarmDirection = 1;
unsigned long lastAlarmStep = 0;

float distance_between(float lat1, float long1, float lat2, float long2);


bool gpsFixIsCurrent() {
  return gpsFixValid && millis() - lastGpsFix <= GPS_FIX_TIMEOUT_MS;
}

bool buttonPressed(DebouncedButton &button) {
  bool reading = digitalRead(button.pin) == HIGH;
  unsigned long now = millis();

  if (reading != button.lastReading) {
    button.lastReading = reading;
    button.lastChange = now;
  }

  if (reading != button.stableState && now - button.lastChange >= BUTTON_DEBOUNCE_MS) {
    button.stableState = reading;
    return button.stableState;
  }

  return false;
}

uint8_t hexValue(char value) {
  if (value >= '0' && value <= '9') {
    return value - '0';
  }
  if (value >= 'A' && value <= 'F') {
    return value - 'A' + 10;
  }
  if (value >= 'a' && value <= 'f') {
    return value - 'a' + 10;
  }
  return 255;
}

bool checksumIsValid(char *sentence) {
  char *checksumPosition = strchr(sentence, '*');
  if (sentence[0] != '$' || checksumPosition == nullptr || checksumPosition[1] == '\0' || checksumPosition[2] == '\0') {
    return false;
  }

  uint8_t checksum = 0;
  for (char *position = sentence + 1; position < checksumPosition; position++) {
    checksum ^= *position;
  }

  uint8_t highNibble = hexValue(checksumPosition[1]);
  uint8_t lowNibble = hexValue(checksumPosition[2]);
  if (highNibble > 15 || lowNibble > 15 || checksum != (highNibble << 4 | lowNibble)) {
    return false;
  }

  *checksumPosition = '\0';
  return true;
}

bool convertCoordinate(const char *rawCoordinate, char hemisphere, float &coordinate) {
  if (rawCoordinate[0] == '\0') {
    return false;
  }

  float nmeaCoordinate = atof(rawCoordinate);
  int degrees = static_cast<int>(nmeaCoordinate / 100.0f);
  float minutes = nmeaCoordinate - degrees * 100.0f;
  if (minutes < 0.0f || minutes >= 60.0f) {
    return false;
  }

  coordinate = degrees + minutes / 60.0f;
  if (hemisphere == 'S' || hemisphere == 'W') {
    coordinate = -coordinate;
  } else if (hemisphere != 'N' && hemisphere != 'E') {
    return false;
  }

  return true;
}

void processNMEA(char *sentence) {
  if (!checksumIsValid(sentence)) {
    return;
  }

  char *fields[7] = {nullptr};
  uint8_t fieldCount = 1;
  fields[0] = sentence;
  for (char *position = sentence; *position != '\0' && fieldCount < 7; position++) {
    if (*position == ',') {
      *position = '\0';
      fields[fieldCount++] = position + 1;
    }
  }

  size_t messageTypeLength = strlen(fields[0]);
  if (fieldCount < 7 || messageTypeLength < 3 || strcmp(fields[0] + messageTypeLength - 3, "RMC") != 0) {
    return;
  }

  if (fields[2][0] != 'A') {
    gpsFixValid = false;
    return;
  }

  float newLatitude;
  float newLongitude;
  if (!convertCoordinate(fields[3], fields[4][0], newLatitude)
      || !convertCoordinate(fields[5], fields[6][0], newLongitude)
      || newLatitude < -90.0f || newLatitude > 90.0f
      || newLongitude < -180.0f || newLongitude > 180.0f) {
    gpsFixValid = false;
    return;
  }

  latitude2 = newLatitude;
  longitude2 = newLongitude;
  gpsFixValid = true;
  lastGpsFix = millis();
  if (anchorSet) {
    distanceNow = distance_between(latitude1, longitude1, latitude2, longitude2);
  }
}

void readGPS(){
  while (GPSSerial.available() > 0) {
    char received = GPSSerial.read();

    if (received == '$') {
      nmeaLength = 0;
      nmeaBuffer[nmeaLength++] = received;
      continue;
    }

    if (nmeaLength == 0) {
      continue;
    }

    if (received == '\n') {
      nmeaBuffer[nmeaLength] = '\0';
      processNMEA(nmeaBuffer);
      nmeaLength = 0;
    } else if (received != '\r') {
      if (nmeaLength < NMEA_BUFFER_SIZE - 1) {
        nmeaBuffer[nmeaLength++] = received;
      } else {
        nmeaLength = 0;
      }
    }
  }
}

float distance_between_123 (float lat1, float long1, float lat2, float long2){ // gibt den Abstand in Metern zwischen zwei 2 GPS-Punkten zurück.
  // Mit vorzeichenbehaftete dezimale Längen- und Breitengrade.
  // Verwendet Großkreis
  // Entfernungsberechnung für eine hypothetische Kugel mit einem Radius von 6372795 Metern.
  // Da die Erde keine exakte Kugel ist, können Rundungsfehler bis zu 0,5% auftreten.
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
  float dis = delta * 6372795;
  if (dis > 999.00){
    return 0;
  }
  return dis;
}

float distance_between (float lat1, float long1, float lat2, float long2){ //  Alternativ Formel um die Distanz zwischen 2 GPS Punkten zu berechnen
   float dy = 111.3 * (lat1 -lat2);
   float lat = (lat1 + lat2) / 2 * 0.01745;
   float dx = 111.3 * cos(lat) * (long1 - long2);
   return sqrt(dx * dx + dy * dy) * 1000;
}

uint16_t setDistanz(uint16_t distance) { // Distanz zum Anker Manuel eingeben werte 0 bis 999 in Metern
  u8x8.clearDisplay();
  u8x8.drawString(0, 0, "Distanz in M:");
  u8x8.setCursor(2, 1);
  u8x8.print(distance);
  u8x8.drawString(0, 2, "Neue Distanz: ");
  u8x8.setCursor(2, 3);
  u8x8.print(distance);
  while (true) {
    readGPS();
    if (buttonPressed(enterButton)) {
      break;
    }

    bool valueChanged = false;
    if (buttonPressed(upButton)) {
      distance = distance == 999 ? 0 : distance + 1;
      valueChanged = true;
    }
    if (buttonPressed(downButton)) {
      distance = distance == 0 ? 999 : distance - 1;
      valueChanged = true;
    }
    if (valueChanged) {
      u8x8.clearLine(3);
      u8x8.setCursor(2, 3);
      u8x8.print(distance);
    }
  }
  u8x8.clearDisplay();
  return distance;
}

void showDistance() { // Ausgabe der Distanz auf dem Displaz hinder dem Menu punkt Distanz
  u8x8.setCursor(13, 2);
  u8x8.print(distance);
}

uint8_t setToleranz(uint8_t toleranz) { // Toleranz in der Distanz Werte 0 bis 50 in Metern
  u8x8.clearDisplay();
  u8x8.drawString(0, 0, "Toleranz in M:");
  u8x8.setCursor(2, 1);
  u8x8.print(toleranz);
  u8x8.drawString(0, 2, "Neue Toleranz: ");
  u8x8.setCursor(2, 3);
  u8x8.print(toleranz);
  while (true) {
    readGPS();
    if (buttonPressed(enterButton)) {
      break;
    }

    bool valueChanged = false;
    if (buttonPressed(upButton) && toleranz < 50) {
      toleranz++;
      valueChanged = true;
    }
    if (buttonPressed(downButton) && toleranz > 0) {
      toleranz--;
      valueChanged = true;
    }
    if (valueChanged) {
      u8x8.clearLine(3);
      u8x8.setCursor(2, 3);
      u8x8.print(toleranz);
    }
  }
  u8x8.clearDisplay();
  return toleranz;
}

void showToleranz() { // Ausgabe der Toleranz auf dem Displaz hinder dem Menu punkt Toleranz
  u8x8.setCursor(13, 3);
  u8x8.print(toleranz);
}

void showPOS1isSet() { // Zeigt an das die POS1 gesetzt ist durch ein * am anfang der ersten Zeile
  if (anchorSet) {
    u8x8.drawString(0, 0, "*");
  } else {
    u8x8.drawString(0, 0, " ");
  }
}

void startAlarm(AlarmMode mode) {
  alarmMode = mode;
  alarmFrequency = 10;
  alarmDirection = 1;
  lastAlarmStep = millis();
  u8x8.clearDisplay();
  if (mode == AlarmMode::Test) {
    u8x8.drawString(2, 3, "Alarm Test");
  } else {
    u8x8.drawString(2, 3, "Alarm!");
  }
  tone(pinALARM, alarmFrequency);
}

void stopAlarm() {
  noTone(pinALARM);
  alarmMode = AlarmMode::None;
  u8x8.clearDisplay();
  menuDirty = true;
}

void updateAlarm() {
  if (alarmMode == AlarmMode::None || millis() - lastAlarmStep < 100) {
    return;
  }

  lastAlarmStep = millis();
  if (alarmDirection > 0 && alarmFrequency >= 2475) {
    if (alarmMode == AlarmMode::Test) {
      stopAlarm();
      return;
    }
    alarmDirection = -1;
  } else if (alarmDirection < 0 && alarmFrequency <= 35) {
    alarmDirection = 1;
  } else {
    alarmFrequency += 25 * alarmDirection;
  }
  tone(pinALARM, alarmFrequency);
}

void info() { // InfoSeite
  u8x8.clearDisplay();

  // Angabe Breiten und Längengrad der aktuellen position
  u8x8.drawString(0, 1, "B:");
  u8x8.setCursor(3, 1);
  u8x8.print(latitude2, 6);

  u8x8.drawString(0, 2, "L:");
  u8x8.setCursor(3, 2);
  u8x8.print(longitude2, 6);

  // Ist der Punkt des Ankers gesetzt
  u8x8.drawString(0, 4, "POS1   :");
  if (anchorSet) {
    u8x8.drawString(9, 4, "YES");
  } else {
    u8x8.drawString(9, 4, "NO ");
  }

  // Aktuelle Entfernung zum Ankerpunkt
  u8x8.drawString(0, 5, "ENTFER.:");
  if (anchorSet && gpsFixIsCurrent()) {
    u8x8.setCursor(9, 5);
    u8x8.print(distanceNow);
  } else {
    u8x8.setCursor(9, 5);
    u8x8.print(0);
  }

  // Eingestellte Distanz
  u8x8.drawString(0, 6, "DISTANZ:");
  u8x8.setCursor(9, 6);
  u8x8.print(distance);

  // Aktuell eingestellte Tollerranz
  u8x8.drawString(0, 7, "TOLERA.:");
  u8x8.setCursor(9, 7);
  u8x8.print(toleranz);

  while (!buttonPressed(enterButton)) {
    readGPS();
  }

  u8x8.clearDisplay();
}

void showPOS1miss(){
  u8x8.clearDisplay();
  u8x8.drawString(0, 3, "POS1 ist nicht  ");
  u8x8.drawString(0, 4, "   gesetzt!");
  unsigned long start = millis();
  while (millis() - start < 3000) {
    readGPS();
  }
  u8x8.clearDisplay();
}

void showGPSFixMiss() {
  u8x8.clearDisplay();
  u8x8.drawString(1, 3, "Kein GPS-Fix");
  unsigned long start = millis();
  while (millis() - start < 3000) {
    readGPS();
  }
  u8x8.clearDisplay();
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
}

void setup() {
  pinMode(pinUP, INPUT);
  pinMode(pinDOWN, INPUT);
  pinMode(pinENTER, INPUT);
  pinMode(pinALARM, OUTPUT);
  u8x8.begin();
  u8x8.setPowerSave(0);
  u8x8.setFont(u8x8_font_chroma48medium8_r);
  GPSSerial.begin(9600);
  delay(100);
  GPSSerial.print(GPS_UPDATE_5_SECONDS);
  delay(100);
  GPSSerial.print(GPS_RMC_ONLY);
}

// Hautprogram
void loop() {
  readGPS();

  bool upPressed = buttonPressed(upButton);
  bool downPressed = buttonPressed(downButton);
  bool enterPressed = buttonPressed(enterButton);

  bool alarmThresholdExceeded = distance != 0 && anchorSet && gpsFixIsCurrent()
      && distanceNow > distance + toleranz;
  if (!alarmThresholdExceeded) {
    anchorAlarmSilenced = false;
    if (alarmMode == AlarmMode::Anchor) {
      stopAlarm();
    }
  }

  updateAlarm();
  if (alarmMode == AlarmMode::None && alarmThresholdExceeded && !anchorAlarmSilenced) {
    startAlarm(AlarmMode::Anchor);
  }

  if (alarmMode != AlarmMode::None) {
    if (enterPressed) {
      if (alarmMode == AlarmMode::Anchor) {
        anchorAlarmSilenced = true;
      }
      stopAlarm();
    }
    return;
  }

  if (menuDirty) {
    u8x8.clearDisplay();
    mainmenu();
    showToleranz();
    showDistance();
    showPOS1isSet();
    menuDirty = false;
  }

  if (upPressed) { // Menu Punkt RunterZehelen
    menuDirty = true;
    menupunkt--;
  }

  if (downPressed) { // Menu Punkt Raufzählen
    menuDirty = true;
    menupunkt++;
  }

  if (!enterPressed) {
    return;
  }

  switch (menupunkt) {
    case 0: // POS1
      if (gpsFixIsCurrent()) {
        latitude1 = latitude2;
        longitude1 = longitude2;
        anchorSet = true;
        distanceNow = 0;
      } else {
        showGPSFixMiss();
      }
      break;
    case 1: // DISTANZ MIT POS2
      if (!anchorSet) {
        showPOS1miss();
      } else if (!gpsFixIsCurrent()) {
        showGPSFixMiss();
      } else {
        distance = min(999UL, static_cast<unsigned long>(round(distance_between(
            latitude1, longitude1, latitude2, longitude2))));
      }
      break;
    case 2: // Distanz
      distance = setDistanz(distance);
      break;
    case 3: // Toleranz
      toleranz = setToleranz(toleranz);
      break;
    case 4: // POS DEL
      latitude1 = 0;
      longitude1 = 0;
      anchorSet = false;
      distanceNow = 0;
      break;
    case 5: // Alarmtest
      startAlarm(AlarmMode::Test);
      break;
    case 6: // Info
      info();
      break;
    default:
      break;
  }

  menuDirty = true;
}
