/*
  -----------------------------------------   Einleitung   ------------------------------------------------

  Projektname: moveUP 
  Autor: Annemarie Kayser
  Anwendung: Tragbares sensorbasiertes Messsystem zur Kontrolle des Sitzverhaltens;
             Ausgabe eines Hinweises, wenn eine krumme Haltung eingenommen oder sich längere 
             Zeit nicht bewegt wurde in Form von Vibration am Rücken. Messung des dynamischen und statischen 
             Sitzverhaltens mithilfe der Gyroskopwerte. 
  Bauteile: Verwendung des 6-Achsen-Beschleunigungssensors MPU 6050 in Verbindung mit dem Esp32 Thing;
            Datenübertragung zwischen dem Esp32 Thing und einem Smartphone erfolgt via Bluetooth Low Energy.
            Ein Vibrationsmotor am Rücken gibt den Hinweis auf eine krumme Haltung und sich zubewegen.
            Die Sensorik wurde in einem kleinen Gehäuse befestigt, welches mit einem Clip am Oberteil befestigt werden kann.
  Letztes Update: 18.02.2023


  ----------------------------------------------------------------------------------------------------------*/

/*------------------------------------   Bibliotheken & Variablen   ----------------------------------------*/


// === Bluetooth Low Energy === //
#include <Wire.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <ArduinoJson.h>
#include <CircularBuffer.h>

#define BLE_DEVICENAME "ESP32-moveUp"
#define SERVICE_UUID "0000ffe0-0000-1000-8000-00805f9b34fb"
#define CHARACTERISTIC_UUID "0000ffe1-0000-1000-8000-00805f9b34fb"
#define DELAYTIME 200

// === Variablen für die ungerade Rückenhaltung === //
int angleAcZ;
int thresholdTime = 30000;
int thresholdTimeStop = 10000;
int counterBentBack = 0;
int arrayCounterBentBack[48];
int thresholdBentBack = -30;
bool startTimerBackBent = true;
unsigned long startTime;
unsigned long startTime2;

// === Variablen für Uhrzeit === //
int hour = 0;
int minute = 0;
int intervall = 60000;
unsigned long startUhr = millis();

// === Einstellung der Vibration === //
int delayVibration = 1500;
int vibrationPin = 26;
String vibration = "VIBON";

// === Konfiguration === //
int nextValue = 0;
int const number = 800;
int valuesAcZ[number];
float angleAcZConfigStraight = 0;
float angleAcZConfigBentBack = 0;
float angleAcZConfigLeanBack = 0;
bool startConfig = false;
bool statusConfig = false;
bool statusConfig2 = false;
bool statusConfig3 = false;
bool measurementCompleted = true;

// === Status der Messung === //
String statusMeasurement = "AUS";

// === zurückgelehnt === //
int counterLeanBack = 0;
int thresholdLeanBack = 20;
int thresholdTimeLeanBack = 30000;
int arrayCounterLeanBack[48];
bool startTimerLeanBack = true;
unsigned long timerStartLeanBack;


// === Mittlaufender Mittelwert === //
const int AverageCount = 10;              // Anzahl der in den Laufenden-Mittelwert aufgenommenen Messungen
float AverageBufferAcZ[AverageCount];
int NextAverageAcZ;
float AverageBufferGyX[AverageCount];
int NextAverageGyX;
float AverageBufferGyY[AverageCount];
int NextAverageGyY;
float AverageBufferGyZ[AverageCount];
int NextAverageGyZ;

// === Bewegung erkennen === //
int counterMovement = 0;
int counterMovementHour = 0;
int counterMovementPhase = 0;
int thresholdMovement = 6000;
int arrayCounterMovement[48];
bool startDetectingMovement = true;
bool measuringTime = false;
unsigned long startTimeMovement;
unsigned long thresholdTimeMovement = 1800000;
unsigned long startMovement;

// === aufrechtes Sitzen === //
int timeUpright = 0;
int arraySittingStraight[48];
bool startTimerBackStraight = false;
bool firstStartTimer = true;
unsigned long startTimeSittingUpright;
unsigned long stopTimeSittingUpright;
unsigned long timeSittingUpright = 0;
unsigned long previousTimeSittingUpright = 0;
unsigned long timeSittingUprightMinute = 0;
unsigned long timeUprightPrevious = 0;

// === Challenge === //
String statusChallenge = "STOPP";
String sendStatusChallenge = "geschafft";
bool startChallenge = true;
bool challengeReceived = false;
int timeChallenge;
unsigned long startTimeChallenge;

// === MPU-6050 === //
const int MPU_addr = 0x68;
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;



/*----------------------------------------   BLE-Verbindung -----------------------------------------------*/

BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
  };

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
  }
};

/* ---------------------------------------- Daten empfangen -----------------------------------------------*/

class MyCharacteristicCallbacks : public BLECharacteristicCallbacks {

  // Diese Funktion wird angesprungen, wenn per BLE Daten empfangen werden
  void onWrite(BLECharacteristic* pCharacteristic) {
    String jsonString = pCharacteristic->getValue().c_str();
    // JSON String auswerten
    StaticJsonDocument<2000> doc;
    deserializeJson(doc, jsonString);

    String statusChal = doc["CHALLENGE"];
    String timeString = doc["TIMECHALLENGE"];
    String vibrationVal = doc["VIBRATION"];
    String vibLength = doc["VIBLENGTH"];
    String statusMeas = doc["STARTMESSUNG"];
    String hourString = doc["HOUR"];
    String minuteString = doc["MINUTE"];
    String thresholdBent = doc["THRESHOLDBENTBACK"];
    String thresholdLean = doc["THRESHOLDLEANBACK"];
    statusConfig = (boolean)doc["STATUSKONFIG"];
    statusConfig2 = (boolean)doc["STATUSKONFIG2"];
    statusConfig3 = (boolean)doc["STATUSKONFIG3"];
    startConfig = (boolean)doc["START"];
    challengeReceived = (boolean)doc["CHALLENGERECEIVED"];
    
    // Überprüfen, ob die gesendeten Daten einen Wert haben
    // Anschließend werden die empfangenen Daten den Variablen zugewiesen
    if (hourString != "null" && minuteString != "null") {
      hour = hourString.toInt();
      minute = minuteString.toInt();

      //Umrechnung der Stunde in halbe Stunden damit Daten
      //jede halbe Stunde im Array gespeichert werden     
      if (minute < 30) {
        hour = hour * 2;
      }

      if (minute >= 30) {
        hour = hour * 2 + 1;
      }
    }

    if (thresholdBent != "null") {
      thresholdBentBack = thresholdBent.toInt();
    }

     if (thresholdLean != "null") {
      thresholdLeanBack = thresholdLean.toInt();
    }

    if (challengeReceived) {
      sendStatusChallenge = "";
    }

    if (vibLength != "null") {
      delayVibration = vibLength.toInt();
    }

    // Umrechnen der Zeit in Millisekunden
    if (timeString != "null") {
      timeChallenge = timeString.toInt() * 1000 * 60;
    }

    if (vibrationVal != "null") {
      vibration = vibrationVal;

      if (vibration == "VIBON") {
        delayVibration = vibLength.toInt();
      }

      if (vibration == "VIBOFF") {
        delayVibration = 0;
      }
    }

    if (statusMeas != "null") {
      statusMeasurement = statusMeas;
      
      if (statusMeas == "AN") {
        digitalWrite(vibrationPin, HIGH);
        delay(300);
        digitalWrite(vibrationPin, LOW);
        delay(300);
        digitalWrite(vibrationPin, HIGH);
        delay(300);
        digitalWrite(vibrationPin, LOW);
        startTimeMovement = millis();
      }
    }

    if (statusChal != "null") {
      statusChallenge = statusChal;
    }
  };
};


/*----------------------------------------------   Setup   ------------------------------------------------*/

void setup() {

  pinMode(vibrationPin, OUTPUT);

  // =============== Initialisierung des 6-Achsen-Sensors ================== //
  Wire.begin();

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  // Konfiguration der Sensitivität des Beschleunigungssensors 
  //     Full Scale Range|Sensitivität| Byte
  //_____________________|____________|___________
  // +-2g  | 16384 LSB/g | 0b00000000 | 0x00
  // +-4g  |  8192 LSB/g | 0b00001000 | 0x08
  // +-8g  |  4096 LSB/g | 0b00010000 | 0x10
  // +-16g |  2048 LSB/g | 0b00011000 | 0x18

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1C);
  Wire.write(0b00000000);  // Messbereich: +/- 2g
  Wire.endTransmission(true);

  // Konfiguration der Sensitivität des Gyroskops
  //  Full ScaleRange    | Sensitivität   | Byte
  //__________________   |________________|___________
  // +/-  250 degrees/s  | 131.0 LSB/deg/s| 0b00000000 | 0x00
  // +/-  500 degrees/s  |  65.5 LSB/deg/s| 0b00001000 | 0x08
  // +/- 1000 degrees/s  |  32.8 LSB/deg/s| 0b00010000 | 0x10
  // +/- 2000 degrees/s  |  16.4 LSB/deg/s| 0b00011000 | 0x18

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1B);
  Wire.write(0b00000000);  // Messbereich +/- 250 degree/s
  Wire.endTransmission(true);

  Serial.begin(38400);

  // ======================= Initialisierung BLE ======================== //
  uint16_t mtu = 128;
  BLEDevice::setMTU(128);

  // Erstellung BLE-Geräte
  BLEDevice::init(BLE_DEVICENAME);

  // Erstellung BLE-Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Erstellung BLE-Service
  BLEService* pService = pServer->createService(SERVICE_UUID);

  // Erstellung BLE-Charakteristik
  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY);

  // Erstellung BLE-Descriptor
  pCharacteristic->addDescriptor(new BLE2902());
  pCharacteristic->setCallbacks(new MyCharacteristicCallbacks());

  // Service starten
  pService->start();

  // "Advertising" starten
  BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);
  BLEDevice::startAdvertising();
  Serial.println("Waiting a client connection to notify...");
}


/*------------------------------------------------   void loop()   ----------------------------------------------------*/

void loop() {

  // ========================== BLE-Verbindung ================================== //

  // trennen
  if (!deviceConnected && oldDeviceConnected) {
    delay(500);
    pServer->startAdvertising();  // Neustart "Advertising"
    oldDeviceConnected = deviceConnected;
  }

  // verbinden 
  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;
  }

  // =========================== Konfiguration ================================== //

  while (startConfig) {

    // ===================== Konfiguration gerade Haltung ====================== //
    if (statusConfig) {

      delay(1000);
      Serial.println("Messung gerade Haltung startet");

      digitalWrite(vibrationPin, HIGH);
      delay(200);
      digitalWrite(vibrationPin, LOW);

      // 800 Werte werden eingelesen und in einem Array gespeichert 
      while (nextValue < number) {
        // Einlesen der Daten des MPU6050-Beschleunigungssensors
        Wire.beginTransmission(MPU_addr);
        Wire.write(0x3F);
        Wire.endTransmission(false);
        Wire.requestFrom(MPU_addr, 2, true);

        // Einlesen der Beschleunigung in z-Richtung
        AcZ = Wire.read() << 8 | Wire.read();

        // Berechnung des mittlaufenden Mittelwertes
        // Rohwert wird dem Array hinzugefügt, NextAverageAcZ wird eins hochgezählt
        AverageBufferAcZ[NextAverageAcZ++] = AcZ;

        // wenn der NextAverageAcZ größer als die Anzahl
        // der Messungen ist, wird der NextAverageAcZ auf 0 gesetzt
        if (NextAverageAcZ >= AverageCount) {
          NextAverageAcZ = 0;
        }

        float averageAcZ = 0;

        for (int i = 0; i < AverageCount; ++i) {
          averageAcZ += AverageBufferAcZ[i];  // Summe der Messungen
        }
        averageAcZ /= AverageCount;  // Mittelwertberechnung

        // Berechnung des Winkels der Rückenhaltung
        angleAcZ = (averageAcZ / 16384.0) * 90;

        // Speichern des Winkels in einem Array
        valuesAcZ[nextValue++] = angleAcZ;
      }

      Serial.println("Messsung abgeschlossen");

      if (measurementCompleted) {
        measurementCompleted = false;

        for (int i = 0; i < number; ++i) {
          angleAcZConfigStraight += valuesAcZ[i];  // Summe der Messungen
        }

        // Berechnung des Mittelwertes der vorher im Array aufgenommenen Daten
        angleAcZConfigStraight /= number;
        Serial.println("Wert für gerade Haltung: " + String(angleAcZConfigStraight));
      }
      statusConfig = false;
    }


    // ============================= Konfiguration ungerade Haltung ============================= //

    if (statusConfig2) {

      delay(1000);
      Serial.println("Messung ungerade Haltung startet");

      digitalWrite(vibrationPin, HIGH);
      delay(200);
      digitalWrite(vibrationPin, LOW);

      while (nextValue < number) {
        // Einlesen der Daten des MPU6050-Beschleunigungssensors
        Wire.beginTransmission(MPU_addr);
        Wire.write(0x3F);
        Wire.endTransmission(false);
        Wire.requestFrom(MPU_addr, 2, true);

        // Einlesen der Beschleunigung in z-Richtung
        AcZ = Wire.read() << 8 | Wire.read();

        // Berechnung des mittlaufenden Mittelwertes
        // Rohwert wird dem Array hinzugefügt, NextAverageAcZ wird eins hochgezählt
        AverageBufferAcZ[NextAverageAcZ++] = AcZ;

        // wenn der NextAverageAcX größer als die Anzahl der Messungen ist,
        // wird der NextAverageAcX auf 0 gesetzt
        if (NextAverageAcZ >= AverageCount) {
          NextAverageAcZ = 0;
        }

        float averageAcZ = 0;

        for (int i = 0; i < AverageCount; ++i) {
          averageAcZ += AverageBufferAcZ[i];  // Summe der Messungen
        }
        averageAcZ /= AverageCount;  // Mittelwertberechnung

        //Berechnung des Winkels der Rückenhaltung
        angleAcZ = (averageAcZ / 16384.0) * 90;

        // Speichern des Winkels in einem Array
        valuesAcZ[nextValue++] = angleAcZ;
      }

      Serial.println("Messsung abgeschlossen");

      if (measurementCompleted) {
        measurementCompleted = false;

        for (int i = 0; i < number; ++i) {
          angleAcZConfigBentBack += valuesAcZ[i];  // Summe der Messungen
        }

        // Berechnung des Mittelwertes der vorher im Array aufgenommenen Winkel
        angleAcZConfigBentBack = angleAcZConfigBentBack / number;
        Serial.println("Wert für ungerade Haltung: " + String(angleAcZConfigBentBack));
      }

      // Berechnung des Grenzwertes zwischen einer geraden und ungeraden Haltung
      int diff = angleAcZConfigBentBack - angleAcZConfigStraight;
      thresholdBentBack = angleAcZConfigStraight + ((diff * 2) / 3.0);

      Serial.println("Grenzwert" + String(thresholdBentBack));
      statusConfig2 = false;
    }

    // ========================= Konfiguration zurückgelehnte Haltung ===================== //

    if (statusConfig3) {

      delay(1000);
      Serial.println("Messung zurückgelehnte Haltung startet");

      digitalWrite(vibrationPin, HIGH);
      delay(200);
      digitalWrite(vibrationPin, LOW);

      while (nextValue < number) {
        // Einlesen der Daten des MPU6050-Beschleunigungssensors
        Wire.beginTransmission(MPU_addr);
        Wire.write(0x3F);
        Wire.endTransmission(false);
        Wire.requestFrom(MPU_addr, 2, true);

        // Einlesen der Beschleunigung in z-Richtung
        AcZ = Wire.read() << 8 | Wire.read();

        // Berechnung des mittlaufenden Mittelwertes
        // Rohwert wird dem Array hinzugefügt, NextAverageAcZ wird eins hochgezählt
        AverageBufferAcZ[NextAverageAcZ++] = AcZ;

        // wenn der NextAverageAcX größer als die Anzahl der Messungen ist,
        // wird der NextAverageAcX auf 0 gesetzt
        if (NextAverageAcZ >= AverageCount) {
          NextAverageAcZ = 0;
        }

        float averageAcZ = 0;

        for (int i = 0; i < AverageCount; ++i) {
          averageAcZ += AverageBufferAcZ[i];  // Summe der Messungen
        }
        averageAcZ /= AverageCount;  // Mittelwertberechnung

        //Berechnung des Winkels der Rückenhaltung
        angleAcZ = (averageAcZ / 16384.0) * 90;

        // Speichern des Winkels in einem Array
        valuesAcZ[nextValue++] = angleAcZ;
      }

      Serial.println("Messsung abgeschlossen");

      if (measurementCompleted) {
        measurementCompleted = false;

        for (int i = 0; i < number; ++i) {
          angleAcZConfigLeanBack += valuesAcZ[i];  // Summe der Messungen
        }

        // Berechnung des Mittelwertes der vorher im Array aufgenommenen Winkel
        angleAcZConfigLeanBack = angleAcZConfigLeanBack / number;
        Serial.println("Wert für zurückgelehnte Haltung: " + String(angleAcZConfigLeanBack));
      }

      // Berechnung des Grenzwertes zwischen einer geraden und zurückgelehnten Haltung
      int diff = angleAcZConfigLeanBack - angleAcZConfigStraight;
      thresholdLeanBack = angleAcZConfigStraight + ((diff * 2) / 3.0);

      Serial.println("Grenzwert" + String(thresholdLeanBack));
      statusConfig3 = false;
      sendConfigData();
    }
    nextValue = 0;
    measurementCompleted = true;
  }

  // ===================================== Start der Challenge =========================================//

  while (statusChallenge == "START") {
    statusMeasurement = "AUS";
    challenge();
  }

  // ===================================== Start der Messung ========================================== //

  while (statusMeasurement == "AN") {

    // === Start der Zeitmessung === //
    // minute wird jede Minute um eins hochgezählt
    if (millis() - startUhr > intervall) {
      startUhr = millis();
      minute++;
      
      // hour wird um eins hochgezählt jede halbe Stunde
      if (minute == 30) {
        minute = 0;
        hour++;

        // zurücksetzen der Daten für die neue halbe Stunde
        counterBentBack = 0;
        counterLeanBack = 0;
        counterMovementHour = 0;
        timeUpright = 0;
      }

      // hour wird um eins hochgezählt jede halbe Stunde
      if (minute == 60) {
        minute = 0;
        hour++;

        // zurücksetzen der Daten für die neue halbe Stunde
        counterBentBack = 0;
        counterLeanBack = 0;
        counterMovementHour = 0;
        timeUpright = 0;
      }
    }

    // === Anylse der Rückenpositon und des Sitzverhaltens === //
    // wenn Rückenhaltung ungerade ist -> calculatingBackPosture = 1
    // wenn Rückenhaltung zurückgelehnt ist -> calculatingBackPosture = 2
    // wenn Bewegung erkannt wurde -> calculatingBackPosture = 3
    if (calculatingBackPosture() == 1 || calculatingBackPosture() == 2 || calculatingBackPosture() == 3) {

      // === ungerade Haltung === //
      if (calculatingBackPosture() == 1) {

        // Zeitmessung der geraden Haltung wird gestoppt
        stopTimeSittingUpright = millis();

        // Einmaliges Starten der Zeitmessung, wenn Rückenhaltung ungerade ist,
        // um die Zeit zumessen, die in dieser Position verblieben wird
        if (startTimerBackBent == true) {
          Serial.println("Eine ungerade Haltung wurde eingenommen");
          startTime = millis();
          startTimerBackBent = false;
          startTimerBackStraight = true;
        }

        // Alarm wird ausgelöst sobald eine bestimmte Grenzzeit überschritten wurde, die in der
        // ungerade Haltung verblieben wurde
        if (millis() - startTime > thresholdTime) {

          digitalWrite(vibrationPin, HIGH);
          delay(delayVibration);
          digitalWrite(vibrationPin, LOW);
          delay(delayVibration);
          digitalWrite(vibrationPin, HIGH);
          delay(delayVibration);
          digitalWrite(vibrationPin, LOW);

          // Messung wird neu gestartet werden
          startTimerBackBent = true;

          // Speichern der Daten im Array
          counterBentBack++;
          arrayCounterBentBack[hour] = counterBentBack;
        }
      }

      // === zurückgelehnte Haltung === //
      if (calculatingBackPosture() == 2) {

        // Zeitmessung der geraden Haltung wird gestoppt
        stopTimeSittingUpright = millis();

        // Einmaliges Starten der Zeitmessung, wenn Rückenhaltung zurückgelehnt ist,
        // um die Zeit zumessen, die in dieser Position verblieben wird
        if (startTimerLeanBack == true) {
          Serial.println("Zurück gelehnt");
          timerStartLeanBack = millis();
          startTimerLeanBack = false;
          startTimerBackStraight = true;
        }

        // Hochzählen der zurückgelehnten Phasen, wenn eine bestimmte Zeit in
        // der zurückgelehnte Position verbracht wurde
        if (millis() - timerStartLeanBack > thresholdTimeLeanBack) {
          //Speichern der Daten im Array
          counterLeanBack++;
          arrayCounterLeanBack[hour] = counterLeanBack;
          //Messung wird neu gestartet werden
          startTimerLeanBack = true;
        }
      }

      // === Erkennung einer Bewegung === //
      if (calculatingBackPosture() == 3) {

        // Zeitmessung der geradne Haltung wird gestoppt
        stopTimeSittingUpright = millis();

        // Einmaliges Starten der Zeitmessung, wenn eine Bewegung erkannt wurde
        if (startDetectingMovement == true) {
          startMovement = millis();
          startDetectingMovement = false;
          measuringTime = true;
        }
        // Hochzählen der Bewegungen
        counterMovement++;
        Serial.println("Bewegung");
      }
    } else {

      // === Start der Zeitmessung mit gerader Rückenhaltung === //
      startTimeSittingUpright = millis();

      // Beim initialen Start der Zeitmessung der aufrechten Haltung wird die Variable stopTimeSittingUpright
      // auf die aktuelle Zeit gesetzt, um die bereits laufende Anfangszeit von millis() rauszurechnen
      if (firstStartTimer == true) {
        stopTimeSittingUpright = millis();
        firstStartTimer = false;
      }

      // Berechnung der ingesamten Zeit mit gerader Haltung
      timeSittingUpright = startTimeSittingUpright - stopTimeSittingUpright;
      stopTimeSittingUpright = startTimeSittingUpright;

      previousTimeSittingUpright += timeSittingUpright;
      timeSittingUprightMinute = previousTimeSittingUpright / 60000;

      // Speichern der Zeit mit gerade Haltung in einem Array
      if (timeSittingUprightMinute != timeUprightPrevious) {
        timeUpright++;
        arraySittingStraight[hour] = timeUpright;
        timeUprightPrevious = timeSittingUprightMinute;
        Serial.println("timeUpright= " + timeUpright);
      }

      // Einmaliges Starten der Zeitmessung, wenn Rückenhaltung wieder gerade ist
      if (startTimerBackStraight == true) {
        startTime2 = millis();
        startTimerBackStraight = false;
      }

      // Wenn die Haltung für eine längere Zeit wieder gerade ist, wird die Stoppuhr
      // für die ungerade und zurückgelehnte Haltung zurück gesetzt
      if (millis() - startTime2 > thresholdTimeStop) {
        startTimerBackBent = true;
        startTimerLeanBack = true;
      }

      // Wenn innerhalb von einer Minute 10 Bewegungen erkannt werden, wird dies als Bewegungsphase erkannt
      // und der entsprechende Counter hochgezählt
      if (millis() - startMovement > 60000 && measuringTime == true) {

        if (counterMovement > 10) {
          counterMovementPhase++;
          counterMovementHour++;

          // Speichern der Bewegungsphase in einem Array
          arrayCounterMovement[hour] = counterMovementHour;
        }
        // Zurücksetzen der Variablen
        counterMovement = 0;
        startDetectingMovement = true;
        measuringTime = false;
      }

      // wenn weniger als 3 Bewegungsphasen über einen längeren Zeitraum (30 Minuten) erkannt wurden,
      // wird eine kurze Vibration ausgelöst, um den Nutzer darauf hinzuweisen, sich zu bewegen
      if (millis() - startTimeMovement > thresholdTimeMovement) {
        if (counterMovementPhase < 3) {

          digitalWrite(vibrationPin, HIGH);
          delay(200);
          digitalWrite(vibrationPin, LOW);
          delay(200);
          digitalWrite(vibrationPin, HIGH);
          delay(200);
          digitalWrite(vibrationPin, LOW);
        }
        counterMovementPhase = 0;
        startTimeMovement = millis();
      }
    }

    // === Senden der Werte an die App per BLE === //
    if (deviceConnected) {
      sendData();
      sendData2();
      sendData3();
      sendData4();
    }
    delay(DELAYTIME);
  }
}



/*------------------------------------------   Funktionen   ---------------------------------------------------*/

// ========================= calculatingBackPosture() ============================ //
int calculatingBackPosture() {

  int pos = 0;

  //Einlesen der Daten des MPU6050-Beschleunigungssensors
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3F);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 2, true);

  AcZ = Wire.read() << 8 | Wire.read();

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 6, true);

  GyX = Wire.read() << 8 | Wire.read();
  GyY = Wire.read() << 8 | Wire.read();
  GyZ = Wire.read() << 8 | Wire.read();


  // Berechnung des mittlaufenden Mittelwertes
  // Rohwert wird dem Array hinzugefügt, NextAverageAcZ wird eins hochgezählt
  AverageBufferAcZ[NextAverageAcZ++] = AcZ;

  // wenn der NextAverageAcZ größer als die Anzahl der Messungen ist,
  // wird der NextAverageAcZ auf 0 gesetzt
  if (NextAverageAcZ >= AverageCount) {
    NextAverageAcZ = 0;
  }

  float averageAcZ = 0;

  for (int i = 0; i < AverageCount; ++i) {
    averageAcZ += AverageBufferAcZ[i];  // Summe der Messungen
  }
  averageAcZ /= AverageCount;  // Mittelwertberechnung

  // Berechnung des mittlaufenden Mittelwertes
  // Rohwert wird dem Array hinzugefügt, NextAverageGyX wird eins hochgezählt
  AverageBufferGyX[NextAverageGyX++] = GyX;

  // wenn der NextAverageGyX größer als die Anzahl der Messungen ist, wird der
  // NextAverageGyX auf 0 gesetzt
  if (NextAverageGyX >= AverageCount) {
    NextAverageGyX = 0;
  }

  float averageGyX = 0;

  for (int i = 0; i < AverageCount; ++i) {
    averageGyX += AverageBufferGyX[i];  // Summe der Messungen
  }
  averageGyX /= AverageCount;  // Mittelwertberechnung

  // Berechnung des mittlaufenden Mittelwertes
  // Rohwert wird dem Array hinzugefügt, NextAverageGyY wird eins hochgezählt
  AverageBufferGyY[NextAverageGyY++] = GyY;

  // wenn der NextAverageGyY größer als die Anzahl der Messungen ist, wird der
  // NextAverageGyY auf 0 gesetzt
  if (NextAverageGyY >= AverageCount) {
    NextAverageGyY = 0;
  }

  float averageGyY = 0;

  for (int i = 0; i < AverageCount; ++i) {
    averageGyY += AverageBufferGyY[i];  // Summe der Messungen
  }
  averageGyY /= AverageCount;  // Mittelwertberechnung

  // Berechnung des mittlaufenden Mittelwertes
  // Rohwert wird dem Array hinzugefügt, NextAverageGyZ wird eins hochgezählt
  AverageBufferGyZ[NextAverageGyZ++] = GyZ;

  // wenn der NextAverageGyZ größer als die Anzahl der Messungen ist, wird der
  // NextAverageGyZ auf 0 gesetzt
  if (NextAverageGyZ >= AverageCount) {
    NextAverageGyZ = 0;
  }

  float averageGyZ = 0;

  for (int i = 0; i < AverageCount; ++i) {
    averageGyZ += AverageBufferGyZ[i];  // Summe der Messungen
  }
  averageGyZ /= AverageCount;  // Mittelwertberechnung


  // Negative Werte in positive umwandeln
  if (averageGyX < 0) {
    averageGyX = -averageGyX;
  }
  if (averageGyY < 0) {
    averageGyY = -averageGyY;
  }
  if (averageGyZ < 0) {
    averageGyZ = -averageGyZ;
  }

  // Aufsummieren der Gyroskopwerte
  int gyro = averageGyX + averageGyY + averageGyZ;

  // Berechnung des Winkels der Rückenhaltung
  angleAcZ = (averageAcZ / 16384.0) * 90;

  // === Abfrage, ob Rückenhaltung ungerade ist === //
  if (angleAcZ < thresholdBentBack && gyro < thresholdMovement) {
    pos = 1;
  }

  // === Abfrage, ob Rückenhaltung zurückgelehnt ist === //
  if (angleAcZ > thresholdLeanBack && gyro < thresholdMovement) {
    pos = 2;
  }

  // === Abfrage, ob eine Bewegung erkannt wurde === //
  if (gyro > thresholdMovement) {
    pos = 3;
  }

  // Rückgabe des Wertes
  switch (pos) {
    case 0: return (pos); break;
    case 1: return (pos); break;
    case 2: return (pos); break;
    case 3: return (pos); break;
  }
}

// ==================================== sendData() ========================================== //

void sendData() {
  // Daten als JSON Objekt kodieren, in JSON String
  // umwandeln und per BT senden
  StaticJsonDocument<2500> doc;
  char jsonstring[2500];

  for (int i = 0; i < 48; i++) {
    doc["arrBent"][i] = arrayCounterBentBack[i];
  }

  serializeJson(doc, jsonstring);
  pCharacteristic->setValue(jsonstring);
  pCharacteristic->notify();
}

void sendData2() {
  // Daten als JSON Objekt kodieren, in JSON String
  // umwandeln und per BT senden
  StaticJsonDocument<2500> doc2;
  char jsonstring[2500];

  for (int i = 0; i < 48; i++) {
    doc2["arrDynamic"][i] = arrayCounterMovement[i];
  }

  serializeJson(doc2, jsonstring);
  pCharacteristic->setValue(jsonstring);
  pCharacteristic->notify();
}

void sendData3() {
  // Daten als JSON Objekt kodieren, in JSON String
  // umwandeln und per BT senden
  StaticJsonDocument<2500> doc3;
  char jsonstring[2500];

  for (int i = 0; i < 48; i++) {
    doc3["arrStraight"][i] = arraySittingStraight[i];
  }

  char challenge[20];
  sprintf(challenge, "%s", sendStatusChallenge);
  doc3["challenge"] = challenge;

  serializeJson(doc3, jsonstring);
  pCharacteristic->setValue(jsonstring);
  pCharacteristic->notify();
}

void sendData4() {
  // Daten als JSON Objekt kodieren, in JSON String
  // umwandeln und per BT senden
  StaticJsonDocument<2500> doc4;
  char jsonstring[2500];

  for (int i = 0; i < 48; i++) {
    doc4["arrLean"][i] = arrayCounterLeanBack[i];
  }

  serializeJson(doc4, jsonstring);
  pCharacteristic->setValue(jsonstring);
  pCharacteristic->notify();
}


// ============================ sendConfigData() ==================================== //

void sendConfigData() {
  // Daten als JSON Objekt kodieren, in JSON String
  // umwandeln und per BT senden
  StaticJsonDocument<200> doc;
  char jsonstring[200];

  char threshold[100];
  sprintf(threshold, "%d", thresholdBentBack);
  doc["thresholdBentBack"] = threshold;
  Serial.println("Daten gesendet");

  char thresholdLean[100];
  sprintf(thresholdLean, "%d", thresholdLeanBack);
  doc["thresholdLeanBack"] = thresholdLean;


  serializeJson(doc, jsonstring);
  pCharacteristic->setValue(jsonstring);
  pCharacteristic->notify();
}

// ======================== sendDataChallenge() ======================================= //

void sendDataChallenge() {
  // Daten als JSON Objekt kodieren, in JSON String
  // umwandeln und per BT senden
  StaticJsonDocument<100> doc;
  char jsonstring[100];

  char challenge[100];
  sprintf(challenge, "%s", sendStatusChallenge);
  doc["challenge"] = challenge;

  serializeJson(doc, jsonstring);
  pCharacteristic->setValue(jsonstring);
  pCharacteristic->notify();
}



// ========================================= challenge () ================================= //
void challenge() {

  // Stoppuhr wird einmalig gestartet
  if (startChallenge == true) {
    startTimeChallenge = millis();
    startChallenge = false;
    Serial.println("Challenge Start");
  }

  // Solgane die Stoppuhr kleiner der Zielzeit ist, werden die Werte des Beschleunigungssensors in
  // z- Richtung eingelsesen  und überprüft, ob eine gerade Haltung beibehalten wird
  if (millis() - startTimeChallenge < timeChallenge) {

    //Einlesen der Daten des MPU6050-Beschleunigungssensors
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3F);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr, 2, true);

    AcZ = Wire.read() << 8 | Wire.read();

    // Berechnung des mittlaufenden Mittelwertes
    // Rohwert wird dem Array hinzugefügt, NextAverageAcZ wird eins hochgezählt
    AverageBufferAcZ[NextAverageAcZ++] = AcZ;

    // wenn der NextAverageAcZ größer als die Anzahl der Messungen ist, wird der
    // NextAverageAcZ auf 0 gesetzt
    if (NextAverageAcZ >= AverageCount) {
      NextAverageAcZ = 0;
    }

    float averageAcZ = 0;

    for (int i = 0; i < AverageCount; ++i) {
      averageAcZ += AverageBufferAcZ[i];  // Summe der Messungen
    }
    averageAcZ /= AverageCount;  // Mittelwertberechnung

    // Berechnung des Winkels der Rückenhaltung
    angleAcZ = (averageAcZ / 16384.0) * 90;

    if (angleAcZ < thresholdBentBack) {

      // Einmaliges Starten der Zeitmessung, wenn Rückenhaltung ungerade ist,
      // um die Zeit zumessen, die in dieser Position verblieben wird
      if (startTimerBackBent == true) {
        startTime = millis();
        startTimerBackBent = false;
        startTimerBackStraight = true;
      }

      // Challenge wird abgebrochen, wenn länger als 10 Sekunden in einer
      // ungerade Haltung verbracht wird
      if (millis() - startTime > 10000) {
        Serial.println("Challenge abgebrochen");
        startTimerBackBent = true;
        sendStatusChallenge = "abgebrochen";
      }
    }

    // Einmaliges Starten der Zeitmessung, wenn Rückenhaltung wieder gerade ist
    if (startTimerBackStraight == true) {
      startTime2 = millis();
      startTimerBackStraight = false;
    }

    // wenn die Haltung für eine längere Zeit wieder gerade ist, wird die
    // Stoppuhr für die ungerade Haltung zurück gesetzt
    if (millis() - startTime2 > 5000) {
      startTimerBackBent = true;
    }

  } else {
    // Challenge beendet
    digitalWrite(vibrationPin, HIGH);
    delay(100);
    digitalWrite(vibrationPin, LOW);
    delay(100);
    digitalWrite(vibrationPin, HIGH);
    delay(100);
    digitalWrite(vibrationPin, LOW);

    Serial.println("Challenge geschafft");
    sendStatusChallenge = "geschafft";
    statusChallenge = "STOPP";
    statusMeasurement = "AN";
    startChallenge = true;
    startTimerBackBent = true;
    sendDataChallenge();
  }
}
