/*
  @file
  @date Created on: 02.05.2023
  @author HTWG/ C.Feng, D.Sept, S.Mewes
*/

// Bibliotheken einbinden
#include <ArduinoModbus.h>
#include <ArduinoRS485.h> 
#include <ArduinoMqttClient.h>
#include <MKRNB.h>
#include <SD.h>
#include <RTCZero.h>
#include <wdt_samd21.h>
#include "arduino_secrets.h" 

// Modbus-Konfiguration
#define SLAVE_ID 0x090 // Device Address des Ecowitt WS90 Windsensors
#define WIND_SPEED_REGISTER 0x9C96 // Register-Adresse der Windgeschwindigkeit
#define WIND_DIRECTION_REGISTER 0x9C98 // Register-Adresse der Windrichtung
#define TIME_OUT 65535

// Bibliotheksinstanz initialisieren
NBClient client;
GPRS gprs;
NB nbAccess;
NB_SMS sms;
MqttClient mqttClient(client);
RTCZero rtc;

// System-Konfiguration
const char PINNUMBER[] = SECRET_PINNUMBER; // Bitte geben Sie Ihre sensiblen PIN Nummer in "arduino_secrets.h" ein
unsigned long POLLING_INTERVAL = 1000; // Verzoegerungszeit in Millisekunden
unsigned int PACKAGE_SIZE = 8; // 8 Messungen werden in ein Paket eingepackt

// MQTT-Konfiguration
const char broker[] = "test.mosquitto.org";
const uint16_t port = 1883;
const uint8_t QoS = 2;
const char topic_windData[] = "HTWG_KN/Winddata";
const char topic_errorMessage[] = "HTWG_KN/Error";

// Array zum speichern der Nummer, von der eine SMS abgerufen wird
char senderNumber[20];

void setup() {
  // Baudrate des Modbus-Geraets (9600 bps), 8 Data bits, No Parity, 1 Stop
  if (!ModbusRTUClient.begin(9600, SERIAL_8N1)) {
    while (1);
  }
  
  // Verbindung zu NB IoT herstellen
  if (nbAccess.begin(PINNUMBER) != NB_READY) {
    while (1);
  }
  sms.clear();

  // SD-Karte initialisieren
  if (!SD.begin()) {
    while (1);
  }

  // Verbindung zu MQTT-Broker herstellen
  if (!mqttClient.connect(broker, port)) {
    while (1);
  }

  // Watchdog Timer mit 8 Sekunden (Auszeit) initialisieren
  wdt_init (WDT_CONFIG_PER_8K);

  rtc.begin();
  rtc.setEpoch(nbAccess.getLocalTime() + 7200); // + 7200 fuer deutsche Zeit (GMT+2)
}

void loop() {
  static unsigned long prevMillis = 0; // Vorherige Millisekunden speichern, um Intervalle zu berechnen
  static int count = 0; // Zaehler fuer die Anzahl der Messungen
  const int maxCount1 = 10; // Maximale Anzahl der Messungen, bevor das Intervall geaendert wird
  const int maxCount2 = 15; // Die Anzahl der zusaetzlichen Messungen mit 5 Messungen pro Sekunde
  int c;

  // Ueberpruefen, ob das festgelegte Intervall seit der letzten Messung vergangen ist
  if (millis() - prevMillis >= POLLING_INTERVAL) {
    prevMillis = millis(); // Aktualisieren der vorherigen Millisekunden

    String timestamp = String(rtc.getDay()) + "." + String(rtc.getMonth()) + "." + String(rtc.getYear()) + "," + String(rtc.getHours()) + ":" + String(rtc.getMinutes()) + ":" + String(rtc.getSeconds());
  
    saveDataToSDCard(timestamp); // Speichern die gelesenen Wind-Daten auf SD Karte

    // Wenn SMS empfangen ist
    if (sms.available()) {
      sms.remoteNumber(senderNumber, 20);

      // Nachrichtenbytes lesen und speichern
      String message = "";
      while ((c = sms.read()) != -1) {
        message += (char)c;
      }
      message.trim(); // Dadurch werden alle fuehrenden oder nachgestellten Leerzeichen entfernt

      // Ueberprueft den Inhalt der Nachricht und reagiert entsprechend
      if (message.equalsIgnoreCase("Send")) {
        // Mit dem Kommando "Send" werden alle Wind-Daten von SD-Karte ueber MQTT Broker gesendet
        // Ueberprueft, ob eine Verbindung zum Internet besteht
        if (nbAccess.status() == NB_READY) {
          // Ueberprueft, ob eine Verbindung zum MQTT-Broker besteht
          if (mqttClient.connected()) {
            sendDataFromSDCard(); // Senden auf SD Karte gespeicherte Wind-Daten ueber MQTT
          } else {
            // Versuchen die Verbindung zum MQTT-Broker wiederherzustellen
            if (mqttClient.connect(broker, port)) {
              // Verbindung zum MQTT-Broker erfolgreich
              sendDataFromSDCard();
            } else {
              // Verbindung zum MQTT-Broker fehlgeschlagen
            }
          }
        } else {
          // Versuchen die Verbindung zum Internet wiederherzustellen
          if (nbAccess.begin(PINNUMBER) == NB_READY) {
            // Versuchen die Verbindung zum MQTT-Broker wiederherzustellen
            if (mqttClient.connect(broker, port)) {
              sendDataFromSDCard(); // Senden auf SD Karte gespeicherte Wind-Daten ueber MQTT
            } else {
              // Verbindung zum MQTT-Broker fehlgeschlagen
            }
          } else {
            // Verbindung zum Internet fehlgeschlagen
          }
        }  
      } else if (message.equalsIgnoreCase("Delete")) {
        // Mit dem Kommando "Delete" wird die Datei "winddata.txt" von SD-Karte entfernt
        SD.remove("winddata.txt");
      } 
        
      sms.flush();
      // Nachricht aus dem Modemspeicher loeschen
      
    }

    mqttClient.poll(); // Signalisieren, dass der Client immer noch verbunden ist
    wdt_reset(); // Den Watchdog fuettern, um einen Neustart zu vermeiden

    count++; // Zaehler erhoehen
    
    if (count < maxCount1) {
      POLLING_INTERVAL = 1000; // 1 Messung pro Sekunde fuer die ersten 10 Sekunden
    } else if (count < maxCount2) {
      POLLING_INTERVAL = 200; // 5 Messungen pro Sekunde fuer die naechsten 1 Sekunde
    } else {
      POLLING_INTERVAL = 1000; // Nach 5 Messungen pro Sekunde, wieder 1 Messung pro Sekunde
      count = 0; // Zaehler zuruecksetzen
    }
  }
}

/**
 * Diese Funktion liest Windgeschwindigkeits- und Windrichtungsdaten ueber Modbus, erstellt einen Datenstring mit dem aktuellen Timestamp, 
 * der Windgeschwindigkeit und der Windrichtung, und speichert diesen Datenstring auf der SD-Karte.
 */
void saveDataToSDCard(String timestamp) {
  // Windgeschwindigkeitsregister und Windrichtungsregister lesen
  uint16_t windSpeedRaw = ModbusRTUClient.holdingRegisterRead(SLAVE_ID, WIND_SPEED_REGISTER);
  uint16_t windDirection = ModbusRTUClient.holdingRegisterRead(SLAVE_ID, WIND_DIRECTION_REGISTER);
  float windSpeed;

  if (windSpeedRaw == TIME_OUT || windDirection == TIME_OUT) {
    // Fehlermeldung ueber MQTT veroeffentlichen
    mqttClient.beginMessage(topic_errorMessage);
    mqttClient.print("Error: Time Out.");
    mqttClient.endMessage();
  } else {
    windSpeed = windSpeedRaw * 0.1; // Rohe Windgeschwindigkeit in tatsaechliche Windgeschwindigkeit umwandeln

    // Format in String
    String dataString = timestamp + "," + String(windSpeed, 1) + "," + String(windDirection);

    // Speichern die Daten auf SD Karte
    File dataFile = SD.open("winddata.txt", FILE_WRITE);
    if (dataFile) {
      dataFile.println(dataString);
      dataFile.close();
    }
  }
}

/**
 * Diese Funktion liest gespeicherte Winddaten von der SD-Karte und veroeffentlicht sie ueber MQTT.
 * Diese Funktion ist fuer die Situation gedacht, wenn das Kommando "Send" empfangen ist und die Verbindung zum MQTT-Broker da ist.
 */
void sendDataFromSDCard() {
  if (SD.exists("winddata.txt")) {
    // Deaktivieren jetzt WDT
    wdt_disable();

    File dataFile = SD.open("winddata.txt");
    if (dataFile) {
      String dataString;
      String totalData;
      int lineCount = 0;

      while (dataFile.available()) {
        dataString = dataFile.readStringUntil('\n');
        totalData += dataString + ";";
        lineCount++;

        // Wenn wir 8 Zeilen erreicht haben, senden wir die Daten
        if (lineCount == PACKAGE_SIZE) {
          mqttClient.beginMessage(topic_windData, false, QoS, false);
          mqttClient.print(totalData);
          mqttClient.endMessage();
          
          // Die Daten und den Zaehler zuruecksetzen
          totalData = "";
          lineCount = 0;
        }
      }
      
      // Senden aller verbleibenden Daten, falls weniger als 8 Zeilen uebrig sind
      if (lineCount > 0) {
        mqttClient.beginMessage(topic_windData, false, QoS, false);
        mqttClient.print(totalData);
        mqttClient.endMessage();
      }
      dataFile.close();

      // Aktivieren nun WDT wieder
      wdt_reEnable();
    } else {
    }
  }
}