/*******************************************************************************
 * LERNÜBERWACHUNGSSYSTEM (Study Monitoring System)
 *
 * Beschreibung: System zur Überwachung und Verwaltung von Lernzeit, Schlaf,
 *               Sitzhaltung und Lernumgebung des Nutzers.
 *
 * Hauptfunktionen:
 * - Verfolgung der Lern- und Schlafzeit in den letzten 24 Stunden
 * - Punkteberechnung basierend auf der Differenz zwischen Schlaf- und Lernzeit
 * - Warnung bei falscher Sitzhaltung (krumm sitzen, zu nah am Tisch)
 * - Überwachung der Umgebung (Temperatur, Luftfeuchtigkeit, Licht)
 * - Erinnerung an Pausen nach 45 Minuten kontinuierlichem Lernen
 * - Warnungen bei Gesundheitsproblemen und schlechten Lerngewohnheiten
 *
 * Autor: [Tra MY Le]
 * Erstellungsdatum: [01.2026]
 * OTH AMberg-Weiden / Logistik Digitalisierung
 * Fach: Capstone Project
 ******************************************************************************/

// ============================================================================
// ERFORDERLICHE BIBLIOTHEKEN (Required Libraries)
// ============================================================================
#include <Arduino.h>          // Grundlegende Bibliothek für Arduino/ESP32
#include <RTClib.h>           // Bibliothek für RTC DS1307 (Echtzeituhr)
#include <Wire.h>             // I2C-Kommunikationsbibliothek (für OLED und RTC)
#include <Adafruit_GFX.h>     // Grundlegende Grafikbibliothek für Displays
#include <Adafruit_SSD1306.h> // Bibliothek für OLED SSD1306 Display
#include <DHT.h>              // Bibliothek für DHT11 Sensor (Temperatur, Luftfeuchtigkeit)
#include <string.h>           // String-Verarbeitungsbibliothek
#include <WiFi.h>             // Wifi
#include <WebServer.h>        // Web

// ============================================================================
// OLED DISPLAY KONFIGURATION - I2C (SSD1306 128x64)
// ============================================================================
#define SCREEN_WIDTH 128 // OLED Display Breite (Pixel)
#define SCREEN_HEIGHT 64 // OLED Display Höhe (Pixel)
#define OLED_RESET -1    // Reset Pin (-1 wenn mit ESP32 Reset gemeinsam genutzt)
// Display-Objekt initialisieren mit I2C-Kommunikation
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
const char* ssid = "ESP32_LIGHT_SENSOR"; //WLAN Name erstellen
const char* password = "12345678";       //Passwort erstellen 

const int lightSensorPin = 34;  
// AO an GPIO34 anschließen (liest den Wertebereich und gibt Lichtintensität zurück)
// D0 an Pin 4 anschließen (gibt den Wert 1/0 zurück)
int lightValue = 0; // Lichtwert initialisieren

WebServer server(80);

// ============================================================================
// RTC DS1307 KONFIGURATION (Real Time Clock - Echtzeituhr)
// ============================================================================
RTC_DS1307 rtc; // Uhrmodul zur Zeitaufzeichnung auch bei Stromausfall

// ============================================================================
// DHT11 SENSOR KONFIGURATION (Temperatur- und Luftfeuchtigkeitssensor)
// ============================================================================
#define DHT_PIN 13          // DHT11 DATA Pin verbunden mit GPIO 13
#define DHT_TYPE DHT11      // Sensortyp (DHT11)
DHT dht(DHT_PIN, DHT_TYPE); // DHT11 Sensor-Objekt initialisieren

// ============================================================================
// TASTEN KONFIGURATION (Buttons)
// ============================================================================
#define BUTTON_STUDY_PIN 26 // Start/Ende Lernsession - GPIO 26
#define BUTTON_SLEEP_PIN 27 // Start/Ende Schlafsession - GPIO 27

// ============================================================================
// LICHT SENSOR UND AUSGABEGERÄTE KONFIGURATION (LDR, LED, Buzzer)
// ============================================================================
#define LDR_PIN 4        // Lichtsensor LDR - GPIO 4
#define LED_RED_PIN 32   // Rote LED (Warnung) - GPIO 32
#define LED_GREEN_PIN 19 // Grüne LED (Pause) - GPIO 19
#define BUZZER_PIN 15    // Summer (active low) - GPIO 15

// ============================================================================
// HC-SR04 ULTRASCHALLSENSOREN KONFIGURATION (Ultrasonic Sensors)
// ============================================================================
// Sensor 1: Erkennung von krummer Sitzhaltung
#define ULTRASONIC1_TRIG_PIN 12 // Trigger Pin - GPIO 12
#define ULTRASONIC1_ECHO_PIN 14 // Echo Pin - GPIO 14

// Sensor 2: Erkennung von zu nahem Sitzen am Tisch
#define ULTRASONIC2_TRIG_PIN 25 // Trigger Pin - GPIO 25
#define ULTRASONIC2_ECHO_PIN 33 // Echo Pin - GPIO 33

// ============================================================================
// SCHWELLENWERT KONFIGURATION (THRESHOLD VALUES) - Alle Werte anpassbar
// ============================================================================
// Schwellenwerte für Lern- und Schlafzeiten
#define STUDY_TIME_MAX_HOURS 6.0 // Warnung wenn Lernzeit > 6 Stunden/24h
#define SLEEP_TIME_MIN_HOURS 7.0 // Warnung wenn Schlafzeit < 7 Stunden/24h

// Konfiguration für kontinuierliches Lernen
#define STUDY_WARNING_TIME_MINUTES 2 // Automatisches Session-Ende nach 45 Minuten
#define GREEN_LED_DURATION_MINUTES 1  // Grüne LED leuchtet 5 Minuten nach Pausenerinnerung

// Luftfeuchtigkeit Schwellenwerte
#define HUMIDITY_HIGH_THRESHOLD 80.0 // Warnung wenn Luftfeuchtigkeit > 80%
#define HUMIDITY_LOW_THRESHOLD 40.0  // Warnung wenn Luftfeuchtigkeit < 40%

// Lernzeitbeschränkung (spätes Lernen verbieten)
#define STUDY_LATE_START_HOUR 0 // Beginn der verbotenen Lernzeit (0h - Mitternacht)
#define STUDY_LATE_END_HOUR 4   // Ende der verbotenen Lernzeit (4h morgens)

// Abstandsschwellenwerte für Ultraschallsensoren
#define ULTRASONIC1_THRESHOLD_CM 20 // Haltungswarnung wenn < 20cm
#define ULTRASONIC2_THRESHOLD_CM 30 // Tischnähe Warnung wenn < 30cm

// Umrechnung in Sekunden für Berechnungen
const unsigned long STUDY_WARNING_TIME = STUDY_WARNING_TIME_MINUTES * 60;
const unsigned long GREEN_LED_DURATION = GREEN_LED_DURATION_MINUTES * 60;

// ============================================================================
// LERNSESSION VERFOLGUNGSVARIABLEN (Study Session Tracking)
// ============================================================================
unsigned long continuousStudyStartTime = 0; // Startzeit des kontinuierlichen Lernens
bool greenLedActive = false;                // Status der grünen LED (Pausenmodus)
unsigned long greenLedStartTime = 0;        // Startzeit der grünen LED

// ============================================================================
// TASTEN-INTERRUPT FLAGS (Button Interrupt Flags)
// ============================================================================
volatile bool studyButtonFlag = false;         // Flag für Tastendruck Lern-Taste
volatile bool sleepButtonFlag = false;         // Flag für Tastendruck Schlaf-Taste
volatile unsigned long lastStudyInterrupt = 0; // Zeitpunkt des letzten Lern-Tasten-Interrupts
volatile unsigned long lastSleepInterrupt = 0; // Zeitpunkt des letzten Schlaf-Tasten-Interrupts
const unsigned long debounceDelay = 200;       // Entprellzeit (ms)

// ============================================================================
// SESSION DATENSTRUKTUR (Session Structure)
// ============================================================================
// Struktur zur Speicherung einer aktiven Lern/Schlaf-Session
struct Session
{
    unsigned long startTime; // Startzeit (Unix Timestamp - Sekunden)
    unsigned long endTime;   // Endzeit (Unix Timestamp - Sekunden)
    bool isActive;           // Aktivstatus (true = aktiv)
};

// Aktuelle Lern- und Schlaf-Session
Session studySession = {0, 0, false}; // Lern-Session initialisieren (inaktiv)
Session sleepSession = {0, 0, false}; // Schlaf-Session initialisieren (inaktiv)

// ============================================================================
// SESSION VERLAUFSSPEICHER (Session History Storage)
// ============================================================================
#define MAX_SESSIONS 100 // Maximale Anzahl gespeicherter Sessions (24h Zyklus)

// Struktur zur Speicherung einer abgeschlossenen Session
struct SessionRecord
{
    unsigned long startTime; // Startzeit (Unix Timestamp)
    unsigned long endTime;   // Endzeit (Unix Timestamp)
    bool isStudy;            // Session-Typ (true = lernen, false = schlafen)
};

// Arrays zur Speicherung des Session-Verlaufs
SessionRecord studySessions[MAX_SESSIONS]; // Verlauf der Lern-Sessions
SessionRecord sleepSessions[MAX_SESSIONS]; // Verlauf der Schlaf-Sessions
int studySessionCount = 0;                 // Anzahl gespeicherter Lern-Sessions
int sleepSessionCount = 0;                 // Anzahl gespeicherter Schlaf-Sessions

// ============================================================================
// DISPLAY UPDATE VARIABLEN (Display Update Variables)
// ============================================================================
unsigned long lastDisplayUpdate = 0; // Zeitpunkt des letzten Display-Updates

// ============================================================================
// ZEITUMRECHNUNGSFUNKTIONEN (Time Conversion Functions)
// ============================================================================

/**
 * Konvertiert DateTime zu Unix Timestamp (Sekunden seit 1/1/1970)
 * @param dt: Zu konvertierendes DateTime-Objekt
 * @return: Unix timestamp (Sekunden)
 */
unsigned long dateTimeToUnix(DateTime dt)
{
    return dt.unixtime();
}

/**
 * Holt aktuelle Zeit von RTC als Unix Timestamp
 * @return: Unix timestamp aktuell (Sekunden)
 */
unsigned long getCurrentUnixTime()
{
    DateTime now = rtc.now();
    return dateTimeToUnix(now);
}

// ============================================================================
// ZEITBERECHNUNGSFUNKTIONEN (Time Calculation Functions)
// ============================================================================

/**
 * Berechnet Gesamtzeit aller Sessions in den letzten 24 Stunden
 * @param sessions: Array mit Session-Verlauf
 * @param count: Anzahl der Sessions im Array
 * @return: Gesamtzeit (Sekunden)
 *
 * Erklärung: Funktion durchläuft alle Sessions und berechnet nur die Zeit,
 * die innerhalb der letzten 24 Stunden liegt
 */
unsigned long calculateTotalTime(SessionRecord *sessions, int count)
{
    unsigned long currentTime = getCurrentUnixTime();
    unsigned long time24HoursAgo = currentTime - (24 * 3600); // 24h = 86400 Sekunden
    unsigned long totalSeconds = 0;

    // Durchläuft alle gespeicherten Sessions
    for (int i = 0; i < count; i++)
    {
        unsigned long sessionStart = sessions[i].startTime;
        unsigned long sessionEnd = sessions[i].endTime;

        // Berechnet nur Sessions die innerhalb der letzten 24 Stunden liegen
        if (sessionEnd > time24HoursAgo)
        {
            // Berechnet die tatsächliche Zeit innerhalb der 24 Stunden
            unsigned long actualStart = (sessionStart > time24HoursAgo) ? sessionStart : time24HoursAgo;
            unsigned long actualEnd = (sessionEnd < currentTime) ? sessionEnd : currentTime;

            if (actualEnd > actualStart)
            {
                totalSeconds += (actualEnd - actualStart);
            }
        }
    }

    return totalSeconds;
}

//---------------------------------
//Funktion zum Lesen des Lichtsensorwerts, Funktion stellt die Webseitengrafik bereit

void handleData() {
  lightValue = analogRead(lightSensorPin);
  // ADC-Wert (0-4095) in Helligkeit (%) umwandeln

   float brightness = map(lightValue, 0, 4095, 0, 100);
  String json = "{\"brightness\": " + String(brightness) + "}";
  server.send(200, "application/json", json);
}

// Funktion für Webgrafiken

void handleRoot() {
  
String html = R"rawliteral(
  <!DOCTYPE html>
  <html>
  <head>
    <title>ESP32 Light Monitor</title>
    <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
  </head>
  <body>
    <h2>Aktuelle Helligkeit</h2>
    <canvas id="lightChart" width="600" height="300"></canvas>
    <script>
      const ctx = document.getElementById('lightChart').getContext('2d');
      const data = {
        labels: [],
        datasets: [{
          label: 'Helligkeit (%)',
          data: [],
          fill: false,
          borderColor: 'rgb(75, 192, 192)',
          tension: 0.1
        }]
      };
      const config = {
        type: 'line',
        data: data,
        options: {
          scales: {
            y: { min: 0, max: 100 }
          }
        }
      };
      const myChart = new Chart(ctx, config);

      async function fetchData() {
        const response = await fetch('/data');
        const json = await response.json();
        const time = new Date().toLocaleTimeString();
        data.labels.push(time);
        data.datasets[0].data.push(json.brightness);
        if(data.labels.length > 20){ // Nur 20 Punkte behalten
          data.labels.shift();
          data.datasets[0].data.shift();
        }
        myChart.update();
      }

      setInterval(fetchData, 1000); // Jede Sekunde aktualisieren
    </script>
  </body>
  </html>
  )rawliteral";

  server.send(200, "text/html", html);
}
//---------------------------------

// ============================================================================
// SESSION VERLAUFSVERWALTUNG (Session History Management)
// ============================================================================

/**
 * Fügt Lern-Session zum Verlauf hinzu
 * @param start: Startzeit (Unix Timestamp)
 * @param end: Endzeit (Unix Timestamp)
 *
 * Erklärung: Wenn Array voll (MAX_SESSIONS erreicht), wird die älteste Session
 * entfernt indem alle Elemente nach links verschoben werden
 */
void addStudySession(unsigned long start, unsigned long end)
{
    if (studySessionCount >= MAX_SESSIONS)
    {
        // Array voll - verschiebe alle Elemente nach links (entferne erstes Element)
        for (int i = 0; i < MAX_SESSIONS - 1; i++)
        {
            studySessions[i] = studySessions[i + 1];
        }
        studySessionCount = MAX_SESSIONS - 1;
    }
    // Füge neue Session am Ende des Arrays hinzu
    studySessions[studySessionCount].startTime = start;
    studySessions[studySessionCount].endTime = end;
    studySessions[studySessionCount].isStudy = true;
    studySessionCount++;
}

/**
 * Fügt Schlaf-Session zum Verlauf hinzu
 * @param start: Thời gian bắt đầu (Unix timestamp)
 * @param end: Thời gian kết thúc (Unix timestamp)
 *
 * Erklärung: Ähnlich wie addStudySession, für Schlaf-Sessions
 */
void addSleepSession(unsigned long start, unsigned long end)
{
    if (sleepSessionCount >= MAX_SESSIONS)
    {
        // Array voll - verschiebe alle Elemente nach links (entferne erstes Element)
        for (int i = 0; i < MAX_SESSIONS - 1; i++)
        {
            sleepSessions[i] = sleepSessions[i + 1];
        }
        sleepSessionCount = MAX_SESSIONS - 1;
    }
    // Füge neue Session am Ende des Arrays hinzu
    sleepSessions[sleepSessionCount].startTime = start;
    sleepSessions[sleepSessionCount].endTime = end;
    sleepSessions[sleepSessionCount].isStudy = false;
    sleepSessionCount++;
}

// ============================================================================
// DISPLAY FORMATIERUNGSFUNKTIONEN (Display Formatting Functions)
// ============================================================================

/**
 * Formatiert Zeit als Dezimalstunden (z.B. 3.32h)
 * @param totalSeconds: Zu konvertierende Sekunden
 * @return: Formatierter String (VD: "3.32h")
 */
String formatTimeDecimal(unsigned long totalSeconds)
{
    float hours = totalSeconds / 3600.0;
    char buffer[15];
    sprintf(buffer, "%.2fh", hours);
    return String(buffer);
}

/**
 * Formatiert Punktestand als Dezimalstunden (z.B. 3.48 oder -4.56)
 * @param scoreSeconds: Punktestand in Sekunden (kann negativ sein)
 * @return: Formatierter String (VD: "3.48" hoặc "-4.56")
 */
String formatScoreDecimal(long scoreSeconds)
{
    float scoreHours = (float)scoreSeconds / 3600.0f;
    char buffer[20];

    // Sicherstellt dass negative Zahlen korrekt angezeigt werden
    if (scoreHours < 0)
    {
        sprintf(buffer, "-%.2f", -scoreHours); // Verarbeitet negative Zahlen
    }
    else
    {
        sprintf(buffer, "%.2f", scoreHours);
    }
    return String(buffer);
}

// ============================================================================
// ULTRASCHALLSENSOR FUNKTIONEN (Ultrasonic Sensor Functions)
// ============================================================================

/**
 * Liest Entfernung vom HC-SR04 Ultraschallsensor
 * @param trigPin: Trigger Pin des Sensors
 * @param echoPin: Echo Pin des Sensors
 * @return: Entfernung in cm (999 bei Fehler oder außerhalb der Reichweite)
 *
 * Funktionsprinzip:
 * 1. Sende 10µs HIGH Puls an Trigger Pin
 * 2. Sensor sendet Ultraschallwelle
 * 3. Misst Zeit bis Echo zurückkommt (über Echo Pin)
 * 4. Berechnet Entfernung: distance = (duration * 0.034) / 2
 *    (0.034 cm/µs ist Schallgeschwindigkeit, /2 weil Welle hin und zurück)
 */
long readUltrasonicDistance(int trigPin, int echoPin)
{
    // Sicherstellt dass Trigger Pin LOW ist
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);

    // Sende 10µs HIGH Puls zur Sensoraktivierung
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    // Misst Zeit bis Echo Pin HIGH wird (Timeout 30ms)
    long duration = pulseIn(echoPin, HIGH, 30000);
    if (duration == 0)
    {
        return 999; // Fehler oder außerhalb der Reichweite
    }

    // Berechnet Entfernung (cm)
    long distance = duration * 0.034 / 2;
    if (distance > 400)
    {
        return 999; // Außerhalb der Sensorreichweite (>4m)
    }
    return distance;
}

// ============================================================================
// INTERRUPT FUNKTIONEN (Interrupt Service Routines - ISR)
// ============================================================================

/**
 * ISR für Lern-Taste (Study Button)
 *
 * Erklärung: Wird aufgerufen wenn Lern-Taste gedrückt wird (fallende Flanke FALLING)
 * Verwendet Entprellung um Rauschen und Fehldrücke zu vermeiden
 */
void IRAM_ATTR studyButtonISR()
{
    unsigned long currentMillis = millis();
    // Verarbeitet nur wenn Entprellzeit (200ms) vergangen ist
    if (currentMillis - lastStudyInterrupt > debounceDelay)
    {
        studyButtonFlag = true;
        lastStudyInterrupt = currentMillis;
    }
}

/**
 * ISR für Schlaf-Taste (Sleep Button)
 *
 * Erklärung: Wird aufgerufen wenn Schlaf-Taste gedrückt wird (fallende Flanke FALLING)
 * Verwendet Entprellung um Rauschen und Fehldrücke zu vermeiden
 */
void IRAM_ATTR sleepButtonISR()
{
    unsigned long currentMillis = millis();
    // Verarbeitet nur wenn Entprellzeit (200ms) vergangen ist
    if (currentMillis - lastSleepInterrupt > debounceDelay)
    {
        sleepButtonFlag = true;
        lastSleepInterrupt = currentMillis;
    }
}

// ============================================================================
//  DISPLAY FUNKTIONEN (Display Functions)
// ============================================================================

/**
 * Hauptdisplay - Zeigt Übersichtsinformationen
 *
 * Display-Layout:
 * Zeile 0 (y=0):  [Zeit HH:MM] | SCORE: [Punktestand]
 * Zeile 1 (y=12): ST: [Lernzeit]h | SL: [Schlafzeit]h
 * Zeile 2 (y=24): T: [Temperatur]°C | H: [Luftfeuchtigkeit]%
 * Zeile 3 (y=36): Dis1: [Entfernung 1]cm
 * Zeile 4 (y=48): Dis2: [Entfernung 2]cm
 */
void mainDisplay()
{
    DateTime now = rtc.now();

    // Berechnet Gesamtlern- und Schlafzeit der letzten 24h
    unsigned long studyTime = calculateTotalTime(studySessions, studySessionCount);
    unsigned long sleepTime = calculateTotalTime(sleepSessions, sleepSessionCount);

    // Addiert Zeit der aktiven Session (falls vorhanden)
    unsigned long currentUnix = getCurrentUnixTime();
    if (studySession.isActive && studySession.startTime > 0)
    {
        studyTime += (currentUnix - studySession.startTime);
    }
    if (sleepSession.isActive && sleepSession.startTime > 0)
    {
        sleepTime += (currentUnix - sleepSession.startTime);
    }

    //Berechnet Punktestand: SCORE = Schlafzeit - Lernzeit
    // Positiver Score = mehr Schlaf als Lernen (gut)
    // Negativer Score = mehr Lernen als Schlaf (nicht gut)
    long score = (long)((long)sleepTime - (long)studyTime);

    display.clearDisplay();

    // Zeile 1: Aktuelle Zeit und Punktestand
    char timeStr[10];
    sprintf(timeStr, "%02d:%02d", now.hour(), now.minute());
    display.setCursor(0, 0);
    display.print(timeStr);
    display.print(" | SCORE:");
    display.print(formatScoreDecimal(score).c_str());

    // Zeile 2: Lernzeit (ST) und Schlafzeit (SL)
    display.setCursor(0, 12);
    display.print("ST:");
    display.print(formatTimeDecimal(studyTime).c_str());
    display.print(" | SL:");
    display.print(formatTimeDecimal(sleepTime).c_str());

    // Zeile 3: Temperatur (T) und Luftfeuchtigkeit (H) von DHT11
    float temperature = dht.readTemperature();
    float humidity = dht.readHumidity();

    // Überprüft ob Sensorlesung erfolgreich war
    if (!isnan(temperature) && !isnan(humidity))
    {
        display.setCursor(0, 24);
        display.print("T:");
        display.print(temperature, 1); // 1 Dezimalstelle
        display.print("C");

        display.print(" | H:");
        display.print(humidity, 1); // 1 Dezimalstelle
        display.print("%");
    }

    // Zeile 4-5: Entfernungen von 2 Ultraschallsensoren
    long distance1 = readUltrasonicDistance(ULTRASONIC1_TRIG_PIN, ULTRASONIC1_ECHO_PIN);
    long distance2 = readUltrasonicDistance(ULTRASONIC2_TRIG_PIN, ULTRASONIC2_ECHO_PIN);

    display.setCursor(0, 36);
    display.print("Dis1:");
    if (distance1 < 999)
    {
        display.print(distance1);
        display.print("cm");
    }
    else
    {
        display.print("---"); // Zeigt --- bei Fehler
    }

    display.setCursor(0, 48);
    display.print("Dis2:");
    if (distance2 < 999)
    {
        display.print(distance2);
        display.print("cm");
    }
    else
    {
        display.print("---"); // Zeigt --- bei Fehler
    }

    display.display();
}

/**
 * Warnungsdisplay - Zeigt Gesundheits- und Lernwarnungen
 *
 * Layout: Zeigt Überschrift "WARMING:" und Liste der Warnungen
 * Geprüfte Warnungen:
 * 1. Zu viel Lernen (> 6 Stunden/24h)
 * 2. Zu wenig Schlaf (< 7 Stunden/24h)
 * 3. Hohe Luftfeuchtigkeit (> 80%)
 * 4. Niedrige Luftfeuchtigkeit (< 40%)
 * 5. Spätes Lernen (0h-4h morgens)
 * 6. Ungenügende Beleuchtung
 * 7. Falsche Sitzhaltung (krumm sitzen)
 * 8. Zu nah am Tisch sitzen
 */
void warmingDisplay()
{
    DateTime now = rtc.now();

    // Berechnet Gesamtlern- und Schlafzeit
    unsigned long studyTime = calculateTotalTime(studySessions, studySessionCount);
    unsigned long sleepTime = calculateTotalTime(sleepSessions, sleepSessionCount);

    // Addiert Zeit der aktiven Session
    unsigned long currentUnix = getCurrentUnixTime();
    if (studySession.isActive && studySession.startTime > 0)
    {
        studyTime += (currentUnix - studySession.startTime);
    }
    if (sleepSession.isActive && sleepSession.startTime > 0)
    {
        sleepTime += (currentUnix - sleepSession.startTime);
    }

    // Liest Luftfeuchtigkeit von DHT11
    float humidity = dht.readHumidity();

    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("WARMING:");

    int yPos = 12;             // Start-Y-Position für Warnungen
    const int lineHeight = 12; // Zeilenhöhe

    // Warnung 1: Zu viel Lernen (> 6 Stunden/24h)
    float studyHours = studyTime / 3600.0;
    if (studyHours > STUDY_TIME_MAX_HOURS)
    {
        display.setCursor(0, yPos);
        display.print("Study too much!");
        yPos += lineHeight;
    }

    // Warnung 2: Zu wenig Schlaf (< 7 Stunden/24h)
    float sleepHours = sleepTime / 3600.0;
    if (sleepHours < SLEEP_TIME_MIN_HOURS)
    {
        display.setCursor(0, yPos);
        display.print("Need sleep more!");
        yPos += lineHeight;
    }

    // Warnung 3: Hohe Luftfeuchtigkeit (> 80%)
    if (!isnan(humidity) && humidity > HUMIDITY_HIGH_THRESHOLD)
    {
        display.setCursor(0, yPos);
        display.print("High hum, open door!");
        yPos += lineHeight;
    }

    // Warnung 4: Niedrige Luftfeuchtigkeit (< 40%)
    if (!isnan(humidity) && humidity < HUMIDITY_LOW_THRESHOLD)
    {
        display.setCursor(0, yPos);
        display.print("Low hum, drink water!");
        yPos += lineHeight;
    }

    // Warnung 5: Spätes Lernen (0h-4h morgens)
    if (studySession.isActive && studySession.startTime > 0)
    {
        DateTime studyStart = DateTime(studySession.startTime);
        int studyHour = studyStart.hour();
        if (studyHour >= STUDY_LATE_START_HOUR && studyHour < STUDY_LATE_END_HOUR)
        {
            display.setCursor(0, yPos);
            display.print("Too late, sleep now!");
            yPos += lineHeight;
        }
    }

    // Warnung 6: Ungenügende Beleuchtung
    // LDR mit INPUT_PULLUP: HIGH = dunkel (Licht benötigt), LOW = hell
    bool lightEnough = digitalRead(LDR_PIN);
    if (lightEnough)
    { // 
        display.setCursor(0, yPos);
        display.print("Turn on lamp!");
        yPos += lineHeight;
    }

    // Warnung 7: Falsche Sitzhaltung (Sensor 1)
    long distance1 = readUltrasonicDistance(ULTRASONIC1_TRIG_PIN, ULTRASONIC1_ECHO_PIN);
    if (distance1 < ULTRASONIC1_THRESHOLD_CM && distance1 > 0)
    {
        display.setCursor(0, yPos);
        display.print("Sit up straight!");
        yPos += lineHeight;
    }

    // Warnung 8: Zu nah am Tisch sitzen (Sensor 2)
    long distance2 = readUltrasonicDistance(ULTRASONIC2_TRIG_PIN, ULTRASONIC2_ECHO_PIN);
    if (distance2 < ULTRASONIC2_THRESHOLD_CM && distance2 > 0)
    {
        display.setCursor(0, yPos);
        display.print("Look far away!");
        yPos += lineHeight;
    }

    display.display();
}

/**
 * Zeigt Textnachricht für 1 Sekunde
 * @param text: Anzuzeigender Textstring
 *
 * Wird verwendet für kurze Nachrichten wie:
 * - "START STUDY" / "END STUDY"
 * - "START SLEEP" / "END SLEEP"
 * - "REST 5 MINUTES"
 */
void textDisplay(String text)
{
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print(text.c_str());
    display.display();
    delay(2000); // Anzeige für 2 Sekunde
}

// ============================================================================
// SETUP FUNKTION - SYSTEMINITIALISIERUNG
// ============================================================================

void setup()
{
    // Serial für Debugging initialisieren
    Serial.begin(115200);
    delay(100);

    // I2C-Kommunikation initialisieren
    Wire.begin();

    // RTC initialisieren
    if (!rtc.begin())
    {
        Serial.println("Couldn't find RTC");
        while (1)
            ; // Stoppt Programm wenn RTC nicht gefunden
    }

    // RTC-Zeit von Compile-Zeit setzen (nur einmalig ausführen)
    //  um Zeit zu setzen, danach wieder kommentieren
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

    // OLED Display initialisieren
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
    {
        Serial.println(F("SSD1306 allocation failed"));
        for (;;)
            ; // Stoppt Programm wenn OLED-Initialisierung fehlschlägt
    }
    display.clearDisplay();
    display.setTextSize(1);              // Schriftgröße 1
    display.setTextColor(SSD1306_WHITE); // Schriftfarbe weiß
    display.setCursor(0, 28);
    display.print("Initializing...");
    display.display();

    // DHT11 Sensor initialisieren
    dht.begin();
    delay(100); // Wartet bis Sensor stabilisiert


    // Tasten mit Pull-up initialisieren
    pinMode(BUTTON_STUDY_PIN, INPUT_PULLUP);
    pinMode(BUTTON_SLEEP_PIN, INPUT_PULLUP);

    // LDR, LED und Summer initialisieren
    pinMode(LDR_PIN, INPUT_PULLUP);
    pinMode(LED_RED_PIN, OUTPUT);
    pinMode(LED_GREEN_PIN, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(LED_RED_PIN, LOW);   // Rote LED ausschalten
    digitalWrite(LED_GREEN_PIN, LOW); // Grüne LED ausschalten
    digitalWrite(BUZZER_PIN, HIGH);   // Summer ausschalten (active low, HIGH = aus)

    // Ultraschallsensoren initialisieren
    pinMode(ULTRASONIC1_TRIG_PIN, OUTPUT);
    pinMode(ULTRASONIC1_ECHO_PIN, INPUT);
    pinMode(ULTRASONIC2_TRIG_PIN, OUTPUT);
    pinMode(ULTRASONIC2_ECHO_PIN, INPUT);
    digitalWrite(ULTRASONIC1_TRIG_PIN, LOW);
    digitalWrite(ULTRASONIC2_TRIG_PIN, LOW);

    // Interrupt-Funktionen für Tasten zuweisen
    // FALLING = aktiviert wenn Taste von HIGH zu LOW wechselt (gedrückt)
    attachInterrupt(digitalPinToInterrupt(BUTTON_STUDY_PIN), studyButtonISR, FALLING);
    attachInterrupt(digitalPinToInterrupt(BUTTON_SLEEP_PIN), sleepButtonISR, FALLING);

    delay(500);

    Serial.println("System initialized");
    mainDisplay(); // Hauptdisplay anzeigen
}

// ============================================================================
// LOOP FUNKTION - HAUPTPROGRAMMSCHLEIFE
// ============================================================================

void loop()
{
    unsigned long currentTime = getCurrentUnixTime();

    // =========================================================================
    // LERN-TASTEN INTERRUPT VERARBEITUNG (Study Button)
    // =========================================================================
    if (studyButtonFlag)
    {
        studyButtonFlag = false; // Flag zurücksetzen

        if (studySession.isActive)
        {
            // Fall 1: Lernen aktiv → Drücken beendet Lernen
            studySession.endTime = currentTime;
            addStudySession(studySession.startTime, studySession.endTime);
            studySession.isActive = false;
            continuousStudyStartTime = 0; // Kontinuierliches Lernen zurücksetzen
            Serial.println("Study session ended");
            textDisplay("END STUDY");
        }
        else
        {
            // Fall 2: Lernen inaktiv → Drücken startet Lernen
            // Wenn Schlaf aktiv, automatisch Schlaf-Session beenden
            if (sleepSession.isActive)
            {
                sleepSession.endTime = currentTime;
                addSleepSession(sleepSession.startTime, sleepSession.endTime);
                sleepSession.isActive = false;
                Serial.println("Sleep session ended (auto)");
                textDisplay("END SLEEP");
            }
            studySession.startTime = currentTime;
            studySession.isActive = true;
            continuousStudyStartTime = currentTime; // Startet Zählung für kontinuierliches Lernen
            Serial.println("Study session started");
            textDisplay("START STUDY");
        }
        mainDisplay();
    }

    // =========================================================================
    // SCHLAF-TASTEN INTERRUPT VERARBEITUNG (Sleep Button)
    // =========================================================================
    if (sleepButtonFlag)
    {
        sleepButtonFlag = false; // Flag zurücksetzen

        if (sleepSession.isActive)
        {
            //Fall 1: Schlaf aktiv → Drücken beendet Schlaf
            sleepSession.endTime = currentTime;
            addSleepSession(sleepSession.startTime, sleepSession.endTime);
            sleepSession.isActive = false;
            Serial.println("Sleep session ended");
            textDisplay("END SLEEP");
        }
        else
        {
            // Fall 2: Schlaf inaktiv → Drücken startet Schlaf
            // Wenn Lernen aktiv, automatisch Lern-Session beenden
            if (studySession.isActive)
            {
                studySession.endTime = currentTime;
                addStudySession(studySession.startTime, studySession.endTime);
                studySession.isActive = false;
                Serial.println("Study session ended (auto)");
                textDisplay("END STUDY");
            }
            sleepSession.startTime = currentTime;
            sleepSession.isActive = true;
            continuousStudyStartTime = 0; // Kontinuierliches Lernen zurücksetzen
            Serial.println("Sleep session started");
            textDisplay("START SLEEP");
        }
        mainDisplay();
    }

    // =========================================================================
    // SENSORPRÜFUNG UND ROTE LED + SUMMER STEUERUNG
    // =========================================================================
    bool redLedOn = false; // Statusverfolgung für rote LED

    // Lichtprüfung (LDR)
    bool lightEnough = digitalRead(LDR_PIN);
    if (lightEnough)
    { // HIGH = dunkel → rote LED einschalten
        redLedOn = true;
    }

    // Haltungsprüfung (Ultraschallsensor 1)
    long distance1 = readUltrasonicDistance(ULTRASONIC1_TRIG_PIN, ULTRASONIC1_ECHO_PIN);
    if (distance1 < ULTRASONIC1_THRESHOLD_CM && distance1 > 0)
    {
        redLedOn = true;
        digitalWrite(BUZZER_PIN, LOW); // Warnton einschalten (active low)
        delay(1000);
        digitalWrite(BUZZER_PIN, HIGH);
        textDisplay("Schiefe Haltung");
        // 
        warmingDisplay(); // 
        delay(2000);
        mainDisplay(); // Zum Hauptbildschirm zurückkehren
        lastDisplayUpdate = millis();
    }

    // Tischnähe Prüfung (Ultraschallsensor 2)
    long distance2 = readUltrasonicDistance(ULTRASONIC2_TRIG_PIN, ULTRASONIC2_ECHO_PIN);
    if (distance2 < ULTRASONIC2_THRESHOLD_CM && distance2 > 0)
    {
        redLedOn = true;
        digitalWrite(BUZZER_PIN, LOW); // Summer einschalten (active low)
        delay(1000);
        digitalWrite(BUZZER_PIN, HIGH);
        textDisplay("zu nah am Tisch");
        // 
        warmingDisplay(); // 
        delay(2000);
        mainDisplay(); // Zum Hauptbildschirm zurückkehren
        lastDisplayUpdate = millis();
    }

    // Rote LED basierend auf allen Bedingungen steuern
    if (redLedOn)
    {
        digitalWrite(LED_RED_PIN, HIGH);
    }
    else
    {
        digitalWrite(LED_RED_PIN, LOW);
        digitalWrite(BUZZER_PIN, HIGH); // ausschalten  (inactive high)
    }

    // =========================================================================
    // 45 MINUTEN KONTINUIERLICHES LERNEN PRÜFEN → PAUSENERINNERUNG
    // =========================================================================
    if (studySession.isActive && continuousStudyStartTime > 0)
    {
        unsigned long continuousStudyDuration = currentTime - continuousStudyStartTime;

        // Wenn 45 Minuten kontinuierliches Lernen erreicht und grüne LED noch nicht aktiv
        if (continuousStudyDuration >= STUDY_WARNING_TIME && !greenLedActive)
        {
            //  Pausenmodus aktivieren
            greenLedActive = true;
            greenLedStartTime = currentTime;

            // Lern-Session automatisch beenden
            studySession.endTime = currentTime;
            addStudySession(studySession.startTime, studySession.endTime);
            studySession.isActive = false;
            continuousStudyStartTime = 0;

            Serial.println("45 minutes study reached - ending session");

            // Grüne LED einschalten
            digitalWrite(LED_GREEN_PIN, HIGH);

            // Summer für 1 Sekunde einschalten
            digitalWrite(BUZZER_PIN, LOW);
            textDisplay("REST 5 MINUTES");
            delay(1000);
            digitalWrite(BUZZER_PIN, HIGH);
        }
    }

    // =========================================================================
    // Verwaltung der grünen LED-Zeit (5 Minuten)
    // =========================================================================
    if (greenLedActive)
    {
        unsigned long greenLedDuration = currentTime - greenLedStartTime;
        if (greenLedDuration >= GREEN_LED_DURATION)
        {
            digitalWrite(LED_GREEN_PIN, LOW); // LED ausschalten
            greenLedActive = false;
        }
    }

    // =========================================================================
    // Periodisch Bildschirm aktualisieren
    // =========================================================================
    if (millis() - lastDisplayUpdate >= 5000)
    {
        warmingDisplay(); // Warnbildschirm anzeigen
        delay(2000);
        mainDisplay(); // Zum Hauptbildschirm zurückkehren
        lastDisplayUpdate = millis();
    }

    delay(10); // Kurze Verzögerung zur Vermeidung von CPU-Überlastung
}
