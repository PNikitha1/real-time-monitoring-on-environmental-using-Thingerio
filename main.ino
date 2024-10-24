/*
  NodeMCU (ESP8266) with Temperature and Pressure Sensors
  --------------------------------------------------------
  Connections:
  
  Temperature Sensor:
    + (VCC)   -> 3V3
    G (GND)   -> GND
    SCL       -> D1
    SDA       -> D2
    
  Pressure Sensor:
    VCC       -> 3V3
    GND       -> GND
    SCK       -> D5
    OUT       -> D6
  Water Level Sensor:
    +         ->3V3
    -         ->GND
    S         ->A0
*/
#include <Arduino.h>
#if defined(ESP32) || defined(ARDUINO_RASPBERRY_PI_PICO_W)
#include <WiFi.h>
#elif defined(ESP8266)
#include <ESP8266WiFi.h>
#elif __has_include(<WiFiNINA.h>)
#include <WiFiNINA.h>
#elif __has_include(<WiFi101.h>)
#include <WiFi101.h>
#elif __has_include(<WiFiS3.h>)
#include <WiFiS3.h>
#endif

#include <ThingerESP8266.h>
#include <ESP_Mail_Client.h>
#include <Wire.h>
#include <Adafruit_BMP085.h> // Include BMP180 library

#define WIFI_SSID "GITAM" 
#define WIFI_PASSWORD "Gitam$$123"
#define USERNAME "GeetanjaliD" //your thinger io account
#define DEVICE_ID "nodemcu"    // your thinger io device id
#define DEVICE_CREDENTIAL "542OSb%QCBHQC?wA"  // your thinger io device credential

ThingerESP8266 thing(USERNAME, DEVICE_ID, DEVICE_CREDENTIAL);

#define SMTP_HOST "smtp.gmail.com"
#define SMTP_PORT esp_mail_smtp_port_587 // port 465 is not available for Outlook.com

/* The log in credentials */
#define AUTHOR_EMAIL "your email account"
#define AUTHOR_PASSWORD "your email password"

/* Recipient email address */
#define RECIPIENT_EMAIL "your recipient email account"

/* Declare the global used SMTPSession object for SMTP transport */
SMTPSession smtp;
// Create an instance of the BMP180 sensor
Adafruit_BMP085 bmp;

unsigned long lastThingerUpdate = 0;
unsigned long lastSensorCheck = 0;
const unsigned long sensorInterval = 60000;  // Check every 60 seconds
const unsigned long thingerInterval = 1000;  // Send data to Thinger every 5 seconds

#define PRESSURE_SCK_PIN  D5   // Serial Clock for pressure sensor
#define PRESSURE_OUT_PIN  D6   // Data Output from pressure sensor
#define WATER_LEVEL_PIN A0 

/* Callback function to get the Email sending status */
void smtpCallback(SMTP_Status status);

void setup()
{
  Serial.begin(115200);

  // Initialize BMP180 sensor
  if (!bmp.begin()) {
    Serial.println("Could not find a valid BMP180 sensor, check wiring!");
    while (1);
  }
   pinMode(PRESSURE_SCK_PIN, OUTPUT);
  digitalWrite(PRESSURE_SCK_PIN, LOW); // Ensure SCK starts LOW
  pinMode(PRESSURE_OUT_PIN, INPUT);
  pinMode(WATER_LEVEL_PIN, INPUT);
  // Initialize WiFi and Thinger.io
  thing.add_wifi(WIFI_SSID, WIFI_PASSWORD);

  // Define Thinger.io resources for temperature and pressure
  thing["temperature"] >> [](pson &out) {
    out = bmp.readTemperature();  // Send temperature to Thinger.io
  };

  thing["pressure"] >> [](pson &out) {
    out = readPressureSensor();  // Send pressure to Thinger.io
  };
  thing["water_level"] >> [](pson &out) {
    out = analogRead(WATER_LEVEL_PIN);  // Send water level to Thinger.io
  };

  Serial.println("NodeMCU Initialization Complete.");

#if defined(ARDUINO_ARCH_SAMD)
  while (!Serial);
#endif

  Serial.println();

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();

  /*  Set the network reconnection option */
  MailClient.networkReconnect(true);

  smtp.debug(1);

  /* Set the callback function to get the sending results */
  smtp.callback(smtpCallback);
}

void loop()
{
  unsigned long currentMillis = millis();

  if (currentMillis - lastSensorCheck >= sensorInterval) {
    lastSensorCheck = currentMillis;
    
    float temperature = bmp.readTemperature();
    float pressure = readPressureSensor();
    int waterLevelValue = analogRead(WATER_LEVEL_PIN);
    
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.println(" °C");
    
    Serial.print("Pressure: ");
    Serial.print(pressure);
    Serial.println(" hPa");

    Serial.print("Water Level: ");
    Serial.println(waterLevelValue);

    if (temperature > 33.0) {
      sendEmailAlertTemp(temperature, "Temperature is too high -  Kindly Turn ON AC");
    } else if (temperature < 10.0) {
      sendEmailAlertTemp(temperature, "Temperature is too low - Kindly Turn off AC");
    }

    if (pressure > 1000) {
      sendEmailAlertPress(pressure, "Pressure is more - Kindly shutdown the geyser");
    } else if (pressure < 400) {
      sendEmailAlertPress(pressure, "Pressure is low - Kindly stop heating the water");
    }

    if (waterLevelValue > 900) {
      sendEmailAlertWater("Water Tank is Almost Full - Kindly Turn Off the Motor  ", waterLevelValue);
    } else if (waterLevelValue < 200) {
      sendEmailAlertWater("Water is Low - Kindly Turn ON The Motor", waterLevelValue);
    }
  }

  if (currentMillis - lastThingerUpdate >= thingerInterval) {
    lastThingerUpdate = currentMillis;
    thing.handle();
  } // Wait for 1 minute before next reading
}
float readPressureSensor() {
  const int NUM_BITS = 16; // Number of bits to read (adjust based on your sensor)
  unsigned long rawData = 0;

  for (int i = 0; i < NUM_BITS; i++) {
    // Generate a clock pulse
    digitalWrite(PRESSURE_SCK_PIN, HIGH);
    delayMicroseconds(10); // Short delay to stabilize

    // Read the data bit
    int bit = digitalRead(PRESSURE_OUT_PIN);
    rawData = (rawData << 1) | bit;

    // Lower the clock
    digitalWrite(PRESSURE_SCK_PIN, LOW);
    delayMicroseconds(10); // Short delay
  }

  // Convert raw data to pressure
  float pressure = convertRawToPressure(rawData);

  return pressure;
}

/**
 * Converts raw pressure data to meaningful pressure value.
 * Replace this function's content with the actual conversion formula from your sensor's datasheet.
 */
float convertRawToPressure(unsigned long raw) {
  // Example: Suppose raw ranges from 0 to 65535 corresponding to 300 hPa to 1100 hPa
  float pressure = 300.0 + ((float)raw / 65535.0) * (1100.0 - 300.0);
  return pressure;
}


/* Function to send email alert when temperature exceeds threshold */
void sendEmailAlertTemp(float temperature,const char* alertMsg)
{
  /* Declare the Session_Config for user defined session credentials */
  Session_Config config;

  /* Set the session config */
  config.server.host_name = SMTP_HOST;
  config.server.port = SMTP_PORT;
  config.login.email = AUTHOR_EMAIL;
  config.login.password = AUTHOR_PASSWORD;
  config.login.user_domain = F("127.0.0.1");

  config.time.ntp_server = F("pool.ntp.org,time.nist.gov");
  config.time.gmt_offset = 3;
  config.time.day_light_offset = 0;

  SMTP_Message message;

  /* Set the message headers */
  message.sender.name = F("Pallapothula");
  message.sender.email = AUTHOR_EMAIL;
  message.subject = "Temperature Alert";

  message.addRecipient(F("Nikitha "), RECIPIENT_EMAIL);

  // Create the email message with the temperature reading
  String textMsg = String(alertMsg) + "\nCurrent Temperature: " + String(temperature) + " °C\n";

  message.text.content = textMsg;
  message.text.charSet = F("utf-8");
  message.text.transfer_encoding = "base64";

  message.priority = esp_mail_smtp_priority::esp_mail_smtp_priority_high;

  if (!smtp.connect(&config))
  {
    MailClient.printf("Connection error, Status Code: %d, Error Code: %d, Reason: %s\n", smtp.statusCode(), smtp.errorCode(), smtp.errorReason().c_str());
    return;
  }

  /* Start sending Email and close the session */
  if (!MailClient.sendMail(&smtp, &message))
    MailClient.printf("Error, Status Code: %d, Error Code: %d, Reason: %s\n", smtp.statusCode(), smtp.errorCode(), smtp.errorReason().c_str());

  smtp.sendingResult.clear();
}
void sendEmailAlertPress(float pressure, const char* alertMsg)
{
  /* Declare the Session_Config for user defined session credentials */
  Session_Config config;

  /* Set the session config */
  config.server.host_name = SMTP_HOST;
  config.server.port = SMTP_PORT;
  config.login.email = AUTHOR_EMAIL;
  config.login.password = AUTHOR_PASSWORD;
  config.login.user_domain = F("127.0.0.1");

  config.time.ntp_server = F("pool.ntp.org,time.nist.gov");
  config.time.gmt_offset = 3;
  config.time.day_light_offset = 0;

  SMTP_Message message;

  /* Set the message headers */
  message.sender.name = F("Pallapothula");
  message.sender.email = AUTHOR_EMAIL;
  message.subject = "Geyser Alert";

  message.addRecipient(F("Nikitha "), RECIPIENT_EMAIL);

  // Create the email message with the temperature reading
  String textMsg = String(alertMsg) + "\nCurrent Pressure: " + String(pressure) + " hPa\n";


  message.text.content = textMsg;
  message.text.charSet = F("utf-8");
  message.text.transfer_encoding = "base64";

  message.priority = esp_mail_smtp_priority::esp_mail_smtp_priority_high;

  if (!smtp.connect(&config))
  {
    MailClient.printf("Connection error, Status Code: %d, Error Code: %d, Reason: %s\n", smtp.statusCode(), smtp.errorCode(), smtp.errorReason().c_str());
    return;
  }

  /* Start sending Email and close the session */
  if (!MailClient.sendMail(&smtp, &message))
    MailClient.printf("Error, Status Code: %d, Error Code: %d, Reason: %s\n", smtp.statusCode(), smtp.errorCode(), smtp.errorReason().c_str());

  smtp.sendingResult.clear();
}

void sendEmailAlertWater(const char* alertMsg, int waterLevelValue)
{
  /* Declare the Session_Config for user defined session credentials */
  Session_Config config;

  /* Set the session config */
  config.server.host_name = SMTP_HOST;
  config.server.port = SMTP_PORT;
  config.login.email = AUTHOR_EMAIL;
  config.login.password = AUTHOR_PASSWORD;
  config.login.user_domain = F("127.0.0.1");

  config.time.ntp_server = F("pool.ntp.org,time.nist.gov");
  config.time.gmt_offset = 3;
  config.time.day_light_offset = 0;

  SMTP_Message message;

  /* Set the message headers */
  message.sender.name = F("Pallapothula");
  message.sender.email = AUTHOR_EMAIL;
  message.subject = "Water Tank Alert";

  message.addRecipient(F("Nikitha "), RECIPIENT_EMAIL);

  // Create the email message with the temperature reading
 String textMsg = String(alertMsg) + "\nCurrent Water Level: " + String(waterLevelValue);

  message.text.content = textMsg;
  message.text.charSet = F("utf-8");
  message.text.transfer_encoding = "base64";

  message.priority = esp_mail_smtp_priority::esp_mail_smtp_priority_high;

  if (!smtp.connect(&config))
  {
    MailClient.printf("Connection error, Status Code: %d, Error Code: %d, Reason: %s\n", smtp.statusCode(), smtp.errorCode(), smtp.errorReason().c_str());
    return;
  }

  /* Start sending Email and close the session */
  if (!MailClient.sendMail(&smtp, &message))
    MailClient.printf("Error, Status Code: %d, Error Code: %d, Reason: %s\n", smtp.statusCode(), smtp.errorCode(), smtp.errorReason().c_str());

  smtp.sendingResult.clear();
}

/* Callback function to get the Email sending status */
void smtpCallback(SMTP_Status status)
{
  /* Print the current status */
  Serial.println(status.info());

  /* Print the sending result */
  if (status.success())
  {
    Serial.println("----------------");
    MailClient.printf("Message sent success: %d\n", status.completedCount());
    MailClient.printf("Message sent failed: %d\n", status.failedCount());
    Serial.println("----------------\n");

    for (size_t i = 0; i < smtp.sendingResult.size(); i++)
    {
      /* Get the result item */
      SMTP_Result result = smtp.sendingResult.getItem(i);

      MailClient.printf("Message No: %d\n", i + 1);
      MailClient.printf("Status: %s\n", result.completed ? "success" : "failed");
      MailClient.printf("Date/Time: %s\n", MailClient.Time.getDateTimeString(result.timestamp, "%B %d, %Y %H:%M:%S").c_str());
      MailClient.printf("Recipient: %s\n", result.recipients.c_str());
      MailClient.printf("Subject: %s\n", result.subject.c_str());
    }
    Serial.println("----------------\n");

    // Clear sending result log
    smtp.sendingResult.clear();
  }
}
