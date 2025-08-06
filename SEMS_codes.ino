//Authentication to Blynk
#define BLYNK_TEMPLATE_ID ""
#define BLYNK_TEMPLATE_NAME ""
#define BLYNK_AUTH_TOKEN ""
#define BLYNK_PRINT Serial

//Necessary libraries
#include <Arduino.h>
#include <Wire.h>
#include <time.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_BME280.h>
#include <BH1750.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <ESP_Mail_Client.h>
#include <LittleFS.h>
#include <esp_task_wdt.h>
#include <BlynkSimpleEsp32.h>

// Pin Definitions
#define MQ135_PIN 34
#define GAS_ALERT_LED_PIN 26   // LED for gas alert
#define TEMP_ALERT_LED_PIN 4  // LED for temperature alert
// NTP Server settings
#define NTP_SERVER "pool.ntp.org"
#define GMT_OFFSET_SEC (-5 * 3600)  // GMT-5 (Eastern Time)
#define DAYLIGHT_OFFSET_SEC 3600    // 1 hour daylight saving time (set to 0 if not applicable)


// I2C Pin Definitions
#define OLED_SDA_PIN 21
#define OLED_SCL_PIN 22
#define BME_SDA_PIN 33
#define BME_SCL_PIN 32
#define BH1750_SDA_PIN 33
#define BH1750_SCL_PIN 32

// I2C Addresses
#define OLED_ADDRESS 0x3C  //address for Oled display
#define BME280_ADDRESS 0x76  // address for BME280
#define BH1750_ADDRESS 0x23  // address for BH1750

// Alert thresholds
#define GAS_ALERT_THRESHOLD 500.0    // PPM threshold for harmful gas
#define HIGH_TEMP_THRESHOLD 30.0     // °C threshold for high temperature
#define LOW_TEMP_THRESHOLD 18.0      // °C threshold for low temperature
#define HIGH_LIGHT_THRESHOLD 800.0  // Lux threshold for high light

// Screen dimensions
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

// WiFi configuration
#define WIFI_SSID ""
#define WIFI_PASSWORD ""
#define MAX_PENDING_ALERTS 10

char auth[]= BLYNK_AUTH_TOKEN;

BlynkTimer timer;

// Create separate TwoWire instances
TwoWire I2COLED = TwoWire(0);    // For the OLED display
TwoWire I2C_SENSOR = TwoWire(1); // For the BME280 and BH1750

// Sensor Objects
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &I2COLED, -1);
Adafruit_BME280 bme;
BH1750 lightMeter;

// Global variables to store sensor data
volatile float mq135Voltage = 0.0;
volatile float mq135PPM = 0.0;      // Added for air quality in PPM
volatile float bmeTemperature = 0.0;
volatile float bmeHumidity = 0.0;
volatile float bmePressure = 0.0;
volatile float lightLevel = 0.0;

// Mutex to manage I2C bus access
SemaphoreHandle_t i2cMutex;

// Alert state variables
bool gasAlertActive = false;
bool tempAlertActive = false;
bool lightAlertActive = false;
bool acOn = false;

// Task handles
TaskHandle_t gasLedTaskHandle = NULL;
TaskHandle_t tempLedTaskHandle = NULL;


struct AlertMessage {
  char eventName[20];
  char message[150];
  bool valid;
};

AlertMessage pendingAlerts[MAX_PENDING_ALERTS];
int pendingAlertCount = 0;


// Function to convert MQ135 voltage to approximate PPM

float voltageToPPM(float voltage) {
  return voltage * 100; 
}



//Send data to the blynk app and dashboard

void sendSensorDataToBlynk() {
  if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
  // Send temperature data to Virtual Pin 0
  Blynk.virtualWrite(V0, bmeTemperature);
  
  // Send humidity data to Virtual Pin 1
  Blynk.virtualWrite(V1, bmeHumidity);
  
  // Send pressure data to Virtual Pin 2
  Blynk.virtualWrite(V2, mq135PPM);
  
  // Send air quality data to Virtual Pin 3
  Blynk.virtualWrite(V3, lightLevel);
  
  // Send light level data to Virtual Pin 4
  Blynk.virtualWrite(V4, bmePressure);
  xSemaphoreGive(i2cMutex);
  
  }
}



// Function that handles alerts using Blynk events 
void sendAlert(const char* eventName, const char* message) {
  Serial.print("Preparing to send Blynk event '");
  Serial.print(eventName);
  Serial.println("'...");
  
  if (Blynk.connected()) {
    Blynk.logEvent(eventName, message);
    delay(150);
    
    Serial.print("Alert sent via Blynk event '");
    Serial.print(eventName);
    Serial.print("': ");
    Serial.println(message);
    
    // Also process any pending alerts if we're connected
    processPendingAlerts();
  } else {
    Serial.println("Blynk not connected! Buffering alert for later delivery");
    
    // Only buffer if we have space
    if (pendingAlertCount < MAX_PENDING_ALERTS) {
      strncpy(pendingAlerts[pendingAlertCount].eventName, eventName, sizeof(pendingAlerts[pendingAlertCount].eventName)-1);
      strncpy(pendingAlerts[pendingAlertCount].message, message, sizeof(pendingAlerts[pendingAlertCount].message)-1);
      pendingAlerts[pendingAlertCount].valid = true;
      pendingAlertCount++;
      
      Serial.print("Alert buffered (");
      Serial.print(pendingAlertCount);
      Serial.println(" alerts pending)");
    } else {
      Serial.println("Alert buffer full! Alert will be lost!");
    }
  }
}


void processPendingAlerts() {
  if (pendingAlertCount > 0 && Blynk.connected()) {
    Serial.print("Processing ");
    Serial.print(pendingAlertCount);
    Serial.println(" pending alerts...");
    
    for (int i = 0; i < pendingAlertCount; i++) {
      if (pendingAlerts[i].valid) {
        Serial.print("Sending buffered alert: ");
        Serial.println(pendingAlerts[i].eventName);
        
        Blynk.logEvent(pendingAlerts[i].eventName, pendingAlerts[i].message);
        delay(150);
        
        pendingAlerts[i].valid = false;
      }
    }
    
    pendingAlertCount = 0;
    Serial.println("All pending alerts processed");
  }
}


// Helper function to get formatted timestamp
String getTimeStamp() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    return "[Time unavailable]";
  }
  char timeString[30];
  strftime(timeString, sizeof(timeString), "%Y-%m-%d %H:%M:%S", &timeinfo);
  return String(timeString);
}

void checkBlynkAndAlerts(void * parameter) {
  for (;;) {
    // Try to process pending alerts every 5 seconds
    if (Blynk.connected()) {
      processPendingAlerts();
    }
    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}


// Task: Blink LED for gas alert
void blinkGasAlertLED(void * parameter) {
  for (;;) {
    if (gasAlertActive) {
      digitalWrite(GAS_ALERT_LED_PIN, HIGH);
      vTaskDelay(pdMS_TO_TICKS(500));
      digitalWrite(GAS_ALERT_LED_PIN, LOW);
      vTaskDelay(pdMS_TO_TICKS(500));
    } else {
      digitalWrite(GAS_ALERT_LED_PIN, LOW);
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }
}

// Task: Blink LED for temperature alert
void blinkTempAlertLED(void * parameter) {
  for (;;) {
    if (tempAlertActive) {
      digitalWrite(TEMP_ALERT_LED_PIN, HIGH);
      vTaskDelay(pdMS_TO_TICKS(acOn ? 1000 : 300)); 
      digitalWrite(TEMP_ALERT_LED_PIN, LOW);
      vTaskDelay(pdMS_TO_TICKS(acOn ? 1000 : 300));
    } else {
      digitalWrite(TEMP_ALERT_LED_PIN, LOW);
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }
}


// Function to display alert message on OLED
void showAlertOnDisplay(const char* alertMessage, bool takeDisplay = true) {
  if (takeDisplay) {
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
      Serial.println("Failed to get mutex for alert display");
      return;
    }
  }
  
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  
  // Title
  display.setCursor(0, 0);
  display.println("!! ALERT !!");
  display.drawLine(0, 9, display.width(), 9, SSD1306_WHITE);
  
  // Split the alert message if it's long
  String message = String(alertMessage);
  int spaceIndex = 0;
  int lineY = 12;
  int maxLineLength = 21; // Characters per line
  
  while (message.length() > 0 && lineY < 64) {
    if (message.length() > maxLineLength) {
      // Find a space to break the line
      spaceIndex = message.substring(0, maxLineLength).lastIndexOf(" ");
      if (spaceIndex <= 0) spaceIndex = maxLineLength; // If no space found, just break at max length
      
      display.setCursor(0, lineY);
      display.println(message.substring(0, spaceIndex));
      message = message.substring(spaceIndex + 1);
    } else {
      display.setCursor(0, lineY);
      display.println(message);
      message = "";
    }
    lineY += 10;
  }
  
  display.display();
  
  if (takeDisplay) {
    xSemaphoreGive(i2cMutex);
  }
}


// Task: Read MQ-135 sensor (analog, no I2C needed)
void readMQ135(void * parameter) {
  // Use static variables to reduce stack usage
  static int analogValue;
  static float voltage;
  static bool previousAlert = false;
  
  for (;;) {
    analogValue = analogRead(MQ135_PIN);
    voltage = analogValue * (3.3 / 4095.0);
    mq135Voltage = voltage;
    mq135PPM = voltageToPPM(voltage);
    
    Serial.printf("MQ-135 -> Analog Value: %d\tVoltage: %.2f V\tApprox PPM: %.2f\n", 
                 analogValue, voltage, mq135PPM);
    
    // Check for harmful gas levels
    previousAlert = gasAlertActive;
    gasAlertActive = (mq135PPM > GAS_ALERT_THRESHOLD);
    
    // If alert status changed from inactive to active
    if (gasAlertActive && !previousAlert) {
      Serial.println("WARNING: Harmful gas levels detected!");
      
      // Show alert on display
      if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        // Create message with less stack usage
        String alertMsg = "Harmful gas! PPM: ";
        alertMsg += String(mq135PPM, 1);
        showAlertOnDisplay(alertMsg.c_str(), false);
        xSemaphoreGive(i2cMutex);
      }
      
      // Send alert
      char alertMsg[150];
      snprintf(alertMsg, sizeof(alertMsg), 
               "%s - Gas Alert: %0.1f PPM detected (threshold: %0.1f PPM)", 
               getTimeStamp().c_str(), mq135PPM, GAS_ALERT_THRESHOLD);
  
      // Send alert 
      sendAlert("aq_alert",alertMsg);
      

    }
    // Alert when conditions return to normal
    else if (!gasAlertActive && previousAlert) {
      char alertMsg[150];
      snprintf(alertMsg, sizeof(alertMsg), 
               "%s - Gas levels have returned to normal: %0.1f PPM (threshold: %0.1f PPM)", 
               getTimeStamp().c_str(), mq135PPM, GAS_ALERT_THRESHOLD);
      
      // Send "back to normal" alert 
      sendAlert("aq_alert", alertMsg);
    }
    
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}

// Task: Read BME280 sensor (temperature, humidity, pressure)
void readBME280(void * parameter) {
  bool previousTempAlert = false;
  bool previousAcState = false;
  
  for (;;) {
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      bmeTemperature = bme.readTemperature();
      bmeHumidity = bme.readHumidity();
      bmePressure = bme.readPressure() / 100.0;  // Convert Pa to hPa
      xSemaphoreGive(i2cMutex);
      
      Serial.printf("BME280 -> Temp: %.2f C, Hum: %.2f %%, Pres: %.2f hPa\n", 
                   bmeTemperature, bmeHumidity, bmePressure);
      
      // Store previous states before updating
      previousTempAlert = tempAlertActive;
      previousAcState = acOn;
      
      // Determine if temp alert is active and if AC should be on
      tempAlertActive = (bmeTemperature > HIGH_TEMP_THRESHOLD || bmeTemperature < LOW_TEMP_THRESHOLD);
      acOn = (bmeTemperature > HIGH_TEMP_THRESHOLD);
      
      // If temperature alert status changed
      if (tempAlertActive && !previousTempAlert) {
        Serial.println("WARNING: Temperature outside normal range!");
        
        // Show alert on display
        if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
          String alertMsg = "Temp: " + String(bmeTemperature, 1) + "C ";
          alertMsg += acOn ? "AC ON" : "Too cold!";
          showAlertOnDisplay(alertMsg.c_str(), false);
          xSemaphoreGive(i2cMutex);
        }
        
        // Create alert message about temperature
        char alertMsg[150];
        if (acOn) {
          snprintf(alertMsg, sizeof(alertMsg), 
                  "%s - Temperature Alert: %0.1f°C is too high! AC activated. (Threshold: %0.1f°C)", 
                  getTimeStamp().c_str(), bmeTemperature, HIGH_TEMP_THRESHOLD);
        } else {
          snprintf(alertMsg, sizeof(alertMsg), 
                  "%s - Temperature Alert: %0.1f°C is too low! (Threshold: %0.1f°C)", 
                  getTimeStamp().c_str(), bmeTemperature, LOW_TEMP_THRESHOLD);
        }
        
        // Send alert 
        sendAlert("temperature", alertMsg);
      }
      // Alert when conditions return to normal
      else if (!tempAlertActive && previousTempAlert) {
        char alertMsg[150];
        snprintf(alertMsg, sizeof(alertMsg), 
                "%s - Temperature has returned to normal range: %0.1f°C (Range: %0.1f°C to %0.1f°C)", 
                getTimeStamp().c_str(), bmeTemperature, LOW_TEMP_THRESHOLD, HIGH_TEMP_THRESHOLD);
        
        // Send "back to normal" alert 
        sendAlert("temperature",alertMsg); 
      }
      // If AC state changed but we're still in alert state
      else if (tempAlertActive && (acOn != previousAcState)) {
        char alertMsg[150];
        if (acOn) {
          snprintf(alertMsg, sizeof(alertMsg), 
                  "%s - AC activated: Temperature %0.1f°C above threshold (%0.1f°C)", 
                  getTimeStamp().c_str(), bmeTemperature, HIGH_TEMP_THRESHOLD);
        } else {
          snprintf(alertMsg, sizeof(alertMsg), 
                  "%s - AC deactivated: Temperature %0.1f°C has dropped below high threshold", 
                  getTimeStamp().c_str(), bmeTemperature);
        }
        
        // Send AC status change alert 
        sendAlert("temperature",alertMsg);
      }
    } else {
      Serial.println("Failed to get mutex for BME280 reading");
    }
    
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}

// Task: Read BH1750 sensor (light level in lux)
void readBH1750(void * parameter) {
  bool previousLightAlert = false;
  
  for (;;) {
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      lightLevel = lightMeter.readLightLevel();
      xSemaphoreGive(i2cMutex);
      
      Serial.printf("BH1750 -> Light Level: %.2f lux\n", lightLevel);
      
      // Store previous state before updating
      previousLightAlert = lightAlertActive;
      lightAlertActive = (lightLevel > HIGH_LIGHT_THRESHOLD);
      
      // If light alert status changed from inactive to active
      if (lightAlertActive && !previousLightAlert) {
        Serial.println("WARNING: Light intensity too high!");
        
        // Show alert on display
        if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
          String alertMsg = "Light intensity high!\nReduce lighting\n" + String(lightLevel, 0) + " lux";
          showAlertOnDisplay(alertMsg.c_str(), false);
          xSemaphoreGive(i2cMutex);
        }
        
        // Create alert message about light level
        char alertMsg[150];
        snprintf(alertMsg, sizeof(alertMsg), 
                "%s - Light Alert: %0.0f lux exceeds threshold (%0.0f lux)", 
                getTimeStamp().c_str(), lightLevel, HIGH_LIGHT_THRESHOLD);
  
        // Send alert 
        sendAlert("light_level",alertMsg);
      }
      // Alert when conditions return to normal
      else if (!lightAlertActive && previousLightAlert) {
        char alertMsg[150];
        snprintf(alertMsg, sizeof(alertMsg), 
                "%s - Light intensity has returned to normal: %0.0f lux (threshold: %0.0f lux)", 
                getTimeStamp().c_str(), lightLevel, HIGH_LIGHT_THRESHOLD);
        
        // Send "back to normal" alert 
        sendAlert("light_level",alertMsg);
      }
    } else {
      Serial.println("Failed to get mutex for BH1750 reading");
    }
    
    vTaskDelay(pdMS_TO_TICKS(2000)); // 2000 ms delay
  }
}

// Task: Update OLED display with sensor data that cycles every 8 seconds
void updateDisplay(void * parameter) {
  int displayState = 0;  // 0=temp, 1=humidity, 2=pressure, 3=AQ, 4=light
  const int totalStates = 5;
  unsigned long lastUpdate = 0;
  
  for (;;) {
    unsigned long currentTime = millis();
    
    // If any alert is active, don't cycle display content
    if (!gasAlertActive && !tempAlertActive && !lightAlertActive) {
      if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        // Clear the buffer
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        
        // Title
        display.setCursor(0, 0);
        display.println("Environment Monitor");
        display.drawLine(0, 9, display.width(), 9, SSD1306_WHITE);
        
        // Now we can show more info with the taller display
        display.setCursor(0, 12);
        display.print("Temp: ");
        display.print(bmeTemperature, 1);
        display.println("C");
        
        display.setCursor(0, 22);
        display.print("Hum: ");
        display.print(bmeHumidity, 1);
        display.println("%");
        
        display.setCursor(0, 32);
        display.print("Air: ");
        display.print(mq135PPM, 1);
        display.println("ppm");
        
        display.setCursor(0, 42);
        display.print("Light: ");
        display.print(lightLevel, 0);
        display.println("lx");
        
        display.setCursor(0, 52);
        display.print("Pres: ");
        display.print(bmePressure, 0);
        display.println("hPa");
        
        // Show system status at the bottom
        if (acOn) {
          display.setCursor(90, 52);
          display.println("AC:ON");
        }
        
        // Actually update the display
        display.display();
        xSemaphoreGive(i2cMutex);
      } else {
        Serial.println("Failed to get mutex for display update");
      }
    }
    
    vTaskDelay(pdMS_TO_TICKS(1000)); // Check every second
  }
}

// WiFi connection function
bool connectToWiFi() {
  Serial.print("Connecting to WiFi");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    return true;
  } else {
    Serial.println("\nFailed to connect to WiFi!");
    return false;
  }
}

// Task: Monitor and maintain WiFi connection
void monitorWiFi(void * parameter) {
  for(;;) {
    if(WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi connection lost. Reconnecting...");
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
      
      // Wait for connection or timeout
      int attempts = 0;
      while(WiFi.status() != WL_CONNECTED && attempts < 20) {
        vTaskDelay(pdMS_TO_TICKS(500));
        Serial.print(".");
        attempts++;
      }
      
      if(WiFi.status() == WL_CONNECTED) {
        Serial.println("\nWiFi reconnected!");
        Serial.print("IP address: ");
        Serial.println(WiFi.localIP());
      } else {
        Serial.println("\nFailed to reconnect to WiFi!");
      }
    }
    
    vTaskDelay(pdMS_TO_TICKS(30000)); // Check every 30 seconds
  }
}



bool initOLED() {
  Serial.println("Initializing OLED display...");
  
  // Initialize I2COLED on pins
  I2COLED.begin(OLED_SDA_PIN, OLED_SCL_PIN);
  
  // Try to initialize the display
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
    Serial.println("SSD1306 allocation failed");
    return false;
  }

  // Set contrast to maximum
  display.ssd1306_command(SSD1306_SETCONTRAST);
  display.ssd1306_command(255);
  
  // Show initial display buffer contents on the screen
  display.display();
  delay(1000); // Pause for 1 second
  
  // Clear the buffer
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 10);
  display.println(" Starting");
  display.setCursor(0, 30);
  display.println(" System...");
  display.display();
  
  delay(2000); // Pause for 2 seconds
  
  Serial.println("OLED display initialized successfully");
  return true;

}

bool initBME280() {
  Serial.println("Initializing BME280 sensor...");
  
  // Initialize I2CBME on pins
  I2C_SENSOR.begin(BME_SDA_PIN, BME_SCL_PIN);
  
  // Default settings from datasheet
  if (!bme.begin(BME280_ADDRESS, &I2C_SENSOR)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    return false;
  }
  
  // Set up sensor with recommended weather monitoring settings
  bme.setSampling(Adafruit_BME280::MODE_NORMAL,    // Operating Mode
                  Adafruit_BME280::SAMPLING_X1,    // Temp. oversampling
                  Adafruit_BME280::SAMPLING_X1,    // Humidity oversampling
                  Adafruit_BME280::SAMPLING_X1,    // Pressure oversampling
                  Adafruit_BME280::FILTER_OFF,     // No filtering
                  Adafruit_BME280::STANDBY_MS_1000); // Standby time

  Serial.println("BME280 initialized successfully");
  return true;
}

bool initBH1750() {
  Serial.println("Initializing BH1750 light sensor..."); 
  
  // Initialize I2CBH on pins
  I2C_SENSOR.begin(BH1750_SDA_PIN, BH1750_SCL_PIN);
  
  // Initialize the sensor
  if (!lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, BH1750_ADDRESS, &I2C_SENSOR)) {
    Serial.println("Error initialising BH1750 light sensor");
    return false;
  }
  
  Serial.println("BH1750 initialized successfully");
  return true;
}
void setupTime() {
  Serial.println("Setting up time with NTP...");
  configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, NTP_SERVER);
  
  Serial.println("Waiting for NTP time sync...");
  time_t now = time(nullptr);
  int attempts = 0;
  const int maxAttempts = 10;
  
  while (now < 8 * 3600 * 2 && attempts < maxAttempts) {
    Serial.print(".");
    delay(500);
    now = time(nullptr);
    attempts++;
  }
  
  if (attempts < maxAttempts) {
    Serial.println("\nTime synchronized successfully!");
    struct tm timeinfo;
    localtime_r(&now, &timeinfo);
    Serial.print("Current time: ");
    Serial.println(asctime(&timeinfo));
  } else {
    Serial.println("\nFailed to get time from NTP server!");
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000); // Give serial monitor time to start
  Blynk.begin(auth, WIFI_SSID, WIFI_PASSWORD, "blynk.cloud", 80);
  timer.setInterval(10000L, sendSensorDataToBlynk);


  //esp_task_wdt_init(30, false);
  Serial.println("\n\n====================================");
  Serial.println("ESP32 Environmental Monitoring System");
  Serial.println("====================================");
  
  // Configure pins
  pinMode(MQ135_PIN, INPUT);
  pinMode(GAS_ALERT_LED_PIN, OUTPUT);
  pinMode(TEMP_ALERT_LED_PIN, OUTPUT);
  
  
  digitalWrite(GAS_ALERT_LED_PIN, LOW);
  digitalWrite(TEMP_ALERT_LED_PIN, LOW);
  
  
  // Create the I2C mutex
  i2cMutex = xSemaphoreCreateMutex();
  if (i2cMutex == NULL) {
    Serial.println("Failed to create I2C mutex");
    while (1) {
      delay(1000);
    }
  }

  // Initialize sensors
  bool oledInitialized = initOLED();
  bool bmeInitialized = initBME280();
  bool bhInitialized = initBH1750();
  
  // Connect to WiFi
  bool wifiConnected = false;
  int wifiAttempts = 0;
  while (!wifiConnected && wifiAttempts < 3) {
  wifiConnected = connectToWiFi();
  if (!wifiConnected) {
    Serial.println("WiFi connection failed. Retrying...");
    delay(2000);
    wifiAttempts++;
  }
}


if (wifiConnected) {
  setupTime();
}

if (!LittleFS.begin(true)) {
  Serial.println("LittleFS initialization failed!");
} else {
  Serial.println("LittleFS initialized successfully");
}

  
  // Display status on OLED
 
if (oledInitialized) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Init Status:");
  display.drawLine(0, 9, display.width(), 9, SSD1306_WHITE);
  
  display.setCursor(0, 12);
  display.print("OLED: ");
  display.println(oledInitialized ? "OK" : "FAIL");
  
  display.setCursor(0, 22);
  display.print("BME280: ");
  display.println(bmeInitialized ? "OK" : "FAIL");
  
  display.setCursor(0, 32);
  display.print("BH1750: ");
  display.println(bhInitialized ? "OK" : "FAIL");
  
  display.setCursor(0, 42);
  display.print("WiFi: ");
  display.println(wifiConnected ? "OK" : "FAIL");
  
  display.setCursor(0, 52);
  display.print("System ready to monitor");
  display.display();
  delay(2000);
}
  // Create alert LED tasks (low priority, core 0)
  xTaskCreatePinnedToCore(monitorWiFi, "WiFiMonitor", 2048, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(blinkGasAlertLED, "GasLedTask", 2048, NULL, 1, &gasLedTaskHandle, 0);
  xTaskCreatePinnedToCore(blinkTempAlertLED, "TempLedTask", 2048, NULL, 1, &tempLedTaskHandle, 0);
  xTaskCreatePinnedToCore(checkBlynkAndAlerts, "BlynkAlertsTask", 3072, NULL, 1, NULL, 0);
  
  // Create sensor tasks on core 1 (application core)
  xTaskCreatePinnedToCore(readMQ135, "ReadMQ135", 4096, NULL, 2, NULL, 1);
  
  if (bmeInitialized) {
    xTaskCreatePinnedToCore(readBME280, "ReadBME280", 3072, NULL, 2, NULL, 1);
  }
  
  if (bhInitialized) {
    xTaskCreatePinnedToCore(readBH1750, "ReadBH1750", 3072, NULL, 2, NULL, 1);
  }
  
  if (oledInitialized) {
    xTaskCreatePinnedToCore(updateDisplay, "UpdateDisplay", 3072, NULL, 1, NULL, 1);
  }
  
  // Send a startup notification email
  if (wifiConnected) {
  char startupMessage[100];
  snprintf(startupMessage, sizeof(startupMessage), 
          "ESP32 Environmental Monitoring System has started.\nAll sensors: %s",
          (oledInitialized && bmeInitialized && bhInitialized) ? "OK" : "Some failures");
  
  
   Blynk.logEvent("system_startup", startupMessage);
  Serial.println("Startup email sent via Blynk");
}
  
  Serial.println("Setup complete, starting monitoring...");
}

void loop() {
  static unsigned long lastReconnectAttempt = 0;
  
  if (!Blynk.connected()) {
    unsigned long currentMillis = millis();
    // Try to reconnect every 3 seconds
    if (currentMillis - lastReconnectAttempt > 3000) {
      lastReconnectAttempt = currentMillis;
      Serial.println("Blynk disconnected, reconnecting...");
      if (Blynk.connect()) {
        Serial.println("Blynk reconnected successfully");
        // Process any pending alerts immediately after reconnection
        processPendingAlerts();
      } else {
        Serial.println("Blynk reconnection failed");
      }
    }
  } else {
    Blynk.run();
  }
  
  timer.run();
  vTaskDelay(pdMS_TO_TICKS(1000));
}