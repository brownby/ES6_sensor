/*
 * PM Tracker
 * MUHAMMAD Nasir
 */

#include <SPI.h>
#include <Wire.h>
//#include "HM3301.h"
#include <ArduinoJson.h>
// Make sure you have these five libraries installed in Documents/Arduino/libraries
#include <Adafruit_I2CDevice.h>
#include <Adafruit_GFX.h>
//#include <Adafruit_SSD1327.h>
#include "Display.h"
#include <Adafruit_ST7735.h>
// #include <Encoder.h>
#include <RotaryEncoder.h>
//#include "SdFat.h"
//#include <LittleFS.h>
#include <SD.h>
#include <TinyGPS++.h>
#include <TimeLib.h>
//#include <RTCZero.h>
#include <ESP32Time.h>
#include <Seeed_BMP280.h>
#include <Ticker.h>
#include <PMserial.h>
#include "pinconfig.h"
#include <WiFi.h>
#include <WebServer.h>

#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

uint16_t LCD_FOREGROUND = ST7735_WHITE;
uint16_t LCD_BACKGROUND = ST7735_BLACK;

#define SAMP_TIME 2500 // number of ms between sensor readings
#define BLINK_TIME 30 // time in ms between LED blinks on successful write to SD
#define GPS_TIME 10000 // time between GPS reads
#define GPS_TIMEOUT 5000 // number of ms before GPS read times out
#define GPS_FIRST_TIMEOUT 600000 // number of ms before first GPS read times out
//#define BLINK_CNT 3 // number of times to blink LED on successful write
//#define SD_CS_PIN 4 // CS pin of SD card, 4 on SD MKR proto shield
#define CUR_YEAR 2024 // for GPS first fix error checking
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3D
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 128

// SD card with custom pins (Not default SPI pins)
#define SD_CS_PIN 15    // Chip Select (CS) pin for SD card
#define SD_MISO 2      // MISO pin for SD card
#define SD_MOSI 33      // MOSI pin for SD card
#define SD_SCK 25       // Clock (SCK) pin for SD card

// Define DHT11 pin and type
#define DHTPIN 27    // GPIO D27 where the DHT11 data pin is connected
#define DHTTYPE DHT11 // DHT11 sensor type

// Create an instance of the SPI class with custom pins
SPIClass customSPI(VSPI);  // Using VSPI, but with custom GPIO pins

// //Left and Right Encoder Pins
// #define ENC_RIGHT_BUTTON 35
// #define ENC_RIGHT_A 39
// #define ENC_RIGHT_B 36
// #define ENC_LEFT_BUTTON 13
// #define ENC_LEFT_A 34
// #define ENC_LEFT_B 12       //swaped with 12

//Left and Right Encoder Pins
#define ENC_RIGHT_BUTTON 13
#define ENC_RIGHT_A 12
#define ENC_RIGHT_B 34    //swaped with 12
#define ENC_LEFT_BUTTON 35
#define ENC_LEFT_A 36
#define ENC_LEFT_B 39

#define MENU_UPDATE_TIME 100 // milliseconds between menu updates
#define DEBUG_PRINT

#define LED_BUILTIN -1
//HM3301 dustSensor;
// SdFat SD;
// BMP280 TPSensor;  //BMP280
DHT dht(DHTPIN, DHTTYPE);
//#define SD LittleFS
File dataFile;  
File metaFile;
File imageFile;
String dataFileName; // YYMMDD_HHMMSS_data.txt
String metaFileName; // YYMMDD_HHMMSS_meta.txt
char * fileList; // list of files on SD card
bool fileListAlloc = false; // variable for tracking if fileList is malloc'd to avoid memory leaks
uint32_t fileCount = 0; // number of files on SD card
char fileToUpload[30];
bool timeSetOnce = false;

// Replace with your network credentials
// const char* ssid     = "TV-LED";
// const char* password = "Lums@12345";

const char* ssid     = "Qosain";
const char* password = "12345678";

WebServer server(80); // Create a web server on port 80
TinyGPSPlus gps;
bool timestampFlag = false;
bool gpsAwake = true;
bool gpsDisplayFail = false;

int utcYear;
int utcMonth;
int utcDay;
int utcHour;
int utcMinute;
int utcSecond;
double latitude;
double longitude;
double altitude;

time_t prevTimeStamp = 0;
uint8_t manualMonth = 1;
uint8_t manualDay = 1;
uint16_t manualYear = CUR_YEAR;
uint8_t manualHour = 0;
uint8_t manualMinute = 0;
int manualsecond = 0;
bool manualTimeEntry = false; // false means use GPS
bool rtcSet = false; // flag to indicate if RTC is set or not
//RTCZero rtc;
ESP32Time rtc;

//Adafruit_SSD1327 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Encoder encRight(ENC_RIGHT_B, ENC_RIGHT_A);
// Encoder encLeft(ENC_LEFT_B, ENC_LEFT_A);
RotaryEncoder encRight(ENC_RIGHT_A, ENC_RIGHT_B, ENC_RIGHT_BUTTON);
RotaryEncoder encLeft(ENC_LEFT_A, ENC_LEFT_B, ENC_LEFT_BUTTON);
Ticker encoderRotary;
Ticker encoderButton;

long encRightOldPosition = 0;
long encLeftOldPosition = 0;

unsigned long prevSampMillis = 0;
unsigned long prevLedMillis = 0;
unsigned long prevGpsMillis = 0;
unsigned long prevMenuMillis = 0;
unsigned long dataStartMillis = 0; // millis() when data collection began
unsigned long curMillis;

volatile bool encRightButtonFlag = false;
volatile bool encRightButtonISREn = false;
volatile bool encLeftButtonFlag = false;
volatile bool encLeftButtonISREn = false;

bool ledFlag = false;
uint8_t ledCount = 0;

bool firstMeasurementFlag = false;

// state = 0 navigating menu
// state = 2 collecting data
// state = 3 uploading data
uint8_t state = 0;
uint8_t prevState = 0;

// page = 0 two choice menu, collect data and upload data
// page = 1 time entry choice menu -> two choices enter timestamp or get from GPS
// page = 2 entering date
// page = 3 entering time
// page = 4 viewing SD card files
// page = 5 data collection screen
uint8_t page = 0;
uint8_t prevPage = 0;
int16_t currentVertMenuSelection = 0;
int16_t currentHoriMenuSelection = 0;
int16_t prevVertMenuSelection = 0;
uint8_t scroll = 0; // count number of times SD page has been scrolled

SerialPM pms(PMSx003, PM_SERIAL);  // PMSx003, UART

// Function declarations for Wifi Module
void handleRoot();
void handleFileList();
void handleFileDownload();

// ISR for encoder position updates
void encoderISR()
{
  encRight.readAB(); // Update position for right encoder
  encLeft.readAB(); // Update position for left encoder
}

void setup() {
  // Initialize Serial port
  Serial.begin(115200);

  // LCD Initialization
  LCD_FOREGROUND = 0b0000001011100000;
  LCD_BACKGROUND = 0b1111111111111111;
  Serial1.begin(9600, SERIAL_8N1, 26, 14); // GPS Serial
  // Wire.begin(32, 27); // I2C Initialization
  // Initialize the DHT sensor
  // dht.begin();
  // Initialize Display
  DisplaySetup();
  display.setRotation(2);
  setLCDBacklight(255);
  display.clearDisplay(LCD_BACKGROUND);
  display.setTextSize(1);
  updateDisplay("Initializing...", 40, false);
  display.display();
  delay(2500);

  // Initialize Pins
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(ENC_RIGHT_BUTTON, INPUT_PULLUP);
  pinMode(ENC_LEFT_BUTTON, INPUT_PULLUP);

  // SD Card Initialization
  initializeSDCard();

  // Wi-Fi Initialization
  WiFi.begin(ssid, password);
  waitForWiFiConnection(); // Blocking connection during setup

  // Web Server Initialization
  initializeWebServer();

  // PM Sensor and GPS Initialization
  initializePMSensorAndGPS();

  // RTC Initialization
  rtc.setTime(0, 0, 0, 1, 1, 2024); // Default time

  // Encoder Initialization
  encLeft.begin();
  encRight.begin();
  encoderRotary.attach_ms(10, encoderISR);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_BUTTON), encRightButtonISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_BUTTON), encLeftButtonISR, FALLING);
  encRightButtonISREn = true;
  encLeftButtonISREn = true;

  // Set Initial State
  state = 0;
}

void initializeSDCard() {
  display.clearDisplay(LCD_BACKGROUND);
  updateDisplay("Checking SD Card", 40, false);
  display.display();
  delay(2500);

  customSPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS_PIN);
  if (!SD.begin(SD_CS_PIN, customSPI, 4000000)) {
    display.clearDisplay(LCD_BACKGROUND);
    updateDisplay("SD card failed", 32, false);
    updateDisplay("Reset device", 48, false);
    display.display();
    while (true); // Halt if SD card fails
  } else {
    display.clearDisplay(LCD_BACKGROUND);
    updateDisplay("SD card detected", 40, false);
    display.display();
  }
  delay(2500);
}

void waitForWiFiConnection() {
  Serial.println("Connecting to WiFi...");
  unsigned long startAttemptTime = millis();
  const unsigned long timeout = 10000; // 10 seconds

  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < timeout) {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nWiFi connection failed. Continuing without connection.");
  }
}

void initializeWebServer() {
  server.serveStatic("/qosain.jpg", SD, "/qosain.jpg");
  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/systemState", HTTP_GET, handleSystemStateAPI);
  server.on("/list", HTTP_GET, handleFileList);
  server.on("/api/fileList", HTTP_GET, handleFileListAPI);
  server.on("/view", handleViewFile);
  server.on("/download", HTTP_GET, handleFileDownload);
  server.on("/delete", HTTP_DELETE, handleFileDelete);
  server.on("/downloadAll", HTTP_GET, handleDownloadAllFiles); // New route for downloading all files
  server.on("/plot", HTTP_GET, handleRealTimeDataDisplay); // Serves the HTML page
  server.on("/api/state", HTTP_GET, handleRealTimeData);    // Serves real-time JSON data
  server.begin();
  Serial.println("Server started");
}

void initializePMSensorAndGPS() {
  pinMode(PM_SET_PIN, OUTPUT);
  digitalWrite(PM_SET_PIN, HIGH);
  PM_SERIAL.begin(9600, SERIAL_8N1, 17, 16);
  pms.init();
  // TPSensor.init();
  dht.begin();

  if (gpsAwake) {
    toggleGps();
  }
}

void loop() {
  
  // check number of milliseconds since Arduino was turned on
  curMillis = millis();

  static unsigned long lastWiFiCheck = 0;
  const unsigned long wifiCheckInterval = 10000; // Check Wi-Fi every 10 seconds

  // Wi-Fi Reconnection Logic
  if (millis() - lastWiFiCheck >= wifiCheckInterval) {
    lastWiFiCheck = millis();
    reconnectWiFi();
  }

  // Handle Web Server Requests
  server.handleClient();

  // display the current page
  displayPage(page);

  if(state == 0) // Navigating menus
  {
    // update the current menu selection
    updateMenuSelection();

    // Check for serial commands
    String msg = "";
    while (Serial.available() > 0)
    {
      char c = Serial.read();
      msg += c;
      // End of command
      if (c == '\n')
      {
        String cmd = String(msg[0]) + String(msg[1]);

        // Get list of all files, if not already created
        if (!fileListAlloc)
        {
          getFileList();
        }
        char allFiles[fileCount][30];
        memcpy(allFiles, fileList, sizeof(allFiles));

        // Send list of files
        if (cmd == "ls")
        {
          #ifdef DEBUG_PRINT
          Serial.println("ls command");
          #endif
          for (int i = 0; i < fileCount; i++)
          {
            Serial.print(allFiles[i]);
            Serial.print('\n');
          }
          Serial.print('\x04');
        }
        // Download files
        else if (cmd == "dl")
        {
          #ifdef DEBUG_PRINT
          Serial.println("dl command");
          #endif

          // Remove command and space from the msg string
          msg = msg.substring(3);

          #ifdef DEBUG_PRINT
          Serial.print("dl args: "); Serial.println(msg);
          #endif

          // Create an array for storing all the files in the argument
          char filesToDownload[fileCount][30];
          String fileName = "";
          uint32_t downloadCount = 0;
          int i = 0;

          // Construct array of files to download
          while (i < msg.length())
          {
            // Encountered a space, add fileName to array
            if (msg[i] == ' ' || msg[i] == '\n')
            {
              #ifdef DEBUG_PRINT
              Serial.print("fileName: "); Serial.println(fileName);
              #endif

              strcpy(filesToDownload[downloadCount], fileName.c_str());

              downloadCount++;

              // Reset fileName
              fileName = "";
            }
            else
            {
              fileName += msg[i];
            }
            i++;
          }
          #ifdef DEBUG_PRINT
          Serial.print("downloadCount: "); Serial.println(downloadCount);
          #endif

          // Upload each file one by one, terminate with end ETX character
          for (int i = 0; i < downloadCount; i++)
          {
            #ifdef DEBUG_PRINT
            Serial.print("Uploading file: "); Serial.println(filesToDownload[i]);
            #endif

            uploadSerial(filesToDownload[i], 100);
            Serial.print('\x03');
          }
          Serial.print('\x04');
        }

        // If we're currently on the page for viewing the files on the SD, don't free() fileList
        if (page != 4)
        {
          free(fileList);
          fileListAlloc = false;
        }
      }
    }

    if(encRightButtonFlag) // select button has been pressed
    {
      if(page == 0) // initial menu
      {
        if(currentVertMenuSelection == 0)
        {
          page = 1; // go to time entry choice page
        }
        else if(currentVertMenuSelection == 1) // upload data
        {
          prevState = state;
          page = 4; // go to page for viewing SD card files
          scroll = 0; // start at the beginning of the file list
          
          // For SDFAt Library
          // #ifdef DEBUG_PRINT
          // Serial.println("\nSD card contents from SD.ls():");
          // SD.ls(LS_R);
          // Serial.println();
          // #endif

          // For SD Library 
        #ifdef DEBUG_PRINT
          Serial.println("\nSD card contents:");
          File root = SD.open("/");  // Open the root directory

          while (File entry = root.openNextFile()) {  // Iterate through files
            Serial.print(entry.name());  // Print file name
            if (entry.isDirectory()) {
              Serial.println("/");  // Indicate directories
            } else {
              Serial.print("\t");
              Serial.println(entry.size());  // Print file size
            }
            entry.close();  // Close each file after reading
          }
          Serial.println();
        #endif


          // Create file list, if not already created
          if (!fileListAlloc)
          {
            getFileList();
          }
        }
      }
      else if (page == 1) // time entry method
      {
        if(currentVertMenuSelection == 0) // Use GPS for time stamp
        {
          manualTimeEntry = false;
          createDataFiles(); // create the names for the data and gps files for data collection
          // State and page are set from within createDataFiles() so that I can set it to different things on success and failure
        }
        else if(currentVertMenuSelection == 1) // Use manual entry + RTC
        {
          page = 2; // enter date
          if(rtcSet)
          {
            manualDay = rtc.getDay();
            manualHour = rtc.getHour();
            manualMinute = rtc.getMinute();
            manualMonth = rtc.getMonth();
            manualYear = rtc.getYear();
          }
        }
      }
      else if (page == 2)
      {
        page = 3; // enter time
      }
      else if (page == 3)
      {
        #ifdef DEBUG_PRINT
        Serial.print("Timestamp set as: ");
        Serial.print(manualMonth);
        Serial.print('/');
        Serial.print(manualDay);
        Serial.print('/');
        Serial.print(manualYear);
        Serial.print(' ');
        if(manualHour < 10) Serial.print('0');
        Serial.print(manualHour);
        Serial.print(':');
        if(manualMinute < 10) Serial.print('0');
        Serial.println(manualMinute);
        #endif

        // set RTC
        // TBD
        //rtc.setDate(manualDay, manualMonth, manualYear % 100); // year is saved as an offset from 2000
        //rtc.setTime(manualHour, manualMinute, 0);

        /////// FOr ESP32 Library
        // Set the date and time using the rtc.setTime() method
        rtc.setTime(manualHour, manualMinute, 0, manualDay, manualMonth, manualYear);
        
        // Get the current time as a time_t value
        time_t now = rtc.getEpoch();

        // Convert time_t to tm structure
        tm* timeInfo = localtime(&now);
        
        if(!rtcSet) rtcSet = true;
        
        manualTimeEntry = true; // flag to indicate that RTC is being used for time stamps, not GPS
        createDataFiles();
        // State and page are set from within createDataFiles() so that I can set it to different things on success and failure
      }
      else if(page == 4)
      {
        // save fileToUpload before uploadSerial is called on it
        memcpy(fileToUpload, fileList + currentVertMenuSelection*30, sizeof(fileToUpload));
        prevState = state;
        state = 3;
      }

      // reset menus for next page
      if(page == 2) currentVertMenuSelection = manualMonth - 1;
      else if(page == 3) currentVertMenuSelection = manualHour;
      else if(page == 4) currentVertMenuSelection = currentVertMenuSelection; // known minor bug here where upon first entering this page the cursor will be on the second selection
      else currentVertMenuSelection = 0;
      currentHoriMenuSelection = 0;
      encRightButtonFlag = false;
      encRightButtonISREn = true;
    }
  }
  else if(state == 2) // Collecting data
  {
    // Blink LED upon successful SD write
    if(ledFlag)
    {
      if(curMillis - prevLedMillis >= BLINK_TIME)
      {
        prevLedMillis = curMillis;
        //blinkLed();
      }
    }

    // Read sensor and update SD card every SAMP_TIME milliseconds
    if(curMillis - prevSampMillis >= SAMP_TIME)
    {
      prevSampMillis = curMillis;
      if(curMillis - prevGpsMillis >= GPS_TIME)
      {
        timestampFlag = true;
        prevGpsMillis = curMillis;
      }
      // createDataFiles();
      updateSampleSD();
    }

    // // Immediate first write and subsequent writes every 4 minutes
    // static unsigned long lastSDUpdate = 0;
    // const unsigned long SDUpdateInterval = 1 * 60 * 1000; // 4 minutes in milliseconds

    // if (lastSDUpdate == 0 || curMillis - lastSDUpdate >= SDUpdateInterval)
    // {
    //     lastSDUpdate = curMillis;
    //     // Perform the SD card update
    //     updateSampleSD();
    // }

  }
  else if(state == 3) // uploading data
  {
    Serial.print(fileToUpload);
    Serial.print(".txt\n");
    uploadSerial(fileToUpload, 2500);
    state = prevState;
    prevState = 3;
    page = 4; // go back to file list
  }

  if(encLeftButtonFlag) // back button has been pressed
  {
    switch(page)
    {
      case 1: // time entry choice menu
        page = 0; // go back to initial menu
        prevState = state;
        state = 0;
        break;
      case 2: // date entry page
        page = 1; // go back to time entry choice menu
        prevState = state;
        state = 0;
        break;
      case 3: // time entry page
        page = 2; // go back to date entry page
        prevState = state;
        state = 0;
        break;
      case 4: // SD card file list menu
        page = 0; // go back to initial menu
        prevState = state;
        state = 0;
        #ifdef DEBUG_PRINT
        Serial.println("free()'ing fileList");
        #endif
        free(fileList); // deallocate memory for list of SD card files
        fileListAlloc = false;
        scroll = 0;
        break;
      case 5: // data collection screen
        page = 1; // go back to time entry choice menu
        prevState = state;
        state = 0;
        break;
    }
    // reset menus for next page
    if(page == 2) currentVertMenuSelection = manualMonth - 1;
    else if(page == 3) currentVertMenuSelection = manualHour;
    else currentVertMenuSelection = 0;
    encLeftButtonFlag = false;
    encLeftButtonISREn = true;
    #ifdef DEBUG_PRINT
    Serial.println("Back button pressed");
    Serial.print("Going to page: ");
    Serial.println(page);
    #endif
  }

}

void reconnectWiFi() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi disconnected. Attempting reconnection...");
    WiFi.begin(ssid, password);

    unsigned long reconnectStart = millis();
    const unsigned long reconnectTimeout = 5000;

    while (WiFi.status() != WL_CONNECTED && millis() - reconnectStart < reconnectTimeout) {
      delay(500);
      Serial.print(".");
    }

    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("\nWiFi reconnected!");
      Serial.print("IP Address: ");
      Serial.println(WiFi.localIP());
    } else {
      Serial.println("\nReconnection failed.");
    }
  }
}

// Function to convert RGB to RGB565
uint16_t RGBtoRGB565(byte r, byte g, byte b) {
    // Scale the 8-bit values to the 5, 6, and 5 bits needed for RGB565
    uint16_t r565 = round(r / 255.0f * 31.0f); // 5-bit red
    uint16_t g565 = round(g / 255.0f * 63.0f); // 6-bit green
    uint16_t b565 = round(b / 255.0f * 31.0f); // 5-bit blue

    // Combine the components into a single 16-bit value
    uint16_t rgb565 = (r565 << 11) | (g565 << 5) | b565;
    return rgb565;
}


// ISR for button being pressed
void encRightButtonISR()
{
  if(encRightButtonISREn == true)
  {
    encRightButtonFlag = true;
    encRightButtonISREn = false;
  }
}

void encLeftButtonISR()
{
  if(encLeftButtonISREn == true)
  {
    encLeftButtonFlag = true;
    encLeftButtonISREn = false;
  }
}

void handleRoot() {
  String html = "<html><body>";

  // Add the company logo at the top of the page
  html += "<div style='text-align:center;'><img src='/qosain.jpg' alt='Company Logo' width='200' height='auto'></div>";

  // Add title and links
  html += "<h1>Qosain Scientific Web Server</h1>";
  html += "<a href='/list'>List Files</a><br>";

  // Add a placeholder for plot link or a message
  html += "<div id='plotSection' style='margin-top: 20px;'></div>";

  // Add JavaScript to dynamically update the plot section
  html += "<script>";
  html += "async function checkState() {";
  html += "  let response = await fetch('/api/systemState');";
  html += "  let data = await response.json();";
  html += "  let plotSection = document.getElementById('plotSection');";

  html += "  if (data.state === 2) {";
  html += "    plotSection.innerHTML = `<a href='/plot'>Real time data and plot</a>`;";
  html += "  } else {";
  html += "    plotSection.innerHTML = `<div style='color: red;'>Data collection is not active. No data and plot available.</div>`;";
  html += "  }";
  html += "}";

  html += "checkState();"; // Call the function to check state when the page loads
  html += "setInterval(checkState, 5000);"; // Periodically check state every 5 seconds
  html += "</script>";

  html += "</body></html>";

  server.send(200, "text/html", html);
}

void handleSystemStateAPI() {
  String json = "{ \"state\": " + String(state) + " }";
  server.send(200, "application/json", json);
}

void handleFileDownload() {
  if (server.hasArg("file")) {
    String path = "/" + server.arg("file");
    
    if (SD.exists(path)) {
      File file = SD.open(path, FILE_READ);
      
      // Get the filename from the path and set the correct content disposition
      String filename = path.substring(path.lastIndexOf("/") + 1);
      server.sendHeader("Content-Disposition", "attachment; filename=" + filename);
      
      // Set the correct MIME type for CSV files
      String mimeType = "text/csv";
      
      // Stream the file with the correct MIME type
      server.streamFile(file, mimeType);
      file.close();
    } else {
      server.send(404, "text/plain", "File not found");
    }
  } else {
    server.send(400, "text/plain", "No file specified");
  }
}

// Function to handle file deletion
void handleFileDelete() {
  if (server.hasArg("file")) {
    String path = "/" + server.arg("file");
    
    if (SD.exists(path)) {
      if (SD.remove(path)) {
        server.send(200, "text/plain", "File deleted successfully");
      } else {
        server.send(500, "text/plain", "Failed to delete file");
      }
    } else {
      server.send(404, "text/plain", "File not found");
    }
  } else {
    server.send(400, "text/plain", "No file specified");
  }
}

void handleFileListAPI() {
  DynamicJsonDocument jsonDoc(1024);
  JsonArray fileList = jsonDoc.createNestedArray("files");

  File dir = SD.open("/");
  while (File entry = dir.openNextFile()) {
    String filename = entry.name();
    fileList.add(filename);
  }
  dir.close();

  String jsonResponse;
  serializeJson(jsonDoc, jsonResponse);
  server.send(200, "application/json", jsonResponse);
}

void handleFileList() {
  String html = "<html><body>";

  // Add company logo
  html += "<div style='text-align:center;'><img src='/qosain.jpg' alt='Company Logo' width='200' height='auto'></div>";

  // Title and dynamic file list
  html += "<h1>Available Files</h1>";

  // "Download All" button
  html += "<button style='margin-bottom:10px;' onclick='downloadAllFiles()'>Download All</button>";

  html += "<ul id='fileList'>Loading...</ul>"; // Placeholder for the dynamic file list

  // JavaScript for fetching and updating the file list
  html += "<script>";
  html += "async function fetchFileList() {";
  html += "  let response = await fetch('/api/fileList');";
  html += "  let data = await response.json();";
  html += "  let fileList = document.getElementById('fileList');";
  html += "  fileList.innerHTML = '';"; // Clear current list
  html += "  data.files.forEach(file => {";
  html += "    if (file.endsWith('.csv')) {"; // Only process .csv files
  html += "      let listItem = document.createElement('li');";
  html += "      listItem.innerHTML = `<a href='/view?file=${file}'>${file}</a> ` +";
  html += "                          `<button onclick='downloadFile(\"${file}\")'>Download</button> ` +";
  html += "                          `<button onclick='deleteFile(\"${file}\")'>Delete</button>`;";
  html += "      fileList.appendChild(listItem);";
  html += "    }";
  html += "  });";
  html += "}";

  // Download a single file
  html += "function downloadFile(fileName) {";
  html += "  window.location.href = `/download?file=${fileName}`;";
  html += "}";

  // Delete a single file
  html += "async function deleteFile(fileName) {";
  html += "  if (confirm(`Are you sure you want to delete ${fileName}?`)) {";
  html += "    let response = await fetch(`/delete?file=${fileName}`, { method: 'DELETE' });";
  html += "    if (response.ok) {";
  html += "      alert(`${fileName} has been deleted.`);";
  html += "      fetchFileList();"; // Refresh the file list
  html += "    } else {";
  html += "      alert('Failed to delete the file.');";
  html += "    }";
  html += "  }";
  html += "}";

  // Download all files
  html += "function downloadAllFiles() {";
  html += "  fetch('/api/fileList').then(response => response.json()).then(data => {";
  html += "    data.files.filter(file => file.endsWith('.csv')).forEach(file => {";
  html += "      let a = document.createElement('a');";
  html += "      a.href = `/download?file=${file}`;";
  html += "      a.download = file;";
  html += "      document.body.appendChild(a);";
  html += "      a.click();";
  html += "      document.body.removeChild(a);";
  html += "    });";
  html += "  });";
  html += "}";

  html += "fetchFileList();";  // Fetch files when the page loads
  html += "setInterval(fetchFileList, 10000);";  // Refresh file list every 10 seconds
  html += "</script>";

  html += "</body></html>";
  server.send(200, "text/html", html);
}

void handleDownloadAllFiles() {
  File root = SD.open("/");
  if (!root) {
    server.send(500, "text/plain", "Failed to open SD card.");
    return;
  }

  String allFiles = "";
  while (File file = root.openNextFile()) {
    if (!file.isDirectory() && String(file.name()).endsWith(".csv")) {
      allFiles += String(file.name()) + "\n";
    }
    file.close();
  }
  root.close();

  if (allFiles == "") {
    server.send(404, "text/plain", "No .csv files found.");
  } else {
    // Response content with file list for download
    server.send(200, "text/plain", "List of files to download:\n" + allFiles);
  }
}

// Function to handle viewing .csv files
void handleViewFile() {
  String filename = server.arg("file");
  
  // Open the file from the SD card
  File file = SD.open("/" + filename);
  if (!file) {
    server.send(404, "text/plain", "File not found");
    return;
  }
  
  // Set content type to plain text for .csv files
  server.streamFile(file, "text/plain");
  file.close();
}

void handleRealTimeDataDisplay() {
  // Check the system state before allowing access to the plot page (Server-side check)
  if (state != 2) {
    // Redirect to the root page if state is not 2
    server.sendHeader("Location", "/");
    server.send(303); // HTTP 303 See Other (redirect)
    return;
  }

  String html = "<html><head>";
  html += "<meta charset='UTF-8'>";  // Ensure UTF-8 encoding is set
  html += "<script src='https://cdn.plot.ly/plotly-2.20.0.min.js'></script>";
  html += "<style>";
  html += "body { font-family: Arial, sans-serif; }";
  html += "h3 { font-weight: normal; }";
  html += ".box {";
  html += "  border: 2px solid black;";
  html += "  padding: 10px;";
  html += "  width: 120px;";
  html += "  height: 110px;";
  html += "  line-height: 1.5;";
  html += "  text-align: center;";
  html += "  background-color: #f4f4f4;";
  html += "  margin: 10px;";
  html += "  display: flex;";
  html += "  flex-direction: column;";
  html += "  justify-content: center;";
  html += "  align-items: center;";
  html += "  border-radius: 8px;";
  html += "  box-shadow: 0 4px 6px rgba(0,0,0,0.1);";
  html += "}";
  html += ".flex-container {";
  html += "  display: flex;";
  html += "  justify-content: center;";
  html += "  gap: 20px;";
  html += "  margin-top: 20px;";
  html += "}";
  html += ".bold { font-weight: bold; }";
  html += ".unit { font-weight: normal; font-size: 0.9em; }";
  html += ".time-date { display: flex; flex-direction: column; align-items: center; }";
  html += "</style>";
  html += "</head><body>";

  html += "<div style='text-align:center;'><img src='/qosain.jpg' alt='Company Logo' width='200' height='auto'></div>";
  html += "<div class='flex-container'>";

  html += "<div class='box'>";
  html += "<h3>Time & Date</h3>";
  html += "<div class='time-date'>";
  html += "<div id='time' class='bold'></div>";
  html += "<div id='date'></div>";
  html += "</div>";
  html += "</div>";

  html += "<div class='box'>";
  html += "<h3>PM1.0</h3>";
  html += "<div><span id='pm1' class='bold'></span><span class='unit'> µg/m³</span></div>";
  html += "</div>";

  html += "<div class='box'>";
  html += "<h3>PM2.5</h3>";
  html += "<div><span id='pm25' class='bold'></span><span class='unit'> µg/m³</span></div>";
  html += "</div>";

  html += "<div class='box'>";
  html += "<h3>PM10.0</h3>";
  html += "<div><span id='pm10' class='bold'></span><span class='unit'> µg/m³</span></div>";
  html += "</div>";

  html += "<div class='box'>";
  html += "<h3>Temperature</h3>";
  html += "<div><span id='temperature' class='bold'></span><span class='unit'> °C</span></div>";
  html += "</div>";

  html += "<div class='box'>";
  html += "<h3>Humidity</h3>";
  html += "<div><span id='humidity' class='bold'></span><span class='unit'> %</span></div>";
  html += "</div>";

  html += "</div>";
  html += "<div id='plot' style='width: 90%; height: 500px; margin: 20px auto;'></div>";

  html += "<script>";
  // Define traces for PM1.0, PM2.5, and PM10.0
  html += "let pm1Data = { x: [], y: [], type: 'scatter', mode: 'lines+markers', name: 'PM1.0', marker: { color: 'red' } };";
  html += "let pm25Data = { x: [], y: [], type: 'scatter', mode: 'lines+markers', name: 'PM2.5', marker: { color: 'blue' } };";
  html += "let pm10Data = { x: [], y: [], type: 'scatter', mode: 'lines+markers', name: 'PM10.0', marker: { color: 'green' } };";

  // Layout to accommodate multiple y-axes
  html += "let layout = {";
  html += "  title: 'PM1.0, PM2.5, and PM10.0 vs Time',";
  html += "  xaxis: { title: 'Time' },";
  html += "  yaxis: { title: 'PM (µg/m³)', range: [0, null] },";
  html += "  showlegend: true,";
  html += "  grid: { rows: 1, columns: 1, pattern: 'independent' }";
  html += "};";

  html += "Plotly.newPlot('plot', [pm1Data, pm25Data, pm10Data], layout);";

  html += "async function updateData() {";
  html += "  let response = await fetch('/api/state');";
  html += "  let state = await response.json();";

  html += "  let currentTime = state.time.split(':');";
  html += "  let hours = currentTime[0].padStart(2, '0');";
  html += "  let minutes = currentTime[1].padStart(2, '0');";
  html += "  let seconds = currentTime[2].padStart(2, '0');";
  html += "  let formattedTime = `${hours}:${minutes}:${seconds}`;";
  html += "  document.getElementById('time').innerHTML = formattedTime;";

  // Fix and reverse the date
  html += "  let dateParts = state.date.split('-');"; // Expecting YYYY-MM-DD
  html += "  let reversedDate = `${dateParts[2]}-${dateParts[1]}-${dateParts[0]}`;"; // Rearrange to DD-MM-YYYY
  html += "  document.getElementById('date').innerHTML = reversedDate;";

  html += "  document.getElementById('pm1').innerHTML = state['PM1.0'];";
  html += "  document.getElementById('pm25').innerHTML = state['PM2.5'];";
  html += "  document.getElementById('pm10').innerHTML = state['PM10.0'];";
  html += "  document.getElementById('temperature').innerHTML = state.temperature;";
  html += "  document.getElementById('humidity').innerHTML = state.humidity;";

  // Update the plot data
  html += "  pm1Data.x.push(formattedTime);";
  html += "  pm1Data.y.push(state['PM1.0']);";
  html += "  pm25Data.x.push(formattedTime);";
  html += "  pm25Data.y.push(state['PM2.5']);";
  html += "  pm10Data.x.push(formattedTime);";
  html += "  pm10Data.y.push(state['PM10.0']);";

  // Update the plot
  html += "  Plotly.update('plot', { x: [pm1Data.x, pm25Data.x, pm10Data.x], y: [pm1Data.y, pm25Data.y, pm10Data.y] });";
  html += "}";
  
  html += "function startUpdates() {";
  html += "  setInterval(updateData, 5000);";
  html += "  updateData();";
  html += "}";

  html += "startUpdates();";
  html += "</script>";
  html += "</body></html>";

  server.send(200, "text/html", html);
}

void handleRealTimeData() {
    bool timeoutFlag = false;
    time_t utcTime;

    // Read sensors
    float humidity = dht.readHumidity();            // Humidity
    float temperature = dht.readTemperature();      // Temperature in Celsius
    pms.read();
    uint16_t pm1p0 = pms.pm01;      // PM1.0 concentration in μg/m³
    uint16_t pm25 = pms.pm25;      // PM2.5 concentration in μg/m³
    uint16_t pm10p0 = pms.pm10;    // PM10.0 concentration in μg/m³

    // Get timestamp
    if (!manualTimeEntry) {
        if (!gpsAwake) toggleGps();
        unsigned long gpsStart = millis();

        while (true) {
            readGps();
            if (millis() - gpsStart >= GPS_TIMEOUT) {
                timeoutFlag = true;
                break;
            }
            if (gps.date.isValid() && gps.time.isValid() && gps.date.year() == CUR_YEAR) {
                setTime(
                    gps.time.hour(), gps.time.minute(), gps.time.second(),
                    gps.date.day(), gps.date.month(), gps.date.year()
                );
                utcTime = now();
                rtc.setTime(second(utcTime), minute(utcTime), hour(utcTime), day(utcTime), month(utcTime), year(utcTime));
                break;
            }
        }
        if (gpsAwake) toggleGps();
    }

    if (manualTimeEntry || timeoutFlag) {
        utcTime = now();
    }

    // Format UTC time
    String formattedDate = String(year(utcTime)) + "-" + String(month(utcTime)) + "-" + String(day(utcTime));
    String formattedTime = String(hour(utcTime)) + ":" + String(minute(utcTime)) + ":" + String(second(utcTime));

    // Create JSON response
    String jsonData = "{";
    jsonData += "\"PM1.0\":" + String(pm1p0) + ",";  // PM1.0
    jsonData += "\"PM2.5\":" + String(pm25) + ",";  // PM2.5
    jsonData += "\"PM10.0\":" + String(pm10p0) + ",";  // PM10.0
    jsonData += "\"temperature\":" + String(temperature, 2) + ",";  // Temperature
    jsonData += "\"humidity\":" + String(humidity, 2) + ",";  // Humidity
    jsonData += "\"date\":\"" + formattedDate + "\",";
    jsonData += "\"time\":\"" + formattedTime + "\"";
    jsonData += "}";

    server.send(200, "application/json", jsonData);
    Serial.println("Real-time data sent to web client:");
    Serial.println(jsonData);
}

// update samples in SD card
void updateSampleSD() {
    bool timeoutFlag = false;
    time_t utcTime;
    unsigned long msTimer;

    if (firstMeasurementFlag) {
        firstMeasurementFlag = false;
        msTimer = 0;
        dataStartMillis = millis();
    } else {
        msTimer = millis() - dataStartMillis;
    }

    float temp;
    float humidity;

    // Read the PM sensor
    pms.read();
    uint16_t PM1p0_atm = pms.pm01;      // PM1.0 concentration in μg/m³
    uint16_t PM2p5_atm = pms.pm25;      // PM2.5 concentration in μg/m³
    uint16_t PM10p0_atm = pms.pm10;     // PM10.0 concentration in μg/m³
    uint16_t count_0p3um = pms.n0p3;    // Number of particles with diameter > 0.3µm
    uint16_t count_0p5um = pms.n0p5;    // Number of particles with diameter > 0.5µm
    uint16_t count_1p0um = pms.n1p0;    // Number of particles with diameter > 1.0µm
    uint16_t count_2p5um = pms.n2p5;    // Number of particles with diameter > 2.5µm
    uint16_t count_5p0um = pms.n5p0;    // Number of particles with diameter > 5.0µm
    uint16_t count_10p0um = pms.n10p0;  // Number of particles with diameter > 10.0µm

    if (timestampFlag) { // If it is time to get a time stamp
        // Read temperature and pressure
        temp = dht.readTemperature();
        humidity = dht.readHumidity();

        // If you chose to use GPS, keep getting time, lat, long, and alt from GPS
        if (!manualTimeEntry) {
            display.clearDisplay(LCD_BACKGROUND);
            display.drawLine(0, display.height() - 12, display.width() - 1, display.height() - 12, LCD_FOREGROUND);
            display.drawLine(display.width() / 2 - 1, display.height() - 10, display.width() / 2 - 1, display.height() - 1, LCD_FOREGROUND);
            display.setTextColor(LCD_FOREGROUND);
            display.setCursor(10, display.height() - 10);
            display.print("Back ");
            // updateDisplay("Reading GPS...", 40, false);
            // display.display();

            // Wake up GPS module
            if (!gpsAwake) {
                toggleGps();
            }
            unsigned long gpsReadCurMillis;
            unsigned long gpsReadStartMillis = millis();
            unsigned long gpsTimeoutMillis = GPS_TIMEOUT;

            // Read GPS data until it's valid
            // 5 second timeout
            while (true) {
                gpsReadCurMillis = millis();
                readGps();

                if (gpsReadCurMillis - gpsReadStartMillis >= gpsTimeoutMillis) {
                    timeoutFlag = true;
                    gpsDisplayFail = true;
                    break;
                }

                if (gps.date.isValid() && gps.time.isValid() && gps.location.isValid() && gps.altitude.isValid() && gps.date.year() == CUR_YEAR) {
                    // Set time for now()
                    setTime(gps.time.hour(), gps.time.minute(), gps.time.second(), gps.date.day(), gps.date.month(), gps.date.year());

                    // Check for stale GPS timestamps
                    if (now() > prevTimeStamp) {
                        prevTimeStamp = now();
                        gpsDisplayFail = false;

                        // Resync RTC every successful GPS read
                        rtc.setTime(
                            gps.time.second(),    // seconds
                            gps.time.minute(),    // minutes
                            gps.time.hour(),      // hours
                            gps.date.day(),       // day
                            gps.date.month(),     // month
                            gps.date.year()       // year
                        );

                        if (!rtcSet) rtcSet = true;
                        break;
                    }
                }

                // Make GPS reads interruptible by the button being pressed
                if (encLeftButtonFlag) {
                    // Put GPS to sleep
                    if (gpsAwake) {
                        toggleGps();
                    }
                    return;
                }
            }

            // Store UTC time
            utcTime = now();
            latitude = gps.location.lat();
            longitude = gps.location.lng();
            altitude = gps.altitude.meters();

            // Put GPS to sleep
            if (gpsAwake) {
                toggleGps();
                if (timeoutFlag) {
                    delay(500); // Add extra delay after timeout to ensure sleep command is properly interpreted before next read
                }
            }

            // Store time from GPS
            utcYear = year(utcTime);
            utcMonth = month(utcTime);
            utcDay = day(utcTime);
            utcHour = hour(utcTime);
            utcMinute = minute(utcTime);
            utcSecond = second(utcTime);
        }

        if (manualTimeEntry || timeoutFlag) { // If manual time entry or GPS timed out, overwrite timestamp with RTC values
            utcYear = year();       // Returns the year (e.g., 2024)
            utcMonth = month();     // Returns the month (1 = January, 12 = December)
            utcDay = day();         // Returns the day of the month
            utcHour = hour();       // Returns the current hour (0-23)
            utcMinute = minute();   // Returns the current minute (0-59)
            utcSecond = second();    // Returns the current second (0-59)
        }
    }

    // Open data file
    dataFile = SD.open(startWithSlash(dataFileName), FILE_APPEND);
    if (dataFile) {
        // Writing metadata and data together
        dataFile.print(msTimer);
        dataFile.print(',');
        dataFile.print(utcYear);
        dataFile.print('-');
        dataFile.print(utcMonth);
        dataFile.print('-');
        dataFile.print(utcDay);
        dataFile.print('T');
        if (utcHour < 10) dataFile.print('0');
        dataFile.print(utcHour);
        dataFile.print(':');
        if (utcMinute < 10) dataFile.print('0');
        dataFile.print(utcMinute);
        dataFile.print(':');
        if (utcSecond < 10) dataFile.print('0');
        dataFile.print(utcSecond);
        dataFile.print("+00:00");

        if (manualTimeEntry || timeoutFlag) {
            dataFile.print(",,,");
        } else {
            dataFile.print(',');
            dataFile.print(latitude, 5);
            dataFile.print(',');
            dataFile.print(longitude, 5);
            dataFile.print(',');
            dataFile.print(altitude);
        }

        dataFile.print(',');
        dataFile.print(temp, 2);
        dataFile.print(',');
        dataFile.print(humidity, 2);
        dataFile.print(",");
        dataFile.print(PM1p0_atm);    // PM1.0 (atmo)
        dataFile.print(",");
        dataFile.print(PM2p5_atm);    // PM2.5 (atmo)
        dataFile.print(",");
        dataFile.print(PM10p0_atm);   // PM10.0 (atmo)
        // dataFile.print(",");
        // dataFile.print(count_0p3um);   // >0.3µm particle count
        // dataFile.print(",");
        // dataFile.print(count_0p5um);   // >0.5µm particle count
        // dataFile.print(",");
        // dataFile.print(count_1p0um);   // >1.0µm particle count
        // dataFile.print(",");
        // dataFile.print(count_2p5um);   // >2.5µm particle count
        // dataFile.print(",");
        // dataFile.print(count_5p0um);   // >5.0µm particle count
        // dataFile.print(",");
        // dataFile.print(count_10p0um);  // >10.0µm particle count
        dataFile.print('\n');
        dataFile.flush(); // Ensure data is written to the file
        dataFile.close();
        Serial.println("Data logged.");
    } else {
        #ifdef DEBUG_PRINT
        Serial.println("Couldn't open data file");
        #endif
        display.clearDisplay(LCD_BACKGROUND);
        updateDisplay("Couldn't open data file", 40, false);
        display.display();
    }
}

// upload SD card data over serial port
void uploadSerial(char * fileName, uint32_t wait)
{
  // disable button ISRs
  encRightButtonISREn = false; 
  encLeftButtonISREn = false;
  uint8_t buffer[512] = {0}; // buffer to read/write data 512 bytes at a time
  uint16_t writeLen = sizeof(buffer);
  display.clearDisplay(LCD_BACKGROUND);
  updateDisplay("Uploading ", 40, false);
  updateDisplay(fileName, 48, false);
  updateDisplay("via serial port", 56, false);
  display.display();

  char fileNameExtension[30];
  strcpy(fileNameExtension, fileName);
  strcat(fileNameExtension, ".txt");

  #ifdef DEBUG_PRINT
  Serial.println("Serial upload initiated");
  Serial.print("Uploading: ");
  #endif
  // Not sending file name at top of file anymore, do this explicitly when it's sent via the upload menu to maintain backwards compatibility
  // Serial.print(fileNameExtension); Serial.print('\n');

  File file = SD.open(startWithSlash(fileNameExtension), FILE_READ);
  // File file = SD.open(fileNameExtension, FILE_READ);
  while(file.available())
  {
    if (file.available() > sizeof(buffer))
    {
      writeLen = sizeof(buffer);
      file.read(buffer, sizeof(buffer));
    }
    else
    {
      writeLen = file.available();  
      file.read(buffer, file.available());
    }
  
    Serial.write(buffer, writeLen);
    memset(buffer, 0, sizeof(buffer));
  }
  file.close();
  delay(wait);
  
  encRightButtonISREn = true;
  encLeftButtonISREn = true;
}

// read GPS module and encode data
void readGps()
{
  while(Serial1.available() > 0)
  {
    int b = Serial1.peek();
    Serial.printf("GPS Data: %d\r\n", b);
    if(gps.encode(Serial1.read()))
    {
      #ifdef DEBUG_PRINT
      // Serial.println("GPS data successfully encoded");
      #endif
    }
  }
}

// toggle GPS between sleep and wake
void toggleGps()
{
  sendGpsCommand("051,0");
  delay(100);
  gpsAwake = !gpsAwake;
  #ifdef DEBUG_PRINT
  if (gpsAwake)
  {
    Serial.println("GPS awake");
  }
  else
  {
    Serial.println("GPS asleep");
  }
  #endif
}

// send GPS command
void sendGpsCommand(const char* cmd)
{
  char cmdBase[] = "PGKC";
  char* finalCmd = strcat(cmdBase, cmd); // data between the $ and * - on which checksum is based
  char checksum = createChecksum(finalCmd);

  Serial1.write('$');
  Serial1.write(finalCmd);
  Serial1.write('*');
  Serial1.print(checksum, HEX);
  Serial1.write("\r\n");

  #ifdef DEBUG_PRINT
  Serial.print("Command sent to GPS: ");
  Serial.write('$');
  Serial.write(finalCmd);
  Serial.write('*');
  Serial.print(checksum, HEX);
  Serial.write("\r\n");
  #endif
}

// create a checksum for GPS command
char createChecksum(char* cmd)
{
  char checksum = 0;

  for(int i = 0; i < strlen(cmd); i++)
  {
    checksum = checksum ^ cmd[i];
  }

  return checksum;
}

String startWithSlash(char* chr){
  if(chr[0] == '/')
    return String(chr);
  return String("/") + String(chr); 
}
String startWithSlash(String& str){
  if(str[0] == '/')
    return str;
  return String("/") + str; 
}
// Create two data files, one for GPS and one for dust data
void createDataFiles()
{
  Serial.println("createDataFiles()");
  bool newFileCreated = false;

  if(!manualTimeEntry)
  {
    // get time stamp from GPS, set RTC
    unsigned long gpsReadCurMillis;
    unsigned long gpsReadStartMillis = millis();
    unsigned long gpsTimeoutMillis = GPS_FIRST_TIMEOUT;

      display.clearDisplay(LCD_BACKGROUND);
      display.drawLine(0, display.height()-12, display.width()-1, display.height()-12, LCD_FOREGROUND);  //-10
      display.drawLine(display.width()/2 - 1, display.height()-10, display.width()/2 - 1, display.height()-1, LCD_FOREGROUND);
      display.setTextColor(LCD_FOREGROUND);
      display.setCursor(10, display.height()-10);       //  -8
      display.print("Back ");
      updateDisplay("Reading GPS...", 40, false);
      updateDisplay("(First GPS read", 56, false);
      updateDisplay("may take a", 64, false);
      updateDisplay("few minutes)", 72, false);
      display.display();

    if(!gpsAwake)
    {
      toggleGps();
    }

    while (true)
    {
      gpsReadCurMillis = millis();

      readGps();

      if (gpsReadCurMillis - gpsReadStartMillis >= gpsTimeoutMillis)
      {
        // timeoutFlag = true;
        gpsDisplayFail = true;
        #ifdef DEBUG_PRINT 
        Serial.println("GPS timeout");
        #endif
        display.clearDisplay(LCD_BACKGROUND);
        display.drawLine(0, display.height()-12, display.width()-1, display.height()-12, LCD_FOREGROUND);     //-10
        display.drawLine(display.width()/2 - 1, display.height()-10, display.width()/2 - 1, display.height()-1, LCD_FOREGROUND);
        display.setTextColor(LCD_FOREGROUND);
        display.setCursor(10, display.height()-10);    //-8
        display.print("Back ");
        updateDisplay("GPS read failed", 40, false);
        updateDisplay("Please enter time", 56, false);
        updateDisplay("manually", 64, false);
        display.display();
        // turn off GPS
        if(gpsAwake)
        {
          toggleGps();
        }
        while(!encLeftButtonFlag); // wait for back button to be pressed
        encLeftButtonFlag = false;
        encLeftButtonISREn = true;
        prevState = state;
        state = 0;
        page = 1;
        return;
      }

      if (gps.date.isValid() && gps.time.isValid() && gps.location.isValid() && gps.altitude.isValid() && gps.date.year() == CUR_YEAR)
      {
        #ifdef DEBUG_PRINT
        Serial.println("GPS data valid");
        #endif

        // set time for now()
        // setTime(gps.time.hour(), gps.time.minute(), gps.time.second(), gps.date.day(), gps.date.month(), gps.date.year());
        // prevTimeStamp = now();

        // GPS data is valid, set RTC
        // TBD
        // rtc.setDay(gps.date.day());
        // rtc.setMonth(gps.date.month());
        // rtc.setYear(gps.date.year() % 100);
        // rtc.setHours(gps.time.hour());
        // rtc.setMinutes(gps.time.minute());
        // rtc.setSeconds(gps.time.second());

        // GPS data is valid, set RTC using ESP32Time methods
        // rtc.setTime(gps.time.second(), gps.time.minute(), gps.time.hour(), 
        //         gps.date.day(), gps.date.month(), gps.date.year());

         if(!rtcSet) rtcSet = true;

        break;
      
      }
      // else{
      //   // setTime(manualHour, manualMinute, manualsecond, manualDay, manualMonth, manualYear);
      //   // rtc.setTime(manualsecond, manualMinute, manualHour, manualDay, manualMonth, manualYear);
      // }

      // make GPS reads interruptible by the button being pressed
      if (encLeftButtonFlag)
      {
        // Put GPS to sleep
        if (gpsAwake)
        {
          toggleGps();
        }
        page = 1; // go back to time entry choice menu
        prevState = state;
        state = 0;
        encLeftButtonFlag = false;
        encLeftButtonISREn = true;
        return;
      }
    }

  }

  // put GPS to sleep if it's woken up
  // if(gpsAwake)
  // {
  //   toggleGps();
  // }

// Get current time from RTC
int year; 
int month;
int day;
int hour;
int minutes;
int seconds;

  // // strings to concatenate into file names
  // char yearStr[3];
  // char monthStr[3];
  // char dayStr[3];
  // char hourStr[3];
  // char minutesStr[3];
  // char secondsStr[3];
  // char baseString[15]; // YYMMDD_HHMMSS_


if(!gpsAwake)
{
  // toggleGps;
  // year = rtc.getYear();
  // month = rtc.getMonth();
  // day = rtc.getDay();
  // hour = rtc.getHour();
  // minutes = rtc.getMinute();
  // seconds = rtc.getSecond();
  year = manualYear;
  month  = manualMonth;
  day = manualDay;
  hour = manualHour;
  minutes = manualMinute;
  seconds = second();
}
else
{
  readGps();
  year = gps.date.year();
  month = gps.date.month();
  day = gps.date.day();
  hour = gps.time.hour();
  minutes = gps.time.minute();
  seconds = gps.time.second();
  // toggleGps();
}
// Build the filename using String class
String fileNameSeed = String(year) + (month < 10 ? "0" : "") + String(month) +
               (day < 10 ? "0" : "") + String(day) + "_" +
               (hour < 10 ? "0" : "") + String(hour) +
               (minutes < 10 ? "0" : "") + String(minutes) +
               (seconds < 10 ? "0" : "") + String(seconds);
               
dataFileName = fileNameSeed + "_data.csv";
metaFileName = fileNameSeed + "_mata.csv";

  #ifdef DEBUG_PRINT
  Serial.print("dataFileName: ");
  Serial.println(dataFileName);
  Serial.print("metaFileName: ");
  Serial.println(metaFileName);
  #endif
  
  File newFile;

  // Create column headers for new data file
  if(!SD.exists(startWithSlash(dataFileName)))
  // if(!SD.exists(dataFileName))
  {
    #ifdef DEBUG_PRINT
    Serial.println("Writing column headers for new data file");
    #endif
    newFile = SD.open(startWithSlash(dataFileName), FILE_WRITE);
    // newFile = SD.open(dataFileName, FILE_WRITE);
    if(newFile)
    {
      // #ifdef DEBUG_PRINT
      // Serial.print("ms,UTC_timestamp,latitude,longitude,altitude,temperature,pressure,PM1.0,PM2.5,PM10.0,0.3um,0.5um,1.0um,2.5um,5.0um,10.0um");
      // #endif
      // newFile.print("ms,UTC_timestamp,latitude,longitude,altitude,temperature,pressure,PM1.0,PM2.5,PM10.0,0.3um,0.5um,1.0um,2.5um,5.0um,10.0um\n");
      #ifdef DEBUG_PRINT
      Serial.print("ms,UTC_timestamp,latitude,longitude,altitude,temperature,humidity,PM1.0,PM2.5,PM10.0");
      #endif
      newFile.print("ms,UTC_timestamp,latitude,longitude,altitude,temperature,humidity,PM1.0,PM2.5,PM10.0\n");
    }
    else 
    {
      #ifdef DEBUG_PRINT
      Serial.println("Couldn't open data file");
      #endif
    }
    newFile.close();
  }

  // collect data and go to data collection screen
  prevState = state;
  state = 2;
  page = 5;
  firstMeasurementFlag = true;
}

// Update current menu selection based on encoders
void updateMenuSelection()
{
  long encRightPosition = encRight.getPosition();
  long encLeftPosition = encLeft.getPosition();
  //Serial.print("EncA"); Serial.print(encRightPosition);
  //Serial.print(", EncB"); Serial.println(encLeftPosition);
  #ifdef DEBUG_PRINT
  // Serial.print("Right encoder position: ");
  // Serial.println(encRightPosition);
  #endif
  if (encRightPosition > encRightOldPosition + 2) // clockwise, go down
  {
    #ifdef DEBUG_PRINT
    Serial.println("Right knob turned cw");
    #endif
    encRightOldPosition = encRightPosition;

    if(curMillis >= prevMenuMillis + MENU_UPDATE_TIME) // only update menu selection every 100ms
    {
      prevMenuMillis = curMillis;
      prevVertMenuSelection = currentVertMenuSelection;
      currentVertMenuSelection++;
      switch (page)
      {
        case 0: case 1: // initial two menus
          if(currentVertMenuSelection > 1) currentVertMenuSelection = 1; // only two choices on these pages
          break;
        case 2: // entering date
          if(currentHoriMenuSelection == 0) // month
          {
            if(currentVertMenuSelection > 11) currentVertMenuSelection = 0;
            manualMonth = currentVertMenuSelection + 1;
          }
          else if(currentHoriMenuSelection == 1) // day
          {
            switch(manualMonth)
            {
              case 4: case 6: case 9: case 11: // Apr, Jun, Sept, Nov
                if(currentVertMenuSelection > 29) currentVertMenuSelection = 0;
                break;
              case 1: case 3: case 5: case 7: case 8: case 10: case 12: // Jan, Mar, May, Jul, Aug, Oct, Dec
                if(currentVertMenuSelection > 30) currentVertMenuSelection = 0;
                break;
              case 2: // February
                if(manualYear % 4 == 0) 
                {
                  if(currentVertMenuSelection > 28) currentVertMenuSelection = 0;
                }
                else
                {
                  if(currentVertMenuSelection > 27) currentVertMenuSelection = 0;
                }
                break;
            }
            manualDay = currentVertMenuSelection + 1;
          }
          else if(currentHoriMenuSelection == 2) // year
          {
            if(currentVertMenuSelection > 2099) 
            {
              currentVertMenuSelection = CUR_YEAR;
            }
            else if(currentVertMenuSelection < CUR_YEAR)
            {
              currentVertMenuSelection = CUR_YEAR;
            }
            manualYear = currentVertMenuSelection;
          }
          break;
        case 3: // entering time
          if(currentHoriMenuSelection == 0) // hour
          {
            if(currentVertMenuSelection > 23) currentVertMenuSelection = 0;
            manualHour = currentVertMenuSelection;
          }
          else if(currentHoriMenuSelection == 1) // minute
          {
            if(currentVertMenuSelection > 59)
            {
              currentVertMenuSelection = 0;
              manualHour++;
              if(manualHour > 23) manualHour = 0;
            }
            manualMinute = currentVertMenuSelection;
          }
          break;
        case 4: // selecting file from SD card
          if(currentVertMenuSelection > fileCount - 1) currentVertMenuSelection = fileCount - 1;
          if(currentVertMenuSelection % 12 == 0 && currentVertMenuSelection != 0 && currentVertMenuSelection != prevVertMenuSelection)
          {
            scroll++;
          }
          break;
      }
      #ifdef DEBUG_PRINT
      Serial.print("Current vert menu selection: ");
      Serial.println(currentVertMenuSelection);
      #endif
    }
  }
  else if (encRightPosition < encRightOldPosition - 2) // counterclockwise, go up
  {
    #ifdef DEBUG_PRINT
    Serial.println("Right knob turned ccw");
    #endif
    encRightOldPosition = encRightPosition;

    if(curMillis >= prevMenuMillis + MENU_UPDATE_TIME) // only update menu selection every 100ms
    {
      prevMenuMillis = curMillis;
      prevVertMenuSelection = currentVertMenuSelection;
      currentVertMenuSelection--;
      switch(page)
      {
        case 0: case 1:
          if (currentVertMenuSelection < 0) currentVertMenuSelection = 0; // stay at top of menu
          break;
        case 2: // entering date
          if(currentHoriMenuSelection == 0) // entering month
          {
            if(currentVertMenuSelection < 0) currentVertMenuSelection = 11; // wrap around
            manualMonth = currentVertMenuSelection + 1;
          }
          else if(currentHoriMenuSelection == 1) // entering day
          {
            if(currentVertMenuSelection < 0)
            {
              switch(manualMonth)
              {
                case 4: case 6: case 9: case 11: // April, June, September, November
                  currentVertMenuSelection = 29;
                  break;
                case 1: case 3: case 5: case 7: case 8: case 10: case 12: // Jan, Mar, May, Jul, Aug, Oct, Dec
                  currentVertMenuSelection = 30;
                  break;
                case 2:
                  if(manualYear % 4 == 0) // leap year
                  {
                    currentVertMenuSelection = 28;
                  }
                  else
                  {
                    currentVertMenuSelection = 27;
                  }
              }
            }
            manualDay = currentVertMenuSelection + 1;
          }
          else if(currentHoriMenuSelection == 2) // entering year
          {
            if(currentVertMenuSelection > 2099)
            {
              currentVertMenuSelection = CUR_YEAR;
            }
            else if(currentVertMenuSelection < CUR_YEAR)
            {
              currentVertMenuSelection = CUR_YEAR;
            }
            manualYear = currentVertMenuSelection;
          }
          break;
        case 3: // entering time
          if(currentHoriMenuSelection == 0) // hour
          {
            if(currentVertMenuSelection < 0) currentVertMenuSelection = 23;
            manualHour = currentVertMenuSelection;
          }
          else if(currentHoriMenuSelection == 1) // minute
          {
            if(currentVertMenuSelection < 0)
            {
              currentVertMenuSelection = 59;
              manualHour--;
              if(manualHour > 200) manualHour = 23;
            }
            manualMinute = currentVertMenuSelection;
          }
          break;
        case 4:
          if (currentVertMenuSelection < 0) currentVertMenuSelection = 0; // stay at top of menu
          if(currentVertMenuSelection % 12 == 11)
          {
            scroll--;
            if(scroll < 0) scroll = 0;
          }
      }
      #ifdef DEBUG_PRINT
      Serial.print("Current vert menu selection: ");
      Serial.println(currentVertMenuSelection);
      #endif
    }
  }

  if(encLeftPosition > encLeftOldPosition + 2)
  {
    #ifdef DEBUG_PRINT
    Serial.println("Left knob turned cw");
    Serial.println(encLeftPosition);
    #endif
    encLeftOldPosition = encLeftPosition;

    if(curMillis >= prevMenuMillis + MENU_UPDATE_TIME)
    {
      prevMenuMillis = curMillis;
      if(page == 2 || page == 3) // date or time entry
      {
        currentHoriMenuSelection++;
        if(page == 2) // date entry
        {
          if(currentHoriMenuSelection > 2) currentHoriMenuSelection = 2;

          if(currentHoriMenuSelection == 0) // month
          {
            currentVertMenuSelection = manualMonth - 1;
          }
          else if(currentHoriMenuSelection == 1) // day
          {
            currentVertMenuSelection = manualDay - 1;
          }
          else if(currentHoriMenuSelection == 2) // year
          {
            currentVertMenuSelection = manualYear;
          }
        }
        else if(page == 3)
        {
          if(currentHoriMenuSelection > 1) currentHoriMenuSelection = 1;

          if(currentHoriMenuSelection == 0) // hour
          {
            currentVertMenuSelection = manualHour;
          }
          else if(currentHoriMenuSelection == 1) // minute
          {
            currentVertMenuSelection = manualMinute;
          }
        }
      }
      #ifdef DEBUG_PRINT
      Serial.print("Current hori menu selection: ");
      Serial.println(currentHoriMenuSelection);
      #endif
    }
  }
  else if(encLeftPosition < encLeftOldPosition - 2)
  {
    #ifdef DEBUG_PRINT
    Serial.println("Left knob turned ccw");
    Serial.println(encLeftPosition);
    #endif
    encLeftOldPosition = encLeftPosition;

    if(curMillis >= prevMenuMillis + MENU_UPDATE_TIME)
    {
      currentHoriMenuSelection--;
      if(page == 2) // date entry
      {
        if(currentHoriMenuSelection < 0) currentHoriMenuSelection = 0;

        if(currentHoriMenuSelection == 0) // month
        {
          currentVertMenuSelection = manualMonth - 1;
        }
        else if(currentHoriMenuSelection == 1) // day
        {
          currentVertMenuSelection = manualDay - 1;
        }
        else if(currentHoriMenuSelection == 2) // year
        {
          currentVertMenuSelection = manualYear;
        }
      }
      else if(page == 3) // time entry
      {
        if(currentHoriMenuSelection < 0) currentHoriMenuSelection = 0;

        if(currentHoriMenuSelection == 0) // hour
        {
          currentVertMenuSelection = manualHour;
        }
        else if(currentHoriMenuSelection == 1) // minute
        {
          currentVertMenuSelection = manualMinute;
        }
      }
      #ifdef DEBUG_PRINT
      Serial.print("Current hori menu selection: ");
      Serial.println(currentHoriMenuSelection);
      #endif
    }
  }
}

// void updateMenuSelection() {
//     long encRightPosition = encRight.getPosition();
//     long encLeftPosition = encLeft.getPosition();

//     // Right Encoder Logic
//     if (encRightPosition > encRightOldPosition + 2) { // Clockwise rotation
//         encRightOldPosition = encRightPosition;

//         if (curMillis >= prevMenuMillis + MENU_UPDATE_TIME) {
//             prevMenuMillis = curMillis;
//             prevVertMenuSelection = currentVertMenuSelection;
//             currentVertMenuSelection++;

//             switch (page) {
//                 case 0: case 1: // Initial menus
//                     if (currentVertMenuSelection > 1) currentVertMenuSelection = 1;
//                     break;
//                 case 2: // Date entry
//                       if (currentHoriMenuSelection == 0) { // Day
//                         switch (manualMonth) { // Validate based on the current month
//                           case 4: case 6: case 9: case 11: // Months with 30 days
//                             if (currentVertMenuSelection > 29) currentVertMenuSelection = 0; // Reset if out of range
//                               break;
//                           case 1: case 3: case 5: case 7: case 8: case 10: case 12: // Months with 31 days
//                               if (currentVertMenuSelection > 30) currentVertMenuSelection = 0; // Reset if out of range
//                               break;
//                           case 2: // February
//                               if (manualYear % 4 == 0) { // Leap year
//                                   if (currentVertMenuSelection > 28) currentVertMenuSelection = 0;
//                               } else { // Non-leap year
//                                   if (currentVertMenuSelection > 27) currentVertMenuSelection = 0;
//                               }
//                               break;
//                       }
//                       manualDay = currentVertMenuSelection + 1; // Set the day (1-based index)
//                   } else if (currentHoriMenuSelection == 1) { // Month
//                       if (currentVertMenuSelection > 11) currentVertMenuSelection = 0; // Reset if out of range
//                       manualMonth = currentVertMenuSelection + 1; // Set the month (1-based index)
//                   } else if (currentHoriMenuSelection == 2) { // Year
//                       if (currentVertMenuSelection > 2099) currentVertMenuSelection = CUR_YEAR; // Upper limit
//                       if (currentVertMenuSelection < CUR_YEAR) currentVertMenuSelection = CUR_YEAR; // Lower limit
//                       manualYear = currentVertMenuSelection; // Set the year
//                   }
//                   break;

//                     // if (currentHoriMenuSelection == 0) { // Month
//                     //     if (currentVertMenuSelection > 11) currentVertMenuSelection = 0;
//                     //     manualMonth = currentVertMenuSelection + 1;
//                     // } else if (currentHoriMenuSelection == 1) { // Day
//                     //     switch (manualMonth) {
//                     //         case 4: case 6: case 9: case 11:
//                     //             if (currentVertMenuSelection > 29) currentVertMenuSelection = 0;
//                     //             break;
//                     //         case 1: case 3: case 5: case 7: case 8: case 10: case 12:
//                     //             if (currentVertMenuSelection > 30) currentVertMenuSelection = 0;
//                     //             break;
//                     //         case 2:
//                     //             if (manualYear % 4 == 0) {
//                     //                 if (currentVertMenuSelection > 28) currentVertMenuSelection = 0;
//                     //             } else {
//                     //                 if (currentVertMenuSelection > 27) currentVertMenuSelection = 0;
//                     //             }
//                     //             break;
//                     //     }
//                     //     manualDay = currentVertMenuSelection + 1;
//                     // } else if (currentHoriMenuSelection == 2) { // Year
//                     //     if (currentVertMenuSelection > 2099) currentVertMenuSelection = CUR_YEAR;
//                     //     if (currentVertMenuSelection < CUR_YEAR) currentVertMenuSelection = CUR_YEAR;
//                     //     manualYear = currentVertMenuSelection;
//                     // }
//                     // break;
//                 case 3: // Time entry
//                     if (currentHoriMenuSelection == 0) { // Hour
//                         if (currentVertMenuSelection > 23) currentVertMenuSelection = 0;
//                         manualHour = currentVertMenuSelection;
//                     } else if (currentHoriMenuSelection == 1) { // Minute
//                         if (currentVertMenuSelection > 59) {
//                             currentVertMenuSelection = 0;
//                             manualHour = (manualHour + 1) % 24;
//                         }
//                         manualMinute = currentVertMenuSelection;
//                     }
//                     break;
//                 case 4: // File selection
//                     if (currentVertMenuSelection > fileCount - 1) currentVertMenuSelection = fileCount - 1;
//                     if (currentVertMenuSelection % 12 == 0 && currentVertMenuSelection != 0 && currentVertMenuSelection != prevVertMenuSelection) {
//                         scroll++;
//                     }
//                     break;
//             }
//         }
//     } else if (encRightPosition < encRightOldPosition - 2) { // Counterclockwise rotation
//         encRightOldPosition = encRightPosition;

//         if (curMillis >= prevMenuMillis + MENU_UPDATE_TIME) {
//             prevMenuMillis = curMillis;
//             prevVertMenuSelection = currentVertMenuSelection;
//             currentVertMenuSelection--;

//             switch (page) {
//                 case 0: case 1:
//                     if (currentVertMenuSelection < 0) currentVertMenuSelection = 0;
//                     break;
//                 case 2: // Date entry
//                     if (currentHoriMenuSelection == 0) { // Day
//                       if (currentVertMenuSelection < 0) {
//                           switch (manualMonth) { // Validate and wrap around based on the current month
//                               case 4: case 6: case 9: case 11: // Months with 30 days
//                                   currentVertMenuSelection = 29; // Wrap to the last valid day
//                                   break;
//                               case 1: case 3: case 5: case 7: case 8: case 10: case 12: // Months with 31 days
//                                   currentVertMenuSelection = 30; // Wrap to the last valid day
//                                   break;
//                               case 2: // February
//                                   currentVertMenuSelection = (manualYear % 4 == 0) ? 28 : 27; // Leap year or not
//                                   break;
//                           }
//                       }
//                       manualDay = currentVertMenuSelection + 1; // Set the day (1-based index)
//                   } else if (currentHoriMenuSelection == 1) { // Month
//                       if (currentVertMenuSelection < 0) currentVertMenuSelection = 11; // Wrap to December
//                       manualMonth = currentVertMenuSelection + 1; // Set the month (1-based index)
//                   } else if (currentHoriMenuSelection == 2) { // Year
//                       if (currentVertMenuSelection < CUR_YEAR) currentVertMenuSelection = CUR_YEAR; // Ensure no year below the current year
//                       manualYear = currentVertMenuSelection; // Set the year
//                   }
//                   break;
                  
//                     // if (currentHoriMenuSelection == 0) { // Month
//                     //     if (currentVertMenuSelection < 0) currentVertMenuSelection = 11;
//                     //     manualMonth = currentVertMenuSelection + 1;
//                     // } else if (currentHoriMenuSelection == 1) { // Day
//                     //     if (currentVertMenuSelection < 0) {
//                     //         switch (manualMonth) {
//                     //             case 4: case 6: case 9: case 11:
//                     //                 currentVertMenuSelection = 29;
//                     //                 break;
//                     //             case 1: case 3: case 5: case 7: case 8: case 10: case 12:
//                     //                 currentVertMenuSelection = 30;
//                     //                 break;
//                     //             case 2:
//                     //                 currentVertMenuSelection = (manualYear % 4 == 0) ? 28 : 27;
//                     //                 break;
//                     //         }
//                     //     }
//                     //     manualDay = currentVertMenuSelection + 1;
//                     // } else if (currentHoriMenuSelection == 2) { // Year
//                     //     if (currentVertMenuSelection < CUR_YEAR) currentVertMenuSelection = CUR_YEAR;
//                     //     manualYear = currentVertMenuSelection;
//                     // }
//                     // break;
//                 case 3: // Time entry
//                     if (currentHoriMenuSelection == 0) { // Hour
//                         if (currentVertMenuSelection < 0) currentVertMenuSelection = 23;
//                         manualHour = currentVertMenuSelection;
//                     } else if (currentHoriMenuSelection == 1) { // Minute
//                         if (currentVertMenuSelection < 0) {
//                             currentVertMenuSelection = 59;
//                             manualHour = (manualHour > 0) ? manualHour - 1 : 23;
//                         }
//                         manualMinute = currentVertMenuSelection;
//                     }
//                     break;
//                 case 4:
//                     if (currentVertMenuSelection < 0) currentVertMenuSelection = 0;
//                     if (currentVertMenuSelection % 12 == 11) scroll--;
//                     if (scroll < 0) scroll = 0;
//                     break;
//             }
//         }
//     }

//     // Left Encoder Logic
//     if (encLeftPosition > encLeftOldPosition + 2) { // Clockwise rotation
//         encLeftOldPosition = encLeftPosition;

//         if (curMillis >= prevMenuMillis + MENU_UPDATE_TIME) {
//             prevMenuMillis = curMillis;

//             if (page == 2 || page == 3) { // Date or Time entry
//                 currentHoriMenuSelection++;
//                 if (page == 2 && currentHoriMenuSelection > 2) currentHoriMenuSelection = 2;
//                 if (page == 3 && currentHoriMenuSelection > 1) currentHoriMenuSelection = 1;
//             }
//         }
//     } else if (encLeftPosition < encLeftOldPosition - 2) { // Counterclockwise rotation
//         encLeftOldPosition = encLeftPosition;

//         if (curMillis >= prevMenuMillis + MENU_UPDATE_TIME) {
//             prevMenuMillis = curMillis;

//             currentHoriMenuSelection--;
//             if (currentHoriMenuSelection < 0) currentHoriMenuSelection = 0;
//         }
//     }
// }

// function for displaying various pages/menus
void displayPage(uint8_t page)
{
  // On all pages add "select" and "back" indicators on the bottom of the screen
  // On data collection page, only show "back"
  display.clearDisplay(LCD_BACKGROUND);
  // display.drawLine(0, display.height()-10, display.width()-1, display.height()-10, LCD_FOREGROUND);
  // display.drawLine(display.width()/2 - 1, display.height()-10, display.width()/2 - 1, display.height()-1, LCD_FOREGROUND);
  display.drawLine(-30, display.height() -12, display.width() -3, display.height() -12, LCD_FOREGROUND);
  display.drawLine((display.width()/2) -1, display.height() -10, (display.width()/2) -1, display.height() -1, LCD_FOREGROUND);
  // display.drawLine((2*display.width()/3)-1, display.height()-10, (2*display.width()/3)-1, display.height()-1, LCD_FOREGROUND);
  display.setTextColor(LCD_FOREGROUND);
  // display.setCursor(10, display.height()-8); // For 1.8 TFT
  display.setCursor(10, display.height() -10); //For 1.44 TFT
  display.print("Back ");
  if(page == 2 || page == 3) // only the date and time page uses the left knob for left-right
  {
    display.cp437(true);
    display.print("\x11\x10");
    display.cp437(false);
  }
  if (page != 5) // no select button on data collection screen
  {
    // display.setCursor((display.width()/2) + 5, display.height()-8);  //1.8
    display.setCursor((display.width()/3) + 25, display.height()-10); //1.44
    display.cp437(true);
    display.print("\x1e\x1f");
    display.cp437(false);
    display.print(" Select");
  }

  switch(page)
  {
    case(0): // Initial menu
    {
      if (currentVertMenuSelection == 0)
      {
        // updateDisplay("Start data collection\n", 0, true);
        // updateDisplay("Upload data", 8, false);
        updateDisplay("Start data collection\n", 32, true);
        //updateDisplay("Upload data", 40, false);
      }
      else if (currentVertMenuSelection == 1)
      {
        // updateDisplay("Start data collection\n", 0, false);
        // updateDisplay("Upload data", 8, true);
        updateDisplay("Start data collection\n", 32, false);
        //updateDisplay("Upload data", 40, true);
      }
      break;
    }
    case(1): // Time entry method menu
    {
      // display.drawLine(0, 10, display.width()-1, 10, LCD_FOREGROUND);
      // updateDisplay("Timestamp method?", 0, false);
      display.drawLine(0, 42, display.width()-1, 42, LCD_FOREGROUND);
      updateDisplay("Timestamp method?", 32, false);
      if (currentVertMenuSelection == 0)        
      {
        updateDisplay("Auto (GPS)\n", 44, true);        //12
        updateDisplay("Manual Entry", 52, false);             //52
        // updateDisplay("Auto (GPS)\n", 12, true);
        // updateDisplay("Manual", 20, false); 
      }
      else if (currentVertMenuSelection == 1)
      {
        updateDisplay("Auto (GPS)\n", 44, false);      //12
        updateDisplay("Manual Entry", 52, true);     //52
        // updateDisplay("Auto (GPS)\n", 12, false);      //12
        // updateDisplay("Manual", 20, true);
      }
      break;
    }
    case(2): // Date entry
    {
      // display.drawLine(0, 10, display.width()-1, 10, LCD_FOREGROUND);
      // updateDisplay("Enter date", 0, false);
      display.drawLine(0, 42, display.width()-1, 42, LCD_FOREGROUND);
      updateDisplay("Enter date", 30, false);    // 0

      char displayMonth[3];
      char displayDay[3];
      char displayYear[5];

      itoa(manualMonth, displayMonth, 10);
      itoa(manualDay, displayDay, 10);
      itoa(manualYear, displayYear, 10);

      display.setTextSize(2);
      display.setCursor(0, 56);
      // display.setTextSize(1);
      // display.setCursor(0, 10);

      if(currentHoriMenuSelection == 0) display.setTextColor(LCD_BACKGROUND, LCD_FOREGROUND);
      if(manualMonth < 10) display.print('0');
      display.print(manualMonth);

      display.setTextColor(LCD_FOREGROUND);
      display.print('/');

      if(currentHoriMenuSelection == 1) display.setTextColor(LCD_BACKGROUND, LCD_FOREGROUND);
      if(manualDay < 10) display.print('0');
      display.print(manualDay);

      display.setTextColor(LCD_FOREGROUND);
      display.print('/');

      if(currentHoriMenuSelection == 2) display.setTextColor(LCD_BACKGROUND, LCD_FOREGROUND);
      display.print(manualYear);

      display.setTextColor(LCD_FOREGROUND);
      display.setTextSize(1);
      break;
    }

//     case (2): // Date entry
// {
//     display.drawLine(0, 42, display.width() - 1, 42, LCD_FOREGROUND);
//     updateDisplay("Enter date", 30, false);

//     char displayDay[3];
//     char displayMonth[3];
//     char displayYear[5];

//     itoa(manualDay, displayDay, 10);
//     itoa(manualMonth, displayMonth, 10);
//     itoa(manualYear, displayYear, 10);

//     display.setTextSize(2);
//     display.setCursor(0, 56);

//     if (currentHoriMenuSelection == 0) 
//         display.setTextColor(LCD_BACKGROUND, LCD_FOREGROUND);
//     if (manualDay < 10) 
//         display.print('0');
//     display.print(manualDay);

//     display.setTextColor(LCD_FOREGROUND);
//     display.print('/');

//     if (currentHoriMenuSelection == 1) 
//         display.setTextColor(LCD_BACKGROUND, LCD_FOREGROUND);
//     if (manualMonth < 10) 
//         display.print('0');
//     display.print(manualMonth);

//     display.setTextColor(LCD_FOREGROUND);
//     display.print('/');

//     if (currentHoriMenuSelection == 2) 
//         display.setTextColor(LCD_BACKGROUND, LCD_FOREGROUND);
//     display.print(manualYear);

//     display.setTextColor(LCD_FOREGROUND);
//     display.setTextSize(1);
//     break;
// }
    case(3): // Time entry
    {
      // display.drawLine(0, 10, display.width()-1, 10, LCD_FOREGROUND);
      // updateDisplay("Enter time (UTC)", 0, false);  
      display.drawLine(0, 42, display.width()-1, 42, LCD_FOREGROUND);
      updateDisplay("Enter time", 32, false);    
      char displayHour[3];
      char displayMinute[3];

      itoa(manualHour, displayHour, 10);
      itoa(manualMinute, displayMinute, 10);

      display.setTextSize(2);
      display.setCursor(0, 56);
      // display.setTextSize(1);
      // display.setCursor(0, 10);

      if(currentHoriMenuSelection == 0) display.setTextColor(LCD_BACKGROUND, LCD_FOREGROUND);
      if(manualHour < 10) display.print('0');
      display.print(manualHour);

      display.setTextColor(LCD_FOREGROUND);
      display.print(':');

      if(currentHoriMenuSelection == 1) display.setTextColor(LCD_BACKGROUND, LCD_FOREGROUND);
      if(manualMinute < 10) display.print('0');
      display.print(manualMinute);

      display.setTextColor(LCD_FOREGROUND);
      display.setTextSize(1);

      break;
    }
    case(4): // Viewing list of files on SD card
    {
      char allFiles[fileCount][30];
      memcpy(allFiles, fileList, sizeof(allFiles));

      char screenFiles[12][30]; // files being displayed on screen

      // #ifdef DEBUG_PRINT
      // Serial.println("List of files on SD found in displayPage:");
      // for(int i = 0; i < fileCount; ++i)
      // {
      //   Serial.println(allFiles[i]);
      // }
      // Serial.println();
      // #endif

      // display.drawLine(0, 10, display.width()-1, 10, LCD_FOREGROUND);
      // updateDisplay("Select a file", 0, false);  
      display.drawLine(0, 42, display.width()-1, 42, LCD_FOREGROUND);
      updateDisplay("Select a file", 32, false);  

      uint8_t numFilesToDisplay = 0;

      // copy next 12 names from allFiles into screenFiles, or fewer if there are fewer than 12 left
      if(fileCount - scroll*12 >= 12)
      {
        memcpy(screenFiles, allFiles[scroll*12], sizeof(screenFiles));
        numFilesToDisplay = 12;
      }
      else if(fileCount - scroll*12 < 12)
      {
        numFilesToDisplay = fileCount - scroll*12;
        memcpy(screenFiles, allFiles[scroll*12], numFilesToDisplay*30); // 30 bytes per file name
      }

      for(uint8_t i = 0; i < numFilesToDisplay; i++)
      {
        if(scroll == 0)
        {
          if(currentVertMenuSelection == i) updateDisplay(screenFiles[i], 12 + i*8, true);
          else updateDisplay(screenFiles[i], 12 + i*8, false);
        }
        else
        {
          if(currentVertMenuSelection == i + 12*scroll) updateDisplay(screenFiles[i], 12 + i*8, true);
          else updateDisplay(screenFiles[i], 12 + i*8, false);
        }          
        
      }
      break;
    }
    case(5): // Data collection
    {
      // uint16_t PM1p0_std = dustSensor.data.PM1p0_std;
      // uint16_t PM2p5_std = dustSensor.data.PM2p5_std;
      uint16_t PM2p5_std = pms.pm25;   // COncentration of PM2.5
      // uint16_t PM10p0_std = dustSensor.data.PM10p0_std;
      // uint16_t PM1p0_atm = dustSensor.data.PM1p0_atm;
      // uint16_t PM2p5_atm = dustSensor.data.PM2p5_atm;
      // uint16_t PM10p0_atm = dustSensor.data.PM10p0_atm;
      //uint16_t count_0p3um = dustSensor.data.count_0p3um;
      // uint16_t count_0p3um = pms.n0p3; // Number of particles with diameter > 0.3µm
      // uint16_t count_0p5um = dustSensor.data.count_0p5um;
      // uint16_t count_1p0um = dustSensor.data.count_1p0um;
      // uint16_t count_2p5um = dustSensor.data.count_2p5um;
      // uint16_t count_5p0um = dustSensor.data.count_5p0um;
      // uint16_t count_10p0um = dustSensor.data.count_10p0um;

      char timeText[50];
      // char pm1p0Text[10];
      // char pm2p5Text[10];
      // char pm10p0Text[10];
      char hourText[10];
      char minuteText[10];
      char monthText[10];
      char dayText[10];
      char yearText[10];

      if (!gpsAwake) {
        if (timeStatus() == timeNotSet) {
            setTime(manualHour, manualMinute, manualsecond, manualDay, manualMonth, manualYear);
            timeSetOnce = true;  // User has manually set the time
        }
        // Use TimeLib's time functions to get time components
        utcYear = year();       // Returns the year (e.g., 2024)
        utcMonth = month();     // Returns the month (1 = January, 12 = December)
        utcDay = day();         // Returns the day of the month
        utcHour = hour();       // Returns the current hour (0-23)
        utcMinute = minute();   // Returns the current minute (0-59)
        utcSecond = second();    // Returns the current second (0-59)

        // itoa(hour(), hourText, 10);
        // itoa(minute(), minuteText, 10);
        // itoa(month(), monthText, 10);
        // itoa(day(), dayText, 10);
        // itoa(year(), yearText, 10);
      }
      else {

        utcYear = gps.date.year();       // Returns the year (e.g., 2024)
        utcMonth = gps.date.month();     // Returns the month (1 = January, 12 = December)
        utcDay = gps.date.day();         // Returns the day of the month
        utcHour = gps.time.hour();       // Returns the current hour (0-23)
        utcMinute = gps.time.minute();   // Returns the current minute (0-59)
        // utcSecond = second();    // Returns the current second (0-59)

        // itoa(gps.time.hour(), hourText, 10);
        // itoa(gps.time.minute(), minuteText, 10);
        // itoa(gps.date.month(), monthText, 10);
        // itoa(gps.date.day(), dayText, 10);
        // itoa(gps.date.year(), yearText, 10);
      }

      itoa(utcHour, hourText, 10);
      itoa(utcMinute, minuteText, 10);
      itoa(utcMonth, monthText, 10);
      itoa(utcDay, dayText, 10);
      itoa(utcYear, yearText, 10);

      display.setTextSize(2);   //2
      // display.setTextSize(2);
      display.setCursor(5, 32); //52
      // display.setCursor(0, 20);  
      // display.print(">0.3um:");
      display.print("PM2.5:");
      display.setCursor(5, 76);   //48
      display.setTextSize(3);
      // display.setCursor(0, 48);
      // display.print(count_0p3um);
      display.print(PM2p5_std);
      display.setTextSize(1);
      display.setCursor(85, 86);   //62
      // display.setCursor(0, 62);
      // display.print("count/0.1L");
      display.print("ug/m3");
      display.setTextSize(1);

      strcpy(timeText, dayText);
      strcat(timeText, "/");
      strcat(timeText, monthText);
      strcat(timeText, "/");
      strcat(timeText, yearText);
      strcat(timeText, " ");
      
      if(utcHour < 10)
          strcat(timeText, "0");
      strcat(timeText, hourText);
      strcat(timeText, ":");
      
    if(utcMinute < 10)
        strcat(timeText, "0");
    strcat(timeText, minuteText);
      
      if(gpsDisplayFail || manualTimeEntry) strcat(timeText, " ");
      else strcat(timeText, "(GPS)");
    display.setCursor(5, 118);
    display.print(timeText);
    // updateDisplay(timeText, 118, false);
      // updateDisplay(timeText, -20, false);
      break;
    }
  }
  display.display();
}

// function for displaying characters to OLED 
void updateDisplay(char* text, uint8_t height, bool bg)
{
  // if(clear) display.clearDisplay(LCD_BACKGROUND);

  display.setTextSize(1);

  if(bg) 
  {
    display.setTextColor(LCD_BACKGROUND, LCD_FOREGROUND);
  }
  else
  { 
    display.setTextColor(LCD_FOREGROUND);
  }
  display.setCursor(0, height);

  display.print(text);

  // if(send) display.display();

}

int cmpstr(void const *a, void const *b)
{
  char const *aa = (char const *)a;
  char const *bb = (char const *)b;

  return -1*strcmp(aa, bb);
}

void getFileList()
{
  // Serial.println("getFileList");
  // Free fileList if it's malloc'd
  // (this should never be the case, but just to be sure)
  if (fileListAlloc)
  {
    // Serial.println("Deallocating files list");
    free(fileList);
    fileListAlloc = false;
  }

// First count files on SD card
#ifdef DEBUG_PRINT
  Serial.println("\nCounting files on SD card");
#endif

File root = SD.open("/");  // Open root directory
fileCount = 0;
//File file;

if (!root) {
    #ifdef DEBUG_PRINT
      Serial.println("Error opening root");
    #endif
    // TODO: figure out how to handle this error
} else {
    File file = root.openNextFile();  // Open the first file in the directory
    
    while (file) {
        if (!file.isDirectory()) {  // Only count regular files
            fileCount++;
            
            #ifdef DEBUG_PRINT
            Serial.print(file.name());
            Serial.print('\t');
            Serial.println(fileCount);
            #endif
        }
        file = root.openNextFile();  // Move to the next file
    }
    root.close();  // Close the root directory
}

  #ifdef DEBUG_PRINT
  Serial.print("\nFile count: ");
  Serial.println(fileCount);
  #endif

  // Now create an array of file names
char filesOnSd[fileCount][30]; // Each file name should be at most 30 characters long
uint32_t curFile = 0;

// Open root directory
//File root = SD.open("/");
if (!root) {
    #ifdef DEBUG_PRINT
    Serial.println("Error opening root directory");
    #endif
    return;  // Exit if the root directory cannot be opened
}

// Loop through files in the root directory
File file = root.openNextFile();
while (file) {
    if (!file.isDirectory()) {  // Only process regular files, not directories
        char fileName[30] = {0};
        strncpy(fileName, file.name(), sizeof(fileName) - 1);  // Copy file name to fileName array

        // Remove the file extension (.txt) to fit on screen
        char shortenedFileName[30] = {0};
        strncpy(shortenedFileName, fileName, strlen(fileName) - 4); // Remove ".txt" (4 characters)
        strcpy(filesOnSd[curFile], shortenedFileName);  // Copy shortened name to filesOnSd array
        curFile++;
    }
    file = root.openNextFile();  // Move to the next file
}
root.close();  // Close root directory

#ifdef DEBUG_PRINT
Serial.println("\nList of files created (pre-sort):");
for (int i = 0; i < fileCount; ++i) {
    Serial.println(filesOnSd[i]);
}
#endif

// Sort filesOnSd alphabetically
qsort(filesOnSd, fileCount, sizeof(filesOnSd[0]), cmpstr);

#ifdef DEBUG_PRINT
Serial.println("\nList of files created (post-sort):");
for (int i = 0; i < fileCount; ++i) {
    Serial.println(filesOnSd[i]);
}
#endif

  // copy contents fo fileList memory location
  // free()'d when the upload menu is left with the back button
  // free()'d when file list is sent over serial
  fileList = (char *)malloc(sizeof(filesOnSd));
  memcpy(fileList, filesOnSd, sizeof(filesOnSd));
  fileListAlloc = true;
}