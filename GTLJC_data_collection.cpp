// ALL THANKS AND GLORY TO THE AND my ONLY GOD AND LORD JESUS CHRIST ALONE
///////////////The Lord's Grace and Mercy//////////////////

#include <IRremote.hpp> // include the library
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include "WiFi.h"
#include "HTTPClient.h"
#include "ArduinoJson.h"
#include <WiFiClientSecure.h>  
#include <LiquidCrystal_I2C.h>
#include <esp_system.h>

String ssid = "";
const char* password = "";
const char* GTLJC_host = "roadanomaly4christalone-d0b8esbucpenbdd7.canadacentral-01.azurewebsites.net";
const int GTLJC_httpsPort = 443;
const char* GTLJC_path_inference = "/api-road-inference-logs/road_anomaly_infer/";
const char* GTLJC_path_predictions = "/api-road-prediction-output/road_anomaly_predict/";
const char* GTLJC_path_verification = "/api-road-verification/road_anomaly_verify/";
const char* GTLJC_path_manual_data = "/api-road-manual-data-collection/road_anomaly_collect_data/";

const char* API_PREDICTION_OUTPUT = "https://roadanomaly4christalone-d0b8esbucpenbdd7.canadacentral-01.azurewebsites.net/api-road-prediction-output/road_anomaly_predict/";
const char* API_VERIFICATION = "https://roadanomaly4christalone-d0b8esbucpenbdd7.canadacentral-01.azurewebsites.net/api-road-verification/road_anomaly_verify/";
const char* API_INFERENCE = "https://roadanomaly4christalone-d0b8esbucpenbdd7.canadacentral-01.azurewebsites.net/api-road-inference-logs/road_anomaly_infer/";
const char* API_INFERENCE_NON_JSON = "https://roadanomaly4christalone-d0b8esbucpenbdd7.canadacentral-01.azurewebsites.net/api-road-inference-logs-raw/road_anomaly_inference_raw/";

const char* API_REGISTERED_ANOMALIES = "https://roadanomaly4christalone-d0b8esbucpenbdd7.canadacentral-01.azurewebsites.net/api-road-in/road_anomaly_in/";

#define IR_RECEIVE_PIN 27
#define IR_SEND_PIN  15
static const int RXPin = 16, TXPin = 17;
static const uint32_t GPSBaud = 9600;

// The TinyGPSPlus object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

// Creating an instance of the LCD_I2C Interface Object
LiquidCrystal_I2C lcd(0x27,20,4); 


String GTLJC_batch_readings = "";
String GTLJC_batch_readings_json_send = "";

String GTLJC_batch_results[2];

int GTLJC_vibration_sensor_input = 27;
int GTLJC_vibration;

int GTLJC_sample_count = 0;
int GTLJC_command = 100;
bool GTLJC_command_given = false;
unsigned long GTLJC_lastCommandTime = 0;
const unsigned long GTLJC_debounceDelay = 300;

int GTLJC_database_transfer_pin = 26;
int GTLJC_label_provided_led = 25;
unsigned long GTLJC_last_interval_ms = 0;

unsigned long GTLJC_timestamp_prev = 0;
unsigned long GTLJC_batch = 0;
long GTLJC_time_to_repeat = 0;

unsigned long GTLJC_interval_of_collection = 60000 * 1; // Graciously defining a 1-minute interval for data collection
unsigned long GTLJC_send_data_period = 60000 * 0.5 ; // A 1-minute space allowed for data transfer


bool GTLJC_predictionsReceived = false;
bool GTLJC_collecting_data = false;

int GTLJC_arr_of_commands[] = {68,64,67,21,25,24,28,82};
bool backend_connection_established = false;
bool verification_process_started = false;
int GTLJC_total_arr_of_commands[] = {69,70,71,68,64,67,21,25,24,28,82,8,12,90,66,74,100};
// String command_names[] = {
//   "resetting system    ",
//   "erasing SD card     ",
//   "sending lbld data   ",
//   "no-movement         ",
//   "smooth road         ",
//   "static-vibration    ",
//   "cracks              ",
//   "bumps               ",
//   "road patch          ",
//   "mild pothole        ",
//   "severe pothole      ",
//   "sending inf data    ",
//   "start data preproc  ",
//   "getting predictions ",
//   "accept predictions  ",
//   "reject predictions  ",
//   "idle                "

// };
String command_names[] = {
  "                    ",
  "                    ",
  "                    ",
  "no-movement         ",
  "smooth road         ",
  "static-vibration    ",
  "cracks              ",
  "bumps               ",
  "road patch          ",
  "mild pothole        ",
  "severe pothole      ",
  "                    ",
  "                    ",
  "                    ",
  "                    ",
  "                    ",
  "idle                "

};


String get_anomaly_name(int GTLJC_command){
  for (int GTLJC_i = 0; GTLJC_i < 17; GTLJC_i++){
    if (GTLJC_command == GTLJC_total_arr_of_commands[GTLJC_i]){
      return command_names[GTLJC_i];
    }
  }
  return ""; 
}

bool logs_checked_at_SD_CARD_wipe = false;

bool commandIsNotRecognized( int& GTLJC_command){
  bool commandNotRecognized = true;
  for (int GTLJC_i = 0; GTLJC_i < 17; GTLJC_i++){
    if (GTLJC_command == GTLJC_total_arr_of_commands[GTLJC_i]){
      commandNotRecognized = false;
    }
  }
  return commandNotRecognized;
}

void commandsToUse(){
  
      delay(3000);
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Anomalies In View:  ");
      lcd.setCursor(0,1);
      lcd.print("no-movement         ");
      lcd.setCursor(0,2);
      lcd.print("smooth road         ");
      lcd.setCursor(0,3);
      lcd.print("static-vibration    ");

      delay(3000);
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Anomalies In View:  ");
      lcd.setCursor(0,1);
      lcd.print("cracks              ");
      lcd.setCursor(0,2);
      lcd.print("bumps               ");
      lcd.setCursor(0,3);
      lcd.print("road patch          ");

      delay(3000);
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Anomalies In View:  ");
      lcd.setCursor(0,1);
      lcd.print("mild pothole        ");
      lcd.setCursor(0,2);
      lcd.print("severe pothole      ");
   

      delay(3000);
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Important commands: ");
      lcd.setCursor(0,1);
      lcd.print("Btn 6 :get pred     ");
      lcd.setCursor(0,2);
      lcd.print("Btn 4 :acq inf data ");
    

      delay(3000);
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Important commands: ");
      lcd.setCursor(0,1);
      lcd.print("Btn CH+ :acquire....");
      lcd.setCursor(0,2);
      lcd.print("   ...labeled data  ");
      lcd.setCursor(0,3);
      lcd.print("Btn CH- :sys reset  ");

      
      delay(3000);
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Important commands:   ");
      lcd.setCursor(0,1);
      lcd.print("Btn 1 :start data...  ");
      lcd.setCursor(0,2);
      lcd.print("   ...preprocessing");
      lcd.setCursor(0,3);
      lcd.print("Btn CH :empty SDcard");

      delay(3000);
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Important commands: ");
      lcd.setCursor(0,1);
      lcd.print("Btn 9 :accept pred  ");
      lcd.setCursor(0,2);
      lcd.print("Btn 7 :reject pred  ");

}



void GTLJC_eraseScreen(){
      lcd.setCursor(0,0);
      lcd.print("                    ");
      lcd.setCursor(0,1);
      lcd.print("                    ");
      lcd.setCursor(0,2);
      lcd.print("                    ");
      lcd.setCursor(0,3);
      lcd.print("                    ");
}


Adafruit_MPU6050 mpu;
void setup()
{
      lcd.init();    
      lcd.backlight();
      lcd.setCursor(0,0);
      lcd.print("   Road Anomaly");
      lcd.setCursor(0,1);
      lcd.print("   Detection And");
      lcd.setCursor(0,2);
      lcd.print(" Information System");
      delay(5000);
      lcd.clear();
      for (int GTLJC_i = 0; GTLJC_i < 2; GTLJC_i++){
        commandsToUse();
      }

      Serial.begin(115200);
      ss.begin(GPSBaud);
      pinMode( GTLJC_vibration_sensor_input , INPUT);
      pinMode(GTLJC_database_transfer_pin , OUTPUT);
      
      //digitalWrite(GTLJC_System_Active_Led, HIGH);

      pinMode(GTLJC_label_provided_led , OUTPUT);
      IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);

      if(!mpu.begin()){
        Serial.println("GLORY TO GOD ALONE, Failed to find mpu chip");
        lcd.setCursor(0,0);
        lcd.print("Failed to find mpu chip");
        while(1)
          delay(10);
      }

      Serial.println("MPU6050 GRACIOUSLY Found");
      mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
      Serial.print("Accelerometer range set to: ");
      Serial.println("+-8G");
      mpu.setGyroRange(MPU6050_RANGE_500_DEG);
      Serial.print("Gyro range set to: ");
      Serial.println("+- 500 deg/s");
      mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
      Serial.print("Filter bandwidth set to: ");
      Serial.println("21 Hz");

      if (!SD.begin()) {
            Serial.println("Card Mount Failed");
            lcd.clear();
            lcd.setCursor(0,0);
            lcd.print("Card Mount Failed");

            return;
      }
      uint8_t cardType = SD.cardType();

      if (cardType == CARD_NONE) {
        Serial.println("No SD card attached");
            lcd.clear();
            lcd.setCursor(0,0);
            lcd.print("No SD card attached");
        return;
      }

      Serial.print("SD Card Type: ");
      if (cardType == CARD_MMC) {
        Serial.println("MMC");
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("MMC");
      } else if (cardType == CARD_SD) {
        Serial.println("SDSC");
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("SDSC");
      } else if (cardType == CARD_SDHC) {
        Serial.println("SDHC");
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("SDHC");
      } else {
        Serial.println("UNKNOWN");
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("UNKNOWN");
      }

      uint64_t cardSize = SD.cardSize() / (1024 * 1024);
      Serial.printf("SD Card Size: %lluMB\n", cardSize);

      // writeFile(SD, "/GTLJC_data.txt","batch,timestamp/colllection_interval,acc_x ,acc_y,acc_z,rot_x,rot_y ,rot_z,lat,long,GPS_speed_kmph,GPS_speed_mps,GPS_altitude_km,GPS_altitude_m,GPS_data_time,GPS_hdop_acc,GPS_n_of_satellite,anomaly,speed_level_on_encounter\n");
      // readFile(SD, "/GTLJC_data.txt");
      Serial.printf("Total space: %lluMB\n", SD.totalBytes() / (1024 * 1024));
      Serial.printf("Used space: %lluMB\n", SD.usedBytes() / (1024 * 1024));

      WiFi.begin(ssid,password);
      Serial.print("Connecting to WiFi");
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Connecting to WiFi..");
      
      while(WiFi.status() != WL_CONNECTED){
        delay(500);
        Serial.print(".");
      }
      Serial.println("\nConnected to WiFi");
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Connected to: ");
      lcd.setCursor(0,1);
      lcd.print(ssid.substring(0,19));
      lcd.setCursor(0,2);
      lcd.print(ssid.substring(19));
      delay(3000);
      lcd.clear();


}

void GTLJC_parseJsonResponse(const String& jsonString){
  const size_t capacity = 4096;
  DynamicJsonDocument doc(capacity);

  DeserializationError error = deserializeJson(doc, jsonString);
  if(error){
      Serial.print("JSON deserialization failed: ");
      Serial.println(error.c_str());
      return;
  }

  Serial.println("\n--- Parsed JSON Fields ---");

  if (!doc.is<JsonArray>()) {
    Serial.println("Expected JSON Array but got something else.");
    return;
  }

  for (JsonObject obj : doc.as<JsonArray>()) {
    Serial.println("--- Entry ---");
    for (JsonPair kv : obj) {
      Serial.print(kv.key().c_str());
      Serial.print(": ");
      Serial.println(kv.value().as<String>());
    }
    Serial.println("--------------");
  }

}

// Gracious routine for fetching predictions
void GTLJC_fetchJsonData(){

    if((WiFi.status() == WL_CONNECTED)){
          HTTPClient http;
          http.begin(API_PREDICTION_OUTPUT);

          int httpCode = http.GET();

          if (httpCode > 0){
              String payload = http.getString();
              Serial.println("\n--- JSON from GET ---");
              // Serial.println(payload);

              GTLJC_parseJsonResponse(payload);
          }
          else{
            Serial.print("GET failed. HTTP error code: ");
            Serial.println(httpCode);
          }

          http.end();

          GTLJC_command = 100;
          // WiFi.disconnect(true);
          delay(5000);

        
    }
}

String fieldNames[] = {
  "batch_id",
  "acc_x", "acc_y", "acc_z",
  "rot_x", "rot_y", "rot_z",
  "speed",
  "latitude", "longitude", 
  "accuracy","timestamp",
  "log_interval"
};

// Gracious routine for sending Json
String GTLJC_sendJsonBatch(const String& rawBatch, const String& GTLJC_url) {        
          WiFiClientSecure client;
          client.setInsecure(); // ❗ Trusts all certificates — for development/testing only
          // HTTPClient http;

          //Restricting client complete communication to within a timeout of 10 seconds
          client.setTimeout(10000);


          Serial.print("🌐 Connecting to ");
          Serial.println(GTLJC_host);

          if(!client.connect(GTLJC_host, GTLJC_httpsPort)){
            Serial.println("❌ HTTPS connection failed");
            lcd.clear();
            lcd.setCursor(0,0);
            lcd.print("HTTPS connection");
            lcd.setCursor(0,1);
            lcd.print("failed");
            backend_connection_established = true;
            return "";
          }

          // Graciously sending HTTP headers and body
          client.println("POST " + String(GTLJC_url) + " HTTP/1.1");
          client.println("Host: " + String(GTLJC_host));
          client.println("Content-Type: text/plain");
          client.println("Connection: close");
          client.print("Content-Length: ");
          client.println(rawBatch.length());
          client.println();  // End of headers
          
          client.write((const uint8_t *)rawBatch.c_str(), rawBatch.length());
          Serial.print(rawBatch);
          // client.print(rawBatch); // Send raw data
          

          // Graciously reading server response
         Serial.println("📨 Server Response:");
          String response = "";

          unsigned long startTime = millis();   // Track start time
          while (client.connected() || client.available()) {
            if (client.available()) {
              char c = client.read();
              response += c;
              Serial.print(c);
            }
          }
          // ✅ Always close the connection
          client.stop();
          Serial.println("\n✅ HTTPS text POST complete.");
          // lcd.setCursor(0,0);
          // lcd.print("HTTPS text POST complete.");
          // lcd.setCursor(0,1);
          // lcd.print("complete");

          GTLJC_command = 100;
          // WiFi.disconnect(true);
          // delay(2000);

          lcd.setCursor(0,2);
          lcd.print("          ");
          return response;
}

int countLoggedLinesWithoutDisplaying(){
    File GTLJC_dataFile = SD.open("/GTLJC_data.txt", FILE_READ);
  if(!GTLJC_dataFile){
    Serial.println("❌ Failed to open file for counting lines.");
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Failed to open file");
    lcd.setCursor(0,1);
    lcd.print("for counting lines.");
    delay(3000);
    
    return 0;
  }

  int GTLJC_lineCount = 0;
  while(GTLJC_dataFile.available()){
    String GTLJC_line = GTLJC_dataFile.readStringUntil('\n');
    if (GTLJC_line.length() > 0){
      ++GTLJC_lineCount;
    }
  }

  GTLJC_dataFile.close();
  Serial.print("✅ Total number of logged lines: ");
  // lcd.clear();
  // lcd.setCursor(0,0);
  // lcd.print("Total number of");
  // lcd.setCursor(0,1);
  // lcd.print("logged lines:");
  // lcd.setCursor(0,2);
  // lcd.print(GTLJC_lineCount);
  // delay(3000);
  Serial.println(GTLJC_lineCount);
  return GTLJC_lineCount ;
}



int countLoggedLines(){
  File GTLJC_dataFile = SD.open("/GTLJC_data.txt", FILE_READ);
  if(!GTLJC_dataFile){
    Serial.println("❌ Failed to open file for counting lines.");
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Failed to open file");
    lcd.setCursor(0,1);
    lcd.print("for counting lines.");
    delay(3000);
    
    return 0;
  }

  int GTLJC_lineCount = 0;
  while(GTLJC_dataFile.available()){
    String GTLJC_line = GTLJC_dataFile.readStringUntil('\n');
    if (GTLJC_line.length() > 0){
      ++GTLJC_lineCount;
    }
  }

  GTLJC_dataFile.close();
  Serial.print("✅ Total number of logged lines: ");
  if(logs_checked_at_SD_CARD_wipe){
    ;
    logs_checked_at_SD_CARD_wipe = false;
  }
  else{
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Total number of");
      lcd.setCursor(0,1);
      lcd.print("logged lines:");
      lcd.setCursor(0,2);
      lcd.print(GTLJC_lineCount);
      delay(3000);
  }

  Serial.println(GTLJC_lineCount);
  return GTLJC_lineCount ;
}


String anomalyTypeStr = "";
String anomalyPredictionScore = "";
void breakString(const String& GTLJC_anomalyStr){
  int white_space_index = GTLJC_anomalyStr.indexOf(" ");
  anomalyTypeStr = GTLJC_anomalyStr.substring(0, white_space_index);
  anomalyPredictionScore = GTLJC_anomalyStr.substring(white_space_index + 1);
}

void GTLJC_parsePredictions(const String& GTLJC_jsonResponse){
  int idx = 0;
  while(true){
    //Graciously extracting inference-data collection start time

    idx = GTLJC_jsonResponse.indexOf("\"timestamp\":", idx);
    if(idx == -1) break;
    idx  += String("\"timestamp\":").length();
    int GTLJC_timestampStart = GTLJC_jsonResponse.indexOf("\"", idx) + 1; // The anomaly_prediction value is also a string and starts with a double quoted which is now omitted
    int GTLJC_timestampEnd = GTLJC_jsonResponse.indexOf("\"", GTLJC_timestampStart);
    String GTLJC_timestampStr = GTLJC_jsonResponse.substring(GTLJC_timestampStart, GTLJC_timestampEnd);
    Serial.print("For Inference Data Collected @ ");
    Serial.print(GTLJC_timestampStr);
    Serial.print(", ");



    //Graciously getting longititude and latitude values from received predictions
    //Latitude Extraction
    idx = GTLJC_jsonResponse.indexOf("\"latitude\":", GTLJC_timestampEnd);
    if(idx == -1) break;
    idx  += String("\"latitude\":").length();

    int GTLJC_latitudeEnd = GTLJC_jsonResponse.indexOf(",",idx);
    String GTLJC_latitudeStr = GTLJC_jsonResponse.substring(idx,GTLJC_latitudeEnd);
    GTLJC_latitudeStr.trim();
    Serial.print("Latitude: ");
    Serial.print(GTLJC_latitudeStr);
    Serial.print(", ");

    //Longitude Extraction
    idx = GTLJC_jsonResponse.indexOf("\"longitude\":", GTLJC_latitudeEnd);
    if(idx == -1) break;
    idx  += String("\"longitude\":").length();

    int GTLJC_longitudeEnd = GTLJC_jsonResponse.indexOf(",",idx);
    String GTLJC_longitudeStr = GTLJC_jsonResponse.substring(idx,GTLJC_longitudeEnd);
    GTLJC_longitudeStr.trim();
    Serial.print("Longitude: ");
    Serial.print(GTLJC_longitudeStr);
    Serial.print(", ");

    // Graciously finding the next "anomaly_prediction" value
    idx = GTLJC_jsonResponse.indexOf("\"anomaly_prediction\":", GTLJC_longitudeEnd);
    if(idx == -1) break;

    idx += String("\"anomaly_prediction\":").length();
    // Graciously skipping the initial quotes
    int GTLJC_predictionStart = GTLJC_jsonResponse.indexOf("\"", idx) + 1; // The anomaly_prediction value is also a string and starts with a double quoted which is now omitted
    int GTLJC_predictionEnd = GTLJC_jsonResponse.indexOf("\"", GTLJC_predictionStart);
    String GTLJC_anomalyStr = GTLJC_jsonResponse.substring(GTLJC_predictionStart, GTLJC_predictionEnd);
    Serial.print("Anomaly Prediction: ");
    Serial.println(GTLJC_anomalyStr);

    idx = GTLJC_predictionEnd;   // Graciously advancing index idx to scan next row of prediction information

    breakString(GTLJC_anomalyStr);

    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Predicted Anomaly: ");
    lcd.setCursor(0,1);
    lcd.print(anomalyTypeStr);
    lcd.setCursor(0,2);
    lcd.print(anomalyPredictionScore);
    lcd.setCursor(0,3);
    lcd.print("Lat:");
    lcd.setCursor(5,3);
    lcd.print(GTLJC_latitudeStr);
    lcd.setCursor(10,3);
    lcd.print("Long:");
    lcd.setCursor(16,3);
    lcd.print(GTLJC_longitudeStr);
    anomalyTypeStr = "";
    anomalyPredictionScore = "";
    delay(3000);

  }
}

void loop()
{    
        // Gracious functions for writing into sd card

        if ( GTLJC_command == 8){  // Collect-Inference-Or-Unlabelled-Data Mode       
          // Graciously accumulating data for 5 minutes (initially this will be smaller)
            
            
            // writeFile(SD, "/GTLJC_data.txt","batch,timestamp/colllection_interval,acc_x ,acc_y,acc_z,rot_x,rot_y ,rot_z,lat,long,GPS_speed_kmph,GPS_speed_mps,GPS_altitude_km,GPS_altitude_m,GPS_data_time,GPS_hdop_acc,GPS_n_of_satellite,anomaly,speed_level_on_encounter\n");
            // readFile(SD, "/GTLJC_data.txt");
            // writeFile(SD, "/GTLJC_data.txt","");           
            long GTLJC_start_collection = millis();
            lcd.setCursor(0,0);
            lcd.print("      Getting");     
            lcd.setCursor(0,1);
            lcd.print("  inference data...");   
            while((millis() - GTLJC_start_collection) < GTLJC_interval_of_collection){
                waitForLabel();
                String GTLJC_label_2 =  GTLJC_batch_results[1];
                if (GTLJC_label_2 != "")
                {
                      //GRACIOUSLY Piling
                      appendFile(SD, "/GTLJC_data.txt", GTLJC_label_2); 
                      Serial.println(GTLJC_label_2);
                      
                }   
            }
            
            GTLJC_command_given = false;
            GTLJC_sample_count = 0;
            ++GTLJC_batch; 
            GTLJC_command = 100;
            int total_no_of_logs = countLoggedLines();  // Gracious count of no of logged lines, just before chunk-wise transfer to backend
            delay(3000);

            File GTLJC_dataFile = SD.open("/GTLJC_data.txt", FILE_READ);
            if(!GTLJC_dataFile){
              Serial.println("❌ Failed to open data file for reading.");
              lcd.clear();
              lcd.setCursor(0,0);
              lcd.print("Failed to open data");
              lcd.setCursor(0,1);
              lcd.print("file for reading.");
              return;
            }

            String GTLJC_batchBuffer = "";
            int GTLJC_lineCount = 0;
            lcd.clear();
            lcd.setCursor(0,0);
            lcd.print("Sending inf data   ");
            lcd.setCursor(0,1);
            lcd.print("Now Sending: "); 
            lcd.setCursor(0,2);
            lcd.print("Remaining: ");
            lcd.setCursor(11,2);
            lcd.print(total_no_of_logs);         
            lcd.setCursor(0,3);
            lcd.print("On backend: ");

            int batch_factor = 0;
            while(GTLJC_dataFile.available()){
                String GTLJC_line = GTLJC_dataFile.readStringUntil('\n');
                if (GTLJC_line.length() == 0) continue; // Graciously skipping empty lines

                GTLJC_batchBuffer += GTLJC_line + "\n";
                ++GTLJC_lineCount;

                // After 100 lines have been collected, send them
                if (GTLJC_lineCount == 100){
                  String rows_received_in_backend = GTLJC_sendJsonBatch(GTLJC_batchBuffer, String(GTLJC_path_inference)); 
                  if(backend_connection_established){
                    break;
                  } 
                  rows_received_in_backend = rows_received_in_backend.substring(rows_received_in_backend.length() - 10, rows_received_in_backend.length() - 1);
                  lcd.setCursor(11,3);
                  lcd.print(rows_received_in_backend);
                  lcd.setCursor(12,1);
                  lcd.print("100 rows"); 
                  lcd.setCursor(0,2);
                  lcd.print("Remaining: ");
                  lcd.setCursor(11,2);
                  batch_factor += 1;
                  lcd.print(total_no_of_logs - batch_factor * 100 );
                  Serial.println("🚀 Sending batch of 100 rows...");                 
                  GTLJC_batchBuffer = "";
                  GTLJC_lineCount = 0;
                }

            }

            backend_connection_established = false;
            lcd.setCursor(11,2);
            lcd.print("      ");
            // Graciously sending the remaining less-than-100-non-zero count of rows
            if (GTLJC_lineCount > 0){
              String rows_received_in_backend_final = GTLJC_sendJsonBatch(GTLJC_batchBuffer, String(GTLJC_path_inference));
              if(backend_connection_established){
                    ;
              }  
              else{
                rows_received_in_backend_final= rows_received_in_backend_final.substring(rows_received_in_backend_final.length() - 10, rows_received_in_backend_final.length() - 1);
                lcd.setCursor(11,3);
                lcd.print(rows_received_in_backend_final);
              }
              Serial.print("🚀 Sending final batch of ");
              Serial.print(GTLJC_lineCount);
              Serial.println(" rows...");
              lcd.setCursor(0,0);
              lcd.print("Sending final batch");     
              lcd.setCursor(12,1);
              lcd.print(String(GTLJC_lineCount) + "rows");  
              lcd.setCursor(11,2);
              lcd.print(total_no_of_logs - batch_factor * 100 - GTLJC_lineCount);
              delay(3000);
              
              // delay(1000) // Inter-batch delay already included in GTLJC_sendJsonBatch routine
 
            }

            GTLJC_dataFile.close();
            if(!backend_connection_established){
              writeFile(SD, "/GTLJC_data.txt", ""); // Graciously emptying acquisition file after sending inference data
            }
           backend_connection_established = false;
           lcd.setCursor(0,1);
           lcd.print("                    ");
           lcd.setCursor(0,3);
           lcd.print("                    ");
        }


        if (GTLJC_command == 90){         //Standalone predictions-collection routine
            lcd.clear();
            lcd.setCursor(0,0);
            lcd.print("     Requesting");    
            lcd.setCursor(0,1);
            lcd.print("     predictions");

            String GTLJC_predictionRequestMessage = "get_predictions\r\n";
            String GTLJC_model_predictions = GTLJC_sendJsonBatch(GTLJC_predictionRequestMessage, GTLJC_path_predictions);
            GTLJC_predictionRequestMessage = "";

            // Verify Data Collected
            if(backend_connection_established){
                  lcd.setCursor(0,2);
                  lcd.print("..request failed..");    
                  
            }  
            else{
                GTLJC_parsePredictions(GTLJC_model_predictions);  // The parse predictions function is to later include an LCD display of predictions
                GTLJC_predictionsReceived = true;
            }

            backend_connection_established = false;
            
            GTLJC_command = 100;
            delay(1000);      


        }
        

        if (GTLJC_command == 12){         //Standalone predictions-collection routine
            lcd.clear();
            lcd.setCursor(0,0);
            lcd.print("Starting");    
            lcd.setCursor(0,1);
            lcd.print("on cloud");  
            lcd.setCursor(0,2);
            lcd.print("data preprocessing..");

            String GTLJC_predictionRequestMessage = "start_data_preprocessing\r\n";
            String preprocessing_status= GTLJC_sendJsonBatch(GTLJC_predictionRequestMessage, GTLJC_path_predictions);
            GTLJC_predictionRequestMessage = "";
            // Verify Data Collected 
            //GTLJC_parsePredictions(GTLJC_model_predictions);  // The parse predictions function is to later include an LCD display of predictions

            String status_substring = "";

            if(backend_connection_established){
                  ;
            }  
            else{
              lcd.clear();
              lcd.setCursor(0,0);
              lcd.print("Starting");    
              lcd.setCursor(0,1);
              lcd.print("on cloud");  
              lcd.setCursor(0,2);
              lcd.print("data preprocessing..");
              lcd.setCursor(0,3);
              lcd.print("...command sent...");
            }
            delay(3000);  
            lcd.clear();
            GTLJC_command = 100;
               
        }

        if(GTLJC_predictionsReceived){
            Serial.println("Gracious message: Predictions received!");
            delay(3000);
            String GTLJC_acceptPredictions = "";
            Serial.println("Accept or Reject Predictions? Press Button 9 or 7 accordingly");
            lcd.clear();
            lcd.setCursor(0,0);
            lcd.print("Accept or ");   
            lcd.setCursor(0,1);
            lcd.print("Reject Predictions?");  
            lcd.setCursor(0,2);
            lcd.print("Press Button 9");  
            lcd.setCursor(0,3);
            lcd.print("or 7 accordingly");

            verification_process_started = true;
            
            while(GTLJC_acceptPredictions == ""){
              waitForLabel();
              if (GTLJC_command == 66) // Decoded command for accepting predictions
              {
                GTLJC_acceptPredictions = "accept";
                lcd.clear();
                lcd.setCursor(0,0);
                lcd.print("Anomaly");  
                lcd.setCursor(8,0);
                lcd.print("accept");
                lcd.setCursor(14,0);
                lcd.print("ed");
                delay(2000);  
                
              }
              else if (GTLJC_command == 74)
              {
                GTLJC_acceptPredictions = "reject";
                lcd.clear();
                lcd.setCursor(0,0);
                lcd.print("Anomaly");  
                lcd.setCursor(8,0);
                lcd.print("reject");
                lcd.setCursor(14,0);
                lcd.print("ed");
                delay(2000);
                
              }
              delay(10);
            }

            Serial.println(GTLJC_acceptPredictions);
            GTLJC_sendJsonBatch(GTLJC_acceptPredictions, GTLJC_path_verification );
            GTLJC_acceptPredictions = "";
            GTLJC_predictionsReceived = false;
            digitalWrite(GTLJC_database_transfer_pin,LOW);
            verification_process_started = false;
            delay(5000);

        }
        else{
          Serial.println("Gracious message: Predictions not received!");
   
          // delay(2000);
          // delay(3000);
        }


        waitForLabel();
        String GTLJC_label =  GTLJC_batch_results[0];
        String GTLJC_label_2 =  GTLJC_batch_results[1];
  
        if (GTLJC_label != "")
        {
              //GRACIOUSLY Piling
              GTLJC_batch_readings += GTLJC_label;
        }   

        if (GTLJC_label_2 != "")
        {
            //GRACIOUSLY Piling Inference data in the correct structure
            GTLJC_batch_readings_json_send +=  GTLJC_label_2;
        }

        Serial.print("Gracious command code: ");
        Serial.println(GTLJC_command);
        if(GTLJC_collecting_data){
          lcd.setCursor(0,0);
          lcd.print("Collecting data...    ");
        }
        else {
          
          if (commandIsNotRecognized (GTLJC_command)){
              lcd.setCursor(0,0);
              lcd.print("Command: weak press");
          }
          else{
            lcd.setCursor(0,0);
            lcd.print("Command:             ");
          }
          
        }
        
        
        if (GTLJC_command == 100){
          lcd.setCursor(0,1);
          lcd.print("                    ");
          if(GTLJC_sample_count == 0){
            lcd.setCursor(0,1);
            lcd.print("(idle)               ");
          }
          else{
            lcd.setCursor(0,1);
            lcd.print("(idle) ..TRY AGAIN..  ");
          }
          
        }
        
        else {
          if (commandIsNotRecognized (GTLJC_command)){
            lcd.setCursor(0,1);
            lcd.print("  ..TRY AGAIN..  ");
          }
          else{
            
            if(get_anomaly_name(GTLJC_command) != ""){
              lcd.setCursor(0,1);
              lcd.print(get_anomaly_name(GTLJC_command));
            } 
            else{
              lcd.setCursor(0,1);
              lcd.print("                    ");
            }

            // lcd.print(String(GTLJC_command) + "             ");
          }
        }

        if (commandIsNotRecognized (GTLJC_command)){
          lcd.setCursor(0,1);
          lcd.print("                    ");
          lcd.setCursor(0,1);
          lcd.print("  ..TRY AGAIN..  ");
        }
        
        for(int GTLJC_i = 0; GTLJC_i < 8; GTLJC_i++){
          if ( GTLJC_command == GTLJC_arr_of_commands[GTLJC_i] ){
                  GTLJC_collecting_data  =  true;
                  String GTLJC_label =  GTLJC_batch_results[0];
                  if (GTLJC_label != "")
                  {
                        //GRACIOUSLY Piling
                       
                        lcd.setCursor(0,0);
                        lcd.print("Collecting data...    ");  
                        appendFile(SD, "/GTLJC_data.txt", GTLJC_label); 
                        Serial.println(GTLJC_label);
                        
                  }   

                // countLoggedLines();  // Gracious count of no of logged lines, just before chunk-wise transfer to backend
                // delay(3000);

                
          }
        }



        if ((millis() - GTLJC_time_to_repeat) < 1000){
                ;
        }
        else if ( GTLJC_command == 70){
              // Graciously erasing out the entire memory
              writeFile(SD, "/GTLJC_data.txt","batch,acc_x,acc_y,acc_z,rot_x,rot_y,rot_z,speed,timestamp,latitude,longitude,accuracy,date_time,anomaly,speed_level\n");
              readFile(SD, "/GTLJC_data.txt");
              GTLJC_batch_readings = "";
              // GTLJC_batch_readings_json_send = "";
              GTLJC_batch = 0;
              GTLJC_command = 100;
              GTLJC_sample_count = 0; 
              GTLJC_timestamp_prev = 0;
              GTLJC_time_to_repeat = millis();

              logs_checked_at_SD_CARD_wipe = true;

              lcd.setCursor(0,2);
              lcd.print("Erasing SD card   ");   
              lcd.setCursor(0,3);
              lcd.print("                  ");  

              // lcd.setCursor(0,3);
              // int no_of_logged_lines_in_SD_card = countLoggedLines();
              // lcd.print("from "); 
              // lcd.setCursor(5,3);
              // lcd.print(no_of_logged_lines_in_SD_card);
              // lcd.setCursor(13,3);
              // lcd.print(" to 0 rows"); 
              


              // digitalWrite(GTLJC_database_transfer_pin, HIGH);
              delay(1000);
              // digitalWrite(GTLJC_database_transfer_pin, LOW); 
              delay(1000);
              lcd.setCursor(0,2);
              lcd.print("                    ");
              lcd.setCursor(0,3);
              lcd.print("                    ");
        }

        if ((millis() - GTLJC_time_to_repeat) < 1000){
                ;
        }
        else if ( GTLJC_command == 71){
            
            int total_no_of_logs = countLoggedLines();  // Gracious count of no of logged lines, just before chunk-wise transfer to backend
            File GTLJC_dataFile = SD.open("/GTLJC_data.txt", FILE_READ);
            if(!GTLJC_dataFile){
              Serial.println("❌ Failed to open data file for reading.");
              lcd.clear();
              lcd.setCursor(0,0);
              lcd.print("Failed to open data");
              lcd.setCursor(0,1);
              lcd.print("file for reading.");
              return;
            }

            String GTLJC_batchBuffer = "";
            int GTLJC_lineCount = 0;
            lcd.clear();
            lcd.setCursor(0,0);
            lcd.print("Sending trn data   ");
            lcd.setCursor(0,1);
            lcd.print("Now Sending: "); 
            lcd.setCursor(0,2);
            lcd.print("Remaining: ");
            lcd.setCursor(11,2);
            lcd.print(total_no_of_logs);         
            lcd.setCursor(0,3);
            lcd.print("On backend: ");

            int batch_factor = 0;
            while(GTLJC_dataFile.available()){
                String GTLJC_line = GTLJC_dataFile.readStringUntil('\n');
                if (GTLJC_line.length() == 0) continue; // Graciously skipping empty lines

                GTLJC_batchBuffer += GTLJC_line + "\n";
                ++GTLJC_lineCount;

                // After 100 lines have been collected, send them
                if (GTLJC_lineCount == 100){
                  String rows_received_in_backend = GTLJC_sendJsonBatch(GTLJC_batchBuffer, String(GTLJC_path_inference)); 
                  if(backend_connection_established){
                    break;
                  } 
                  rows_received_in_backend = rows_received_in_backend.substring(rows_received_in_backend.length() - 10, rows_received_in_backend.length() - 1);
                  lcd.setCursor(11,3);
                  lcd.print(rows_received_in_backend);
                  lcd.setCursor(12,1);
                  lcd.print("100 rows"); 
                  lcd.setCursor(0,2);
                  lcd.print("Remaining: ");
                  lcd.setCursor(11,2);
                  batch_factor += 1;
                  lcd.print(total_no_of_logs - batch_factor * 100 );
                  Serial.println("🚀 Sending batch of 100 rows...");                 
                  GTLJC_batchBuffer = "";
                  GTLJC_lineCount = 0;
                }

            }

            backend_connection_established = false;
            lcd.setCursor(11,2);
            lcd.print("      ");
            // Graciously sending the remaining less-than-100-non-zero count of rows
            if (GTLJC_lineCount > 0){
              String rows_received_in_backend_final = GTLJC_sendJsonBatch(GTLJC_batchBuffer, String(GTLJC_path_inference));
              if(backend_connection_established){
                    ;
              }  
              else{
                rows_received_in_backend_final= rows_received_in_backend_final.substring(rows_received_in_backend_final.length() - 10, rows_received_in_backend_final.length() - 1);
                lcd.setCursor(11,3);
                lcd.print(rows_received_in_backend_final);
              }
              Serial.print("🚀 Sending final batch of ");
              Serial.print(GTLJC_lineCount);
              Serial.println(" rows...");
              lcd.setCursor(0,0);
              lcd.print("Sending final batch");     
              lcd.setCursor(12,1);
              lcd.print(String(GTLJC_lineCount) + "rows");  
              lcd.setCursor(11,2);
              lcd.print(total_no_of_logs - batch_factor * 100 - GTLJC_lineCount);
              delay(3000);
              
              // delay(1000) // Inter-batch delay already included in GTLJC_sendJsonBatch routine
 
            }

            GTLJC_dataFile.close();
            if(!backend_connection_established){
              writeFile(SD, "/GTLJC_data.txt", ""); // Graciously emptying acquisition file after sending inference data
            }
            backend_connection_established = false;
            lcd.setCursor(0,1);
            lcd.print("                    ");
            lcd.setCursor(0,3);
            lcd.print("                    ");
            GTLJC_command = 100; 
            
        }

        if ((millis() - GTLJC_time_to_repeat) < 1000){
                ;
        }
        else if ( GTLJC_command == 69){
          lcd.clear();
          lcd.setCursor(0,0);
          lcd.print("Restarting...");
          lcd.setCursor(0,1);
          lcd.print("Hardware.....");
          delay(5000);
          esp_restart();
        }



        
        //delay(2000);
}

void waitForLabel()
{
  
  if (GTLJC_sample_count == 0){
    digitalWrite(GTLJC_label_provided_led, HIGH);
  }
  else{
    digitalWrite(GTLJC_label_provided_led, LOW);
  }
  
  String GTLJC_label = "";
  String GTLJC_label_2 = "";


  if (IrReceiver.decode()) {
      unsigned long GTLJC_currentTime = millis();

      if (IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT){
          ;
      }
      else{
        GTLJC_command = IrReceiver.decodedIRData.command;
        GTLJC_command_given = true;
        Serial.println(GTLJC_command);     
        lcd.setCursor(0,1);
        lcd.print(GTLJC_command); 
        
      }  
      // GTLJC_lastCommandTime = GTLJC_currentTime;
        IrReceiver.resume();
  }

     
  
  //digitalWrite(GTLJC_label_provided_led, HIGH);
 
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  String acc_x = String(a.acceleration.x);
  String acc_y = String(a.acceleration.y);
  String acc_z = String(a.acceleration.z);

  String rot_x = String(g.gyro.x);
  String rot_y = String(g.gyro.y);
  String rot_z = String(g.gyro.z);
  String lat, lng, GPS_speed_kmph, GPS_speed_mps, GPS_hdop_acc, GPS_altitude_km, GPS_altitude_m, GPS_data_time, GPS_n_of_satellite;
  
  GPS_n_of_satellite = printInt(gps.satellites.value(), gps.satellites.isValid(),2);
  GPS_hdop_acc = printFloat(gps.hdop.hdop(), gps.hdop.isValid(), 6, 1);
  lat = printFloat(gps.location.lat(), gps.location.isValid(), 11, 6);
  lng = printFloat(gps.location.lng(), gps.location.isValid(), 12, 6);
  GPS_speed_kmph = printFloat(gps.speed.kmph(), gps.speed.isValid(), 6, 2);
  GPS_speed_mps = printFloat(gps.speed.mps(), gps.speed.isValid(), 6, 2);
  GPS_altitude_km = printFloat(gps.altitude.kilometers(), gps.altitude.isValid(), 7, 2);
  GPS_altitude_m = printFloat(gps.altitude.meters(), gps.altitude.isValid(), 7, 2);
  GPS_data_time = printDateTime(gps.date, gps.time);
  //printInt(gps.location.age(), gps.location.isValid(), 5);
 

  printInt(gps.charsProcessed(), true, 6);
  printInt(gps.sentencesWithFix(), true, 10);
  printInt(gps.failedChecksum(), true, 9);
  Serial.println();
  
  smartDelay(0);

  if (millis() > 5000 && gps.charsProcessed() < 10)
    Serial.println(F("No GPS data received: check wiring"));
    // lcd.clear();
    // lcd.setCursor(0,0);
    // lcd.print("No GPS data");     
    // lcd.setCursor(0,1);
    // lcd.print("received:"); 
    // lcd.setCursor(0,2);
    // lcd.print("check wiring"); 
    

  unsigned long GTLJC_timestamp = millis();
  String GTLJC_line_values = String(GTLJC_batch) + "," + acc_x + "," + acc_y + "," + acc_z + "," + rot_x + "," + rot_y + "," + rot_z + "," + GPS_speed_mps + ","  + String(GTLJC_sample_count * 24) + "," + lat + "," + lng + ","  + GPS_hdop_acc + ","  + GPS_data_time + "," ;
  String GTLJC_line_values_send = String(GTLJC_batch) + "," + 
                                                acc_x + "," + 
                                                acc_y + "," + 
                                                acc_z + "," + 
                                                rot_x + "," + 
                                                rot_y + "," + 
                                                rot_z + "," + 
                                                GPS_speed_mps + ","  + 
                                                lat + "," + 
                                                lng + ","  + 
                                                GPS_hdop_acc + ","  + 
                                                GPS_data_time + "," + 
                                                String(GTLJC_sample_count * 24) + "\r\n";
                                                
  GTLJC_label_2 = GTLJC_line_values_send;
  if (GTLJC_command_given)
  {

    GTLJC_sample_count++;
    switch(GTLJC_command){
      case 68:
        GTLJC_label = GTLJC_line_values + "no-movement,LOW\n";   
        // GTLJC_label_2 = GTLJC_line_values_send;
        break;
        
      case 64:
        GTLJC_label = GTLJC_line_values + "smooth,AVERAGE\n";
        // GTLJC_label_2 = GTLJC_line_values_send; 
        break;

      case 67:
        GTLJC_label = GTLJC_line_values + "static-vibration,HIGH\n";
        // GTLJC_label_2 = GTLJC_line_values_send; 
        break;

      case 7:
        GTLJC_label = GTLJC_line_values + "crack,LOW\n";
        // GTLJC_label_2 = GTLJC_line_values_send; 
        break;

      case 21:
        GTLJC_label = GTLJC_line_values + "crack,AVERAGE\n";
        // GTLJC_label_2 = GTLJC_line_values_send; 
        break;

      case 9:
        GTLJC_label = GTLJC_line_values + "crack,HIGH\n";
        // GTLJC_label_2 = GTLJC_line_values_send; 
        break;

      case 22:
        GTLJC_label = GTLJC_line_values + "bump,LOW\n";
        // GTLJC_label_2 = GTLJC_line_values_send; 
        break;
      
      case 25:
        GTLJC_label = GTLJC_line_values + "bump,AVERAGE\n";
        // GTLJC_label_2 = GTLJC_line_values_send; 
        break;

      case 13:
        GTLJC_label = GTLJC_line_values + "bump,HIGH\n";
        // GTLJC_label_2 = GTLJC_line_values_send; 
        break;
      
      // case 12: Sends prompt to preprocess already-sent inference data
      //   GTLJC_label = GTLJC_line_values + "road-patch,LOW\n";
      //   GTLJC_label_2 = GTLJC_line_values_send; 
      //   break;

      case 24:
        GTLJC_label = GTLJC_line_values + "road-patch,AVERAGE\n";
        // GTLJC_label_2 = GTLJC_line_values_send; 
        break;

      case 94:
        GTLJC_label = GTLJC_line_values + "road-patch,HIGH\n";
        // GTLJC_label_2 = GTLJC_line_values_send; 
        break;

      // case 8:
      // // Handling automated inference data collection
      //   GTLJC_label = GTLJC_line_values + "pothole_mild,LOW\n";
      //   GTLJC_label_2 = GTLJC_line_values_send; 
      //   break;
      
      case 28:
        GTLJC_label = GTLJC_line_values + "pothole_mild,AVERAGE\n";
        // GTLJC_label_2 = GTLJC_line_values_send; 
        break;
 
      // case 90: Reserved for an "Get predictions" command
      //   GTLJC_label = GTLJC_line_values + "pothole_mild,HIGH\n";
      //   GTLJC_label_2 = GTLJC_line_values_send; 
      //   break;

      // case 66: Reserved for an "accept predictions" command
      //   GTLJC_label = GTLJC_line_values + "POTHOLE-SEVERE,LOW\n";
      //   break;

      case 82:
        GTLJC_label = GTLJC_line_values + "pothole_severe,AVERAGE\n";
        // GTLJC_label_2 = GTLJC_line_values_send; 
        break;

      // case 74: Reserved for a "reject predictions" command
      //   GTLJC_label = GTLJC_line_values + "POTHOLE-SEVERE,HIGH\n";
      //   break;
      
    }

  }
  
  if ((GTLJC_command != 8) && (GTLJC_sample_count == 101)){
    GTLJC_command_given = false;
    GTLJC_sample_count = 0;
    GTLJC_timestamp_prev = 0;
    ++GTLJC_batch; 
    GTLJC_command = 100;
    GTLJC_collecting_data  =  false;
    
  }
  Serial.print("GRACIOUS No of samples: ");
  Serial.println(GTLJC_sample_count);
  // lcd.clear();
  if(!verification_process_started && commandIsNotRecognized (GTLJC_command)){
    lcd.setCursor(0,2);
    lcd.print("after 100th count "); 
    lcd.setCursor(0,3);
    lcd.print(GTLJC_sample_count);
     
  }
  else if (!verification_process_started && GTLJC_command == 100){ 
    lcd.setCursor(0,3);
    lcd.print("              ");

    if(GTLJC_sample_count == 0){
      lcd.setCursor(0,2);
      lcd.print("                 "); 
      lcd.setCursor(0,3);
      lcd.print("              ");
    }
    else{
      lcd.setCursor(0,2);
      lcd.print("after 100th count ");
      lcd.setCursor(0,3);
      lcd.print(GTLJC_sample_count);
    }
    
  }
  else if (!verification_process_started ) {
    lcd.setCursor(0,2);
    lcd.print("No of samples:    ");     
    lcd.setCursor(0,3);
    lcd.print("              "); 
    lcd.setCursor(0,3);
    lcd.print(GTLJC_sample_count);
  }
  else{
    ;
  }
  
  GTLJC_batch_results[0] = GTLJC_label;
  GTLJC_batch_results[1] = GTLJC_label_2;
}

// Gracious functions to handle gps input

// This custom version of delay() ensures that the gps object
// is being "fed".
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}

static String printFloat(float val, bool valid, int len, int prec)
{
  char GTLJC_sz[32];
  if (!valid)
  {
    while (len-- > 1)
      Serial.print('*');
    Serial.print(' ');
  }
  else
  {
    // Serial.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i=flen; i<len; ++i)
      Serial.print(' ');
  }

 
  smartDelay(0);
  sprintf(GTLJC_sz, "%.6f",val);
  return String(GTLJC_sz);
}

static String printInt(unsigned long val, bool valid, int len)
{
  char GTLJC_sz[32] = "*****************";
  if (valid)
    sprintf(GTLJC_sz, "%ld", val);
  GTLJC_sz[len] = 0;
  for (int i=strlen(GTLJC_sz); i<len; ++i)
    GTLJC_sz[i] = ' ';
  if (len > 0) 
    GTLJC_sz[len-1] = ' ';
  // Serial.print(GTLJC_sz);
  smartDelay(0);
  return String(GTLJC_sz);
}

static String printDateTime(TinyGPSDate &d, TinyGPSTime &t)
{
  char sz[32];
  if (!d.isValid() && !t.isValid())
  {
    Serial.print(F("********** "));
  }
  else
  {
    
    sprintf(sz, "%02d/%02d/%02d %02d:%02d:%02d:%f", d.month(), d.day(), d.year(), t.hour(), t.minute(), t.second(), t.centisecond());
    // Serial.print(sz);
    return String(sz);
  }

  printInt(d.age(), d.isValid(), 5);
  smartDelay(0);
  return sz;
}

static void printStr(const char *str, int len)
{
  int slen = strlen(str);
  for (int i=0; i<len; ++i)
    Serial.print(i<slen ? str[i] : ' ');
  smartDelay(0);
}

// Gracious functions for writing into sd card

void readFile(fs::FS &fs, const char *path) {
  Serial.printf("Reading file: %s\n", path);

  File file = fs.open(path);
  if (!file) {
    Serial.println("Failed to open file for reading");
    return;
  }

  Serial.print("Read from file: ");
  while (file.available()) {
    Serial.write(file.read());
  }
  file.close();
}

void writeFile(fs::FS &fs, const char *path, String message) {
  Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  if (file.print(message)) {
    Serial.println("File written");
  } else {
    Serial.println("Write failed");
  }
  file.close();
}

void appendFile(fs::FS &fs, const char *path, String message) {
  Serial.printf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    Serial.println("Failed to open file for appending");
    return;
  }
  if (file.print(message)) {
    Serial.println("Message appended");
  } else {
    Serial.println("Append failed");
  }
  file.close();
}
