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

const char* ssid = "unworthy slave TO CHRIST";
const char* password = "FORCHRIST";
const char* GTLJC_host = "roadanomaly4christalone.pythonanywhere.com";
const int GTLJC_httpsPort = 443;
const char* GTLJC_path_inference = "/api-road-inference-logs/road_anomaly_infer/";
const char* GTLJC_path_predictions = "/api-road-prediction-output/road_anomaly_predict/";

const char* API_PREDICTION_OUTPUT = "https://roadanomaly4christalone.pythonanywhere.com/api-road-prediction-output/road_anomaly_predict/";
const char* API_VERIFICATION = "https://roadanomaly4christalone.pythonanywhere.com/api-road-verification/road_anomaly_verify/";
const char* API_INFERENCE = "https://roadanomaly4christalone.pythonanywhere.com/api-road-inference-logs/road_anomaly_infer/";
const char* API_INFERENCE_NON_JSON = "https://roadanomaly4christalone.pythonanywhere.com/api-road-inference-logs-raw/road_anomaly_inference_raw/";

#define IR_RECEIVE_PIN 27
#define IR_SEND_PIN  15
static const int RXPin = 16, TXPin = 17;
static const uint32_t GPSBaud = 9600;

// The TinyGPSPlus object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);


String GTLJC_batch_readings = "";
String GTLJC_batch_readings_json_send = "";

String GTLJC_batch_results[2];

int GTLJC_vibration_sensor_input = 27;
int GTLJC_vibration;

int GTLJC_sample_count = 0;
int GTLJC_command = 100;
bool GTLJC_command_given = false;

int GTLJC_database_transfer_pin = 26;
int GTLJC_label_provided_led = 25;
unsigned long GTLJC_last_interval_ms = 0;

unsigned long GTLJC_timestamp_prev = 0;
unsigned long GTLJC_batch = 0;
long GTLJC_time_to_repeat = 0;

unsigned long GTLJC_interval_of_collection = 60000 * 1; // Graciously defining a 5-minute interval for data collection
unsigned long GTLJC_send_data_period = 60000 * 0.5 ; // A 1-minute space allowed for data transfer

bool GTLJC_countIsComplete = false;
bool GTLJC_collectionEnded = false;

Adafruit_MPU6050 mpu;

void setup()
{

      Serial.begin(115200);
      ss.begin(GPSBaud);
      pinMode( GTLJC_vibration_sensor_input , INPUT);
      pinMode(GTLJC_database_transfer_pin , OUTPUT);
      
      //digitalWrite(GTLJC_System_Active_Led, HIGH);

      pinMode(GTLJC_label_provided_led , OUTPUT);
      IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);

      if(!mpu.begin()){
        Serial.println("GLORY TO GOD ALONE, Failed to find mpu chip");
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
            return;
      }
      uint8_t cardType = SD.cardType();

      if (cardType == CARD_NONE) {
        Serial.println("No SD card attached");
        return;
      }

      Serial.print("SD Card Type: ");
      if (cardType == CARD_MMC) {
        Serial.println("MMC");
      } else if (cardType == CARD_SD) {
        Serial.println("SDSC");
      } else if (cardType == CARD_SDHC) {
        Serial.println("SDHC");
      } else {
        Serial.println("UNKNOWN");
      }

      uint64_t cardSize = SD.cardSize() / (1024 * 1024);
      Serial.printf("SD Card Size: %lluMB\n", cardSize);

      // writeFile(SD, "/GTLJC_data.txt","batch,timestamp/colllection_interval,acc_x ,acc_y,acc_z,rot_x,rot_y ,rot_z,lat,long,GPS_speed_kmph,GPS_speed_mps,GPS_altitude_km,GPS_altitude_m,GPS_data_time,GPS_hdop_acc,GPS_n_of_satellite,anomaly,speed_level_on_encounter\n");
      // readFile(SD, "/GTLJC_data.txt");
      Serial.printf("Total space: %lluMB\n", SD.totalBytes() / (1024 * 1024));
      Serial.printf("Used space: %lluMB\n", SD.usedBytes() / (1024 * 1024));

      WiFi.begin(ssid,password);
      Serial.print("Connecting to WiFi");
      while(WiFi.status() != WL_CONNECTED){
        delay(500);
        Serial.print(".");
      }
      Serial.println("\nConnected to WiFi");



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
void GTLJC_sendJsonBatch(const String& rawBatch) {
          
          WiFiClientSecure client;
          client.setInsecure(); // ❗ Trusts all certificates — for development/testing only
          // HTTPClient http;

          Serial.print("🌐 Connecting to ");
          Serial.println(GTLJC_host);

          if(!client.connect(GTLJC_host, GTLJC_httpsPort)){
            Serial.println("❌ HTTPS connection failed");
            return;
          }

          // Graciously sending HTTP headers and body
          client.println("POST " + String(GTLJC_path_inference) + " HTTP/1.1");
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
          while (client.connected() || client.available()) {
            if (client.available()) {
              char c = client.read();
              Serial.print(c);
            }
          }

          // ✅ Always close the connection
          client.stop();
          Serial.println("\n✅ HTTPS text POST complete.");

          GTLJC_command = 100;
          // WiFi.disconnect(true);
          delay(2000);
}

void countLoggedLines(){
  File GTLJC_dataFile = SD.open("/GTLJC_data.txt", FILE_READ);
  if(!GTLJC_dataFile){
    Serial.println("❌ Failed to open file for counting lines.");
    return;
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
  Serial.println(GTLJC_lineCount);
}

void GTLJC_sendPredictionRequest(const String& requestMessage){
          WiFiClientSecure client;
          client.setInsecure(); // ❗ Trusts all certificates — for development/testing only
          // HTTPClient http;

          Serial.print("🌐 Connecting to ");
          Serial.println(GTLJC_host);

          if(!client.connect(GTLJC_host, GTLJC_httpsPort)){
            Serial.println("❌ HTTPS connection failed");
            return;
          }

          // Graciously sending HTTP headers and body
          client.println("POST " + String(GTLJC_path_predictions) + " HTTP/1.1");
          client.println("Host: " + String(GTLJC_host));
          client.println("Content-Type: text/plain");
          client.println("Connection: close");
          client.print("Content-Length: ");
          client.println(requestMessage.length());
          client.println();  // End of headers
          
          client.write((const uint8_t *)requestMessage.c_str(), requestMessage.length());
          Serial.print(requestMessage);
          // client.print(rawBatch); // Send raw data
          

          // Graciously reading server response
          Serial.println("📨 Server Response:");
          while (client.connected() || client.available()) {
            if (client.available()) {
              char c = client.read();
              Serial.print(c);
            }
          }

          // ✅ Always close the connection
          client.stop();
          Serial.println("\n✅ HTTPS prediction request-text POST complete.");

          GTLJC_command = 100;
         
          // WiFi.disconnect(true);
          delay(2000);
}

void loop()
{
        if ( GTLJC_command == 8){  // Collect-Inference-Or-Unlabelled-Data Mode       
          // Graciously accumulating data for 5 minutes (initially this will be smaller)
            
            // writeFile(SD, "/GTLJC_data.txt","batch,timestamp/colllection_interval,acc_x ,acc_y,acc_z,rot_x,rot_y ,rot_z,lat,long,GPS_speed_kmph,GPS_speed_mps,GPS_altitude_km,GPS_altitude_m,GPS_data_time,GPS_hdop_acc,GPS_n_of_satellite,anomaly,speed_level_on_encounter\n");
            readFile(SD, "/GTLJC_data.txt");
            writeFile(SD, "/GTLJC_data.txt","");           
            long GTLJC_start_collection = millis();
            while((millis() - GTLJC_start_collection) < GTLJC_interval_of_collection){
                waitForLabel();
                String GTLJC_label_2 =  GTLJC_batch_results[1];
                if (GTLJC_label_2 != "")
                {
                      //GRACIOUSLY Piling
                      GTLJC_batch_readings_json_send += GTLJC_label_2;
                      appendFile(SD, "/GTLJC_data.txt", GTLJC_label_2); 
                      Serial.println(GTLJC_label_2);
                      
                }   
            }
            
            GTLJC_command_given = false;
            GTLJC_sample_count = 0;
            ++GTLJC_batch; 
            GTLJC_command = 100;
            countLoggedLines();  // Gracious count of no of logged lines, just before chunk-wise transfer to backend
            delay(3000);

            File GTLJC_dataFile = SD.open("/GTLJC_data.txt", FILE_READ);
            if(!GTLJC_dataFile){
              Serial.println("❌ Failed to open data file for reading.");
              return;
            }

            String GTLJC_batchBuffer = "";
            int GTLJC_lineCount = 0;

            while(GTLJC_dataFile.available()){
                String GTLJC_line = GTLJC_dataFile.readStringUntil('\n');
                if (GTLJC_line.length() == 0) continue; // Graciously skipping empty lines

                GTLJC_batchBuffer += GTLJC_line + "\n";
                ++GTLJC_lineCount;

                // After 100 lines have been collected, send them
                if (GTLJC_lineCount == 100){
                  Serial.println("🚀 Sending batch of 100 rows...");
                  GTLJC_sendJsonBatch(GTLJC_batchBuffer);
                  // delay(1000) // Inter-batch delay already included in GTLJC_sendJsonBatch routine

                  GTLJC_batchBuffer = "";
                  GTLJC_lineCount = 0;
                }

            }

            // Graciously sending the remaining less-than-100-non-zero count of rows
            if (GTLJC_lineCount > 0){
              Serial.print("🚀 Sending final batch of ");
              Serial.print(GTLJC_lineCount);
              Serial.println(" rows...");
              GTLJC_sendJsonBatch(GTLJC_batchBuffer);
              // delay(1000) // Inter-batch delay already included in GTLJC_sendJsonBatch routine
 
            }

            GTLJC_dataFile.close();

            // Graciously getting predictions and taking verification steps
            // String GTLJC_predictionRequestMessage = "get_predictions";
            // GTLJC_sendPredictionRequest( GTLJC_predictionRequestMessage );
            //  GTLJC_predictionRequestMessage = "";
            // Serial.print("Checking if request message is emptied, request-message-content : ");
            // Serial.println( GTLJC_predictionRequestMessage );
            // GTLJC_fetchJsonData();  // Graciously getting predictions
        
        }


        if ( GTLJC_command == 90){         // Collect-Labelled-Data-Mode
            String GTLJC_predictionRequestMessage = "get_predictions\r\n";
            GTLJC_sendPredictionRequest( GTLJC_predictionRequestMessage );
            GTLJC_predictionRequestMessage = "";
            Serial.print("Checking if request message is emptied, request-message-content : ");
            Serial.println( GTLJC_predictionRequestMessage );
        
        }

        
        //  GTLJC_last_interval_ms = millis();
        // if(WiFi.status() != WL_CONNECTED)
        // {
        //   connectWiFi();
        // }
        // else
        
        //GTLJC_vibration = pulseIn (GTLJC_vibration_sensor_input, HIGH);
        waitForLabel();
        String GTLJC_label =  GTLJC_batch_results[0];
        String GTLJC_label_2 =  GTLJC_batch_results[1];

    
    
        //digitalWrite(GTLJC_label_provided_led, LOW);
    
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

        

        // WiFiClient client;
        Serial.print("Gracious command code: ");
        Serial.println(GTLJC_command);
        
        if ( GTLJC_command == 70)
        {
        // Graciously saving collected data here
              if ((millis() - GTLJC_time_to_repeat) < 1000){
                ;
              } else {
                if (gps.location.isValid())
                {
                  for (int GTLJC_count = 0; GTLJC_count < 2; GTLJC_count++){
                    digitalWrite(GTLJC_database_transfer_pin,HIGH);
                    delay(500);
                    digitalWrite(GTLJC_database_transfer_pin, LOW);
                    delay(500);
                  }                  
                }
                digitalWrite(GTLJC_database_transfer_pin, HIGH);
                // appendFile(SD, "/GTLJC_data.txt",GTLJC_batch_readings );
                appendFile(SD, "/GTLJC_data.txt", GTLJC_batch_readings);      
                readFile(SD, "/GTLJC_data.txt");
                //delay(2000);
              }
                // Graciously the delay doubles due to the yet-present Ir code of 70 propagating from Ir decoder block in waitForLabel() 
              //Serial.print(GTLJC_batch_readings);
              GTLJC_batch_readings = "";
              // GTLJC_batch_readings_json_send = "";
              GTLJC_command = 100;
              GTLJC_sample_count = 0; 
              GTLJC_timestamp_prev = 0;
              GTLJC_time_to_repeat = millis();
                
              digitalWrite(GTLJC_database_transfer_pin, LOW);
              

        }

        if ((millis() - GTLJC_time_to_repeat) < 1000){
                ;
        }
        else if ( GTLJC_command == 71){

              // Graciously erasing out a batch
              GTLJC_batch_readings = "";   
              // GTLJC_batch_readings_json_send = "";  
              GTLJC_command = 100;
              GTLJC_sample_count = 0; 
              GTLJC_timestamp_prev = 0;
              GTLJC_time_to_repeat = millis();
              digitalWrite(GTLJC_database_transfer_pin, HIGH);
              delay(1000);
              digitalWrite(GTLJC_database_transfer_pin, LOW);
              delay(1000);
        }

        if ((millis() - GTLJC_time_to_repeat) < 1000){
                ;
        }
        else if ( GTLJC_command == 69){
              // Graciously erasing out the entire memory
              writeFile(SD, "/GTLJC_data.txt","batch,acc_x,acc_y,acc_z,rot_x,rot_y,rot_z,speed,latitude,longitude,accuracy,timestamp,log_interval\n");
              GTLJC_batch_readings = "";
              // GTLJC_batch_readings_json_send = "";
              GTLJC_batch = 0;
              GTLJC_command = 100;
              GTLJC_sample_count = 0; 
              GTLJC_timestamp_prev = 0;
              GTLJC_time_to_repeat = millis();
              digitalWrite(GTLJC_database_transfer_pin, HIGH);
              delay(1000);
              digitalWrite(GTLJC_database_transfer_pin, LOW);
              delay(1000);
        }

        if ((millis() - GTLJC_time_to_repeat) < 1000){
                ;
        }
        else if ( GTLJC_command == 66){
          // To graciously handle outgoing inference data 
          GTLJC_sendJsonBatch(GTLJC_batch_readings_json_send);
          // Serial.print(GTLJC_batch_readings_json_send);
          // GTLJC_batch_readings_json_send = "";
          


        }

        if ((millis() - GTLJC_time_to_repeat) < 1000){
                ;
        }
        else if ( GTLJC_command == 74){
          // To graciously handle incoming predictions 
          GTLJC_fetchJsonData();

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
      GTLJC_command = IrReceiver.decodedIRData.command;
      GTLJC_command_given = true;
      Serial.println(GTLJC_command);
      // delay(5000);
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

  unsigned long GTLJC_timestamp = GTLJC_sample_count * 3;
  // GTLJC_timestamp_prev = GTLJC_timestamp;
  // Serial.print("GRACIOUS Previous timestamp");
  // Serial.println(GTLJC_timestamp_prev);
  // delay(5000);
  //String GTLJC_line_values = String(GTLJC_batch) + "," + acc_x + "," + acc_y + "," + acc_z + "," + rot_x + "," + rot_y + "," + rot_z + "," + lat + "," + lng + "," + GPS_speed_kmph + "," + GPS_speed_mps + "," + GPS_hdop_acc + ","  + String(GTLJC_timestamp) + ","  + GPS_altitude_km + "," + GPS_altitude_m + "," + GPS_data_time  + "," + GPS_n_of_satellite + "," ;
  String GTLJC_line_values = String(GTLJC_batch) + "," + acc_x + "," + acc_y + "," + acc_z + "," + rot_x + "," + rot_y + "," + rot_z + "," + GPS_speed_mps + ","  + String(GTLJC_sample_count) + "," + lat + "," + lng + ","  + GPS_hdop_acc + ","  + GPS_data_time  + "," ;
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
                                                String(GTLJC_sample_count) + "\r\n";
                                                
  // String GTLJC_label_2 = GTLJC_line_values_send;
  // GTLJC_label_2 = GTLJC_line_values_send;
  if (GTLJC_command_given)
  {

    GTLJC_sample_count++;
    switch(GTLJC_command){
      case 68:
        GTLJC_label = GTLJC_line_values + "no-movement,LOW\n";   
        GTLJC_label_2 = GTLJC_line_values_send;
        break;

      case 64:
        GTLJC_label = GTLJC_line_values + "smooth,AVERAGE\n";
        GTLJC_label_2 = GTLJC_line_values_send; 
        break;

      case 67:
        GTLJC_label = GTLJC_line_values + "walking,HIGH\n";
        GTLJC_label_2 = GTLJC_line_values_send; 
        break;

      case 7:
        GTLJC_label = GTLJC_line_values + "crack,LOW\n";
        GTLJC_label_2 = GTLJC_line_values_send; 
        break;

      case 21:
        GTLJC_label = GTLJC_line_values + "crack,AVERAGE\n";
        GTLJC_label_2 = GTLJC_line_values_send; 
        break;

      case 9:
        GTLJC_label = GTLJC_line_values + "crack,HIGH\n";
        GTLJC_label_2 = GTLJC_line_values_send; 
        break;

      case 22:
        GTLJC_label = GTLJC_line_values + "bump,LOW\n";
        GTLJC_label_2 = GTLJC_line_values_send; 
        break;
      
      case 25:
        GTLJC_label = GTLJC_line_values + "bump,AVERAGE\n";
        GTLJC_label_2 = GTLJC_line_values_send; 
        break;

      case 13:
        GTLJC_label = GTLJC_line_values + "bump,HIGH\n";
        GTLJC_label_2 = GTLJC_line_values_send; 
        break;
      
      case 12:
        GTLJC_label = GTLJC_line_values + "road-patch,LOW\n";
        GTLJC_label_2 = GTLJC_line_values_send; 
        break;

      case 24:
        GTLJC_label = GTLJC_line_values + "road-patch,AVERAGE\n";
        GTLJC_label_2 = GTLJC_line_values_send; 
        break;

      case 94:
        GTLJC_label = GTLJC_line_values + "road-patch,HIGH\n";
        GTLJC_label_2 = GTLJC_line_values_send; 
        break;

      case 8:
      // Handling automated inference data collection
        GTLJC_label = GTLJC_line_values + "pothole_mild,LOW\n";
        GTLJC_label_2 = GTLJC_line_values_send; 
        break;
      
      case 28:
        GTLJC_label = GTLJC_line_values + "pothole_mild,AVERAGE\n";
        GTLJC_label_2 = GTLJC_line_values_send; 
        break;

      // case 90:
      //   GTLJC_label = GTLJC_line_values + "pothole_mild,HIGH\n";
      //   GTLJC_label_2 = GTLJC_line_values_send; 
      //   break;

      // case 66:
      //   GTLJC_label = GTLJC_line_values + "POTHOLE-SEVERE,LOW\n";
      //   break;

      case 82:
        GTLJC_label = GTLJC_line_values + "pothole_severe,AVERAGE\n";
        GTLJC_label_2 = GTLJC_line_values_send; 
        break;

      // case 74:
      //   GTLJC_label = GTLJC_line_values + "POTHOLE-SEVERE,HIGH\n";
      //   break;
      
    }

  }
  
  if ((GTLJC_command != 8) && (GTLJC_sample_count == 100)){
    GTLJC_command_given = false;
    GTLJC_sample_count = 0;
    GTLJC_timestamp_prev = 0;
    ++GTLJC_batch; 
    GTLJC_command = 100;
    
  }
  Serial.print("GRACIOUS No of samples: ");
  Serial.println(GTLJC_sample_count);
  //delay(5000);
  // return GTLJC_label;
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
