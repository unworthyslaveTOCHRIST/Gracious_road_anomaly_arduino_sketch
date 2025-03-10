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

#define IR_RECEIVE_PIN 27
#define IR_SEND_PIN  15
static const int RXPin = 16, TXPin = 17;
static const uint32_t GPSBaud = 9600;

// The TinyGPSPlus object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);


String GTLJC_batch_readings = "";

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
      readFile(SD, "/GTLJC_data.txt");
      Serial.printf("Total space: %lluMB\n", SD.totalBytes() / (1024 * 1024));
      Serial.printf("Used space: %lluMB\n", SD.usedBytes() / (1024 * 1024));

}

void loop()
{
   GTLJC_last_interval_ms = millis();
  // if(WiFi.status() != WL_CONNECTED)
  // {
  //   connectWiFi();
  // }
  // else
  
    //GTLJC_vibration = pulseIn (GTLJC_vibration_sensor_input, HIGH);
    String GTLJC_label =  waitForLabel();
    
    //digitalWrite(GTLJC_label_provided_led, LOW);
    
        if (GTLJC_label != "")
        {
              //GRACIOUSLY Piling
                GTLJC_batch_readings += GTLJC_label;
        }      
              // WiFiClient client;
        Serial.print(GTLJC_command);
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
                appendFile(SD, "/GTLJC_data.txt",GTLJC_batch_readings );
                readFile(SD, "/GTLJC_data.txt");
                delay(2000);
              }
                // Graciously the delay doubles due to the yet-present Ir code of 70 propagating from Ir decoder block in waitForLabel() 
              //Serial.print(GTLJC_batch_readings);
              GTLJC_batch_readings = "";
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
              writeFile(SD, "/GTLJC_data.txt","batch,timestamp/colllection_interval,acc_x ,acc_y,acc_z,rot_x,rot_y ,rot_z,lat,long,GPS_speed_kmph,GPS_speed_mps,GPS_altitude_km,GPS_altitude_m,GPS_data_time,GPS_hdop_acc,GPS_n_of_satellite,anomaly,speed_level_on_encounter\n");
              GTLJC_batch_readings = "";
              GTLJC_command = 100;
              GTLJC_sample_count = 0; 
              GTLJC_timestamp_prev = 0;
              GTLJC_time_to_repeat = millis();
              digitalWrite(GTLJC_database_transfer_pin, HIGH);
              delay(1000);
              digitalWrite(GTLJC_database_transfer_pin, LOW);
              delay(1000);
        }
        
      
        //delay(2000);
}

String waitForLabel()
{
  
  if (GTLJC_sample_count == 0){
    digitalWrite(GTLJC_label_provided_led, HIGH);
  }
  else{
    digitalWrite(GTLJC_label_provided_led, LOW);
  }
  
  String GTLJC_label = "";

  if (IrReceiver.decode()) {
      GTLJC_command = IrReceiver.decodedIRData.command;
      GTLJC_command_given = true;
      Serial.println(GTLJC_command);
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

  unsigned long GTLJC_timestamp = millis() -  GTLJC_last_interval_ms + GTLJC_timestamp_prev;
  GTLJC_timestamp_prev = GTLJC_timestamp;
  // Serial.print("GRACIOUS Previous timestamp");
  // Serial.println(GTLJC_timestamp_prev);
  // delay(5000);
  String GTLJC_line_values = String(GTLJC_batch) + "," + String(GTLJC_timestamp) + "," + acc_x + "," + acc_y + "," + acc_z + "," + rot_x + "," + rot_y + "," + rot_z + "," + lat + "," + lng + "," + GPS_speed_kmph + "," + GPS_speed_mps + "," + GPS_altitude_km + "," + GPS_altitude_m + "," + GPS_data_time + "," + GPS_hdop_acc + "," + GPS_n_of_satellite + "," ;
 
  if (GTLJC_command = 0){
    GTLJC_command_given = false;
  }

  if (GTLJC_command_given)
  {

    GTLJC_sample_count++;
    switch(GTLJC_command){
      case 68:
        GTLJC_label = GTLJC_line_values + "SMOOTH,LOW\n";
        break;

      case 64:
        GTLJC_label = GTLJC_line_values + "SMOOTH,AVERAGE\n";
        break;

      case 67:
        GTLJC_label = GTLJC_line_values + "SMOOTH,HIGH\n";
        break;

      case 7:
        GTLJC_label = GTLJC_line_values + "CRACKED/CORRUGATED,LOW\n";
        break;

      case 21:
        GTLJC_label = GTLJC_line_values + "CRACKED/CORRUGATED,AVERAGE\n";
        break;

      case 9:
        GTLJC_label = GTLJC_line_values + "CRACKED/CORRUGATED,HIGH\n";
        break;

      case 22:
        GTLJC_label = GTLJC_line_values + "BUMP,LOW\n";
        break;
      
      case 25:
        GTLJC_label = GTLJC_line_values + "BUMP,AVERAGE\n";
        break;

      case 13:
        GTLJC_label = GTLJC_line_values + "BUMP,HIGH\n";
        break;
      
      case 12:
        GTLJC_label = GTLJC_line_values + "ROAD-PATCH,LOW\n";
        break;

      case 24:
        GTLJC_label = GTLJC_line_values + "ROAD-PATCH,AVERAGE\n";
        break;

      case 94:
        GTLJC_label = GTLJC_line_values + "ROAD-PATCH,HIGH\n";
        break;

      case 8:
        GTLJC_label = GTLJC_line_values + "POTHOLE-MILD,LOW\n";
        break;
      
      case 28:
        GTLJC_label = GTLJC_line_values + "POTHOLE-MILD,AVERAGE\n";
        break;

      case 90:
        GTLJC_label = GTLJC_line_values + "POTHOLE-MILD,HIGH\n";
        break;

      case 66:
        GTLJC_label = GTLJC_line_values + "POTHOLE-SEVERE,LOW\n";
        break;

      case 82:
        GTLJC_label = GTLJC_line_values + "POTHOLE-SEVERE,AVERAGE\n";
        break;

      case 74:
        GTLJC_label = GTLJC_line_values + "POTHOLE-SEVERE,HIGH\n";
        break;
      
    }

  }
  
  if (GTLJC_sample_count == 333){
    GTLJC_command_given = false;
    GTLJC_sample_count = 0;
    GTLJC_timestamp_prev = 0;
    ++GTLJC_batch; 
    GTLJC_command = 100;
  }
  Serial.print("GRACIOUS No of samples: ");
  Serial.println(GTLJC_sample_count);
  //delay(5000);
  return GTLJC_label;


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
