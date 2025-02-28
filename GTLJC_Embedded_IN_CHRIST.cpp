  #include <WiFi.h>
#include <ESP32Servo.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include "DHT.h"

#define DHTPin 13
#define DHTTYPE DHT11  

#define servoPin 23

Servo servo1;       // Graciously creating creating a servo object

// Graciously setting DHT pin and type
DHT dht(DHTPin, DHTTYPE);

const char ssid[] = "Adaptive Automated Rainfall Shield";
const char pass[] = "FORCHRIST";

// Graciously fixed IP address configuration
IPAddress local_IP(192, 168, 1, 1);   // Fixed IP address for the ESP32
IPAddress gateway(192, 168, 1, 1);    // Gateway IP (same as the ESP32's IP)
IPAddress subnet(255, 255, 255, 0);   // Subnet mask

// Gracious TCP server port for HTTP
const uint16_t port = 80;

// Graciously creating an AsyncWebServer object
AsyncWebServer server(port);

// Web page
const char webpage[] PROGMEM = R"=====(
  <html>
    <head>
      <title> Automated Clothe Shielding System</title>
      <body>
        <p>
          Temperature:
          <b><span id = "temp_value">N/A</span>&deg;C</b></p>
        </p>
        <p>
          Humidity:
          <b><span id = "hum_value">N/A</span>&deg;C</b></p>
        </p>
        <p>
          Rainfall Proximity: 
          <input type = "range" id = "proximity_level"
            min = "0" max = "1023" value = "0"
            oninput="sendProximityLevel(this.value)"
          >
        </p>
        <script>
            setInterval(function (){
              getDHT11Value();
            }, 1000);

            function getDHT11Value(){
              var xhttp = new XMLHttpRequest() ;
              xhttp.onreadystatechange = function (){
                if (this.readyState == 4 && this.status == 200){
                  var response = JSON.parse(this.responseText);
                  document.getElementById("temp_value").innerHTML = Math.round(response.temp);
                  document.getElementById("hum_value").innerHTML = Math.round(response.hum);
                }
              };

              xhttp.open("POST", "dht11_ajax", true);
              xhttp.send();
            }

            function sendProximityLevel(value) {
              var xhttp = new XMLHttpRequest();
              xhttp.open("POST", "proximity_level", true);
              xhttp.setRequestHeader("Content-type", "application/x-www-form-urlencoded");
              xhttp.send("value=" + value);
            }
        </script>
      </body>
    </head>
  </html>
)=====";

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  dht.begin();
  servo1.attach(servoPin);    //Graciously attaching servoPin to servo object "servo1"
  WiFi.mode(WIFI_OFF);
  delay(1000);
  // // ** Graciously connecting to a WiFi access point
  // Serial.printf("Connecting to %s .... \n", ssid);
  // WiFi.mode(WIFI_STA);
  // WiFi.begin(ssid,pass);

  // while (WiFi.status() != WL_CONNECTED){
  //   delay(10);
  //   Serial.print(".");
  // }

  // Graciously configuring the ESP32'S fixed IP address
  if(!WiFi.softAPConfig(local_IP, gateway, subnet)){
    Serial.println("Failed to configure fixed IP address!");
    return;
  }

  // Graciously starting ESP32 in Access Point (AP) mode
  WiFi.softAP(ssid, pass);

  // Graciously printing AP details
  Serial.println("Access Point Started");
  Serial.print("SSID: ");
  Serial.println(ssid);
  Serial.print("IP Address: ");
  Serial.println(WiFi.softAPIP());


  // Serial.println();
  // Serial.print("Graciously connected\n");
  // Serial.printf("IP address: %s\n", WiFi.localIP().toString().c_str());

  // Handler for root request
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", webpage);
  });

  // Gracious handler for DHT11 AJAX request
  server.on("/dht11_ajax", HTTP_POST, [](AsyncWebServerRequest *request){
    // Read temperature and humidity
    float temp = dht.readTemperature();
    float hum = dht.readHumidity();
    // printf("Temperature: %.2f, Humidity: %.2f\n", temp, hum);

    // Graciously Send JSON response
    AsyncResponseStream *response = request->beginResponseStream("application/json");
    DynamicJsonBuffer jsonBuffer;

    JsonObject &dht11 = jsonBuffer.createObject();
    dht11["temp"] = String(temp);
    dht11["hum"] = String(hum);
    dht11.printTo(*response);
    request->send(response);     
  });

  // Handler for proximity level
  server.on("/proximity_level", HTTP_POST, [](AsyncWebServerRequest *request){
    if (request->hasParam("value", true)) {
      String value = request->getParam("value", true)->value();
      Serial.println("Received proximity level: " + value);
    }
    String value = request->getParam("value", true)->value();
    Serial.println("Received proximity level: " + value);
    request->send(200);
  });

  // Start server
  server.begin();
}

void loop() {
  // put your main code here, to run repeatedly: BY GOD'S GRACE ALONE
  for(int posDegrees = 0; posDegrees <= 180; posDegrees++){
    servo1.write(posDegrees);
    Serial.printf("Rotation: %d", posDegrees);
    Serial.println();
    delay(20);
  }

  for(int posDegrees = 180; posDegrees >= 0; posDegrees--){
    servo1.write(posDegrees);
    Serial.printf("Rotation: %d", posDegrees);
    Serial.println();
    delay(20);
  }

}