#include "DFRobot_AHT20.h"
#include <Wire.h>
#include "ScioSense_ENS160.h"
#include <ESP8266WiFi.h>
// #include <WiFi.h>
#include <iostream>
#include <map>
#include <ArduinoJson.h> 

class AHT20Sensor {
  private:
    DFRobot_AHT20 aht20;
  public:
    AHT20Sensor() {}

  void init(){
    uint8_t status;
     while((status = aht20.begin()) != 0){
        Serial.print("AHT20 sensor initialization failed. error status : ");
        Serial.println(status);
        delay(1000);
    }

    Serial.println(status);
  }

  int isReady(){
   return aht20.startMeasurementReady(/* crcEn = */true);
  }

  float getTemperature() {
      return aht20.getTemperature_C();
  }

  float getHumidity(){
    return aht20.getHumidity_RH();
  }
}; // AHT class end

class Ens160Sensor {
  private:
    ScioSense_ENS160 ens160; // Move the object declaration here

  public:
    Ens160Sensor() : ens160(ENS160_I2CADDR_1) {} // Constructor to initialize ens160 object

    void init() {
      Serial.println("------------------------------------------------------------");
      Serial.println("ENS160 - Digital air quality sensor");
      Serial.println();
      Serial.println("Sensor readout in standard mode");
      Serial.println();
      Serial.println("------------------------------------------------------------");
      delay(1000);
      
      Serial.print("ENS160...");
      ens160.begin();
      Serial.println(ens160.available() ? "done." : "failed!");
      if (ens160.available()) {
        // Print ENS160 versions
        Serial.print("\tRev: "); Serial.print(ens160.getMajorRev());
        Serial.print("."); Serial.print(ens160.getMinorRev());
        Serial.print("."); Serial.println(ens160.getBuild());
      
        Serial.print("\tStandard mode ");
        Serial.println(ens160.setMode(ENS160_OPMODE_STD) ? "done." : "failed!");
      }
    } //INIT END
    

    bool isReady (){
      bool ready =  ens160.available();
      ens160.measure(0);
      return ready;
    }
    
    uint16_t getTVOC() {
      return ens160.getTVOC();  
    }

    uint16_t getECO2() {
      return ens160.geteCO2();
    }
  uint8_t getAQI() {
    return ens160.getAQI();
  }
  
}; // Ens160Sensor END

class SensorData {
private:
  AHT20Sensor aht;
  Ens160Sensor ens;

public: 
  SensorData() {}

  void setUp() {
    while (!Serial) {}
    aht.init();
    ens.init();
  }

  std::map<std::string, int>AHTData(){
      int hummidity = 0;
      int tempreture = 0;

    if (aht.isReady()) {
      hummidity = aht.getHumidity();
      tempreture = aht.getTemperature();
    }


      std::map<std::string, int> ahtMap = {
        {"hummidity", hummidity},
        {"tempreture", tempreture},
      };
      return ahtMap;
  };

  std::map<std::string, int>AQIData() {
    
    int tvoc = 0;
    int co2 = 0;
    int aqi = 0;

    if(ens.isReady()) {
        tvoc = ens.getTVOC();
        co2 = ens.getECO2();
        aqi = ens.getAQI();
    }

    std::map<std::string, int> aqiMap = {
          {"tvoc", tvoc},
          {"co2",  co2},
          {"aqi", aqi}
      };
    
    return aqiMap;
  };

  std::map<std::string, int> getAllData() {
    std::map<std::string, int> ahtData = AHTData();

    std::map<std::string, int> aqiData = AQIData();

    // Merge the two maps into a single map
    std::map<std::string, int> mergedData;
    mergedData.insert(ahtData.begin(), ahtData.end());
    mergedData.insert(aqiData.begin(), aqiData.end());

    return mergedData;
  }
};


class Communicator {
 private:
  const char* ssid = "KQPV";
  const char* password = "66642312";

  public:
  Communicator() {}

  void connect() {
    WiFi.begin(ssid, password);

    Serial.print("Connecting to ");
    Serial.println(ssid);
    
    while (WiFi.status() != WL_CONNECTED) {
      delay(1000);
      Serial.print(".");
    }

    Serial.println("\nConnected to WiFi");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  };
  
  bool sendSensorData(const std::map<std::string, int>& data) {
    String payload;
    StaticJsonDocument<256> doc;
    // Convert the data map to a JSON payload
    for (const auto& entry : data) {
        doc[entry.first] = entry.second;
    }
    serializeJson(doc, payload);
    Serial.print("\n");
    Serial.print(payload);
    Serial.print("\n");
    return true;
  };

}; //END Communicator Class

SensorData sensors;
Communicator communicator;

void setup(){
  Serial.begin(115200);
  communicator.connect();
  sensors.setUp();
}

void loop(){
 communicator.sendSensorData(sensors.getAllData());
  delay(5000);
}