#include "DFRobot_AHT20.h"
#include <Wire.h>
#include "ScioSense_ENS160.h"

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
};

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

AHT20Sensor tempHum;
Ens160Sensor air;

void setup(){
    Serial.begin(115200);
    while (!Serial) {}
    tempHum.init();    
    air.init();
  }
  
void loop(){
  Serial.print("UDRIII \n");
  if(tempHum.isReady()) {
    Serial.print(tempHum.getTemperature());
    Serial.print(" C, ");
    Serial.print(tempHum.getHumidity());
    Serial.print(" % ");
  }

  if(air.isReady()) {
    Serial.print("AQI:");Serial.print(air.getAQI());Serial.print("\t");
    Serial.print("TVOC:");Serial.print(air.getTVOC());Serial.print("ppb\t");
    Serial.print("CO2:");Serial.print(air.getECO2());Serial.print("ppm\t");
  }
    Serial.print("\n");
  delay(5000);
}
