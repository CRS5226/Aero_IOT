#include "DHT.h"

#define DHTPIN 2     
#define DHTTYPE DHT11   
int sensorValue;
DHT dht(DHTPIN, DHTTYPE);

void setup() {
  Serial.begin(115200);
//  Serial.println(F("DHTxx test!"));

  dht.begin();
}

void loop() {
  delay(1000);

  float h = dht.readHumidity();
  float t = dht.readTemperature();
  float f = dht.readTemperature(true);

  if (isnan(h) || isnan(t) || isnan(f)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }

  float hif = dht.computeHeatIndex(f, h);
  float hic = dht.computeHeatIndex(t, h, false);

  Serial.print(F("Temperature: "));
  Serial.print(t);
  Serial.print(F("°C \n"));

//  Serial.print(F("Humidity: "));
//  Serial.print(h);
//  Serial.print(F("%  Temperature: "));
//  Serial.print(t);
//  Serial.print(F("°C "));
//  Serial.print(f);
//  Serial.print(F("°F  Heat index: "));
//  Serial.print(hic);
//  Serial.print(F("°C "));
//  Serial.print(hif);
//  Serial.println(F("°F"));
  delay(200);
  sensorValue = analogRead(A0);       // read analog input pin 0
  Serial.print("AirQua: ");
  Serial.print(sensorValue, DEC); 
  Serial.println(" PPM \n"); 
  delay(1000);  
}
