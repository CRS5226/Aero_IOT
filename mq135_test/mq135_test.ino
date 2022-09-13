
//////////// FOR ARDIUNO ////////////////////
int sensorValue;
void setup() {
  Serial.begin(115200);                            // sets the serial port to 115200
}
void loop() {
  sensorValue = analogRead(A0);       // read analog input pin 0
  Serial.print("AirQua=");
  Serial.print(sensorValue, DEC);               // prints the value read
  Serial.println(" PPM");
  delay(1000);                                   // wait 100ms for next reading
}

























///////////////////////////////// FOR ESP8266 /////////////////////////////////
//#include <ESP8266WiFi.h>
//String apiKey = "Z7GR4DBCLYJ5ZIZO"; // Enter your Write API key from ThingSpeak
//const char *ssid = "Dev"; // replace with your wifi ssid and wpa2 key
//const char *pass = "9619177458";
//const char* server = "api.thingspeak.com";
//WiFiClient client;
//void setup()
//{
//  Serial.begin(115200);
//  delay(10);
//  Serial.println("Connecting to ");
//  Serial.println(ssid);
//  WiFi.begin(ssid, pass);
//  while (WiFi.status() != WL_CONNECTED)
//  {
//    delay(500);
//    Serial.print(".");
//  }
//  Serial.println("");
//  Serial.println("WiFi connected");
//}
//void loop()
//{
//  float h = analogRead(A0);
//  if (isnan(h))
//  {
//    Serial.println("Failed to read from MQ-5 sensor!");
//    return;
//  }
//
//  if (client.connect(server, 80)) // "184.106.153.149" or api.thingspeak.com
//  {
//    String postStr = apiKey;
//    postStr += "&field1=";
//    postStr += String(h / 1023 * 100);
//    postStr += "r\n";
//    client.print("POST /update HTTP/1.1\n");
//    client.print("Host: api.thingspeak.com\n");
//    client.print("Connection: close\n");
//    client.print("X-THINGSPEAKAPIKEY: " + apiKey + "\n");
//    client.print("Content-Type: application/x-www-form-urlencoded\n");
//    client.print("Content-Length: ");
//    client.print(postStr.length());
//    client.print("\n\n");
//    client.print(postStr);
//
//
//    Serial.print("Gas Level: ");
//    Serial.println(h / 1023 * 100);
//    Serial.println("Data Send to Thingspeak");
//  }
//  delay(500);
//  client.stop();
//  Serial.println("Waiting...");
//
//  // thingspeak needs minimum 15 sec delay between updates.
//  delay(1500);
//}
