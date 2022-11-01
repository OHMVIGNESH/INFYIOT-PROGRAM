/*Feature list
   Mobile App 2 with return response
   Restart and factory reset
   Modbus slave and its configuration
   RTD, ThermoCouple, Flow, Analog
   NTP time
   Updated Error code for Temperature sensors (restart, n/w error, RTD & Thermocouple)
   Calibration, interval, OTA, Reset,Restart, Caution/Critical with MQTT response
   Status LED
   Modified interval function
   SSID With space fixed
   HeartBeat
   ErrorCode
   ConnectionStatus with will_Set message
*/
/*Error codes
   RTD High Threshold = 10
   RTD Low Threshold = 11
   REFIN- > 0.85 x Bias =12
   REFIN- < 0.85 x Bias - FORCE- open =13
   RTDIN- < 0.85 x Bias - FORCE- open =14
   Under/Over voltage = 15
   Exemption error = 16
   Cold Junction Range Fault = 17
   Thermocouple Range Fault = 18
   Cold Junction High Fault = 19
   Cold Junction Low Fault = 20
   Thermocouple High Fault = 21
   Thermocouple Low Fault = 22
   Over/Under Voltage Fault = 23
   Thermocouple Open Fault = 24
*/
#include <PubSubClient.h> //https://www.arduino.cc/reference/en/libraries/pubsubclient/
#include <WiFi.h>
#include <ArduinoJson.h>
#include <ESP32httpUpdate.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <Adafruit_MAX31865.h>
#include <Adafruit_MAX31856.h>
#include <EEPROM.h>
#include <HTTPClient.h>
#include <modbus.h>
#include <modbusDevice.h>
#include <modbusRegBank.h>
#include <modbusSlave.h>

//Error Code
int flag = 1, flager;

//delay
int period;
unsigned long time_now = 0;
int heatbeatperiod;
unsigned long heatbeattime_now = 0;

//Modbus
modbusDevice regBank;
modbusSlave slave;

//reset and restart
float pressLength_milliSeconds = 0;

// Define the *minimum* length of time, in milli-seconds, that the button must be pressed for a particular option to occur
int optionOne_milliSeconds = 100;
int optionTwo_milliSeconds = 2000;
int buttonState;

//FOTA version and URL
const int FW_VERSION = 2; // version of this firmware
const char* fwUrlBase = "https://165.22.208.52/"; // firmware server url

//MQTT Settings
String topicin, topicout, macadd;
char topic_in[50], topic_out[50], cid[50], broker[50], portadd[50], wifissid[50], wifipswd[50], ptp1char[50], ptp2char[50], ptp3char[50], pterc[50];
WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
int value = 0;
String textin, essid, epaswd, epssid, eppswd;
bool retain = false;
int connectflag = 1;

// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

// Variables to save date and time
String formattedDate;
String dayStamp;
String timeStamp;

//Variables
int i = 0;
bool rc;
int statusCode;
const char* ssid = "text";
const char* passphrase = "text";
String st;
String content;

// Global variable
String firstValue, sssid, secondValue, pswd, thirdvalue, brokeradd, fourthvalue, port, fifthvalue, ptp1, sixthvalue, ptp2, seventhvalue, ptp3, eigthvalue, stp1, ninthvalue, ip01;
String tenthvalue, i1ce, eleventhvalue, i1tc, tewelthvalue, i1tw, thirteenthvalue, ip02, fourteenthvalue, i2ce, fifteenthvalue, i2it,  sixteenthvalue, i2ma, seventeenthvalue, i2mi;
String eighteenthvalue, i2at, nineteenthvalue, ip03, twentethvalue, i3ce, twentyonethvalue, i3it, twentytwothvalue, i3mi, twentythreethvalue, i3ma, twentyfourthvalue, i3at;
String twentyfivthvalue, ddur, twentysixthvalue, rest;
String condition, threshold1, threshold2, typeno, esid, epass = "";
byte mac[6];

//Function Decalration
bool testWifi();
HTTPClient httpClient;

// Set web server port number to 80
WiFiServer server(80);
//Pin declaration
int resetbutton = 13, wifistatusled = 25, mqttstatusled = 26;;

// RTD sensor
// use hardware SPI, just pass in the CS pin
Adafruit_MAX31865 rtd = Adafruit_MAX31865(27);

//Thermocouple sensor
#define DRDY_PIN 5
Adafruit_MAX31856 maxthermo = Adafruit_MAX31856(4);

// The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
#define RREF      430.0
// The 'nominal' 0-degrees-C resistance of the sensor
// 100.0 for PT100, 1000.0 for PT1000
#define RNOMINAL  100.0

//Digital Flow meter
#define Digital_SENSOR  33
long currentMillis = 0;
long previousMillis = 0;
int interval = 1000;
float calibrationFactor = 1;
volatile byte pulseCount;
byte pulse1Sec = 0;
float flowRate;
unsigned int flowMilliLitres;
unsigned long totalMilliLitres;
bool checkforupdates = false;
void IRAM_ATTR pulseCounter() {
  pulseCount++;
}
void setup() {
  Serial.begin(115200);
  pinMode(resetbutton, INPUT_PULLUP);
  pinMode(mqttstatusled, OUTPUT);
  pinMode(wifistatusled, OUTPUT);
  digitalWrite(mqttstatusled, LOW);
  digitalWrite(wifistatusled, LOW);
  Serial.println("Disconnecting previously connected WiFi");
  WiFi.disconnect();
  EEPROM.begin(512); //Initialasing EEPROM
  delay(10);
  Serial.println();
  Serial.println();
  Serial.println("Startup");
  for (i = 0; i < 32; ++i)
  {
    esid += char(EEPROM.read(i));
  }
  Serial.println();
  Serial.print("SSID: ");
  Serial.println(esid);
  String sl = esid.c_str();
  Serial.println(sl.length());
  sl.replace("%20", " ");
  sl.replace("%26", "&");
  Serial.print("SSID: ");
  epssid = sl.c_str();
  Serial.println(epssid);
  delay(1000);
  Serial.println("Reading EEPROM pass");

  for (i = 32; i < 96; ++i)
  {
    epass += char(EEPROM.read(i));
  }
  Serial.print("PASS: ");
  Serial.println(epass);
  String pl = epass.c_str();
  Serial.println(pl.length());
  pl.replace("%20", " ");
  pl.replace("%26", "&");
  Serial.print("SSID: ");
  eppswd = pl.c_str();
  Serial.println(eppswd);


  Serial.print("esid.c_str(): ");
  essid = epssid.c_str();
  epaswd = eppswd.c_str();
  Serial.println(essid.length());

  if (essid.length() > 0) {
    WiFi.begin(epssid.c_str(), eppswd.c_str());
    if (testWifi()) {
      Serial.println("Succesfully Connected!!!");
      digitalWrite(wifistatusled, HIGH);
      // Initialize a NTPClient to get time
      timeClient.begin();    // GMT +1 = 3600 // GMT +8 = 28800 // GMT -1 = -3600 // GMT 0 = 0
      timeClient.setTimeOffset(0);

      for (i = 96; i < 111; ++i)
      {
        brokeradd += char(EEPROM.read(i));
      }
      Serial.print("brokeradd: ");
      Serial.println(brokeradd);

      for (i = 111; i < 116; ++i)
      {
        port += char(EEPROM.read(i));
      }
      Serial.print("port: ");
      Serial.println(port);

      for (i = 116; i < 126; ++i)
      {
        ptp1 += char(EEPROM.read(i));
      }
      Serial.print("ptp1: ");
      Serial.println(ptp1);

      for (i = 126; i < 142; ++i)
      {
        ptp2 += char(EEPROM.read(i));
      }
      Serial.print("ptp2: ");
      Serial.println(ptp2);

      for (i = 142; i < 162; ++i)
      {
        ptp3 += char(EEPROM.read(i));
      }
      Serial.print("ptp3: ");
      Serial.println(ptp3);

      for (i = 162; i < 182; ++i)
      {
        stp1 += char(EEPROM.read(i));
      }
      Serial.print("stp1: ");
      Serial.println(stp1);

      for (i = 182; i < 184; ++i)
      {
        ip01 += char(EEPROM.read(i));
      }
      Serial.print("ip01: ");
      Serial.println(ip01);

      for (i = 184; i < 188; ++i)
      {
        i1ce += char(EEPROM.read(i));
      }
      Serial.print("i1ce: ");
      Serial.println(i1ce);

      for (i = 188; i < 192; ++i)
      {
        i1tc += char(EEPROM.read(i));
      }
      Serial.print("i1tc: ");
      Serial.println(i1tc);

      for (i = 192; i < 194; ++i)
      {
        ip02 += char(EEPROM.read(i));
      }
      Serial.print("ip02: ");
      Serial.println(ip02);

      for (i = 194; i < 198; ++i)
      {
        i2ce += char(EEPROM.read(i));
      }
      Serial.print("i2ce: ");
      Serial.println(i2ce);

      for (i = 198; i < 200; ++i)
      {
        ip03 += char(EEPROM.read(i));
      }
      Serial.print("ip03: ");
      Serial.println(ip03);

      for (i = 200; i < 205; ++i)
      {
        i3ce += char(EEPROM.read(i));
      }
      Serial.print("i3ce: ");
      Serial.println(i3ce);

      for (i = 205; i < 210; ++i)
      {
        ddur += char(EEPROM.read(i));
      }
      Serial.print("ddur: ");
      Serial.println(ddur);

      for (i = 210; i < 215; ++i)
      {
        rest += char(EEPROM.read(i));
      }
      Serial.print("rest: ");
      Serial.println(rest);

      for (i = 215; i < 216; ++i)
      {
        i1tw += char(EEPROM.read(i));
      }
      Serial.print("i1tw: ");
      Serial.println(i1tw);

      for (i = 216; i < 217; ++i)
      {
        i2it += char(EEPROM.read(i));
      }
      Serial.print("i2it: ");
      Serial.println(i2it);

      for (i = 217; i < 222; ++i)
      {
        i2ma += char(EEPROM.read(i));
      }
      Serial.print("i2ma: ");
      Serial.println(i2ma);

      for (i = 222; i < 227; ++i)
      {
        i2mi += char(EEPROM.read(i));
      }
      Serial.print("i2mi: ");
      Serial.println(i2mi);

      for (i = 227; i < 233; ++i)
      {
        i2at += char(EEPROM.read(i));
      }
      Serial.print("i2at: ");
      Serial.println(i2at);

      for (i = 233; i < 235; ++i)
      {
        i3it += char(EEPROM.read(i));
      }
      Serial.print("i3it: ");
      Serial.println(i3it);

      for (i = 235; i < 240; ++i)
      {
        i3ma += char(EEPROM.read(i));
      }
      Serial.print("Read i3ma: ");
      Serial.println(i3ma);

      for (i = 240; i < 245; ++i)
      {
        i3mi += char(EEPROM.read(i));
      }
      Serial.print("Read i3mi: ");
      Serial.println(i3mi);

      for (i = 245; i < 251; ++i)
      {
        i3at += char(EEPROM.read(i));
      }
      Serial.print("i3at: ");
      Serial.println(i3at);

      topicin = stp1;      //for subscribe
      topicout = ptp1;    //for publish
      macadd = getMAC();
      macadd.toCharArray(cid, 50);
      topicin.toCharArray(topic_in, 50);
      topicout.toCharArray(topic_out, 50);
      brokeradd.toCharArray(broker, 50);
      int portadd = port.toInt();
      client.setServer(broker, portadd);
      client.setCallback(callback);
      if (i1tw == "1") {
        pinMode(DRDY_PIN, INPUT);
        if (!maxthermo.begin()) {
          Serial.println("Could not initialize thermocouple.");
        }
        maxthermo.setThermocoupleType(MAX31856_TCTYPE_K);
        Serial.print("Thermocouple type: ");
        switch (maxthermo.getThermocoupleType() ) {
          case MAX31856_TCTYPE_B: Serial.println("B Type"); break;
          case MAX31856_TCTYPE_E: Serial.println("E Type"); break;
          case MAX31856_TCTYPE_J: Serial.println("J Type"); break;
          case MAX31856_TCTYPE_K: Serial.println("K Type"); break;
          case MAX31856_TCTYPE_N: Serial.println("N Type"); break;
          case MAX31856_TCTYPE_R: Serial.println("R Type"); break;
          case MAX31856_TCTYPE_S: Serial.println("S Type"); break;
          case MAX31856_TCTYPE_T: Serial.println("T Type"); break;
          case MAX31856_VMODE_G8: Serial.println("Voltage x8 Gain mode"); break;
          case MAX31856_VMODE_G32: Serial.println("Voltage x8 Gain mode"); break;
          default: Serial.println("Unknown"); break;
        }
        maxthermo.setConversionMode(MAX31856_CONTINUOUS);
      }
      if (i1tw == "2") {
        rtd.begin(MAX31865_2WIRE);  // 2WIRE
      }
      if (i1tw == "3") {
        rtd.begin(MAX31865_3WIRE);  // 3WIRE
      }
      if (i1tw == "4") {
        rtd.begin(MAX31865_4WIRE);  //4WIRE
      }
      else {
        // do nothing
      }
      if (ip03 == "1") {
        if (i3it == "2") {
          pinMode(Digital_SENSOR, INPUT_PULLUP);
          pulseCount = 0;
          flowRate = 0.0;
          flowMilliLitres = 0;
          totalMilliLitres = 0;
          previousMillis = 0;
          attachInterrupt(digitalPinToInterrupt(Digital_SENSOR), pulseCounter, FALLING);
        }
        else {
          //do nothing
        }
      }
      else {
        //do nothing
      }

      String baudrt, slaveid;
      int budrt = 9600, machineid = 1;
      for (i = 350; i < 358; ++i)
      {
        baudrt += char(EEPROM.read(i));
      }
      Serial.print("baudrt: ");
      Serial.println(baudrt);
      budrt = baudrt.toInt();
      Serial.print("budrt: ");
      Serial.println(budrt);
      if (budrt == 0) {
        budrt = 9600;
        Serial.print("budrt: ");
        Serial.println(budrt);
      }
      else {
        budrt = baudrt.toInt();
        Serial.print("budrt: ");
        Serial.println(budrt);
      }
      Serial.print("budrt: ");
      Serial.println(budrt);
      for (i = 358; i < 362; ++i)
      {
        slaveid += char(EEPROM.read(i));
      }
      Serial.print("slaveid: ");
      Serial.println(slaveid);
      machineid = slaveid.toInt();
      if (machineid == 0) {
        machineid = 1;
        Serial.print("machineid: ");
        Serial.println(machineid);
      }
      else {
        machineid = slaveid.toInt();
        Serial.print("machineid: ");
        Serial.println(machineid);
      }
      Serial.print("machineid: ");
      Serial.println(machineid);

      regBank.setId(int(machineid));
      regBank.add(40001);
      regBank.add(40002);
      regBank.add(40003);
      slave._device = &regBank;
      slave.setBaud(int(budrt), SERIAL_8N1, 16, 17);
      return;
    }
  }
  else
  {
    apmode();
  }
}
void apmode() {
  digitalWrite(mqttstatusled, LOW);
  digitalWrite(wifistatusled, LOW);
  Serial.println("Turning the HotSpot On");
  WiFi.mode(WIFI_MODE_APSTA);
  WiFi.macAddress(mac);
  String mac = getMAC();
  String apssid = "INFYIOT" + mac;
  Serial.println(mac);
  char apssidchar[50];
  apssid.toCharArray(apssidchar, 49);
  WiFi.softAP(apssidchar, "1122334455"); // password protected ap
  IPAddress IP = WiFi.softAPIP();
  server.begin();
  while (1) {
    WiFiClient client = server.available();     // Listen for incoming clients
    httpClient.begin(client, "http://192.168.4.1/");
    String temp_clientResponse = "";            // variable to store http get buffer
    if (client) {                              // If a new client connects,
      Serial.println("New Client.");          // print a message out in the serial port
      while (client.connected()) {           // loop while the client's connected
        Serial.println("New Client connected");
        Serial.println("Trying to read data");
        temp_clientResponse = client.readStringUntil('\n');
        Serial.println(temp_clientResponse);
        if (client.available()) {
          Serial.println("New Client available.");
        }
        else {
          Serial.println("But Client is not available");
        }
        int serialresponselength = temp_clientResponse.length();
        Serial.println(serialresponselength);
        if (serialresponselength > 150) {
          int i1 = temp_clientResponse.indexOf('&');                  //index id of the splited string
          String firstValue = temp_clientResponse.substring(0, i1);  //from index id get the string as GET /?SSID=abc
          String ssidstring = firstValue.substring(6, 10);            //from GET /?SSID=abc split SSID string for validation
          Serial.println(firstValue);              //GET /?SSID=abc
          Serial.println(ssidstring);             //SSID
          if (ssidstring == "ssid") {
            Serial.println("Done");
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: Close");
            client.println("");
            int httpResponseCode = httpClient.POST("200");
            int i1 = temp_clientResponse.indexOf('&');
            int i2 = temp_clientResponse.indexOf('&', i1 + 1);
            int i3 = temp_clientResponse.indexOf('&', i2 + 1);
            int i4 = temp_clientResponse.indexOf('&', i3 + 1);
            int i5 = temp_clientResponse.indexOf('&', i4 + 1);
            int i6 = temp_clientResponse.indexOf('&', i5 + 1);
            int i7 = temp_clientResponse.indexOf('&', i6 + 1);
            int i8 = temp_clientResponse.indexOf('&', i7 + 1);
            int i9 = temp_clientResponse.indexOf('&', i8 + 1);
            int i10 = temp_clientResponse.indexOf('&', i9 + 1);
            int i11 = temp_clientResponse.indexOf('&', i10 + 1);
            int i12 = temp_clientResponse.indexOf('&', i11 + 1);
            int i13 = temp_clientResponse.indexOf('&', i12 + 1);
            int i14 = temp_clientResponse.indexOf('&', i13 + 1);
            int i15 = temp_clientResponse.indexOf('&', i14 + 1);
            int i16 = temp_clientResponse.indexOf('&', i15 + 1);
            int i17 = temp_clientResponse.indexOf('&', i16 + 1);
            int i18 = temp_clientResponse.indexOf('&', i17 + 1);
            int i19 = temp_clientResponse.indexOf('&', i18 + 1);
            int i20 = temp_clientResponse.indexOf('&', i19 + 1);
            int i21 = temp_clientResponse.indexOf('&', i20 + 1);
            int i22 = temp_clientResponse.indexOf('&', i21 + 1);
            int i23 = temp_clientResponse.indexOf('&', i22 + 1);
            int i24 = temp_clientResponse.indexOf('&', i23 + 1);
            int i25 = temp_clientResponse.indexOf('&', i24 + 1);
            int i26 = temp_clientResponse.indexOf('&', i25 + 1);
            int i27 = temp_clientResponse.indexOf('&', i26 + 1);
            firstValue = temp_clientResponse.substring(0, i1);
            sssid = firstValue.substring(11);
            secondValue = temp_clientResponse.substring(i1 + 1, i2);
            pswd = secondValue.substring(5);
            thirdvalue = temp_clientResponse.substring(i2 + 1, i3);
            brokeradd = thirdvalue.substring(5);
            fourthvalue = temp_clientResponse.substring(i3 + 1, i4);
            port = fourthvalue.substring(5);
            fifthvalue = temp_clientResponse.substring(i4 + 1, i5);
            ptp1 = fifthvalue.substring(5);
            sixthvalue = temp_clientResponse.substring(i5 + 1, i6);
            ptp2 = sixthvalue.substring(5);
            seventhvalue = temp_clientResponse.substring(i6 + 1, i7);
            ptp3 = seventhvalue.substring(5);
            eigthvalue = temp_clientResponse.substring(i7 + 1, i8);
            stp1 = eigthvalue.substring(5);
            ninthvalue = temp_clientResponse.substring(i8 + 1, i9);
            ip01 = ninthvalue.substring(5);
            tenthvalue = temp_clientResponse.substring(i9 + 1, i10);
            i1ce = tenthvalue.substring(5);
            eleventhvalue = temp_clientResponse.substring(i10 + 1, i11);
            i1tc = eleventhvalue.substring(5);
            tewelthvalue = temp_clientResponse.substring(i11 + 1, i12);
            i1tw = tewelthvalue.substring(5);
            thirteenthvalue = temp_clientResponse.substring(i12 + 1, i13);
            ip02 = thirteenthvalue.substring(5);
            fourteenthvalue = temp_clientResponse.substring(i13 + 1, i14);
            i2ce = fourteenthvalue.substring(5);
            fifteenthvalue = temp_clientResponse.substring(i14 + 1, i15);
            i2it  = fifteenthvalue.substring(5);
            sixteenthvalue = temp_clientResponse.substring(i15 + 1, i16);
            i2mi = sixteenthvalue.substring(5);
            seventeenthvalue = temp_clientResponse.substring(i16 + 1, i17);
            i2ma = seventeenthvalue.substring(5);
            eighteenthvalue = temp_clientResponse.substring(i17 + 1, i18);
            i2at = eighteenthvalue.substring(5);
            nineteenthvalue = temp_clientResponse.substring(i18 + 1, i19);
            ip03 = nineteenthvalue.substring(5);
            twentethvalue = temp_clientResponse.substring(i19 + 1, i20);
            i3ce = twentethvalue.substring(5);
            twentyonethvalue = temp_clientResponse.substring(i20 + 1, i21);
            i3it = twentyonethvalue.substring(5);
            twentytwothvalue = temp_clientResponse.substring(i21 + 1, i22);
            i3ma = twentytwothvalue.substring(5);
            twentythreethvalue = temp_clientResponse.substring(i22 + 1, i23);
            i3mi  = twentythreethvalue.substring(5);
            twentyfourthvalue = temp_clientResponse.substring(i23 + 1, i24);
            i3at = twentyfourthvalue.substring(5);
            twentyfivthvalue = temp_clientResponse.substring(i24 + 1, i25);
            ddur = twentyfivthvalue.substring(5);
            twentysixthvalue = temp_clientResponse.substring(i25 + 1, i26);
            rest = twentysixthvalue.substring(5);

            Serial.println(firstValue);
            Serial.println(sssid);
            Serial.println(secondValue);
            Serial.println(pswd);
            Serial.println(thirdvalue);
            Serial.println(brokeradd);
            Serial.println(fourthvalue);
            Serial.println(port);
            Serial.println(fifthvalue);
            Serial.println(ptp1 );
            Serial.println(sixthvalue);
            Serial.println(ptp2);
            Serial.println(seventhvalue);
            Serial.println(ptp3);
            Serial.println(eigthvalue);
            Serial.println(stp1 );
            Serial.println(ninthvalue);
            Serial.println(ip01 );
            Serial.println(tenthvalue);
            Serial.println(i1ce);
            Serial.println(eleventhvalue);
            Serial.println(i1tc);
            Serial.println(tewelthvalue);
            Serial.println(i1tw );
            Serial.println(thirteenthvalue);
            Serial.println(ip02);
            Serial.println(fourteenthvalue);
            Serial.println(i2ce);
            Serial.println(fifteenthvalue);
            Serial.println(i2it);
            Serial.println(sixteenthvalue);
            Serial.println(i2mi);
            Serial.println(seventeenthvalue);
            Serial.println(i2ma);
            Serial.println(eighteenthvalue);
            Serial.println(i2at);
            Serial.println(nineteenthvalue);
            Serial.println(ip03);
            Serial.println(twentethvalue);
            Serial.println(i3ce);
            Serial.println(twentyonethvalue);
            Serial.println(i3it);
            Serial.println(twentytwothvalue);
            Serial.println(i3ma);
            Serial.println(twentythreethvalue);
            Serial.println(i3mi);
            Serial.println(twentyfourthvalue);
            Serial.println(i3at);
            Serial.println(twentyfivthvalue);
            Serial.println(ddur);
            Serial.println(twentysixthvalue);
            Serial.println(rest);
            Serial.println("start");
            delay(5000);
            store();
            break;
          }
          else {
            Serial.println("Failed");
            client.println("HTTP/1.1 400 Failed");
            client.println("Content-type:text/html");
            client.println("Connection: keep-alive");
            client.println("");
            int httpResponseCode = httpClient.POST("400");
          }
        }
        if (serialresponselength < 150 || serialresponselength == 150) {
          Serial.println("Failed");
          client.println("HTTP/1.1 400 Failed");
          client.println("Content-type:text/html");
          client.println("Connection: keep-alive");
          client.println("");
          int httpResponseCode = httpClient.POST("400");
        }
      }
    }
  }
}
/**to get the mac id of the ESP to set as a topic for mqtt***/
String getMAC()
{
  uint8_t mac[6];
  String result;
  WiFi.macAddress(mac);
  //result = String(mac);
  //snprintf( result, sizeof( result ), "%02x%02x%02x%02x%02x%02x", mac[ 0 ], mac[ 1 ], mac[ 2 ], mac[ 3 ], mac[ 4 ], mac[ 5 ] );
  result = String(mac[0], HEX) + String(mac[1], HEX) + String(mac[2], HEX) + String(mac[3], HEX) + String(mac[4], HEX) + String(mac[5], HEX);
  return result;
}
void store() {
  Serial.println(sssid);
  Serial.println(pswd);
  if (sssid.length() > 0 && pswd.length()) {
    Serial.println("Clearing EEPROM");
    for (i = 0; i < 96; ++i) {
      EEPROM.write(i, 0);
    }
    Serial.println("Writing EEPROM ssid");
    for (i = 0; i < sssid.length() ; ++i) {
      EEPROM.write(i, sssid[i]);
      Serial.println("wrote");
      Serial.println(sssid[i]);
    }
    Serial.println("Writing EEPROM pswd");
    for (i = 0; i < pswd.length() ; ++i) {
      EEPROM.write(32 + i, pswd[i]);
      Serial.println("wrote");
      Serial.println(pswd[i]);
    }
    Serial.println("Writing EEPROM mqtt server name");
    for (i = 0; i < brokeradd.length() ; ++i) {
      EEPROM.write(96 + i, brokeradd[i]);
      Serial.println("wrote");
      Serial.println(brokeradd[i]);
    }
    Serial.println("Writing EEPROM mqtt port");
    for (i = 0; i < port.length() ; ++i) {
      EEPROM.write(111 + i, port[i]);
      Serial.println("wrote");
      Serial.println(port[i]);
    }
    Serial.println("Writing EEPROM mqtt ptp1");
    for (i = 0; i < ptp1.length() ; ++i) {
      EEPROM.write(116 + i, ptp1[i]);
      Serial.println("wrote");
      Serial.println(ptp1[i]);
    }
    Serial.println("Writing EEPROM mqtt ptp2");
    for (i = 0; i < ptp2.length() ; ++i) {
      EEPROM.write(126 + i, ptp2[i]);
      Serial.println("wrote");
      Serial.println(ptp2[i]);
    }
    Serial.println("Writing EEPROM mqtt ptp3");
    for (i = 0; i < ptp3.length() ; ++i) {
      EEPROM.write(142 + i, ptp3[i]);
      Serial.println("wrote");
      Serial.println(ptp3[i]);
    }
    Serial.println("Writing EEPROM mqtt stp1");
    for (i = 0; i < stp1.length() ; ++i) {
      EEPROM.write(162 + i, stp1[i]);
      Serial.println("wrote");
      Serial.println(stp1[i]);
    }
    Serial.println("Writing EEPROM mqtt ip01");
    for (i = 0; i < ip01.length() ; ++i) {
      EEPROM.write(182 + i, ip01[i]);
      Serial.println("wrote");
      Serial.println(ip01[i]);
    }
    Serial.println("Writing EEPROM mqtt i1ce");
    for (i = 0; i < i1ce.length() ; ++i) {
      EEPROM.write(184 + i, i1ce[i]);
      Serial.println("wrote");
      Serial.println(i1ce[i]);
    }
    Serial.println("Writing EEPROM mqtt i1tc");
    for (i = 0; i < i1tc.length() ; ++i) {
      EEPROM.write(188 + i, i1tc[i]);
      Serial.println("wrote");
      Serial.println(i1tc[i]);
    }
    Serial.println("Writing EEPROM mqtt ip02");
    for (i = 0; i < ip02.length() ; ++i) {
      EEPROM.write(192 + i, ip02[i]);
      Serial.println("wrote");
      Serial.println(ip02[i]);
    }
    Serial.println("Writing EEPROM mqtt i2ce");
    for (i = 0; i < i2ce.length() ; ++i) {
      EEPROM.write(194 + i, i2ce[i]);
      Serial.println("wrote");
      Serial.println(i2ce[i]);
    }
    Serial.println("Writing EEPROM mqtt ip03");
    for (i = 0; i < i3ce.length() ; ++i) {
      EEPROM.write(198 + i, ip03[i]);
      Serial.println("wrote");
      Serial.println(ip03[i]);
    }
    Serial.println("Writing EEPROM mqtt i3ce");
    for (i = 0; i < i3ce.length() ; ++i) {
      EEPROM.write(200 + i, i3ce[i]);
      Serial.println("wrote");
      Serial.println(i3ce[i]);
    }
    Serial.println("Writing EEPROM mqtt ddur");
    for (i = 0; i < ddur.length() ; ++i) {
      EEPROM.write(205 + i, ddur[i]);
      Serial.println("wrote");
      Serial.println(ddur[i]);
    }
    Serial.println("Writing EEPROM mqtt rest");
    for (i = 0; i < rest.length() ; ++i) {
      EEPROM.write(210 + i, rest[i]);
      Serial.println("wrote");
      Serial.println(rest[i]);
    }
    Serial.println("Writing EEPROM mqtt i1tw");
    for (i = 0; i < i1tw.length() ; ++i) {
      EEPROM.write(215 + i, i1tw[i]);
      Serial.println("wrote");
      Serial.println(i1tw[i]);
    }
    Serial.println("Writing EEPROM mqtt i2it");
    for (i = 0; i < i2it.length() ; ++i) {
      EEPROM.write(216 + i, i2it[i]);
      Serial.println("wrote");
      Serial.println(i2it[i]);
    }
    Serial.println("Writing EEPROM mqtt i2ma");
    for (i = 0; i < i2ma.length() ; ++i) {
      EEPROM.write(217 + i, i2ma[i]);
      Serial.println("wrote");
      Serial.println(i2ma[i]);
    }
    Serial.println("Writing EEPROM mqtt i2mi");
    for (i = 0; i < i2mi.length() ; ++i) {
      EEPROM.write(222 + i, i2mi[i]);
      Serial.println("wrote");
      Serial.println(i2mi[i]);
    }
    Serial.println("Writing EEPROM mqtt i2at");
    for (i = 0; i < i2at.length() ; ++i) {
      EEPROM.write(227 + i, i2at[i]);
      Serial.println("wrote");
      Serial.println(i2at[i]);
    }
    Serial.println("Writing EEPROM mqtt i3it");
    for (i = 0; i < i3it.length() ; ++i) {
      EEPROM.write(233 + i, i3it[i]);
      Serial.println("wrote");
      Serial.println(i3it[i]);
    }
    Serial.println("Writing EEPROM mqtt i3ma");
    for (i = 0; i < i3ma.length() ; ++i) {
      EEPROM.write(235 + i, i3ma[i]);
      Serial.println("wrote");
      Serial.println(i3ma[i]);
    }
    Serial.println("Writing EEPROM mqtt i3mi");
    for (i = 0; i < i3mi.length() ; ++i) {
      EEPROM.write(240 + i, i3mi[i]);
      Serial.println("wrote");
      Serial.println(i3mi[i]);
    }
    Serial.println("Writing EEPROM mqtt i3at");
    for (i = 0; i < i3at.length() ; ++i) {
      EEPROM.write(245 + i, i3at[i]);
      Serial.println("wrote");
      Serial.println(i3at[i]);
    }
    EEPROM.commit();
    delay(500);
    ESP.restart();
  }
}
void callback(char* topic, byte* message, unsigned int length) {
  for (i = 0; i < length; i++) {
    textin += (char)message[i];
  }
  textin = textin;
  Serial.println(textin);
  //delay(10000);
  StaticJsonDocument <900> subdata;
  deserializeJson(subdata, message);
  String  datamacid = subdata["mac"];
  int datacondition = subdata["data"]["measurement_condition"]; //1 => ON, 2 => less_than_or_equal, 3 => greater_than_or_equal, 4 => Range
  int alerttypeid = subdata["data"]["alerttypeid"];  // caution (1) or critical (2)
  int inputid = subdata["inputid"]; // 1 for RTD, 2 for pressure, 3 for flow
  int categ = subdata["data"]["op"]; // 1 - Add, 2 - Delete, 3- Update
  int cmdtype = subdata["cmdtypeid"]; //  1 => Calibaration Command, 2 => Interval Command, 3 => Reboot Command, 4 => Alert Command, 5 => OTA Command
  //6 => reset Command
  int cmdidr = subdata["cmdid"];
  Serial.println(datamacid);
  Serial.println(datacondition);
  Serial.println(alerttypeid);
  Serial.println(inputid);
  Serial.println(categ);
  Serial.println(cmdtype);
  delay(5000);
  if (macadd == datamacid) {
    Serial.println("mac passed");
    if (cmdtype == 4) {
      Serial.println("======================================");
      Serial.println(cmdtype);
      if (inputid == 1) {
        Serial.println(inputid);
        int altypet = subdata["data"]["alerttypeid"];
        Serial.println(altypet);
        if (altypet == 1) {
          Serial.println("Caution Add/Update");
          if (categ == 1 || categ == 3) {
            Serial.println(categ);
            Serial.println("======================================");
            Serial.println("Clearing EEPROM");
            for (int i = 251; i < 262; ++i) {
              EEPROM.write(i, 0);
            }
            int condi = subdata["data"]["measurement_condition"];
            condition = String(condi);
            Serial.println("Writing EEPROM mqtt datacondition");
            for (int i = 0; i < 252 ; ++i) {
              EEPROM.write(251 + i, condition[i]);
              //Serial.println("wrote");
              //Serial.println(condition[i]);
            }
            int thres1 = subdata["data"]["threshold1"];
            threshold1 = String(thres1);
            Serial.println("Writing EEPROM mqtt threshold1");
            for (int i = 0; i < threshold1.length() ; ++i) {
              EEPROM.write(252 + i, threshold1[i]);
              //Serial.println("wrote");
              //Serial.println(threshold1[i]);
            }
            int thres2 = subdata["data"]["threshold2"];
            threshold2 = String(thres2);
            Serial.println("Writing EEPROM mqtt threshold12");
            for (int i = 0; i < threshold2.length() ; ++i) {
              EEPROM.write(257 + i, threshold2[i]);
              //Serial.println("wrote");
              //Serial.println(threshold2[i]);
            }
            int typenumb = subdata["data"]["alerttypeid"];
            typeno = String(typenumb);
            Serial.println("Writing EEPROM mqtt alerttypeid");
            for (int i = 0; i < 263; ++i) {
              EEPROM.write(262 + i, typeno[i]);
              //Serial.println("wrote");
              //Serial.println(typeno[i]);
            }
            EEPROM.commit();
            delay(500);
          }
          if (categ == 2) {
            Serial.println("Caution Delete");
            Serial.println("Clearing EEPROM");
            for (int i = 251; i < 263; ++i) {
              EEPROM.write(i, 0);
            }
            EEPROM.commit();
            delay(500);
          }
          else {
            //Do nothing
          }
        }
        if (altypet == 2) {
          if (categ == 1 || categ == 3) {
            Serial.println("Critical Add/Update");
            Serial.println(categ);
            Serial.println("======================================");
            Serial.println("Clearing EEPROM");
            for (int i = 263; i < 275; ++i) {
              EEPROM.write(i, 0);
            }
            int condi = subdata["data"]["measurement_condition"];
            condition = String(condi);
            Serial.println("Writing EEPROM mqtt datacondition");
            for (int i = 0; i < 264 ; ++i) {
              EEPROM.write(263 + i, condition[i]);
              //Serial.println("wrote");
              //Serial.println(condition[i]);
            }
            int thres1 = subdata["data"]["threshold1"];
            threshold1 = String(thres1);
            Serial.println("Writing EEPROM mqtt threshold1");
            for (int i = 0; i < threshold1.length() ; ++i) {
              EEPROM.write(264 + i, threshold1[i]);
              //Serial.println("wrote");
              //Serial.println(threshold1[i]);
            }
            int thres2 = subdata["data"]["threshold2"];
            threshold2 = String(thres2);
            Serial.println("Writing EEPROM mqtt threshold12");
            for (int i = 0; i < threshold2.length() ; ++i) {
              EEPROM.write(269 + i, threshold2[i]);
              //Serial.println("wrote");
              //Serial.println(threshold2[i]);
            }
            int typenumb = subdata["data"]["alerttypeid"];
            typeno = String(typenumb);
            Serial.println("Writing EEPROM mqtt alerttypeid");
            for (int i = 0; i < 275; ++i) {
              EEPROM.write(274 + i, typeno[i]);
              //Serial.println("wrote");
              //Serial.println(typeno[i]);
            }
            EEPROM.commit();
            delay(500);
          }
          if (categ == 2) {
            Serial.println("Critical Delete");
            Serial.println("Clearing EEPROM");
            for (int i = 263; i < 275; ++i) {
              EEPROM.write(i, 0);
            }
            EEPROM.commit();
            delay(500);
          }
          else {
            //Do nothing
          }
        }
      }
      if (inputid == 2) {
        Serial.println(inputid);
        int altypet = subdata["data"]["alerttypeid"];
        Serial.println(altypet);
        if (altypet == 1) {
          Serial.println("Caution Add/Update");
          if (categ == 1 || categ == 3) {
            Serial.println(categ);
            Serial.println("======================================");
            Serial.println("Clearing EEPROM");
            for (int i = 275; i < 287; ++i) {
              EEPROM.write(i, 0);
            }
            int condi = subdata["data"]["measurement_condition"];
            condition = String(condi);
            Serial.println("Writing EEPROM mqtt datacondition");
            for (int i = 0; i < 276 ; ++i) {
              EEPROM.write(275 + i, condition[i]);
              //Serial.println("wrote");
              //Serial.println(condition[i]);
            }
            int thres1 = subdata["data"]["threshold1"];
            threshold1 = String(thres1);
            Serial.println("Writing EEPROM mqtt threshold1");
            for (int i = 0; i < threshold1.length() ; ++i) {
              EEPROM.write(276 + i, threshold1[i]);
              //Serial.println("wrote");
              //Serial.println(threshold1[i]);
            }
            int thres2 = subdata["data"]["threshold2"];
            threshold2 = String(thres2);
            Serial.println("Writing EEPROM mqtt threshold12");
            for (int i = 0; i < threshold2.length() ; ++i) {
              EEPROM.write(281 + i, threshold2[i]);
              //Serial.println("wrote");
              //Serial.println(threshold2[i]);
            }
            int typenumb = subdata["data"]["alerttypeid"];
            typeno = String(typenumb);
            Serial.println("Writing EEPROM mqtt alerttypeid");
            for (int i = 0; i < 287; ++i) {
              EEPROM.write(286 + i, typeno[i]);
              //Serial.println("wrote");
              //Serial.println(typeno[i]);
            }
            EEPROM.commit();
            delay(500);
          }
          if (categ == 2) {
            Serial.println("Caution Delete");
            Serial.println("Clearing EEPROM");
            for (int i = 275; i < 287; ++i) {
              EEPROM.write(i, 0);
            }
            EEPROM.commit();
            delay(500);
          }
          else {
            //Do nothing
          }
        }
        if (altypet == 2) {
          if (categ == 1 || categ == 3) {
            Serial.println("Critical Add/Update");
            Serial.println(categ);
            Serial.println("======================================");
            Serial.println("Clearing EEPROM");
            for (int i = 287; i < 299; ++i) {
              EEPROM.write(i, 0);
            }
            int condi = subdata["data"]["measurement_condition"];
            condition = String(condi);
            Serial.println("Writing EEPROM mqtt datacondition");
            for (int i = 0; i < 288 ; ++i) {
              EEPROM.write(287 + i, condition[i]);
              //Serial.println("wrote");
              //Serial.println(condition[i]);
            }
            int thres1 = subdata["data"]["threshold1"];
            threshold1 = String(thres1);
            Serial.println("Writing EEPROM mqtt threshold1");
            for (int i = 0; i < threshold1.length() ; ++i) {
              EEPROM.write(288 + i, threshold1[i]);
              //Serial.println("wrote");
              //Serial.println(threshold1[i]);
            }
            int thres2 = subdata["data"]["threshold2"];
            threshold2 = String(thres2);
            Serial.println("Writing EEPROM mqtt threshold12");
            for (int i = 0; i < threshold2.length() ; ++i) {
              EEPROM.write(293 + i, threshold2[i]);
              //Serial.println("wrote");
              //Serial.println(threshold2[i]);
            }
            int typenumb = subdata["data"]["alerttypeid"];
            typeno = String(typenumb);
            Serial.println("Writing EEPROM mqtt alerttypeid");
            for (int i = 0; i < 299; ++i) {
              EEPROM.write(298 + i, typeno[i]);
              //Serial.println("wrote");
              //Serial.println(typeno[i]);
            }
            EEPROM.commit();
            delay(500);
          }
          if (categ == 2) {
            Serial.println("Critical Delete");
            Serial.println("Clearing EEPROM");
            for (int i = 287; i < 299; ++i) {
              EEPROM.write(i, 0);
            }
            EEPROM.commit();
            delay(500);
          }
          else {
            //Do nothing
          }
        }
      }
      if (inputid == 3) {
        Serial.println(inputid);
        int altypet = subdata["data"]["alerttypeid"];
        Serial.println(altypet);
        if (altypet == 1) {
          Serial.println("Caution Add/Update");
          if (categ == 1 || categ == 3) {
            Serial.println(categ);
            Serial.println("======================================");
            Serial.println("Clearing EEPROM");
            for (int i = 299; i < 311; ++i) {
              EEPROM.write(i, 0);
            }
            int condi = subdata["data"]["measurement_condition"];
            condition = String(condi);
            Serial.println("Writing EEPROM mqtt datacondition");
            for (int i = 0; i < 300 ; ++i) {
              EEPROM.write(299 + i, condition[i]);
              //Serial.println("wrote");
              //Serial.println(condition[i]);
            }
            int thres1 = subdata["data"]["threshold1"];
            threshold1 = String(thres1);
            Serial.println("Writing EEPROM mqtt threshold1");
            for (int i = 0; i < threshold1.length() ; ++i) {
              EEPROM.write(300 + i, threshold1[i]);
              //Serial.println("wrote");
              //Serial.println(threshold1[i]);
            }
            int thres2 = subdata["data"]["threshold2"];
            threshold2 = String(thres2);
            Serial.println("Writing EEPROM mqtt threshold12");
            for (int i = 0; i < threshold2.length() ; ++i) {
              EEPROM.write(305 + i, threshold2[i]);
              //Serial.println("wrote");
              //Serial.println(threshold2[i]);
            }
            int typenumb = subdata["data"]["alerttypeid"];
            typeno = String(typenumb);
            Serial.println("Writing EEPROM mqtt alerttypeid");
            for (int i = 0; i < 311; ++i) {
              EEPROM.write(310 + i, typeno[i]);
              //Serial.println("wrote");
              //Serial.println(typeno[i]);
            }
            EEPROM.commit();
            delay(500);
          }
          if (categ == 2) {
            Serial.println("Caution Delete");
            Serial.println("Clearing EEPROM");
            for (int i = 299; i < 311; ++i) {
              EEPROM.write(i, 0);
            }
            EEPROM.commit();
            delay(500);
          }
          else {
            //Do nothing
          }
        }
        if (altypet == 2) {
          if (categ == 1 || categ == 3) {
            Serial.println("Critical Add/Update");
            Serial.println(categ);
            Serial.println("======================================");
            Serial.println("Clearing EEPROM");
            for (int i = 311; i < 323; ++i) {
              EEPROM.write(i, 0);
            }
            int condi = subdata["data"]["measurement_condition"];
            condition = String(condi);
            Serial.println("Writing EEPROM mqtt datacondition");
            for (int i = 0; i < 312 ; ++i) {
              EEPROM.write(311 + i, condition[i]);
              //Serial.println("wrote");
              //Serial.println(condition[i]);
            }
            int thres1 = subdata["data"]["threshold1"];
            threshold1 = String(thres1);
            Serial.println("Writing EEPROM mqtt threshold1");
            for (int i = 0; i < threshold1.length() ; ++i) {
              EEPROM.write(312 + i, threshold1[i]);
              //Serial.println("wrote");
              //Serial.println(threshold1[i]);
            }
            int thres2 = subdata["data"]["threshold2"];
            threshold2 = String(thres2);
            Serial.println("Writing EEPROM mqtt threshold12");
            for (int i = 0; i < threshold2.length() ; ++i) {
              EEPROM.write(317 + i, threshold2[i]);
              //Serial.println("wrote");
              //Serial.println(threshold2[i]);
            }
            int typenumb = subdata["data"]["alerttypeid"];
            typeno = String(typenumb);
            Serial.println("Writing EEPROM mqtt alerttypeid");
            for (int i = 0; i < 323; ++i) {
              EEPROM.write(322 + i, typeno[i]);
              //Serial.println("wrote");
              //Serial.println(typeno[i]);
            }
            EEPROM.commit();
            delay(500);
          }
          if (categ == 2) {
            Serial.println("Critical Delete");
            Serial.println("Clearing EEPROM");
            for (int i = 311; i < 323; ++i) {
              EEPROM.write(i, 0);
            }
            EEPROM.commit();
            delay(500);
          }
          else {
            //Do nothing
          }
        }
      }
      StaticJsonDocument<900> retdoc;
      JsonObject JSONencoder = retdoc.to<JsonObject>();
      JSONencoder["mac"] = macadd;
      JSONencoder["cmdid"] = cmdidr; //cmdidr
      JSONencoder["cmdtypeid"] = cmdtype;
      JSONencoder["code"] = 200;
      JSONencoder["message"] = "success";
      char buffer[900];
      serializeJson(JSONencoder, buffer);
      //Serial.println(buffer);
      ptp3.toCharArray(ptp3char, 50);
      boolean rc = client.publish(ptp3char, buffer, 2);
    }

    if (cmdtype == 1) {
      if (inputid == 1) {
        float multifac = subdata["data"]["coef_value"];
        String MF = String(multifac);
        Serial.println("Received value");
        Serial.println(MF);
        Serial.println("Clearing EEPROM");
        for (i = 184; i < 188; ++i) {
          EEPROM.write(i, 0);
        }
        Serial.println("Writing EEPROM mqtt i1ce");
        for (i = 0; i < MF.length() ; ++i) {
          EEPROM.write(184 + i, MF[i]);
          Serial.println("wrote");
          Serial.println(MF[i]);
        }
        EEPROM.commit();
        delay(1000);
      }
      if (inputid == 2) {
        float multifac = subdata["data"]["coef_value"];
        String MF = String(multifac);
        Serial.println("Received value");
        Serial.println(MF);
        Serial.println("Clearing EEPROM");
        for (i = 194; i < 198; ++i) {
          EEPROM.write(i, 0);
        }
        for (i = 0; i < MF.length() ; ++i) {
          EEPROM.write(194 + i, MF[i]);
          Serial.println("wrote");
          Serial.println(MF[i]);
        }
        EEPROM.commit();
        delay(1000);
      }
      if (inputid == 3) {
        float multifac = subdata["data"]["coef_value"];
        String MF = String(multifac);
        Serial.println("Received value");
        Serial.println(MF);
        Serial.println("Clearing EEPROM");
        for (i = 200; i < 205; ++i) {
          EEPROM.write(i, 0);
        }
        for (i = 0; i < MF.length() ; ++i) {
          EEPROM.write(200 + i, MF[i]);
          Serial.println("wrote");
          Serial.println(MF[i]);
        }
        EEPROM.commit();
        delay(1000);
      }
      StaticJsonDocument<150> retdoc;
      JsonObject JSONencoder = retdoc.to<JsonObject>();
      JSONencoder["mac"] = macadd;
      JSONencoder["cmdid"] = cmdidr;
      JSONencoder["cmdtypeid"] = cmdtype;
      JSONencoder["code"] = 200;
      JSONencoder["message"] = "success";
      char buffer[150];
      serializeJson(JSONencoder, buffer);
      Serial.println(buffer);
      ptp3.toCharArray(ptp3char, 50);
      rc = client.publish(ptp3char, buffer, retain);
    }
    if (cmdtype == 2) {
      String cmdddur = subdata["data"]["interval"];
      Serial.println(cmdddur);
      delay(1000);
      Serial.println("Clearing EEPROM");
      for (i = 205; i < 210; ++i) {
        EEPROM.write(i, 0);
      }
      Serial.println("Writing EEPROM mqtt ddur");
      for (i = 0; i < cmdddur.length() ; ++i) {
        EEPROM.write(205 + i, cmdddur[i]);
        Serial.println("wrote");
        Serial.println(cmdddur[i]);
      }
      EEPROM.commit();
      delay(500);
      StaticJsonDocument<150> retdoc;
      JsonObject JSONencoder = retdoc.to<JsonObject>();
      JSONencoder["mac"] = macadd;
      JSONencoder["cmdid"] = cmdidr;
      JSONencoder["cmdtypeid"] = cmdtype;
      JSONencoder["code"] = 200;
      JSONencoder["message"] = "success";
      char buffer[150];
      serializeJson(JSONencoder, buffer);
      Serial.println(buffer);
      ptp3.toCharArray(ptp3char, 50);
      rc = client.publish(ptp3char, buffer, retain);
    }
    if (cmdtype == 3) {
      StaticJsonDocument<150> retdoc;
      JsonObject JSONencoder = retdoc.to<JsonObject>();
      JSONencoder["mac"] = macadd;
      JSONencoder["cmdid"] = cmdidr;
      JSONencoder["cmdtypeid"] = cmdtype;
      JSONencoder["code"] = 200;
      JSONencoder["message"] = "success";
      char buffer[900];
      serializeJson(JSONencoder, buffer);
      Serial.println(buffer);
      ptp3.toCharArray(ptp3char, 50);
      rc = client.publish(ptp3char, buffer, retain);
      delay(500);
      formattedDate = timeClient.getFormattedDate();
      StaticJsonDocument<150> doc2;
      JsonObject JSONencoder2 = doc2.to<JsonObject>();
      JSONencoder2["mac"] = macadd;
      JSONencoder2["datetime"] = formattedDate;
      JSONencoder2["connstatus"] = 2;
      JSONencoder2["reason"] = "Disconnected by software-restart";
      char buffer2[150];
      serializeJson(JSONencoder2, buffer2);
      Serial.println(buffer2);
      String tpec = "ConnectionStatus";
      tpec.toCharArray(pterc, 50);
      rc = client.publish(pterc, buffer2, retain);
      delay(1000);
      ESP.restart();
    }
    if (cmdtype == 5) {
      StaticJsonDocument<150> retdoc;
      JsonObject JSONencoder = retdoc.to<JsonObject>();
      JSONencoder["mac"] = macadd;
      JSONencoder["cmdid"] = cmdidr;
      JSONencoder["cmdtypeid"] = cmdtype;
      JSONencoder["code"] = 200;
      JSONencoder["message"] = "success";
      char buffer[150];
      serializeJson(JSONencoder, buffer);
      Serial.println(buffer);
      ptp3.toCharArray(ptp3char, 50);
      rc = client.publish(ptp3char, buffer, retain);
      delay(1000);
      /**this line helps to call the the http update check function***/
      //checkForUpdates();
    checkforupdates = true;
    return;
    }
    if (cmdtype == 6) {
      StaticJsonDocument<150> retdoc;
      JsonObject JSONencoder = retdoc.to<JsonObject>();
      JSONencoder["mac"] = macadd;
      JSONencoder["cmdid"] = cmdidr;
      JSONencoder["cmdtypeid"] = cmdtype;
      JSONencoder["code"] = 200;
      JSONencoder["message"] = "success";
      char buffer[150];
      serializeJson(JSONencoder, buffer);
      Serial.println(buffer);
      ptp3.toCharArray(ptp3char, 50);
      rc = client.publish(ptp3char, buffer, retain);
      delay(1000);
      formattedDate = timeClient.getFormattedDate();
      StaticJsonDocument<150> doc2;
      JsonObject JSONencoder2 = doc2.to<JsonObject>();
      JSONencoder2["mac"] = macadd;
      JSONencoder2["datetime"] = formattedDate;
      JSONencoder2["connstatus"] = 2;
      JSONencoder2["reason"] = "Disconnected by software-reset";
      char buffer2[150];
      serializeJson(JSONencoder2, buffer2);
      Serial.println(buffer2);
      String tpec = "ConnectionStatus";
      tpec.toCharArray(pterc, 50);
      rc = client.publish(pterc, buffer2, retain);
      delay(1000);
      devicereset();
    }
    if (cmdtype == 7) {
      int slv = subdata["data"]["slave"];
      int bd = subdata["data"]["baudrate"];
      String sve = String(slv);
      String baud = String(bd);
      Serial.println("Received value");
      Serial.println(sve);
      Serial.println(baud);
      Serial.println("Clearing EEPROM");
      for (i = 350; i < 362; ++i) {
        EEPROM.write(i, 0);
      }
      for (i = 0; i < baud.length() ; ++i) {
        EEPROM.write(350 + i, baud[i]);
        Serial.println("wrote");
        Serial.println(baud[i]);
      }
      for (i = 0; i < sve.length() ; ++i) {
        EEPROM.write(358 + i, sve[i]);
        Serial.println("wrote");
        Serial.println(sve[i]);
      }
      EEPROM.commit();
      delay(1000);
      StaticJsonDocument<150> retdoc;
      JsonObject JSONencoder = retdoc.to<JsonObject>();
      JSONencoder["mac"] = macadd;
      JSONencoder["cmdid"] = cmdidr;
      JSONencoder["cmdtypeid"] = cmdtype;
      JSONencoder["code"] = 200;
      JSONencoder["message"] = "success";
      char buffer[150];
      serializeJson(JSONencoder, buffer);
      Serial.println(buffer);
      ptp3.toCharArray(ptp3char, 50);
      rc = client.publish(ptp3char, buffer, retain);
      delay(1000);
      ESP.restart();
    }
    if (cmdtype == 8) {
      String master_slave = subdata["data"]["slave"];
      String master_baudrate = subdata["data"]["baudrate"];
      String master_parity = subdata["data"]["parity"];
      String master_tagid1 = subdata["data"]["tags"][0]["tagId"];
      String master_tagid2 = subdata["data"]["tags"][1]["tagId"];
      String master_tagid3 = subdata["data"]["tags"][2]["tagId"];
      Serial.println(master_slave);
      Serial.println(master_baudrate);
      Serial.println(master_parity);
      Serial.println(master_tagid1);
      Serial.println(master_tagid2);
      Serial.println(master_tagid3);
      delay(2000);
      Serial.println("Clearing EEPROM");
      for (int i = 401; i < 440; ++i) {
        EEPROM.write(i, 0);
      }
      Serial.println("Writing EEPROM master_slave");
      for (int i = 0; i < master_slave.length() ; ++i) {
        EEPROM.write(401 + i, master_slave[i]);
        Serial.println("wrote");
        Serial.println(master_slave[i]);
      }
      Serial.println("Writing EEPROM master_baudrate");
      for (int i = 0; i < master_baudrate.length() ; ++i) {
        EEPROM.write(405 + i, master_baudrate[i]);
        Serial.println("wrote");
        Serial.println(master_baudrate[i]);
      }
      Serial.println("Writing EEPROM master_parity");
      for (int i = 0; i < master_parity.length() ; ++i) {
        EEPROM.write(412 + i, master_parity[i]);
        Serial.println("wrote");
        Serial.println(master_parity[i]);
      }
      Serial.println("Writing EEPROM master_tagid1");
      for (int i = 0; i < master_tagid1.length() ; ++i) {
        EEPROM.write(415 + i, master_tagid1[i]);
        Serial.println("wrote");
        Serial.println(master_tagid1[i]);
      }
      Serial.println("Writing EEPROM master_tagid2");
      for (int i = 0; i < master_tagid2.length() ; ++i) {
        EEPROM.write(423 + i, master_tagid2[i]);
        Serial.println("wrote");
        Serial.println(master_tagid2[i]);
      }
      Serial.println("Writing EEPROM master_tagid3");
      for (int i = 0; i < master_tagid3.length() ; ++i) {
        EEPROM.write(430 + i, master_tagid3[i]);
        Serial.println("wrote");
        Serial.println(master_tagid3[i]);
      }
      EEPROM.commit();
      delay(500);
      StaticJsonDocument<100> retdoc;
      JsonObject JSONencoder = retdoc.to<JsonObject>();
      JSONencoder["mac"] = macadd;
      JSONencoder["cmdid"] = cmdidr;
      JSONencoder["cmdtypeid"] = cmdtype;
      JSONencoder["code"] = 200;
      JSONencoder["message"] = "success";
      char buffer[100];
      serializeJson(JSONencoder, buffer);
      Serial.println(buffer);
      ptp3.toCharArray(ptp3char, 50);
      boolean rc = client.publish(ptp3char, buffer, retain);      
      delay(1000);
      ESP.restart();
    }
    if (cmdtype == 9) {
      String cmdddur = subdata["data"]["interval"];
      delay(1000);
      for (i = 365; i < 370; ++i) {
        EEPROM.write(i, 0);
      }
      for (i = 0; i < cmdddur.length() ; ++i) {
        EEPROM.write(365 + i, cmdddur[i]);
      }
      EEPROM.commit();
      delay(500);
      StaticJsonDocument<150> retdoc;
      JsonObject JSONencoder = retdoc.to<JsonObject>();
      JSONencoder["mac"] = macadd;
      JSONencoder["cmdid"] = cmdidr;
      JSONencoder["cmdtypeid"] = cmdtype;
      JSONencoder["code"] = 200;
      JSONencoder["message"] = "success";
      char buffer[150];
      serializeJson(JSONencoder, buffer);
      ptp3.toCharArray(ptp3char, 50);
      rc = client.publish(ptp3char, buffer, retain);
    }
  }
}
void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    digitalWrite(mqttstatusled, LOW);
    formattedDate = timeClient.getFormattedDate();
    StaticJsonDocument<120> doc2;
    JsonObject JSONencoder2 = doc2.to<JsonObject>();
    JSONencoder2["mac"] = macadd;
    JSONencoder2["connstatus"] = 3;
    JSONencoder2["reason"] = "Ungracefully disconnected ";
    char buffer2[120];
    serializeJson(JSONencoder2, buffer2);
    Serial.println(buffer2);
    flager = 1;
    if (client.connect(cid, "ConnectionStatus", 2, false, buffer2)) {
      Serial.println("connected");
      digitalWrite(mqttstatusled, HIGH);
      Serial.println(topic_in);
      client.subscribe(topic_in);
      flager = 1;
      loop();
      break;
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      flager = 1;
      Serial.println(" try again in 5 seconds");
      delay(5);
    }
  }
  connectflag = 1;
}
void loop() {
  if (WiFi.status() == WL_CONNECTED) {
    if (!client.connected()) {
      reconnect();
    }
    client.loop();
  if(checkforupdates)
  {
    checkForUpdates();
  }
    while (!timeClient.update()) {
      timeClient.forceUpdate();
    }
    formattedDate = timeClient.getFormattedDate();
    if (connectflag == 1) {
      Serial.println("Send Connected.........................................");
      StaticJsonDocument<100> doc2;
      JsonObject JSONencoder2 = doc2.to<JsonObject>();
      JSONencoder2["mac"] = macadd;
      JSONencoder2["datetime"] = formattedDate;
      JSONencoder2["connstatus"] = 1;
      JSONencoder2["reason"] = "connected";
      char buffer2[100];
      serializeJson(JSONencoder2, buffer2);
      String tpec = "ConnectionStatus";
      tpec.toCharArray(pterc, 50);
      rc = client.publish(pterc, buffer2, retain);
      delay(1000);
      connectflag = 0;
    }
    if (flag == 1) {
      StaticJsonDocument<150> retdoc;
      JsonObject JSONencoder = retdoc.to<JsonObject>();
      JSONencoder["mac"] = macadd;
      JSONencoder["datetime"] = formattedDate;
      JSONencoder["inputid"] = 0;
      JSONencoder["errcode"] = 25;
      char buffer[150];
      serializeJson(JSONencoder, buffer);
      Serial.println(buffer);
      String tpec = "ErrorCode";
      tpec.toCharArray(pterc, 50);
      rc = client.publish(pterc, buffer, retain);
      flag = 0;
    }
    if (flager == 1) {
      StaticJsonDocument<150> retdoc;
      JsonObject JSONencoder = retdoc.to<JsonObject>();
      JSONencoder["mac"] = macadd;
      JSONencoder["datetime"] = formattedDate;
      JSONencoder["inputid"] = 0;
      JSONencoder["errcode"] = 26;
      char buffer[150];
      serializeJson(JSONencoder, buffer);
      Serial.println(buffer);
      String tpec = "ErrorCode";
      tpec.toCharArray(pterc, 50);
      rc = client.publish(pterc, buffer, retain);
      flager = 0;
    }
    if (flager == 0) {
      String tere = "";
      String prre = "";
      String flre = "";
      for (i = 330; i < 335; ++i)
      {
        tere += char(EEPROM.read(i));
      }
      for (i = 335; i < 340; ++i)
      {
        prre += char(EEPROM.read(i));
      }
      for (i = 340; i < 345; ++i)
      {
        flre += char(EEPROM.read(i));
      }
      int Te = tere.toInt();
      float Pr = prre.toFloat();
      float Fl = flre.toFloat();
      StaticJsonDocument<200> retdoc;
      JsonObject JSONencoder = retdoc.to<JsonObject>();
      JSONencoder["mac"] = macadd;
      JSONencoder["timestamp"] = formattedDate;
      JSONencoder["last_temperature"] = Te;
      JSONencoder["last_pressure"] = Pr;
      JSONencoder["last_flow"] = Fl;
      char buffer[200];
      serializeJson(JSONencoder, buffer);
      Serial.println(buffer);
      String tpec = "retaindata";
      tpec.toCharArray(pterc, 50);
      rc = client.publish(pterc, buffer, retain);
      flager = 2;
    }
    buttonState = digitalRead(resetbutton);
    Serial.print("buttonState = ");
    Serial.println(buttonState);
    while (buttonState == LOW ) {
      delay(100);  //if you want more resolution, lower this number
      buttonState = digitalRead(resetbutton);
      Serial.print("buttonState = ");
      Serial.println(buttonState);
      pressLength_milliSeconds = pressLength_milliSeconds + 100;
      //display how long button is has been held
      Serial.print("ms = ");
      Serial.println(pressLength_milliSeconds);
    }
    if (pressLength_milliSeconds >= optionTwo_milliSeconds) {
      Serial.println("Reset");
      formattedDate = timeClient.getFormattedDate();
      StaticJsonDocument<200> doc2;
      JsonObject JSONencoder2 = doc2.to<JsonObject>();
      JSONencoder2["mac"] = macadd;
      JSONencoder2["datetime"] = formattedDate;
      JSONencoder2["connstatus"] = 2;
      JSONencoder2["reason"] = "Disconnected due to hardware reset";
      char buffer2[200];
      serializeJson(JSONencoder2, buffer2);
      Serial.println(buffer2);
      String tpec = "ConnectionStatus";
      tpec.toCharArray(pterc, 50);
      rc = client.publish(pterc, buffer2, retain);
      delay(500);
      devicereset();
    }
    else if (pressLength_milliSeconds >= optionOne_milliSeconds) {
      Serial.println("Restart");
      formattedDate = timeClient.getFormattedDate();
      StaticJsonDocument<200> doc2;
      JsonObject JSONencoder2 = doc2.to<JsonObject>();
      JSONencoder2["mac"] = macadd;
      JSONencoder2["datetime"] = formattedDate;
      JSONencoder2["connstatus"] = 2;
      JSONencoder2["reason"] = "Disconnected due to hardware restart";
      char buffer2[200];
      serializeJson(JSONencoder2, buffer2);
      Serial.println(buffer2);
      String tpec = "ConnectionStatus";
      tpec.toCharArray(pterc, 50);
      rc = client.publish(pterc, buffer2, retain);
      delay(500);
      ESP.restart();
    }
    pressLength_milliSeconds = 0;
    int tempresult = 0;
    float presresult = 0;
    float flowresult = 0;
    int temperror = 0;
    if (ip01 == "1") {
      String ipo1mf;
      for (i = 184; i < 188; ++i)
      {
        ipo1mf += char(EEPROM.read(i));
      }
      //Serial.println("MFTemp: ");
      //Serial.println(ipo1mf);
      float MFTemp = ipo1mf.toFloat();
      //Serial.println("*******************************************");
      //Serial.println(MFTemp);
      if (i1tc == "1") {
        int count = 0;
        while (digitalRead(DRDY_PIN)) {
          if (count++ > 200) {
            count = 0;
            Serial.print(".");
          }
        }
        int thermo = maxthermo.readThermocoupleTemperature();
        uint8_t maxthermofault = maxthermo.readFault();
        if (maxthermofault) {
          Serial.print("maxthermofault 0x"); Serial.println(maxthermofault, HEX);
          tempresult = 0;
          if (maxthermofault & MAX31856_FAULT_CJRANGE) {
            temperror = 17;
            Serial.println("Cold Junction Range Fault");
          }
          if (maxthermofault & MAX31856_FAULT_TCRANGE) {
            temperror = 18;
            Serial.println("Thermocouple Range Fault");
          }
          if (maxthermofault & MAX31856_FAULT_CJHIGH) {
            temperror = 19;
            Serial.println("Cold Junction High Fault");
          }
          if (maxthermofault & MAX31856_FAULT_CJLOW) {
            temperror = 20;
            Serial.println("Cold Junction Low Fault");
          }
          if (maxthermofault & MAX31856_FAULT_TCHIGH) {
            temperror = 21;
            Serial.println("Thermocouple High Fault");
          }
          if (maxthermofault & MAX31856_FAULT_TCLOW) {
            temperror = 22;
            Serial.println("Thermocouple Low Fault");
          }
          if (maxthermofault & MAX31856_FAULT_OVUV) {
            temperror = 23;
            Serial.println("Over/Under Voltage Fault");
          }
          if (maxthermofault & MAX31856_FAULT_OPEN) {
            temperror = 24;
            Serial.println("Thermocouple Open Fault");
          }
        }
        else {
          tempresult = thermo * MFTemp;
          temperror = 0;
          if (tempresult < -10) {
            tempresult = thermo * MFTemp;
            temperror = 16;
          }
        }
      }
      if (i1tc == "100" || i1tc == "1000") {
        uint16_t rtdvalue = rtd.readRTD();
        float ratio = rtdvalue;
        ratio /= 32768;
        int rtdte = rtd.temperature(RNOMINAL, RREF);
        uint8_t fault = rtd.readFault();
        //Serial.println(fault);
        if (fault) {
          Serial.print("Fault 0x"); Serial.println(fault, HEX);
          rtd.clearFault();
          tempresult = 0;
          if (fault & MAX31865_FAULT_HIGHTHRESH) {
            Serial.println("RTD High Threshold");
            temperror = 10;
          }
          if (fault & MAX31865_FAULT_LOWTHRESH) {
            Serial.println("RTD Low Threshold");
            temperror = 11;
          }
          if (fault & MAX31865_FAULT_REFINLOW) {
            Serial.println("REFIN- > 0.85 x Bias");
            temperror = 12;
          }
          if (fault & MAX31865_FAULT_REFINHIGH) {
            Serial.println("REFIN- < 0.85 x Bias - FORCE- open");
            temperror = 13;
          }
          if (fault & MAX31865_FAULT_RTDINLOW) {
            Serial.println("RTDIN- < 0.85 x Bias - FORCE- open");
            temperror = 14;
          }
          if (fault & MAX31865_FAULT_OVUV) {
            Serial.println("Under/Over voltage");
            temperror = 15;
          }
          rtd.clearFault();
        }
        else {
          tempresult = rtdte * MFTemp;
          temperror = 0;
          if (tempresult < -10) {
            tempresult = 0;
            temperror = 16;
          }
        }
      }
      Serial.println("********TEMPERATURE Alert DATA**********");
      Serial.println("********TEMPERATURE Alert Caution**********");
      String alerttypeno = "";
      for (i = 262; i < 263; ++i)
      {
        alerttypeno += char(EEPROM.read(i));
      }
      Serial.print("alerttypeno: ");
      Serial.println(alerttypeno);
      int alerttypenumber1 = alerttypeno.toInt();
      Serial.print("alerttypenumber1: ");
      Serial.println(alerttypenumber1);

      Serial.println("********TEMPERATURE Alert Critical**********");
      String alerttypenotemp = "";
      for (i = 274; i < 275; ++i)
      {
        alerttypenotemp += char(EEPROM.read(i));
      }
      Serial.print("alerttypenotemp: ");
      Serial.println(alerttypenotemp);
      int alerttypenumber2 = alerttypenotemp.toInt();

      Serial.print("alerttypenumber2: ");
      Serial.println(alerttypenumber2);

      if (alerttypenumber1 == 1) {
        Serial.println("*********************TEMPERATURE Alert Caution**********************");
        String alertcondition = "";
        String alertthreshold1 = "";
        String alertthreshold2 = "";
        String alerttypeno = "";
        for (i = 251; i < 252; ++i)
        {
          alertcondition += char(EEPROM.read(i));
        }
        for (i = 252; i < 257; ++i)
        {
          alertthreshold1 += char(EEPROM.read(i));
        }
        for (i = 257; i < 262; ++i)
        {
          alertthreshold2 += char(EEPROM.read(i));
        }
        for (i = 262; i < 263; ++i)
        {
          alerttypeno += char(EEPROM.read(i));
        }
        Serial.println("Temperature Caution");
        Serial.print("alertcondition: ");
        Serial.println(alertcondition);
        Serial.print("alertthreshold1: ");
        Serial.println(alertthreshold1);
        Serial.print("alertthreshold2: ");
        Serial.println(alertthreshold2);
        Serial.print("alerttypeno: ");
        Serial.print(alerttypeno);

        float alertthres1 = alertthreshold1.toFloat();
        float alertthres2 = alertthreshold2.toFloat();
        int alerttypenumber = alerttypeno.toInt();
        if (alertcondition == "1") {
          if (alertthres1 == tempresult) {
            StaticJsonDocument<200> retalert;
            JsonObject JSONencoder = retalert.to<JsonObject>();
            JSONencoder["mac"] = macadd;
            JSONencoder["ipid"] = 1;
            JSONencoder["value"] = tempresult;
            JSONencoder["datetime"] = formattedDate;
            JSONencoder["alerttypeid"] = alerttypenumber;
            char buffer[200];
            serializeJson(JSONencoder, buffer);
            Serial.println(buffer);
            ptp2.toCharArray(ptp2char, 50);
            rc = client.publish(ptp2char, buffer, retain);
          }
        }
        if (alertcondition == "2") {
          Serial.println("alertcondition 2");
          bool trial = (alertthres1 <= tempresult);
          Serial.println(trial);
          if (alertthres1 >= tempresult) {
            StaticJsonDocument<200> retalert;
            JsonObject JSONencoder = retalert.to<JsonObject>();
            JSONencoder["mac"] = macadd;
            JSONencoder["ipid"] = 1;
            JSONencoder["value"] = tempresult;
            JSONencoder["datetime"] = formattedDate;
            JSONencoder["alerttypeid"] = alerttypenumber;
            char buffer[200];
            serializeJson(JSONencoder, buffer);
            Serial.println(buffer);
            ptp2.toCharArray(ptp2char, 50);
            rc = client.publish(ptp2char, buffer, retain);
          }
        }
        if (alertcondition == "3") {
          if (alertthres1 <= tempresult) {
            StaticJsonDocument<200> retalert;
            JsonObject JSONencoder = retalert.to<JsonObject>();
            JSONencoder["mac"] = macadd;
            JSONencoder["ipid"] = 1;
            JSONencoder["value"] = tempresult;
            JSONencoder["datetime"] = formattedDate;
            JSONencoder["alerttypeid"] = alerttypenumber;
            char buffer[200];
            serializeJson(JSONencoder, buffer);
            Serial.println(buffer);
            ptp2.toCharArray(ptp2char, 50);
            rc = client.publish(ptp2char, buffer, retain);
          }
        }
        if (alertcondition == "4") {
          Serial.println("alertcondition 4");
          bool trial = (tempresult >= alertthres1 && tempresult <= alertthres2);
          Serial.println(trial);
          if (tempresult >= alertthres1 && tempresult <= alertthres2) {
            StaticJsonDocument<200> retalert;
            JsonObject JSONencoder = retalert.to<JsonObject>();
            JSONencoder["mac"] = macadd;
            JSONencoder["ipid"] = 1;
            JSONencoder["value"] = tempresult;
            JSONencoder["datetime"] = formattedDate;
            JSONencoder["alerttypeid"] = alerttypenumber;
            char buffer[200];
            serializeJson(JSONencoder, buffer);
            Serial.println(buffer);
            ptp2.toCharArray(ptp2char, 50);
            rc = client.publish(ptp2char, buffer, retain);
          }
        }
      }
      if (alerttypenumber2 == 2) {
        Serial.println("*********************TEMPERATURE Alert Critical**********************");
        String alertcondition1 = "";
        String alertthreshold11 = "";
        String alertthreshold21 = "";
        String alerttypeno1 = "";
        for (i = 263; i < 264; ++i)
        {
          alertcondition1 += char(EEPROM.read(i));
        }
        for (i = 264; i < 269; ++i)
        {
          alertthreshold11 += char(EEPROM.read(i));
        }
        for (i = 269; i < 274; ++i)
        {
          alertthreshold21 += char(EEPROM.read(i));
        }
        for (i = 299; i < 301; ++i)
        {
          alerttypeno1 += char(EEPROM.read(i));
        }
        Serial.println("Temperature Critical");
        Serial.print("alertcondition1: ");
        Serial.println(alertcondition1);
        Serial.print("alertthreshold11: ");
        Serial.println(alertthreshold11);
        Serial.print("alertthreshold21: ");
        Serial.println(alertthreshold21);
        Serial.print("alerttypeno1: ");
        Serial.println(alerttypeno1);

        float alertthres11 = alertthreshold11.toFloat();
        float alertthres21 = alertthreshold21.toFloat();
        int alerttypenumber1 = alerttypeno1.toInt();
        Serial.print("alertthres11: ");
        Serial.println(alertthres11);
        Serial.print("alertthres21: ");
        Serial.println(alertthres21);
        Serial.print("alerttypenumber1: ");
        Serial.println(alerttypenumber1);
        Serial.println("____________________________________________");
        if (alertcondition1 == "1") {
          if (alertthres11 == tempresult) {
            StaticJsonDocument<200> retalert;
            JsonObject JSONencoder = retalert.to<JsonObject>();
            JSONencoder["mac"] = macadd;
            JSONencoder["ipid"] = 1;
            JSONencoder["value"] = tempresult;
            JSONencoder["datetime"] = formattedDate;
            JSONencoder["alerttypeid"] = alerttypenumber1;
            char buffer[200];
            serializeJson(JSONencoder, buffer);
            Serial.println(buffer);
            ptp2.toCharArray(ptp2char, 50);
            rc = client.publish(ptp2char, buffer, retain);
          }
        }
        if (alertcondition1 == "2") {
          Serial.println("alertcondition 2");
          bool trial = (alertthres11 <= tempresult);
          Serial.println(trial);
          if (alertthres11 >= tempresult) {
            StaticJsonDocument<200> retalert;
            JsonObject JSONencoder = retalert.to<JsonObject>();
            JSONencoder["mac"] = macadd;
            JSONencoder["ipid"] = 1;
            JSONencoder["value"] = tempresult;
            JSONencoder["datetime"] = formattedDate;
            JSONencoder["alerttypeid"] = alerttypenumber1;
            char buffer[200];
            serializeJson(JSONencoder, buffer);
            Serial.println(buffer);
            ptp2.toCharArray(ptp2char, 50);
            rc = client.publish(ptp2char, buffer, retain);
          }
        }
        if (alertcondition1 == "3") {
          if (alertthres11 <= tempresult) {
            StaticJsonDocument<200> retalert;
            JsonObject JSONencoder = retalert.to<JsonObject>();
            JSONencoder["mac"] = macadd;
            JSONencoder["ipid"] = 1;
            JSONencoder["value"] = tempresult;
            JSONencoder["datetime"] = formattedDate;
            JSONencoder["alerttypeid"] = alerttypenumber1;
            char buffer[200];
            serializeJson(JSONencoder, buffer);
            Serial.println(buffer);
            ptp2.toCharArray(ptp2char, 50);
            rc = client.publish(ptp2char, buffer, retain);
          }
        }
        if (alertcondition1 == "4") {
          Serial.println("alertcondition 4");
          bool trial = (tempresult >= alertthres11 && tempresult <= alertthres21);
          Serial.println(trial);
          if (tempresult >= alertthres11 && tempresult <= alertthres21) {
            StaticJsonDocument<200> retalert;
            JsonObject JSONencoder = retalert.to<JsonObject>();
            JSONencoder["mac"] = macadd;
            JSONencoder["ipid"] = 1;
            JSONencoder["value"] = tempresult;
            JSONencoder["datetime"] = formattedDate;
            JSONencoder["alerttypeid"] = alerttypenumber1;
            char buffer[200];
            serializeJson(JSONencoder, buffer);
            Serial.println(buffer);
            ptp2.toCharArray(ptp2char, 50);
            rc = client.publish(ptp2char, buffer, retain);
          }
        }
      }
    }
    if (ip01 == "0") {
      tempresult = 0;
      temperror = 0;
    }
    if (ip02 == "1") {
      String i2coeff;
      for (i = 194; i < 198; ++i)
      {
        i2coeff += char(EEPROM.read(i));
      }
      //Serial.println("MFPresMFPres: ");
      //Serial.println(i2coeff);
      float MFPres = i2coeff.toFloat();
      Serial.println("*******************************************");
      //Serial.println(MFPres);
      if (i2it == "1") {
        float presmax = i2mi.toInt();
        float presmin = i2ma.toInt();

        //Serial.print("presmax");
        //Serial.println(presmax);
        //Serial.print("presmin");
        //Serial.println(presmin);
        int sensorValue = 0;
        float vge = 0;
        float voltage;
        if (i2at == "1") {
          for (int i = 0; i < 10; i++) {
            sensorValue = sensorValue + analogRead(Digital_SENSOR);
            voltage = sensorValue * (3.3 / 4095.0);
            vge = vge + voltage;
          }
          vge = vge / 10;
          int deciconv = float(vge) * 100;
          float vge = deciconv / 100;
          //float pressure = map(voltage, 0, 3.3, presmin, presmax);
          float OldMin = 0;
          float OldMax = 3.3;
          float OldRange = (OldMax - OldMin);
          float NewRange = (presmax - presmin);
          //float pressure = (((vge - OldMin) * NewRange) / OldRange) + presmin;
          float pressure = ( (vge -  OldMin) / (OldMax - OldMin) ) * (presmax - presmin) + presmin;
          // float pressure = vge;
          presresult = MFPres * pressure;


        }
        if (i2at == "2") {
          vge = 0;
          for (int i = 0 ; i < 10; i++) {
            float vo = analogRead(33) * (3.3 / 4095.0);
            vge = vge + vo;
          }
          vge = vge / 10;
          //int deciconv = float(vge) * 100;
          //float vge = deciconv / 100;
          //Serial.println("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&");
          //Serial.println(analogRead(Digital_SENSOR));
          //Serial.println(vge);
          //Serial.println(deciconv);
          //Serial.println(vge1);
          //Serial.println("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&");
          float OldMin = 0;
          float OldMax = 3.3;
          float OldRange = (OldMax - OldMin);
          float NewRange = (presmax - presmin);
          //float pressure = (((vge - OldMin) * NewRange) / OldRange) + presmin;
          float pressure = ( (vge -  OldMin) / (OldMax - OldMin) ) * (presmax - presmin) + presmin;
          // float pressure = vge;
          presresult = MFPres * pressure;
          // presresult = (presresult, 2);
        }
        if (i2at == "3") {
          vge = 0;
          for (int i = 0 ; i < 10; i++) {
            float vo = analogRead(33) * (3.3 / 4095.0);
            vge = vge + vo;
          }
          vge = vge / 10;
          //int deciconv = float(vge) * 100;
          //float vge = deciconv / 100;
          //Serial.println("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&");
          //Serial.println(analogRead(Digital_SENSOR));
          //Serial.println(vge);
          //Serial.println(deciconv);
          //Serial.println(vge1);
          //Serial.println("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&");
          //float pressure = map(current, 0, 20, presmin, presmax);
          float OldMin = 0;
          float OldMax = 3.3;
          float OldRange = (OldMax - OldMin);
          float NewRange = (presmax - presmin);
          //float pressure = (((vge - OldMin) * NewRange) / OldRange) + presmin;
          float pressure = ( (vge -  OldMin) / (OldMax - OldMin) ) * (presmax - presmin) + presmin;
          // float pressure = vge;
          presresult = MFPres * pressure;
          // presresult = (presresult, 2);
        }
        if (i2at == "4") {
          vge = 0;
          for (int i = 0 ; i < 10; i++) {
            float vo = analogRead(33) * (3.3 / 4095.0);
            vge = vge + vo;
          }
          vge = vge / 10;
          //int deciconv = float(vge) * 100;
          //float vge = deciconv / 100;
          //Serial.println("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&");
          //Serial.println(analogRead(Digital_SENSOR));
          //Serial.println(vge);
          //Serial.println(deciconv);
          //Serial.println(vge1);
          //Serial.println("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&");
          //float pressure = map(current, 4, 20, presmin, presmax);
          float OldMin = 0;
          float OldMax = 3.3;
          float OldRange = (OldMax - OldMin);
          float NewRange = (presmax - presmin);
          //float pressure = (((vge - OldMin) * NewRange) / OldRange) + presmin;
          float pressure = ( (vge -  OldMin) / (OldMax - OldMin) ) * (presmax - presmin) + presmin;
          // float pressure = vge;
          presresult = MFPres * pressure;
          // presresult = (presresult, 2);
        }
        else {
          //do nothing
        }
      }
      else {
        //do nothing
      }
      Serial.println("*********************PRESSURE Alert DATA**********************");
      Serial.println("*********************PRESSURE Alert Caution**********************");
      String alerttypenopressure  = "";
      for (int i = 286; i < 287; ++i)
      {
        alerttypenopressure += char(EEPROM.read(i));
      }
      Serial.print("alerttypenopressure: ");
      Serial.println(alerttypenopressure);
      int alerttypenumberpress = alerttypenopressure.toInt();
      Serial.print("alerttypenumberpress: ");
      Serial.println(alerttypenumberpress);

      Serial.println("*********************PRESSURE Alert Critical**********************");
      String alerttypenopressure1  = "";
      for (int i = 298; i < 299; ++i)
      {
        alerttypenopressure1 += char(EEPROM.read(i));
      }
      Serial.print("alerttypenopressure1: ");
      Serial.println(alerttypenopressure1);
      int alerttypenumberpress1 = alerttypenopressure1.toInt();
      Serial.print("alerttypenumberpress1: ");
      Serial.println(alerttypenumberpress1);
      if (alerttypenumberpress == 1) {
        Serial.println("*********************PRESSURE Alert Caution**********************");
        String alertconditionpressure = "";
        String alertthreshold1pressure = "";
        String alertthreshold2pressure = "";
        String alerttypenopressure  = "";
        for (int i = 275; i < 276; ++i)
        {
          alertconditionpressure += char(EEPROM.read(i));
        }
        for (int i = 276; i < 281; ++i)
        {
          alertthreshold1pressure += char(EEPROM.read(i));
        }
        for (int i = 281; i < 286; ++i)
        {
          alertthreshold2pressure += char(EEPROM.read(i));
        }
        for (int i = 286; i < 287; ++i)
        {
          alerttypenopressure += char(EEPROM.read(i));
        }

        Serial.print("alertconditionpressure: ");
        Serial.println(alertconditionpressure);
        Serial.print("alertthreshold1pressure: ");
        Serial.println(alertthreshold1pressure);
        Serial.print("alertthreshold2pressure: ");
        Serial.println(alertthreshold2pressure);
        Serial.print("alerttypenopressure: ");
        Serial.println(alerttypenopressure);

        float alertthres1press = alertthreshold1pressure.toFloat();
        float alertthres2press = alertthreshold2pressure.toFloat();
        if (alertconditionpressure == "1") {
          int alerttypenumberpress = alerttypenopressure.toInt();
          if (presresult == alertthres1press) {
            StaticJsonDocument<900> retalert;
            JsonObject JSONencoder = retalert.to<JsonObject>();
            JSONencoder["mac"] = macadd;
            JSONencoder["ipid"] = 2;
            JSONencoder["value"] = presresult;
            JSONencoder["datetime"] = formattedDate;
            JSONencoder["alerttypeid"] = alerttypenumberpress;
            char buffer[900];
            serializeJson(JSONencoder, buffer);
            Serial.println(buffer);
            ptp2.toCharArray(ptp2char, 50);
            boolean rc = client.publish(ptp2char, buffer, retain);
          }
        }
        if (alertconditionpressure == "2") {
          Serial.println("alertcondition 2");
          bool trial = (alertthres1press <= tempresult);
          Serial.println(trial);
          if (alertthres1press >= presresult) {
            StaticJsonDocument<900> retalert;
            JsonObject JSONencoder = retalert.to<JsonObject>();
            JSONencoder["mac"] = macadd;
            JSONencoder["ipid"] = 2;
            JSONencoder["value"] = presresult;
            JSONencoder["datetime"] = formattedDate;
            JSONencoder["alerttypeid"] = alerttypenumberpress;
            char buffer[900];
            serializeJson(JSONencoder, buffer);
            Serial.println(buffer);
            ptp2.toCharArray(ptp2char, 50);
            boolean rc = client.publish(ptp2char, buffer, retain);
          }
        }
        if (alertconditionpressure == "3") {
          if (alertthres1press <= presresult) {
            StaticJsonDocument<900> retalert;
            JsonObject JSONencoder = retalert.to<JsonObject>();
            JSONencoder["mac"] = macadd;
            JSONencoder["ipid"] = 2;
            JSONencoder["value"] = presresult;
            JSONencoder["datetime"] = formattedDate;
            JSONencoder["alerttypeid"] = alerttypenumberpress;
            char buffer[900];
            serializeJson(JSONencoder, buffer);
            Serial.println(buffer);
            ptp2.toCharArray(ptp2char, 50);
            boolean rc = client.publish(ptp2char, buffer, retain);
          }
        }
        if (alertconditionpressure == "4") {
          Serial.println("alertcondition 4");
          bool trial = (presresult >= alertthres1press && presresult <= alertthres2press);
          Serial.println(trial);
          if (presresult >= alertthres1press && presresult <= alertthres2press) {
            StaticJsonDocument<900> retalert;
            JsonObject JSONencoder = retalert.to<JsonObject>();
            JSONencoder["mac"] = macadd;
            JSONencoder["ipid"] = 2;
            JSONencoder["value"] = presresult;
            JSONencoder["datetime"] = formattedDate;
            JSONencoder["alerttypeid"] = alerttypenumberpress;
            char buffer[900];
            serializeJson(JSONencoder, buffer);
            Serial.println(buffer);
            ptp2.toCharArray(ptp2char, 50);
            boolean rc = client.publish(ptp2char, buffer, retain);
          }
        }
      }
      if (alerttypenumberpress1 == 2) {
        Serial.println("*********************PRESSURE Alert Critical**********************");
        String alertconditionpressure = "";
        String alertthreshold1pressure = "";
        String alertthreshold2pressure = "";
        String alerttypenopressure  = "";
        for (int i = 287; i < 288; ++i)
        {
          alertconditionpressure += char(EEPROM.read(i));
        }
        for (int i = 288; i < 293; ++i)
        {
          alertthreshold1pressure += char(EEPROM.read(i));
        }
        for (int i = 293; i < 298; ++i)
        {
          alertthreshold2pressure += char(EEPROM.read(i));
        }
        for (int i = 298; i < 299; ++i)
        {
          alerttypenopressure += char(EEPROM.read(i));
        }

        Serial.print("alertconditionpressure: ");
        Serial.println(alertconditionpressure);
        Serial.print("alertthreshold1pressure: ");
        Serial.println(alertthreshold1pressure);
        Serial.print("alertthreshold2pressure: ");
        Serial.println(alertthreshold2pressure);
        Serial.print("alerttypenopressure: ");
        Serial.println(alerttypenopressure);

        float alertthres1press = alertthreshold1pressure.toFloat();
        float alertthres2press = alertthreshold2pressure.toFloat();
        int alerttypenumberpress = alerttypenopressure.toInt();
        if (alertconditionpressure == "1") {
          if (presresult == alertthres1press) {
            StaticJsonDocument<900> retalert;
            JsonObject JSONencoder = retalert.to<JsonObject>();
            JSONencoder["mac"] = macadd;
            JSONencoder["ipid"] = 2;
            JSONencoder["value"] = presresult;
            JSONencoder["datetime"] = formattedDate;
            JSONencoder["alerttypeid"] = alerttypenumberpress;
            char buffer[900];
            serializeJson(JSONencoder, buffer);
            Serial.println(buffer);
            ptp2.toCharArray(ptp2char, 50);
            boolean rc = client.publish(ptp2char, buffer, retain);
          }
        }
        if (alertconditionpressure == "2") {
          Serial.println("alertcondition 2");
          bool trial = (alertthres1press <= tempresult);
          Serial.println(trial);
          if (alertthres1press >= presresult) {
            StaticJsonDocument<900> retalert;
            JsonObject JSONencoder = retalert.to<JsonObject>();
            JSONencoder["mac"] = macadd;
            JSONencoder["ipid"] = 2;
            JSONencoder["value"] = presresult;
            JSONencoder["datetime"] = formattedDate;
            JSONencoder["alerttypeid"] = alerttypenumberpress;
            char buffer[900];
            serializeJson(JSONencoder, buffer);
            Serial.println(buffer);
            ptp2.toCharArray(ptp2char, 50);
            boolean rc = client.publish(ptp2char, buffer, retain);
          }
        }
        if (alertconditionpressure == "3") {
          if (alertthres1press <= presresult) {
            StaticJsonDocument<900> retalert;
            JsonObject JSONencoder = retalert.to<JsonObject>();
            JSONencoder["mac"] = macadd;
            JSONencoder["ipid"] = 2;
            JSONencoder["value"] = presresult;
            JSONencoder["datetime"] = formattedDate;
            JSONencoder["alerttypeid"] = alerttypenumberpress;
            char buffer[900];
            serializeJson(JSONencoder, buffer);
            Serial.println(buffer);
            ptp2.toCharArray(ptp2char, 50);
            boolean rc = client.publish(ptp2char, buffer, retain);
          }
        }
        if (alertconditionpressure == "4") {
          Serial.println("alertcondition 4");
          bool trial = (presresult >= alertthres1press && presresult <= alertthres2press);
          Serial.println(trial);
          if (presresult >= alertthres1press && presresult <= alertthres2press) {
            StaticJsonDocument<900> retalert;
            JsonObject JSONencoder = retalert.to<JsonObject>();
            JSONencoder["mac"] = macadd;
            JSONencoder["ipid"] = 2;
            JSONencoder["value"] = presresult;
            JSONencoder["datetime"] = formattedDate;
            JSONencoder["alerttypeid"] = alerttypenumberpress;
            char buffer[900];
            serializeJson(JSONencoder, buffer);
            Serial.println(buffer);
            ptp2.toCharArray(ptp2char, 50);
            boolean rc = client.publish(ptp2char, buffer, retain);
          }
        }
      }
    }
    if (ip02 == "0") {
      presresult = 0;
    }
    if (ip03 == "1") {
      String i3coeff = "";
      for (i = 200; i < 205; ++i)
      {
        i3coeff += char(EEPROM.read(i));
      }
      Serial.println("MFFlow: ");
      Serial.println(i3coeff);
      float MFFlow = i3coeff.toFloat();
      //Serial.println("*******************************************");
      //Serial.println(MFFlow);
      if (i3it == "1") {
        float flowmax = i3ma.toInt();
        float flowmin = i3mi.toInt();
        Serial.print("flowmax");
        Serial.println(flowmax);
        Serial.print("flowmin");
        Serial.println(flowmin);
        int sensorValue = 0;
        float voltage;
        float vge;
        if (i3at == "1") {
          vge = 0;
          for (int i = 0 ; i < 10; i++) {
            float vo = analogRead(33) * (3.3 / 4095.0);
            vge = vge + vo;
          }
          vge = vge / 10;
          //int deciconv = float(vge) * 100;
          //float vge = deciconv / 100;
          Serial.println("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&");
          Serial.println(analogRead(Digital_SENSOR));
          Serial.println(vge);
          //Serial.println(deciconv);
          //Serial.println(vge1);
          Serial.println("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&");
          float OldMin = 0;
          float OldMax = 3.3;
          float OldRange = (OldMax - OldMin);
          float NewRange = (flowmax - flowmin);
          //float flowRate = (((vge - OldMin) * NewRange) / OldRange) + flowmin;
          float flowRate = ( (vge -  OldMin) / (OldMax - OldMin) ) * (flowmax - flowmin) + flowmin;
          //flowresult = vge;
          flowresult = flowRate * MFFlow;

        }
        if (i3at == "2") {
          vge = 0;
          for (int i = 0 ; i < 10; i++) {
            float vo = analogRead(33) * (3.3 / 4095.0);
            vge = vge + vo;
          }
          vge = vge / 10;
          //int deciconv = float(vge) * 100;
          //float vge = deciconv / 100;
          Serial.println("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&");
          Serial.println(analogRead(Digital_SENSOR));
          Serial.println(vge);
          //Serial.println(deciconv);
          //Serial.println(vge1);
          Serial.println("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&");
          float OldMin = 0;
          float OldMax = 3.3;
          float OldRange = (OldMax - OldMin);
          float NewRange = (flowmax - flowmin);
          //float flowRate = (((vge - OldMin) * NewRange) / OldRange) + flowmin;
          float flowRate = ( (vge -  OldMin) / (OldMax - OldMin) ) * (flowmax - flowmin) + flowmin;
          //flowresult = vge;
          flowresult = flowRate * MFFlow;

        }
        if (i3at == "3") {
          vge = 0;
          for (int i = 0 ; i < 10; i++) {
            float vo = analogRead(33) * (3.3 / 4095.0);
            vge = vge + vo;
          }
          vge = vge / 10;
          //int deciconv = float(vge) * 100;
          //float vge = deciconv / 100;
          Serial.println("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&");
          Serial.println(analogRead(Digital_SENSOR));
          Serial.println(vge);
          //Serial.println(deciconv);
          //Serial.println(vge1);
          Serial.println("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&");
          float OldMin = 0;
          float OldMax = 3.3;
          float OldRange = (OldMax - OldMin);
          float NewRange = (flowmax - flowmin);
          //float flowRate = (((vge - OldMin) * NewRange) / OldRange) + flowmin;
          float flowRate = ( (vge -  OldMin) / (OldMax - OldMin) ) * (flowmax - flowmin) + flowmin;
          //flowresult = vge;
          flowresult = flowRate * MFFlow;

        }
        if (i3at == "4") {
          vge = 0;
          for (int i = 0 ; i < 10; i++) {
            float vo = analogRead(33) * (3.3 / 4095.0);
            vge = vge + vo;
          }
          vge = vge / 10;
          //int deciconv = float(vge) * 100;
          //float vge = deciconv / 100;
          Serial.println("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&");
          Serial.println(analogRead(Digital_SENSOR));
          Serial.println(vge);
          //Serial.println(deciconv);
          //Serial.println(vge1);
          Serial.println("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&");
          float OldMin = 0;
          float OldMax = 3.3;
          float OldRange = (OldMax - OldMin);
          float NewRange = (flowmax - flowmin);
          float flowRate = (((vge - OldMin) * NewRange) / OldRange) + flowmin;
          flowresult = flowRate * MFFlow;
          Serial.println("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&");
          Serial.println(flowmin);
          Serial.println(flowmax);
          Serial.println(flowRate);
          Serial.println(flowresult);
          Serial.println("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&");
        }
        else {
          //do nothing
        }
      }
      if (i3it == "2") {
        currentMillis = millis();
        if (currentMillis - previousMillis > interval) {
          pulse1Sec = pulseCount;
          pulseCount = 0;
          flowRate = ((1000.0 / (millis() - previousMillis)) * pulse1Sec) / calibrationFactor;
          Serial.print("Flow rate: ");
          Serial.print(float(flowRate));  // Print the integer part of the variable
          Serial.print("L/min");
          Serial.println("\t");       // Print tab space
          flowresult = flowRate * MFFlow;
        }
      }
      else {
        //do nothing
      }
      String alerttypenoflw = "";
      for (int i = 310; i < 311; ++i)
      {
        alerttypenoflw += char(EEPROM.read(i));
      }
      Serial.print("alerttypenoflw: ");
      Serial.println(alerttypenoflw);
      int alerttypenumberflow = alerttypenoflw.toInt();
      Serial.print("alerttypenumberflow: ");
      Serial.println(alerttypenumberflow);

      String alerttypenoflw1 = "";
      for (int i = 322; i < 323; ++i)
      {
        alerttypenoflw1 += char(EEPROM.read(i));
      }
      Serial.print("alerttypenoflw1: ");
      Serial.println(alerttypenoflw1);
      int alerttypenumberflow1 = alerttypenoflw1.toInt();
      Serial.print("alerttypenumberflow1: ");
      Serial.println(alerttypenumberflow1);
      if (alerttypenumberflow == 1) {
        String alertconditionflw = "";
        String alertthreshold1flw = "";
        String alertthreshold2flw = "";
        String alerttypenoflw = "";
        for (int i = 299; i < 300; ++i)
        {
          alertconditionflw += char(EEPROM.read(i));
        }
        for (int i = 300; i < 305; ++i)
        {
          alertthreshold1flw += char(EEPROM.read(i));
        }
        for (int i = 305; i < 310; ++i)
        {
          alertthreshold2flw += char(EEPROM.read(i));
        }
        for (int i = 310; i < 311; ++i)
        {
          alerttypenoflw += char(EEPROM.read(i));
        }

        Serial.println("alertconditionflw: ");
        Serial.println(alertconditionflw);
        Serial.println("alertthreshold1flw: ");
        Serial.println(alertthreshold1flw);
        Serial.println("alertthreshold2flw: ");
        Serial.println(alertthreshold2flw);
        Serial.println("alerttypenoflw: ");
        Serial.println(alerttypenoflw);

        float alertthres1flow = alertthreshold1flw.toFloat();
        float alertthres2flow = alertthreshold2flw.toFloat();
        int alerttypenumberflow = alerttypenoflw.toInt();
        Serial.println("alerttypenumberflow: ");
        Serial.println(alerttypenumberflow);
        if (alertconditionflw == "1") {
          if (alertthres1flow == flowresult) {
            StaticJsonDocument<900> retalert;
            JsonObject JSONencoder = retalert.to<JsonObject>();
            JSONencoder["mac"] = macadd;
            JSONencoder["ipid"] = 3;
            JSONencoder["value"] = flowresult;
            JSONencoder["datetime"] = formattedDate;
            JSONencoder["alerttypeid"] = alerttypenumberflow;
            char buffer[900];
            serializeJson(JSONencoder, buffer);
            Serial.println(buffer);
            ptp2.toCharArray(ptp2char, 50);
            boolean rc = client.publish(ptp2char, buffer, retain);
          }
        }
        if (alertconditionflw == "2") {
          Serial.println("alertcondition 2");
          bool trial = (alertthres1flow <= flowresult);
          Serial.println(trial);
          if (alertthres1flow >= flowresult) {
            StaticJsonDocument<900> retalert;
            JsonObject JSONencoder = retalert.to<JsonObject>();
            JSONencoder["mac"] = macadd;
            JSONencoder["ipid"] = 3;
            JSONencoder["value"] = flowresult;
            JSONencoder["datetime"] = formattedDate;
            JSONencoder["alerttypeid"] = alerttypenumberflow;
            char buffer[900];
            serializeJson(JSONencoder, buffer);
            Serial.println(buffer);
            ptp2.toCharArray(ptp2char, 50);
            boolean rc = client.publish(ptp2char, buffer, retain);
          }
        }
        if (alertconditionflw == "3") {
          if (alertthres1flow <= flowresult) {
            StaticJsonDocument<900> retalert;
            JsonObject JSONencoder = retalert.to<JsonObject>();
            JSONencoder["mac"] = macadd;
            JSONencoder["ipid"] = 3;
            JSONencoder["value"] = flowresult;
            JSONencoder["datetime"] = formattedDate;
            JSONencoder["alerttypeid"] = alerttypenumberflow;
            char buffer[900];
            serializeJson(JSONencoder, buffer);
            Serial.println(buffer);
            ptp2.toCharArray(ptp2char, 50);
            boolean rc = client.publish(ptp2char, buffer, retain);
          }
        }
        if (alertconditionflw == "4") {
          Serial.println("alertcondition 4");
          bool trial = (flowresult >= alertthres1flow && flowresult <= alertthres2flow);
          Serial.println(trial);
          if (flowresult >= alertthres1flow && flowresult <= alertthres2flow) {
            StaticJsonDocument<900> retalert;
            JsonObject JSONencoder = retalert.to<JsonObject>();
            JSONencoder["mac"] = macadd;
            JSONencoder["ipid"] = 3;
            JSONencoder["value"] = flowresult;
            JSONencoder["datetime"] = formattedDate;
            JSONencoder["alerttypeid"] = alerttypenumberflow;
            char buffer[900];
            serializeJson(JSONencoder, buffer);
            Serial.println(buffer);
            ptp2.toCharArray(ptp2char, 50);
            boolean rc = client.publish(ptp2char, buffer, retain);
          }
        }
      }
      if (alerttypenumberflow1 == 2) {
        String alertconditionflw = "";
        String alertthreshold1flw = "";
        String alertthreshold2flw = "";
        String alerttypenoflw = "";
        for (int i = 311; i < 312; ++i)
        {
          alertconditionflw += char(EEPROM.read(i));
        }
        for (int i = 312; i < 317; ++i)
        {
          alertthreshold1flw += char(EEPROM.read(i));
        }
        for (int i = 317; i < 322; ++i)
        {
          alertthreshold2flw += char(EEPROM.read(i));
        }
        for (int i = 322; i < 323; ++i)
        {
          alerttypenoflw += char(EEPROM.read(i));
        }

        Serial.println("alertconditionflw: ");
        Serial.println(alertconditionflw);
        Serial.println("alertthreshold1flw: ");
        Serial.println(alertthreshold1flw);
        Serial.println("alertthreshold2flw: ");
        Serial.println(alertthreshold2flw);
        Serial.println("alerttypenoflw: ");
        Serial.println(alerttypenoflw);

        float alertthres1flow = alertthreshold1flw.toFloat();
        float alertthres2flow = alertthreshold2flw.toFloat();
        int alerttypenumberflow = alerttypenoflw.toInt();
        Serial.println("alerttypenumberflow: ");
        Serial.println(alerttypenumberflow);
        if (alertconditionflw == "1") {
          if (alertthres1flow == flowresult) {
            StaticJsonDocument<900> retalert;
            JsonObject JSONencoder = retalert.to<JsonObject>();
            JSONencoder["mac"] = macadd;
            JSONencoder["ipid"] = 3;
            JSONencoder["value"] = flowresult;
            JSONencoder["datetime"] = formattedDate;
            JSONencoder["alerttypeid"] = alerttypenumberflow;
            char buffer[900];
            serializeJson(JSONencoder, buffer);
            Serial.println(buffer);
            ptp2.toCharArray(ptp2char, 50);
            boolean rc = client.publish(ptp2char, buffer, retain);
          }
        }
        if (alertconditionflw == "2") {
          Serial.println("alertcondition 2");
          bool trial = (alertthres1flow <= flowresult);
          Serial.println(trial);
          if (alertthres1flow >= flowresult) {
            StaticJsonDocument<900> retalert;
            JsonObject JSONencoder = retalert.to<JsonObject>();
            JSONencoder["mac"] = macadd;
            JSONencoder["ipid"] = 3;
            JSONencoder["value"] = flowresult;
            JSONencoder["datetime"] = formattedDate;
            JSONencoder["alerttypeid"] = alerttypenumberflow;
            char buffer[900];
            serializeJson(JSONencoder, buffer);
            Serial.println(buffer);
            ptp2.toCharArray(ptp2char, 50);
            boolean rc = client.publish(ptp2char, buffer, retain);
          }
        }
        if (alertconditionflw == "3") {
          if (alertthres1flow <= flowresult) {
            StaticJsonDocument<900> retalert;
            JsonObject JSONencoder = retalert.to<JsonObject>();
            JSONencoder["mac"] = macadd;
            JSONencoder["ipid"] = 3;
            JSONencoder["value"] = flowresult;
            JSONencoder["datetime"] = formattedDate;
            JSONencoder["alerttypeid"] = alerttypenumberflow;
            char buffer[900];
            serializeJson(JSONencoder, buffer);
            Serial.println(buffer);
            ptp2.toCharArray(ptp2char, 50);
            boolean rc = client.publish(ptp2char, buffer, retain);
          }
        }
        if (alertconditionflw == "4") {
          Serial.println("alertcondition 4");
          bool trial = (flowresult >= alertthres1flow && flowresult <= alertthres2flow);
          Serial.println(trial);
          if (flowresult >= alertthres1flow && flowresult <= alertthres2flow) {
            StaticJsonDocument<900> retalert;
            JsonObject JSONencoder = retalert.to<JsonObject>();
            JSONencoder["mac"] = macadd;
            JSONencoder["ipid"] = 3;
            JSONencoder["value"] = flowresult;
            JSONencoder["datetime"] = formattedDate;
            JSONencoder["alerttypeid"] = alerttypenumberflow;
            char buffer[900];
            serializeJson(JSONencoder, buffer);
            Serial.println(buffer);
            ptp2.toCharArray(ptp2char, 50);
            boolean rc = client.publish(ptp2char, buffer, retain);
          }
        }
      }
    }
    if (ip03 == "0") {
      flowresult = 0;
    }

    String eepromtemp = String(tempresult).c_str();
    String eeprompres = String(presresult).c_str();
    String eepromflow = String(flowresult).c_str();

    //Serial.println(eepromtemp.length());
    //Serial.println("writing temp data");
    for (i = 0; i < eepromtemp.length() ; ++i) {
      //Serial.println(eepromtemp[i]);
      EEPROM.write(330 + i, eepromtemp[i]);
      //Serial.println(eepromtemp[i]);
    }
    //Serial.println("writing pressure data");
    for (i = 0; i < eeprompres.length() ; ++i) {
      EEPROM.write(335 + i, eeprompres[i]);
      //Serial.println(eeprompres[i]);
    }
    //Serial.println("writing flow data");
    for (i = 0; i < eepromflow.length() ; ++i) {
      EEPROM.write(340 + i, eepromflow[i]);
      //Serial.println(eepromflow[i]);
    }
    EEPROM.commit();
    delay(500);

    String tere = "";
    String prre = "";
    String flre = "";
    for (i = 330; i < 335; ++i)
    {
      tere += char(EEPROM.read(i));
    }
    for (i = 335; i < 340; ++i)
    {
      prre += char(EEPROM.read(i));
    }
    for (i = 340; i < 345; ++i)
    {
      flre += char(EEPROM.read(i));
    }

    int Te = tere.toInt();
    float Pr = prre.toFloat();
    float Fl = flre.toFloat();

    float signalstrength = WiFi.RSSI();
    //Serial.println(signalstrength);
    if (temperror > 0) {
      StaticJsonDocument<100> doc;
      JsonObject JSONencoder = doc.to<JsonObject>();
      JSONencoder["mac"] = macadd;
      JSONencoder["datetime"] = formattedDate;
      JSONencoder["inputid"] = 1;
      JSONencoder["errcode"] = temperror;
      char buffer[100];
      serializeJson(JSONencoder, buffer);
      Serial.println(buffer);
      String tpec = "ErrorCode";
      tpec.toCharArray(pterc, 50);
      rc = client.publish(pterc, buffer, retain);
    }

    StaticJsonDocument<150> doc1;
    JsonObject JSONencoder1 = doc1.to<JsonObject>();
    JSONencoder1["mac"] = macadd;
    JSONencoder1["datetime"] = formattedDate;
    JSONencoder1["ssid"] = String(essid);
    JSONencoder1["pswd"] = String(epaswd);
    JSONencoder1["rssi"] = signalstrength;
    JSONencoder1["fwver"] = FW_VERSION;
    JSONencoder1["battery"] = "0";
    char buffer1[150];
    serializeJson(JSONencoder1, buffer1);
    Serial.println(buffer1);
    String tpec1 = "HeartBeat";
    tpec1.toCharArray(pterc, 50);

    StaticJsonDocument<900> doc;
    JsonObject JSONencoder = doc.to<JsonObject>();
    JSONencoder["mac"] = macadd;
    JSONencoder["dt"] = formattedDate;
    JSONencoder["Subtopic"] = String(topic_in);
    JsonArray array = doc.createNestedArray("temp");
    JsonObject nested = array.createNestedObject();
    nested["ipid"] = "1";
    nested["value"] = tempresult;
    nested["error_code"] = temperror;
    JsonArray array1 = doc.createNestedArray("pressure");
    JsonObject nested1 = array1.createNestedObject();
    nested1["ipid"] = "2";
    nested1["value"] = presresult;
    JsonArray array2 = doc.createNestedArray("flow");
    JsonObject nested2 = array2.createNestedObject();
    nested2["ipid"] = "3";
    nested2["value"] = flowresult;
    char buffer[900];
    serializeJson(JSONencoder, buffer);
    //Serial.println(buffer);
    ptp1.toCharArray(ptp1char, 50);
    regBank.set(40001, tempresult);
    regBank.set(40002, presresult);
    regBank.set(40003, flowresult);
    String dura;
    for (i = 205; i < 210; ++i)
    {
      dura += char(EEPROM.read(i));
    }
    //Serial.println("dura: ");
    //Serial.println(dura);
    int sleepsec = dura.toInt();
    //Serial.println(sleepsec);
    int sleepms = sleepsec * 1000;
    //Serial.println(sleepms);
    //delay(sleepms);
    period = sleepms;
    if (millis() > time_now + period) {
      time_now = millis();
      rc = client.publish(topic_out, buffer, retain);
    }
    String heatbeatdura;
    for (i = 365; i < 370; ++i)
    {
      heatbeatdura += char(EEPROM.read(i));
    }
    int heatbeatsleepsec = heatbeatdura.toInt();
    int heatbeatsleepms = heatbeatsleepsec * 1000;
    heatbeatperiod = heatbeatsleepms;
    //Serial.println("==================================================================");
    //Serial.println(heatbeatsleepsec);
    //Serial.println(heatbeatsleepms);
    //Serial.println(heatbeatperiod);
    //Serial.println("==================================================================");
    if (heatbeatperiod == 0) {
      rc = client.publish(pterc, buffer1, retain);
    }
    if (heatbeatperiod > 0) {
      if (millis() > heatbeattime_now + heatbeatperiod) {
        heatbeattime_now = millis();
        rc = client.publish(pterc, buffer1, retain);
      }
    }
    slave.run();  /// Run  Slave Modbus RTU
  }
  else {
    Serial.println("N/W Failed");
    ESP.restart();
  }
}
/**http update check at the http url and compare current version and the existing version in the server and also the macid's of the device if update available it downloads and compile the binary file function***/
void checkForUpdates() {
  String mac = getMAC();
  Serial.println(mac);
  String fwURL = String( fwUrlBase );
  fwURL.concat( mac );
  String fwVersionURL = fwURL;
  fwVersionURL.concat( ".version" );
  Serial.println(FW_VERSION);
  Serial.println( "Checking for firmware updates." );
  Serial.print( "MAC address: " );
  Serial.println( mac );
  Serial.print( "Firmware version URL: " );
  Serial.println( fwVersionURL );

  HTTPClient httpClient;
  httpClient.begin( fwVersionURL );
  int httpCode = httpClient.GET();
  if ( httpCode == 200 ) {
    String newFWVersion = httpClient.getString();

    Serial.print( "Current firmware version: " );
    Serial.println( FW_VERSION );
    Serial.print( "Available firmware version: " );
    Serial.println( newFWVersion );

    int newVersion = newFWVersion.toInt();

    if ( newVersion > FW_VERSION ) {
      Serial.println( "Preparing to update" );

      String fwImageURL = fwURL;
      fwImageURL.concat( ".bin" );
      t_httpUpdate_return ret = ESPhttpUpdate.update( fwImageURL );
      switch (ret) {
        case HTTP_UPDATE_FAILED:
          Serial.printf("HTTP_UPDATE_FAILD Error (%d): %s", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
          break;

        case HTTP_UPDATE_NO_UPDATES:
          Serial.println("HTTP_UPDATE_NO_UPDATES");
          break;
      }
    }
    else {
      delay(100);
      Serial.println( "Already on latest version" );
    }
  }
  else {
    Serial.print( "Firmware version check failed, got HTTP response code " );
    Serial.println( httpCode );
    delay(100);
  }
  httpClient.end();
}
bool testWifi(void) {
  int c = 0;
  Serial.println("waiting for wifi to connect");
  while (1) {
    buttonState = digitalRead(resetbutton);
    Serial.print("buttonState = ");
    Serial.println(buttonState);
    while (buttonState == LOW ) {
      delay(100);  //if you want more resolution, lower this number
      buttonState = digitalRead(resetbutton);
      pressLength_milliSeconds = pressLength_milliSeconds + 100;
      //display how long button is has been held
      Serial.print("ms = ");
      Serial.println(pressLength_milliSeconds);
    }
    if (pressLength_milliSeconds >= optionTwo_milliSeconds) {
      Serial.println("Reset");
      devicereset();
    }
    else if (pressLength_milliSeconds >= optionOne_milliSeconds) {
      Serial.println("Restart");
      ESP.restart();
    }
    pressLength_milliSeconds = 0;
    Serial.print("esid: ");
    Serial.println(epssid);
    Serial.print("epass: ");
    Serial.println(epass);
    //WiFi.begin(esid.c_str(), epass.c_str());
    if (WiFi.status() == WL_CONNECTED) {
      return true;
    }
    delay(500);
    digitalWrite(wifistatusled, LOW);
    Serial.println("Connecting..");
    if (c > 100) {
      ESP.restart();
    }
    c++;
  }
  Serial.println("Connect timeout");
  return false;
}
void devicereset() {
  Serial.println("Clearing EEPROM");
  for (i = 0; i < 512; ++i) {
    EEPROM.write(i, 0);
    Serial.print("Done..");
    Serial.println(i);
  }
  EEPROM.commit();
  delay(500);
  ESP.restart();
}
