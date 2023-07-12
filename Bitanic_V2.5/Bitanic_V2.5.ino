// Author       : Firdaus Nuur Rhamadhan
// Project      : Bitanic 2.5
// Code Version : 1.0
// Last Update  : 05/07/2023
// Description  : Development process of Bitanic 2.0

//#include <Arduino.h>
#include <string.h>
#include <SPI.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <RTClib.h>
#include <EEPROM.h>
#include <WiFi.h>
#include <WiFiManager.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>

#include <esp_task_wdt.h>

// Panel Declaration
String project = "Bitanic 2.0";
String id = "BT05";
String clientID = "bitanicV2 " + id;

// WiFi Setting
const char* ssid = "Setting";
const char* password = "admin1234";
const char* mqttServer = "broker.emqx.io";
const int mqttPort = 1883;
const char* mqttUser = "";
const char* mqttPassword = "";

WiFiManager wm;
WiFiClient wifiClient;
PubSubClient client(wifiClient);

// Publis Topic
String publisTopic = "bitanic";
// Subscribe Topic
String subscribeTopic = "bitanic/" + id;

#define LED 2
#define RELAY1 25 // 33
#define RELAY2 26
#define BUZZER 12
#define soilMoisture1 13
#define soilMoisture2 14 
#define analogPin 34

#define led1 18
#define led2 23
#define ledWiFi 11
// #define SCREEN_WIDTH 128
// #define SCREEN_HEIGHT 64
#define DHTPIN1 15
// #define DHTPIN2 26
#define DHTTYPE DHT22
#define WDT_TIMEOUT 3

// Variable Declaration
const int soilMoisture1Threshold = 500;
const int soilMoisture2Threshold = 500;
int soilMoisture1Value = 0;
int soilMoisture2Value = 0;
String soilMoisture1Status = "";
String soilMoisture2Status = "";
int motor1Value = 0;
int motor2Value = 0;
String motor1Status = "";
String motor2Status = "";
float humidity = 0;
float temperature = 0;
float heatIndex = 0;
String timeNow = "";
String dateNow = "";
String statusWiFi = "";
String statusMQTT = "";
bool lockWiFi = false;

LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27, 16, 2);

RTC_DS1307 rtc;
DHT dht(DHTPIN1, DHTTYPE);
// DHT dht2(DHTPIN2, DHTTYPE);

int Detik, PortValue, countWifi, count2, State1, GlobalCount, h1, h2, LockTime, CountTime, CountMotor, TimeMotor, CountMinggu, TotalMinggu, OnDay, SaveOn, CountDayOn, countEND;
float t1, t2;
unsigned long GlobalClock;
String DATA[10], INPUTDATA, TimeNOW, DateNOW;
bool Lock1, Lock2, Lock3, Lock4, LockDate, MotorLock, LockMQTT;
// String ssid, password;
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

void sendData(const char* topic, const char* payload) {
  client.publish(topic, payload);
  Serial.println("Sent: " + String(payload));
}
void buzz(int delayTime, int repeat) {
  for (int i = 0; i < repeat; i++)
  {
    tone(BUZZER, 2000);
    delay(delayTime);
    tone(BUZZER, 0);
    delay(delayTime);
  }
}
void lcdPrint(String text1, String text2) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(text1);
  lcd.setCursor(0, 1);
  lcd.print(text2);
}
void sendMotorStatus(String id, int motor1Status, int motor2Status, String publishTopic) {
  DynamicJsonDocument doc(1024);
  doc["ID"] = id;
  doc["MOTOR"]["STATUS_MOTOR1"] = motor1Status;
  doc["MOTOR"]["STATUS_MOTOR2"] = motor2Status;
  String statusMotorData;
  serializeJson(doc, statusMotorData);
  sendData(publishTopic.c_str(), statusMotorData.c_str());
}

class Task {
  public:
    Task(unsigned long interval, void (*callback)())
    {
      this->interval = interval;
      this->callback = callback;
      this->nextRun = millis() + interval;
    }

    void update()
    {
      if (millis() >= nextRun)
      {
        nextRun = millis() + interval;
        callback();
      }
    }

  private:
    unsigned long interval;
    unsigned long nextRun;
    void (*callback)();
};

Task mqttUpdate(30000, []() {
  // Kirim data JSON ke MQTT
  DynamicJsonDocument doc(1024);
  doc["ID"] = id;
  doc["TELEMETRI"]["soil1"] = soilMoisture1Value;
  doc["TELEMETRI"]["soil2"] = soilMoisture2Value;
  doc["TELEMETRI"]["motor1"] = motor1Status;
  doc["TELEMETRI"]["motor2"] = motor2Status;
  doc["TELEMETRI"]["temperature"] = temperature;
  doc["TELEMETRI"]["humidity"] = humidity;
  doc["TELEMETRI"]["heatIndex"] = heatIndex;
  doc["TELEMETRI"]["time"] = timeNow;
  doc["TELEMETRI"]["date"] = dateNow;
  String telemetriData;
  serializeJson(doc, telemetriData);
  sendData(publisTopic.c_str(), telemetriData.c_str());
});
Task dataUpdate(1000, []() {
  DateTime now = rtc.now(); // get the current time
  // = = = = = = = = = = = = = variable feed = = = = = = = = = = = = =
  soilMoisture1Value = digitalRead(soilMoisture1); // read soil moisture 1
  delay(500);
  soilMoisture2Value = digitalRead(soilMoisture2); // read soil moisture 2
  soilMoisture1Status = (soilMoisture1Value == 0) ? "Basah" : "Kering";
  soilMoisture2Status = (soilMoisture2Value == 0) ? "Basah" : "Kering";
  motor1Status = (motor1Value == 1) ? "Hidup" : "Mati";
  motor2Status = (motor2Value == 1) ? "Hidup" : "Mati";

  motor1Value = digitalRead(RELAY1);
  motor2Value = digitalRead(RELAY2);
  // = = = = = = = = = = = = = DHT11 = = = = = = = = = = = = =

  humidity = dht.readHumidity();
  temperature = dht.readTemperature();
  heatIndex = dht.computeHeatIndex(temperature, humidity, false);
  timeNow = String(now.hour(), DEC) + ":" + String(now.minute(), DEC) + ":" + String(now.second(), DEC);
  dateNow = String(now.day(), DEC) + "/" + String(now.month(), DEC) + "/" + String(now.year(), DEC);
});
int lcdMilis = 0;
int lcdQueue = 0;
Task lcdUpdate(1500, []() {
  // = = = = = = = = = = = = = LCD Print = = = = = = = = = = = = =
  // Start Milis for print LCD every 1000ms
  if (millis() - lcdMilis >= 1500)
  {
    lcdMilis = millis();
    if (lcdQueue == 0)
    {
      lcdPrint(timeNow, dateNow);
      lcdQueue = 1;
    }
    else if (lcdQueue == 1)
    {
      lcdPrint("Soil 1 : " + soilMoisture1Status, "Soil 2 : " + soilMoisture2Status);
      lcdQueue = 2;
    }
    else if (lcdQueue == 2)
    {
      lcdPrint("Motor 1 : " + motor1Status, "Motor 2 : " + motor2Status);
      lcdQueue = 3;
    }
    else if (lcdQueue == 3)
    {
      lcdPrint("T : " + String(temperature) + " C", "H : " + String(humidity) + " %");
      lcdQueue = 0;
    }
  }
  // End Milis for print LCD every 1000ms
});

void Send() {
  DateTime now = rtc.now();
  DynamicJsonDocument doc(1023);
  doc["ID"] = id;
  doc["SSID"] = ssid;
  doc["PASSWORD"] = password;
  doc["STATUS_MOTOR1"] = digitalRead(RELAY1);
  doc["STATUS_MOTOR2"] = digitalRead(RELAY2);
  doc["DATE"] = String() + now.day() + "-" + now.month() + "-" + now.year();
  doc["TIME"] = String() + now.hour() + ":" + now.minute() + ":" + now.second();
  doc["DHT1Temp"] = t1;
  doc["DHT1Hum"] = h1;
  doc["DHT2Temp"] = t2;
  doc["DHT2Hum"] = h2;
  doc["ONTIME1"] = String() + EEPROM.readString(100) + "  " + EEPROM.readString(110) + "Menit";
  doc["ONTIME2"] = String() + EEPROM.readString(120) + "  " + EEPROM.readString(130) + "Menit";
  doc["ONTIME3"] = String() + EEPROM.readString(140) + "  " + EEPROM.readString(150) + "Menit";
  doc["ONTIME4"] = String() + EEPROM.readString(160) + "  " + EEPROM.readString(170) + "Menit";
  doc["ONTIME5"] = String() + EEPROM.readString(180) + "  " + EEPROM.readString(190) + "Menit";
  doc["TOTALPEKAN"] = EEPROM.readString(200);
  doc["CURRENTPEKAN"] = EEPROM.readString(205);
  doc["DET_HARI"] = EEPROM.readString(300);
  doc["HISTORY"] = EEPROM.readString(320);
  String data = "";
  serializeJson(doc, data);
  serializeJson(doc, Serial);
  Serial.println("Sending message to MQTT topic..");
  client.beginPublish("bitanic", data.length(), false);
  client.print(data);
  client.endPublish();
}
void callback(char* topic, byte* payload, unsigned int length) {
  for (int i = 0; i < length; i++) {
    char c = (char)payload[i];
    if (c == ',') {
      INPUTDATA.trim();
      DATA[count2] = INPUTDATA;
      count2++;
      INPUTDATA = "";
    } else if (c == '*') {
      if (DATA[0] == id) {
        DynamicJsonDocument doc(1024);
        doc["ID"] = id;
        doc["STATUS_AKTIF"] = 1;
        String statusData;
        serializeJson(doc, statusData);
        sendData(publisTopic.c_str(), statusData.c_str());
        lcdPrint("ID : " + id, "Status : Aktif");
        buzz(500, 3);
      }
      else if (DATA[0] == "MOTOR1") {
        if (DATA[1] == "1") {
          digitalWrite(RELAY1, HIGH);
          digitalWrite(led1, HIGH);
          lcdPrint("MOTOR 1", "Hidup");
          sendMotorStatus(id, motor1Value, motor2Value, publisTopic);
          buzz(500, 3);
        } else {
          digitalWrite(RELAY1, LOW);
          digitalWrite(led1, LOW);
          lcdPrint("MOTOR 1", "Mati");
          sendMotorStatus(id, motor1Value, motor2Value, publisTopic);
          buzz(500, 3);
        }
      } else if (DATA[0] == "MOTOR2") {
        if (DATA[1] == "1") {
          digitalWrite(RELAY2, HIGH);
          digitalWrite(led2, HIGH);
          lcdPrint("MOTOR 2", "Hidup");
          sendMotorStatus(id, motor1Value, motor2Value, publisTopic);
          buzz(500, 3);
        } else {
          digitalWrite(RELAY2, LOW);
          digitalWrite(led2, LOW);
          lcdPrint("MOTOR 2", "Mati");
          sendMotorStatus(id, motor1Value, motor2Value, publisTopic);
          buzz(500, 3);
        }
      } else if (DATA[0] == "GETDATA") {
        Serial.println(EEPROM.readString(0));
        Serial.println(EEPROM.readString(30));
        Send();
        lcdPrint("SENDING DATA", "OK...");
        buzz(500, 3);
      } else if (DATA[0] == "SETRTC") {
        rtc.adjust(DateTime(DATA[1].toInt(), DATA[2].toInt(), DATA[3].toInt(), DATA[4].toInt(), DATA[5].toInt(), DATA[6].toInt()));
        Serial.println("RTC UPDATED");
        lcdPrint("RTC UPDATED", "OK...");
        buzz(500, 3);
      } else if (DATA[0] == "RESET") {
        Serial.println("RESTARTING ESP");
        lcdPrint("RESTARTING ESP", "PLEASE WAIT...");
        buzz(300, 5);
        ESP.restart();
      } else if (DATA[0] == "SETONTIME1") {
        EEPROM.writeString(100, DATA[1]);
        EEPROM.writeString(110, DATA[2]);
        Serial.println("SETONTIME1 OK");
        lcdPrint("SET ONTIME 1", "OK...");
        buzz(500, 3);
      } else if (DATA[0] == "SETONTIME2") {
        EEPROM.writeString(120, DATA[1]);
        EEPROM.writeString(130, DATA[2]);
        Serial.println("SETONTIME2 OK");
        lcdPrint("SET ONTIME 2", "OK...");
        buzz(500, 3);
      } else if (DATA[0] == "SETONTIME3") {
        EEPROM.writeString(140, DATA[1]);
        EEPROM.writeString(150, DATA[2]);
        Serial.println("SETONTIME3 OK");
        lcdPrint("SET ONTIME 3", "OK...");
        buzz(500, 3);
      } else if (DATA[0] == "SETONTIME4") {
        EEPROM.writeString(160, DATA[1]);
        EEPROM.writeString(170, DATA[2]);
        Serial.println("SETONTIME4 OK");
        lcdPrint("SET ONTIME 4", "OK...");
        buzz(500, 3);
      } else if (DATA[0] == "SETONTIME5") {
        EEPROM.writeString(180, DATA[1]);
        EEPROM.writeString(190, DATA[2]);
        Serial.println("SETONTIME5 OK");
        lcdPrint("SET ONTIME 5", "OK...");
        buzz(500, 3);
      } else if (DATA[0] == "SETMINGGU") {
        countEND = 0;
        EEPROM.writeString(200, DATA[1]);
        EEPROM.writeString(205, "0");
        Serial.println("SETMINGGU OK");
        lcdPrint("SET MINGGU", "OK...");
        buzz(500, 3);
      } else if (DATA[0] == "SETHARI") {
        String TT;
        if (DATA[1] == "1") {
          EEPROM.writeString(210, "Sunday");
          TT = "1";
        } else {
          EEPROM.writeString(210, "NONE");
          TT = "0";
        } if (DATA[2] == "1") {
          EEPROM.writeString(220, "Monday");
          TT += "1";
        } else {
          EEPROM.writeString(220, "NONE");
          TT += "0";
        } if (DATA[3] == "1") {
          EEPROM.writeString(230, "Tuesday");
          TT += "1";
        } else {
          EEPROM.writeString(230, "NONE");
          TT += "0";
        }  if (DATA[4] == "1") {
          EEPROM.writeString(240, "Wednesday");
          TT += "1";
        } else {
          EEPROM.writeString(240, "NONE");
          TT += "0";
        }  if (DATA[5] == "1") {
          EEPROM.writeString(250, "Thursday");
          TT += "1";
        } else {
          EEPROM.writeString(250, "NONE");
          TT += "0";
        }  if (DATA[6] == "1") {
          EEPROM.writeString(260, "Friday");
          TT += "1";
        } else {
          EEPROM.writeString(260, "NONE");
          TT += "0";
        }  if (DATA[7] == "1") {
          EEPROM.writeString(270, "Saturday");
          TT += "1";
        } else {
          EEPROM.writeString(270, "NONE");
          TT += "0";
        }
        EEPROM.writeString(300, TT);
        Serial.println("SETHARI OK");
        lcdPrint("SETHARI", "OK...");
        buzz(500, 3);
      } else if (DATA[0] == "RESETALL") {
        digitalWrite(RELAY1, LOW);
        digitalWrite(led1, LOW);
        digitalWrite(RELAY2, LOW);
        digitalWrite(led2, LOW);
        EEPROM.writeString(100, "00:00:00");
        EEPROM.writeString(110, "0");
        EEPROM.writeString(120, "00:00:00");
        EEPROM.writeString(130, "0");
        EEPROM.writeString(140, "00:00:00");
        EEPROM.writeString(150, "0");
        EEPROM.writeString(160, "00:00:00");
        EEPROM.writeString(170, "0");
        EEPROM.writeString(180, "00:00:00");
        EEPROM.writeString(190, "0");
        EEPROM.writeString(200, "0");
        EEPROM.writeString(205, "0");
        EEPROM.writeString(210, "NONE");
        EEPROM.writeString(220, "NONE");
        EEPROM.writeString(230, "NONE");
        EEPROM.writeString(240, "NONE");
        EEPROM.writeString(250, "NONE");
        EEPROM.writeString(260, "NONE");
        EEPROM.writeString(270, "NONE");
        EEPROM.writeString(280, "0");
        EEPROM.writeString(300, "0000000");
        EEPROM.writeString(310, "0");
        EEPROM.writeString(320, "0000000");
        Serial.println("RESETALL OK");
        lcdPrint("RESET ALL", "OK...");
        buzz(500, 3);
      } else {
        Serial.println("COMMAND UNDEFINED");
        lcdPrint("COMMAND", "UNDEFINED");
        buzz(200, 7);
      }
      count2 = 0;
      INPUTDATA = "";
    } else {
      INPUTDATA += c;
    }
  }
  EEPROM.commit();
  delay(200);
  digitalWrite(BUZZER, LOW);
}

void setup() {
  Serial.begin(9600);
  EEPROM.begin(512);

  rtc.begin();
  dht.begin();

  // LCD Initialization
  lcd.begin();
  lcd.backlight();
  lcd.clear();

  lcdPrint("Bitaniv V2.0", id);
  buzz(100, 2);
  buzz(500, 1);
  String wifiAP = "BitanicV2 " + id;
  lcdPrint(wifiAP.c_str(), "192.162.4.1");


  bool res;
  res = wm.autoConnect(wifiAP.c_str());
  if (!res) {
    Serial.println("Failed to connect");
    lcdPrint("WiFi Connect", "Failed");
    ESP.restart();
  }
  else {
    //if you get here you have connected to the WiFi
    Serial.println("connected...yeey :)");
    lcdPrint("WiFi", "Connected");
    buzz(100, 4);
    buzz(700, 1);
  }

  pinMode(LED, OUTPUT);
  pinMode(RELAY1, OUTPUT);
  pinMode(RELAY2, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(soilMoisture1, INPUT);
  pinMode(soilMoisture2, INPUT);

  digitalWrite(RELAY1, LOW);
  digitalWrite(led1, LOW);
  digitalWrite(RELAY2, LOW);
  digitalWrite(led2, LOW);
  digitalWrite(LED, LOW);
  digitalWrite(BUZZER, LOW);
  LockTime = 0;


  DateTime now = rtc.now();
  while (WiFi.status() != WL_CONNECTED) {
    DateTime now = rtc.now();
    digitalWrite(LED, LOW);
    Detik++;
    State1 = 0;
    lockWiFi = true;
    lcdPrint("Connecting WiFi", "");
    Serial.println("WIFI NOT CONNECTED");
    delay(1000);
    if (Detik == 5) {
      Lock1 == false;
      break;
    }
  }
  if (WiFi.status() == WL_CONNECTED) {
    State1 = 1;
    Lock1 = true;
    lcdPrint("WiFi Connected", "");
    digitalWrite(LED, HIGH);
    Serial.println("WIFI CONNECTED");
  }
  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);
  while (!client.connected() && Lock1 == true) {
    Serial.println("Connecting to MQTT...");
    String clientName = "BitanicV2-" + id;
    if (client.connect(clientName.c_str(), mqttUser, mqttPassword )) {
      State1 = 3;
      lcdPrint("MQTT Connected", "");
      Serial.println("MQTT CONNECTED");
    } else {
      State1 = 4;
      lcdPrint("MQTT Failed", "MQTT Failed");
      digitalWrite(LED, LOW);
      delay(500);
      digitalWrite(LED, HIGH);
      delay(100);
      Serial.print("failed with state ");
      Serial.print(client.state());
    }
  }
  Detik = 0;

  client.subscribe(subscribeTopic.c_str());
}

void loop() {
  if (client.connected() == false) {
    lcdPrint("MQTT", "Disconnected");
    buzz(200, 7);
    ESP.restart();
  }
  if (lockWiFi == true) {
    digitalWrite(ledWiFi, HIGH);
  }
  else if (lockWiFi == false) {
    digitalWrite(ledWiFi, LOW);
  }

  OnDay = 0;
  SaveOn = EEPROM.readString(280).toInt();
  if (EEPROM.readString(100) != "00:00:00") {
    OnDay++;
  } if (EEPROM.readString(120) != "00:00:00") {
    OnDay++;
  } if (EEPROM.readString(140) != "00:00:00") {
    OnDay++;
  } if (EEPROM.readString(160) != "00:00:00") {
    OnDay++;
  } if (EEPROM.readString(180) != "00:00:00") {
    OnDay++;
  }
  DateTime now = rtc.now();
  if (now.second() != GlobalClock) {
    CountDayOn = 0;
    //    GlobalCount++;
    if (WiFi.status() != WL_CONNECTED) {
      Lock1 = false;
      State1 = 0;
    }
    if (!client.connected()) {
      Lock1 = false;
      State1 = 4;
    }
    if (Lock1 == false) {
      if (digitalRead(RELAY1) != HIGH || digitalRead(RELAY2) != HIGH) {
        digitalWrite(LED, LOW);
        Detik++;
        if (Detik == 5) {
          State1 = 5;
          ESP.restart();
        }
      }
    }
    if (EEPROM.readString(210) == "Sunday") {
      CountDayOn++;
    } if (EEPROM.readString(220) == "Monday") {
      CountDayOn++;
    } if (EEPROM.readString(230) == "Tuesday") {
      CountDayOn++;
    } if (EEPROM.readString(240) == "Wednesday") {
      CountDayOn++;
    } if (EEPROM.readString(250) == "Thursday") {
      CountDayOn++;
    } if (EEPROM.readString(260) == "Friday") {
      CountDayOn++;
    } if (EEPROM.readString(270) == "Saturday") {
      CountDayOn++;
    }
    if (EEPROM.readString(200) != "END") {
      if (EEPROM.readString(310).toInt() == CountDayOn) {
        if (EEPROM.readString(200) != "0") {
          int YY = EEPROM.readString(205).toInt();
          YY++;
          EEPROM.writeString(205, String() + YY);
        }
        EEPROM.writeString(320, "0000000");
        EEPROM.writeString(310, "0");
      }
      if (LockTime > 0 && Lock4 == false) {
        digitalWrite(RELAY1, HIGH);
        digitalWrite(led1, HIGH);
        digitalWrite(RELAY2, HIGH);
        digitalWrite(led2, HIGH);
        int KK = EEPROM.readString(280).toInt();
        KK++;
        EEPROM.writeString(280, String() + KK);
        if (EEPROM.readString(280).toInt() == OnDay) {
          EEPROM.writeString(280, "0   ");
          KK = EEPROM.readString(310).toInt();
          KK++;
          EEPROM.writeString(310, String() + KK);
          int LL = 320 + now.dayOfTheWeek();
          EEPROM.writeChar(LL, '1');
        }
        Send();
        Lock4 = true;
      }
    }
    if (now.hour() == 12) {
      int LL = 320 + now.dayOfTheWeek();
      EEPROM.writeChar(LL, '1');
    }
    if (EEPROM.readString(205) == EEPROM.readString(200) && EEPROM.readString(200) != 0) {
      EEPROM.writeString(200, "END");
      EEPROM.writeString(205, "0");
    }
    if (Lock4 == true) {
      TimeMotor++;
    }
    if (TimeMotor >= CountMotor * 60 && Lock4 == true) {
      digitalWrite(RELAY1, LOW);
      digitalWrite(led1, LOW);
      digitalWrite(RELAY2, LOW);
      digitalWrite(led2, LOW);
      Send();
      Lock4 = false;
      LockTime = 0;
      TimeMotor = 0;
    }
    String Load = String() + daysOfTheWeek[now.dayOfTheWeek()] + ", ";
    if (now.day() < 10) {
      Load += String() + "0" + now.day();
    } else {
      Load += now.day(), DEC;
    }
    Load += "/";
    if (now.month() < 10) {
      Load += String() + "0" + now.month();
    } else {
      Load += now.month(), DEC;
    }
    Load += String() + "/" + now.year();
    Load += "\n      ";
    if (now.hour() < 10) {
      Load += String() + "0" + now.hour();
    } else {
      Load += now.hour(), DEC;
    }
    Load += ":";
    if (now.minute() < 10) {
      Load += String() + "0" + now.minute();
    } else {
      Load += now.minute(), DEC;
    }
    Load += ":";
    if (now.second() < 10) {
      Load += String() + "0" + now.second();
    } else {
      Load += now.second(), DEC;
    }
    //    Load += "\n\nDHT1   :\nDHT2   :\nMOTOR1 :\nMOTOR2 :\n   WIFI CONNECTED\nA";
    if (now.second() % 10 <= 4) {
      h1 = humidity;
      t1 = temperature;
      h2 = 0;
      t2 = 0;

      if (isnan(h1) || isnan(t1)) {
        Serial.println(F("Failed to read from DHT sensor 1!"));
        Load += "\n\nDHT1   :NOT CONNECT\n";
      } else {
        Load += String() + "\n\nDHT1  :" + t1 + "'C " + h1 + "%\n";
      }
      if (isnan(h2) || isnan(t2)) {
        Serial.println(F("Failed to read from DHT sensor 2!"));
        Load += "DHT2   :NOT CONNECT\n";
      } else {
        Load += String() + "DHT2  :" + t2 + "'C " + h2 + "%\n";
      }
      if (digitalRead(RELAY1) == 1) {
        Load += String() + "RELAY1:NYALA\n";
      } else {
        Load += String() + "RELAY1:MATI\n";
      }
      if (digitalRead(RELAY2) == 1) {
        Load += String() + "RELAY2:NYALA\n";
      } else {
        Load += String() + "RELAY2:MATI\n";
      }
      if (State1 == 0) {
        Load += "  WIFI NOT CONNECTED";
      } else if (State1 == 2) {
        Load += "   WIFI CONNECTED";
      } else if (State1 == 3) {
        Load += "    CONNECTION OK";
      } else if (State1 == 4) {
        Load += "    NO CONNECTION";
      } else if (State1 == 5) {
        Load += "    RESTARTING...";
        delay(1000);
        ESP.restart();
      }
    } else {
      Load += "\n\nDEV :" + id;
      Load += String() + "\nSSID:" + ssid;
      Load += "\nSERV:broker.hivemq";
      Load += "\nPORT:1883\n";
      if (State1 == 0) {
        Load += "  WIFI NOT CONNECTED";
      } else if (State1 == 2) {
        Load += "   WIFI CONNECTED";
      } else if (State1 == 3) {
        Load += "    CONNECTION OK";
      } else if (State1 == 4) {
        Load += "    NO CONNECTION";
      } else if (State1 == 5) {
        Load += "    RESTARTING...";
        delay(1000);
        ESP.restart();
      }
    }
    // Serial.println("\n[585]");
    // Serial.println(Load);
    client.loop();
    GlobalClock = now.second();
  }
  if (digitalRead(RELAY1) == HIGH || digitalRead(RELAY2) == HIGH) {
    MotorLock = true;
  } else {
    MotorLock = false;
  }
  if (now.hour() < 10) {
    TimeNOW = String() + "0" + now.hour();
  } else {
    TimeNOW = now.hour(), DEC;
  }
  TimeNOW += ":";
  if (now.minute() < 10) {
    TimeNOW += String() + "0" + now.minute();
  } else {
    TimeNOW += now.minute(), DEC;
  }
  TimeNOW += ":";
  if (now.second() < 10) {
    TimeNOW += String() + "0" + now.second();
  } else {
    TimeNOW += now.second(), DEC;
  }
  if (EEPROM.readString(200) != "END") {
    TotalMinggu = EEPROM.readString(200).toInt();
    DateNOW = daysOfTheWeek[now.dayOfTheWeek()];
    if (DateNOW == EEPROM.readString(210) || DateNOW == EEPROM.readString(220) || DateNOW == EEPROM.readString(230) || DateNOW == EEPROM.readString(240) || DateNOW == EEPROM.readString(250) || DateNOW == EEPROM.readString(260) || DateNOW == EEPROM.readString(270)) {
      LockDate = true;
    } else {
      LockDate = false;
    }
    if (LockDate == true && CountMinggu <= TotalMinggu && EEPROM.readString(200).toInt() > 0) {
      if (TimeNOW == EEPROM.readString(100) && EEPROM.readString(100) != "00:00:00") {
        LockTime = 1;
      } else if (TimeNOW == EEPROM.readString(120) && EEPROM.readString(120) != "00:00:00") {
        LockTime = 2;
      } else if (TimeNOW == EEPROM.readString(140) && EEPROM.readString(140) != "00:00:00") {
        LockTime = 3;
      } else if (TimeNOW == EEPROM.readString(160) && EEPROM.readString(160) != "00:00:00") {
        LockTime = 4;
      } else if (TimeNOW == EEPROM.readString(180) && EEPROM.readString(180) != "00:00:00") {
        LockTime = 5;
      }
      if (LockTime == 1) {
        CountMotor = EEPROM.readString(110).toInt();
      } else if (LockTime == 2) {
        CountMotor = EEPROM.readString(130).toInt();
      } else if (LockTime == 3) {
        CountMotor = EEPROM.readString(150).toInt();
      } else if (LockTime == 4) {
        CountMotor = EEPROM.readString(170).toInt();
      } else if (LockTime == 5) {
        CountMotor = EEPROM.readString(190).toInt();
      }
    }
    countEND = 0;
  }
  if (EEPROM.readString(200) == "END") {
    EEPROM.writeString(200, "END");
    EEPROM.writeString(205, "0");
    EEPROM.writeString(280, "0");
    EEPROM.writeString(310, "0");
    EEPROM.writeString(320, "0000000");
  }
  EEPROM.commit();
  CountDayOn = 0;

  int soilmoisture1Value = digitalRead(soilMoisture1);
  int soilmoisture2Value = digitalRead(soilMoisture2);

  if (soilmoisture1Value == HIGH) {
    digitalWrite(RELAY1, HIGH);
    sendMotorStatus(id, 1, motor2Value, publisTopic);
    delay (5000);
  } else {
    digitalWrite(RELAY1, LOW);
    sendMotorStatus(id, 0, motor2Value, publisTopic);
  }

  if (soilmoisture2Value == HIGH) {
    sendMotorStatus(id, motor1Value, 1, publisTopic);
    digitalWrite(RELAY2, HIGH);  // Menghidupkan relay jika kelembaban terdeteksi
    delay (5000);
  } else {
    sendMotorStatus(id, motor1Value, 0, publisTopic);
    digitalWrite(RELAY2, LOW); // Mematikan relay jika kelembaban tidak terdeteksi
  }

  delay(1000); // Menunda pembacaan selama 1 detik

  dataUpdate.update();
  lcdUpdate.update();
  mqttUpdate.update();
}
