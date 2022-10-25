/*
 * ESP8266-01 with multiple DS10B20 temperature sensors
 * reads sensors asyncronsly for less blocking time
 *
 */
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <OneWire.h>

const bool DEBUG_ONEWIRE = false;
const int nsensors = 1;
byte sensors[][8] = {
   { 0x28, 0xFF, 0xFB, 0xDB, 0xC0, 0x17, 0x05, 0xE5 }
};
int16_t tempraw[nsensors];
unsigned long nextrun = 0;

const String WIFI_SSID = "_YOUR_SSID_"
const String WIFI_PASSPHRASE = "_YOUR_PASSPHRASE_"
const long utcOffsetInSeconds = -18000; // UTC-5
const String days[7]={"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
const String months[12]={"January", "February", "March", "April", "May", "June", "July", "August", "September", "October", "November", "December"};
const int TEMP_PIN = D4;
const int RELAY_PIN = D7;
const float MIN_TEMP = 80.0;
const float MAX_TEMP = 80.3;

bool HEATING = false;
int on_time = 0;

// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", utcOffsetInSeconds);

OneWire ds(TEMP_PIN);

void setup() {
  Serial.begin(115200);
  Serial.println();

  WiFi.begin(WIFI_SSID, WIFI_PASSPHRASE);
  Serial.print("Connecting");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  Serial.print("Connected, IP address: ");
  Serial.println(WiFi.localIP());
  timeClient.begin();
}

void loop()
{
  ds18process(); //call this every loop itteration, the more calls the better.
  timeClient.update();
  String formattedTime = timeClient.getFormattedTime();
  int currentSecond = timeClient.getSeconds();

  if (millis() > nextrun) {
    float temp = ds18temp(1, tempraw[0]);
    if (currentSecond % 30 == 0) {
      Serial.print(formattedTime);
      Serial.print(": ");
      Serial.print(temp);
      Serial.print("f (heating: ");
      Serial.print(HEATING);
      Serial.print(")");
      Serial.println();
    }
    nextrun = millis() + 1000;

    if (HEATING) {
      on_time += 1;
    }

    if (temp < MIN_TEMP) {
      if (digitalRead(RELAY_PIN) != LOW) {
        Serial.print(formattedTime);
        Serial.print(": ");
        Serial.println("Turning on heater...");
        pinMode(RELAY_PIN, OUTPUT);
        digitalWrite(RELAY_PIN, LOW);
        HEATING = true;
        on_time = 0;
      }
    } else if (temp > MAX_TEMP) {
      if (digitalRead(RELAY_PIN) != HIGH) {
        Serial.print(formattedTime);
        Serial.print(": ");
        Serial.print("Turning off heater after ");
        Serial.print(on_time);
        Serial.print(" seconds. (not really, need to update to use time difference)");
        Serial.println();
        pinMode(RELAY_PIN, OUTPUT);
        digitalWrite(RELAY_PIN, HIGH);
        HEATING = false;
        on_time = 0;
      }
    }
  }
}

/* Process the sensor data in stages.
 * each stage will run quickly. the conversion
 * delay is done via a millis() based delay.
 * a 5 second wait between reads reduces self
 * heating of the sensors.
 */
void ds18process() {
  static byte stage = 0;
  static unsigned long timeNextStage = 0;
  static byte sensorindex = 100;
  byte i, j;
  byte present = 0;
  byte type_s;
  byte data[12];
  byte addr[8];

  if(stage == 0 && millis() > timeNextStage) {
    if (!ds.search(addr)) {
      //no more, reset search and pause
      ds.reset_search();
      timeNextStage = millis() + 5000; //5 seconds until next read
      return;
    } else {
      if (OneWire::crc8(addr, 7) != addr[7]) {
        Serial.println("CRC is not valid!");
        return;
      }
      //got one, start stage 1
      stage = 1;
    }
  }
  if(stage==1) {
    if (DEBUG_ONEWIRE) {
      Serial.print("ROM =");
      for ( i = 0; i < 8; i++) {
        Serial.write(' ');
        Serial.print(addr[i], HEX);
      }
    }
    //find sensor
    for(j=0; j<nsensors; j++){
      sensorindex = j;
      for(i=0; i<8; i++){
        if(sensors[j][i] != addr[i]) {
          sensorindex = 100;
          break; // stop the i loop
        }
      }
      if (sensorindex < 100) {
        break; //found it, stop the j loop
      }
    }
    if(sensorindex == 100) {
      Serial.println("  Sensor not found in array");
      stage = 0;
      return;
    }
    if (DEBUG_ONEWIRE) {
      Serial.print("  index="); Serial.println(sensorindex);
    }

    ds.reset();
    ds.select(sensors[sensorindex]);
    ds.write(0x44, 0);        // start conversion, with parasite power off at the end
    stage = 2; //now wait for stage 2
    timeNextStage = millis() + 1000; //wait 1 seconds for the read
  }

  if (stage == 2 && millis() > timeNextStage) {
    // the first ROM byte indicates which chip
    switch (sensors[sensorindex][0]) {
      case 0x10:
        if (DEBUG_ONEWIRE) {
          Serial.print("  Chip = DS18S20");  // or old DS1820
          Serial.print("  index="); Serial.println(sensorindex);
        }
        type_s = 1;
        break;
      case 0x28:
        if (DEBUG_ONEWIRE) {
          Serial.print("  Chip = DS18B20");
          Serial.print("  index="); Serial.println(sensorindex);
        }
        type_s = 0;
        break;
      case 0x22:
        if (DEBUG_ONEWIRE) {
          Serial.print("  Chip = DS1822");
          Serial.print("  index="); Serial.println(sensorindex);
        }
        type_s = 0;
        break;
      default:
        if (DEBUG_ONEWIRE) {
          Serial.println("Device is not a DS18x20 family device.");
        }
        stage=0;
        return;
    }

    present = ds.reset();
    ds.select(sensors[sensorindex]);
    ds.write(0xBE);         // Read Scratchpad

    if (DEBUG_ONEWIRE) {
      Serial.print("  Data = ");
      Serial.print(present, HEX);
      Serial.print(" ");
    }
    for ( i = 0; i < 9; i++) {           // we need 9 bytes
      data[i] = ds.read();
      if (DEBUG_ONEWIRE) {
        Serial.print(data[i], HEX);
        Serial.print(" ");
      }
    }
    if (DEBUG_ONEWIRE) {
      Serial.print(" CRC=");
      Serial.print(OneWire::crc8(data, 8), HEX);
      Serial.print(" index="); Serial.print(sensorindex);
      Serial.println();
    }

    int16_t raw = (data[1] << 8) | data[0];
    if (type_s) {
      raw = raw << 3; // 9 bit resolution default
      if (data[7] == 0x10) {
        // "count remain" gives full 12 bit resolution
        raw = (raw & 0xFFF0) + 12 - data[6];
      }
    } else {
      byte cfg = (data[4] & 0x60);
      // at lower res, the low bits are undefined, so let's zero them
      if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
      else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
      else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
      //// default is 12 bit resolution, 750 ms conversion time
    }
    tempraw[sensorindex] = raw;
    stage=0;
  }
}

/* Converts raw temp to Celsius or Fahrenheit
 * scale: 0=celsius, 1=fahrenheit
 * raw: raw temp from sensor
 *
 * Call at any time to get the last save temperature
 */
float ds18temp(byte scale, int16_t raw)
{
  switch(scale) {
    case 0: //Celsius
      return (float)raw / 16.0;
      break;
    case 1: //Fahrenheit
      return (float)raw / 16.0 * 1.8 + 32.0;
      break;
    default: //er, wut
      return -255;
  }
}

