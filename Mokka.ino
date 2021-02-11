#include "EEPROM.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include <aREST.h>
#include "RTClib.h"
#include <Wire.h>
#include "SHT2x.h"
#include <LiquidCrystal_I2C.h>
#include <Button.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <HTU21D.h>
#include <MQUnifiedsensor.h>

// go to aREST.h lib to config variables, functions, and buffer size
//#define OUTPUT_BUFFER_SIZE 50000
//#define NUMBER_VARIABLES 15
//#define NUMBER_FUNCTIONS 15

int addrfan[4] = {2, 4, 6, 8};
int addrheat = 10, addrcool = 12, addrFanTempMin = 16, addrFanTempMax = 14,
    addrHeatMin = 16, addrHeatMax = 18, addrCoolMin = 20, addrCoolMax = 22;
int addr = 0;

#define EEPROM_SIZE 100
#define ONE_WIRE_BUS 15

const char *ssidAP = "Mustika Controller";
const char *passwordAP = "mustikajaya";

const char *ssid = "Mie Goyeng";
const char *password = "digodogsek";

int fan1, fan2, fan3, fan4, cooler1, heater1, delayButton = 100, hTempMax, hTempMin, cTempMax, cTempMin;
int menu, state, stateFTMax, stateFTMin, stateHTMax, stateHTMin, stateCTMax, stateCTMin, count, adc, fTempMin, fTempMax;
float ppm, temp, hum;

//variabel controlling untuk REST API
int apiFan1(String command);
int apiFan2(String command);
int apiFan3(String command);
int apiFan4(String command);
int apiHeater(String command);
int apiCooler(String command);
int apiFTempMin(String command);
int apiFTempMax(String command);
int apiHTempMin(String command);
int apiHTempMax(String command);
int apiCTempMin(String command);
int apiCTempMax(String command);

// Create aREST instance
aREST rest = aREST();

int relayON = LOW;   //relay nyala
int relayOFF = HIGH; //relay mati

long n, f, n1, f1, n2, f2, amo, timeSecond, timer1, timer2, timer3, timeNow;

enum sub
{
  algoritma,
  awal,
  sensor,
  menu1,
  menu2,
  menu3,
  menu4,
  menu5,
  menu6,
  menu7,
  menu8,
  menu9,
  menu10,
  menu11,
  menu12,
  menu13,
  setfan1,
  setfan2,
  setfan3,
  setfan4,
  cooler,
  heater,
  timeon,
  timeoff,
  timeon1,
  timeoff1,
  timeon2,
  timeoff2,
  ya,
  tidak
};

// Debouncing
unsigned long interval = 5000, previous = 0, current;

SHT2x SHT2x;

OneWire oneWire(ONE_WIRE_BUS);
//DallasTemperature sensors(&oneWire);
//HTU21D myHTU21D(HTU21D_RES_RH12_TEMP14);
RTC_DS3231 rtc;
LiquidCrystal_I2C lcd(0x27, 20, 4);

char daysOfTheWeek[7][12] = {"Minggu", "Senin", "Selasa", "Rabu", "Kamis", "Jumat", "Sabtu"};
String second, minute, hour, dayOfWeek, dayOfMonth, month, year;

int okButton = 12;
int upButton = 18;
int downButton = 19;
int cancelButton = 13;
int ledPin1 = 2;
int ledPin2 = 4;
int ledPin3 = 16;
int ledPin4 = 17;
int ledPin5 = 5;
int ledPin6 = 26;

const int relay1 = 2;  // 26
const int relay2 = 4;  // 25
const int relay3 = 25; // 33
const int relay4 = 32; //32
const int relay5 = 33; // 2
const int relay6 = 26; // 4

#define ok !digitalRead(okButton)
#define cancel !digitalRead(cancelButton)
#define up !digitalRead(upButton)
#define down !digitalRead(downButton)

WiFiServer server(80);

#define AMMONIA_PIN 39
//AMONIA
#define RL 47    //The value of resistor RL is 47K
#define m -0.263 //Enter calculated Slope
#define b 0.42   //Enter calculated intercept

/************************Hardware Related Macros************************************/
#define Board ("ESP-32") // Wemos ESP-32 or other board, whatever have ESP32 core.
#define Pin (36)         //IO25 for your ESP32 WeMos Board, pinout here: https://i.pinimg.com/originals/66/9a/61/669a618d9435c702f4b67e12c40a11b8.jpg
/***********************Software Related Macros************************************/
#define Type ("MQ-135")          //MQ135 or other MQ Sensor, if change this verify your a and b values.
#define Voltage_Resolution (3.3) // 3V3 <- IMPORTANT. Source: https://randomnerdtutorials.com/esp32-adc-analog-read-arduino-ide/
#define ADC_Bit_Resolution (12)  // ESP-32 bit resolution. Source: https://randomnerdtutorials.com/esp32-adc-analog-read-arduino-ide/
#define RatioMQ135CleanAir (60)  // Ratio of your sensor, for this example an MQ-3
/*****************************Globals***********************************************/
MQUnifiedsensor MQ135(Board, Voltage_Resolution, ADC_Bit_Resolution, Pin, Type);
/*****************************Globals***********************************************/

void setup()
{
  Serial.begin(9600);
  lcd.init();
  lcd.backlight();

  SHT2x.begin();

  state = awal;
  EEPROM.begin(EEPROM_SIZE);

  pinMode(upButton, INPUT_PULLUP);
  pinMode(downButton, INPUT_PULLUP);
  pinMode(cancelButton, INPUT_PULLUP);
  pinMode(okButton, INPUT_PULLUP);

  pinMode(relay1, OUTPUT);
  pinMode(relay2, OUTPUT);
  pinMode(relay3, OUTPUT);
  pinMode(relay4, OUTPUT);
  pinMode(relay5, OUTPUT);
  pinMode(relay6, OUTPUT);

  digitalWrite(relay1, relayOFF);
  digitalWrite(relay2, relayOFF);
  digitalWrite(relay3, relayOFF);
  digitalWrite(relay4, relayOFF);
  digitalWrite(relay5, relayOFF);
  digitalWrite(relay6, relayOFF);

  pinMode(ledPin1, OUTPUT);
  pinMode(ledPin2, OUTPUT);
  pinMode(ledPin3, OUTPUT);
  pinMode(ledPin4, OUTPUT);
  pinMode(ledPin5, OUTPUT);
  pinMode(ledPin6, OUTPUT);

  //read var value from eeprom
  fan1 = EEPROM.read(addrfan[0]);
  if (fan1 > 1)
  {
    fan1 = 1;
    EEPROM.write(addrfan[0], fan1);
    EEPROM.commit();
  }
  fan2 = EEPROM.read(addrfan[1]);
  if (fan2 > 1)
  {
    fan2 = 1;
    EEPROM.write(addrfan[1], fan2);
    EEPROM.commit();
  }
  fan3 = EEPROM.read(addrfan[2]);
  if (fan3 > 1)
  {
    fan3 = 1;
    EEPROM.write(addrfan[2], fan3);
    EEPROM.commit();
  }
  fan4 = EEPROM.read(addrfan[3]);
  if (fan4 > 1)
  {
    fan4 = 1;
    EEPROM.write(addrfan[3], fan4);
    EEPROM.commit();
  }
  cooler1 = EEPROM.read(addrcool);
  if (cooler1 > 1)
  {
    cooler1 = 1;
    EEPROM.write(addrcool, cooler1);
    EEPROM.commit();
  }
  heater1 = EEPROM.read(addrheat);
  if (heater1 > 1)
  {
    heater1 = 0;
    EEPROM.write(addrheat, heater1);
    EEPROM.commit();
  }
  fTempMin = EEPROM.read(addrFanTempMin);
  if (fTempMin > 100)
  {
    fTempMin = 20;
    EEPROM.write(addrFanTempMin, fTempMin);
    EEPROM.commit();
  }
  fTempMax = EEPROM.read(addrFanTempMax);
  if (fTempMax > 100)
  {
    fTempMax = 50;
    EEPROM.write(addrFanTempMax, fTempMax);
    EEPROM.commit();
  }
  temp = 24;
  hum = 70;
  ppm = 5;

  //initialize variable and function in rest
  rest.variable("temperature", &temp);
  rest.variable("humidity", &hum);
  rest.variable("ammonia", &ppm);
  rest.variable("fan1", &fan1);
  rest.variable("fan2", &fan2);
  rest.variable("fan3", &fan3);
  rest.variable("fan4", &fan4);
  rest.variable("heater", &heater1);
  rest.variable("cooler", &cooler1);
  rest.variable("fmintemp", &fTempMin);
  rest.variable("fmaxtemp", &fTempMax);
  rest.variable("hmintemp", &hTempMin);
  rest.variable("hmaxtemp", &hTempMax);
  rest.variable("cmintemp", &cTempMin);
  rest.variable("cmaxtemp", &cTempMax);

  rest.function("apiFan1", apiFan1);
  rest.function("apiFan2", apiFan2);
  rest.function("apiFan3", apiFan3);
  rest.function("apiFan4", apiFan4);
  rest.function("apiHeater", apiHeater);
  rest.function("apiCooler", apiCooler);
  rest.function("apiFTempMin", apiFTempMin);
  rest.function("apiFTempMax", apiFTempMax);
  rest.function("apiHTempMin", apiHTempMin);
  rest.function("apiHTempMax", apiHTempMax);
  rest.function("apiCTempMin", apiCTempMin);
  rest.function("apiCTempMax", apiCTempMax);

  // Give name & ID to the device (ID should be 6 characters long)
  rest.set_id("1");
  rest.set_name("MUSTIKA_CONTROLLER");
  Serial.println("REST ID = 1 & NAME = MUSTIKA_CONTROLLER");

  WiFi.begin(ssid, password);
  Serial.print("Connecting to: ");
  Serial.print(ssid);
  Serial.print(" ");
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Connecting to.. ");
  lcd.setCursor(0, 1);
  lcd.print(ssid);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  lcd.clear();
  lcd.setCursor(0, 1);
  lcd.print("WiFi Connected!");
  delay(2000);

  WiFi.softAP(ssidAP, passwordAP);
  server.begin();
  Serial.println(WiFi.localIP());
}

void loop()
{
  //Handle client rest, taruh di loop paling atas
  WiFiClient client = server.available();
  rest.handle(client);
  current = millis();
  //rtc
  DateTime now = rtc.now();

  uint32_t start = micros();
  hum = SHT2x.GetHumidity();
  temp = SHT2x.GetTemperature();
  if (hum <= 0 && temp <= 0)
  {
    hum = 20;
    temp = 28;
  }
  Serial.print("Humidity(%RH): ");
  Serial.print(hum, 0);
  Serial.print("\tTemperature(C): ");
  Serial.print(temp, 0);

  uint32_t stop = micros();
  Serial.print("\tRead Time: ");
  Serial.println(stop - start);
  delay(1000);

  //amonia
  MQ135.update();           // Update data, the arduino will be read the voltage on the analog pin
  amo = MQ135.readSensor(); // Sensor will read PPM concentration using the model and a and b values setted before or in the setup
  ppm = amo;
  Serial.println(amo);
  //  MQ135.serialDebug(); // Will print the table on the serial port
  delay(500); //Sampling frequency
  //amonia
//   amo = 300 * 1000;
  count = timeSecond;

  if (timeSecond - count < 300000)
  {
    count = timeSecond;
  }

  timeSecond = (esp_timer_get_time() * 0.000001);

  if (timeSecond - count < 5)

  {
    WiFiClient client = server.available();
    rest.handle(client);
    kirim_data();
  }

  // Serial.print(fTempMin);
  // Serial.print(" ");
  // Serial.print(fTempMax);
  // Serial.print(" ");
  // Serial.print(fan1);
  // Serial.print(" ");
  // Serial.print(fan2);
  // Serial.print(" ");
  // Serial.print(fan3);
  // Serial.print(" ");
  // Serial.print(fan4);
  // Serial.print(" ");
  // Serial.print(cooler1);
  // Serial.print(" ");
  // Serial.println(heater1);
  // delay(1000);
  // #include "displayMenu.h"

switch (state)
{
case algoritma:
    lcd.setCursor(0, 0);
    lcd.print("Algoritma");
    lcd.setCursor(10, 0);
    lcd.print("HUM: ");
    lcd.print(hum);
    lcd.setCursor(0, 1);
    lcd.print("T: ");
    lcd.print(temp);
    lcd.setCursor(11, 1);
    lcd.print("PPM: ");
    lcd.print(ppm);
    lcd.setCursor(2, 2);
    lcd.print(daysOfTheWeek[now.dayOfTheWeek()]);
    lcd.print(",");
    lcd.print(now.day(), DEC);
    lcd.print('/');
    lcd.print(now.month(), DEC);
    lcd.print('/');
    lcd.print(now.year(), DEC);
    lcd.setCursor(6, 3);
    lcd.print(now.hour(), DEC);
    lcd.print(':');
    lcd.print(now.minute(), DEC);
    lcd.print(':');
    lcd.print(now.second(), DEC);
    timeNow = timeSecond - timer1;
    //fan1
    if (stateTemp == 0)
    {
        stateTemp = 1;
        timer1 = timeSecond;
        //        timer2 = timeSecond;
        digitalWrite(relay1, relayON);
        //        digitalWrite(relay5, relayON);
    }

    if (fan1 > 0 && temp < fan1 && stateTemp == 1)
    {
        if (timeSecond - timer1 > f)
        {
            timer1 = timeSecond;
            digitalWrite(relay1, relayON);
            digitalWrite(ledPin1, HIGH);
        }
        else if (timeSecond - timer1 > n)
        {
            digitalWrite(relay1, relayOFF);
            digitalWrite(ledPin1, LOW);
        }
    }
    else
    {
        stateTemp = 0;
        digitalWrite(relay1, relayOFF);
        //        digitalWrite(relay5, relayOFF);
    }

    //fan2
    if (fan2 > 0 && temp > fan2)
    {
        digitalWrite(relay2, relayON);
        digitalWrite(ledPin2, HIGH);
    }
    else
    {
        digitalWrite(relay2, relayOFF);
        digitalWrite(ledPin2, LOW);
    }

    //fan3
    if (fan3 > 0 && temp > fan3)
    {
        digitalWrite(relay3, relayON);
        digitalWrite(ledPin3, HIGH);
    }
    else
    {
        digitalWrite(relay3, relayOFF);
        digitalWrite(ledPin3, LOW);
    }

    //fan4
    if (fan4 > 0 && temp > fan4)
    {
        digitalWrite(relay4, relayON);
        digitalWrite(ledPin4, HIGH);
    }
    else
    {
        digitalWrite(relay4, relayOFF);
        digitalWrite(ledPin4, LOW);
    }

    if (stateTemp1 == 0)
    {
        stateTemp1 = 1;
        timer3 = timeSecond;
        digitalWrite(relay6, relayON);
    }

    //heater
    if (heater1 > 0 && temp < heater1 && stateTemp1 == 1)
    {
        if (timeSecond - timer3 > f2)
        {
            timer3 = timeSecond;
            digitalWrite(relay6, relayON);
        }
        else if (timeSecond - timer3 > n2)
        {
            digitalWrite(relay6, relayOFF);
        }
    }
    else
    {
        stateTemp1 = 0;
        digitalWrite(relay6, relayOFF);
    }

    //cooler
    if (stateTemp2 == 0)
    {
        stateTemp2 = 1;
        timer2 = timeSecond;
        digitalWrite(relay5, relayON);
    }

    if (cooler1 > 0 && temp > cooler1 && stateTemp2 == 1)
    {
        if (timeSecond - timer2 > f1)
        {
            timer2 = timeSecond;
            digitalWrite(relay5, relayON);
            digitalWrite(ledPin5, HIGH);
        }
        else if (timeSecond - timer2 > n1)
        {
            digitalWrite(relay5, relayOFF);
            digitalWrite(ledPin5, LOW);
        }
    }
    else
    {
        stateTemp2 = 0;
        digitalWrite(relay5, relayOFF);
        digitalWrite(ledPin5, LOW);
    }

    if (ok)
    {
        delay(delayButton);
        while (ok)
        {
            lcd.clear();
        }
        state = menu1;
    }
    break;

case awal:
    lcd.setCursor(0, 0);
    lcd.print(" MENU ");
    lcd.setCursor(10, 0);
    lcd.print("HUM: ");
    lcd.print(hum);
    lcd.setCursor(0, 1);
    lcd.print("T: ");
    lcd.print(temp);
    lcd.setCursor(11, 1);
    lcd.print("PPM: ");
    lcd.print(ppm);
    lcd.setCursor(2, 2);
    lcd.print(daysOfTheWeek[now.dayOfTheWeek()]);
    lcd.print(",");
    lcd.print(now.day(), DEC);
    lcd.print('/');
    lcd.print(now.month(), DEC);
    lcd.print('/');
    lcd.print(now.year(), DEC);
    lcd.setCursor(6, 3);
    lcd.print(now.hour(), DEC);
    lcd.print(':');
    lcd.print(now.minute(), DEC);
    lcd.print(':');
    lcd.print(now.second(), DEC);
    digitalWrite(relay1, relayOFF);
    digitalWrite(relay2, relayOFF);
    digitalWrite(relay3, relayOFF);
    digitalWrite(relay4, relayOFF);
    digitalWrite(relay5, relayOFF);
    digitalWrite(relay6, relayOFF);
    digitalWrite(ledPin1, LOW);
    digitalWrite(ledPin2, LOW);
    digitalWrite(ledPin3, LOW);
    digitalWrite(ledPin4, LOW);
    digitalWrite(ledPin5, LOW);
    digitalWrite(ledPin6, LOW);
    if (ok)
    {
        delay(delayButton);
        state = menu1;
        lcd.clear();
    };
    if (cancel)
    {
        delay(delayButton);
        state = algoritma;
        lcd.clear();
        stateTemp = 0;
        stateTemp1 = 0;
        stateTemp2 = 0;
        n = (timeron * 60);
        f = n + (timeroff * 60);
        n1 = (timeron1 * 60);
        f1 = n1 + (timeroff1 * 60);
        n2 = (timeron2 * 60);
        f2 = n1 + (timeroff2 * 60);
        EEPROM.write(addrfan[0], fan1);
        EEPROM.write(addrfan[1], fan2);
        EEPROM.write(addrfan[2], fan3);
        EEPROM.write(addrfan[3], fan4);
        EEPROM.write(addrcool, cooler1);
        EEPROM.write(addrheat, heater1);
        EEPROM.write(addrFanTempMin, fTempMin);
        EEPROM.write(addrFanTempMax, fTempMax);
        EEPROM.commit();
    };
    break;

case menu1:
    digitalWrite(ledPin1, HIGH);
    lcd.setCursor(0, 0);
    lcd.print(">Temp Fan1     ");
    lcd.print(fan1);
    lcd.setCursor(0, 1);
    lcd.print(" Temp Fan2     ");
    lcd.print(fan2);
    lcd.setCursor(0, 2);
    lcd.print(" Temp Fan3     ");
    lcd.print(fan3);
    lcd.setCursor(0, 3);
    lcd.print(" Temp Fan4     ");
    lcd.print(fan4);
    if (ok)
    {
        delay(delayButton);
        state = setfan1;
        digitalWrite(ledPin1, LOW);
        lcd.clear();
    };
    if (cancel)
    {
        delay(delayButton);
        state = awal;
        digitalWrite(ledPin1, LOW);
        lcd.clear();
    };
    if (up)
    {
        delay(delayButton);
        state = menu13;
        digitalWrite(ledPin1, LOW);
        lcd.clear();
    };
    if (down)
    {
        delay(delayButton);
        state = menu2;
        digitalWrite(ledPin1, LOW);
        lcd.clear();
    };
    break;

case menu2:
    digitalWrite(ledPin2, HIGH);
    lcd.setCursor(0, 0);
    lcd.print(" Temp Fan1     ");
    lcd.print(fan1);
    lcd.setCursor(0, 1);
    lcd.print(">Temp Fan2     ");
    lcd.print(fan2);
    lcd.setCursor(0, 2);
    lcd.print(" Temp Fan3     ");
    lcd.print(fan3);
    lcd.setCursor(0, 3);
    lcd.print(" Temp Fan4     ");
    lcd.print(fan4);
    if (ok)
    {
        delay(delayButton);
        state = setfan2;
        digitalWrite(ledPin2, LOW);
        lcd.clear();
    };
    if (cancel)
    {
        delay(delayButton);
        state = awal;
        digitalWrite(ledPin2, LOW);
        lcd.clear();
    };
    if (up)
    {
        delay(delayButton);
        state = menu1;
        digitalWrite(ledPin2, LOW);
        lcd.clear();
    };
    if (down)
    {
        delay(delayButton);
        state = menu3;
        digitalWrite(ledPin2, LOW);
        lcd.clear();
    };
    break;

case menu3:
    digitalWrite(ledPin3, HIGH);
    lcd.setCursor(0, 0);
    lcd.print(" Temp Fan1     ");
    lcd.print(fan1);
    lcd.setCursor(0, 1);
    lcd.print(" Temp Fan2     ");
    lcd.print(fan2);
    lcd.setCursor(0, 2);
    lcd.print(">Temp Fan3     ");
    lcd.print(fan3);
    lcd.setCursor(0, 3);
    lcd.print(" Temp Fan4     ");
    lcd.print(fan4);
    if (ok)
    {
        delay(delayButton);
        state = setfan3;
        digitalWrite(ledPin3, LOW);
        lcd.clear();
    };
    if (cancel)
    {
        delay(delayButton);
        state = awal;
        digitalWrite(ledPin3, LOW);
        lcd.clear();
    };
    if (up)
    {
        delay(delayButton);
        state = menu2;
        digitalWrite(ledPin3, LOW);
        lcd.clear();
    };
    if (down)
    {
        delay(delayButton);
        state = menu4;
        digitalWrite(ledPin3, LOW);
        lcd.clear();
    };
    break;

case menu4:
    digitalWrite(ledPin4, HIGH);
    lcd.setCursor(0, 0);
    lcd.print(" Temp Fan1     ");
    lcd.print(fan1);
    lcd.setCursor(0, 1);
    lcd.print(" Temp Fan2     ");
    lcd.print(fan2);
    lcd.setCursor(0, 2);
    lcd.print(" Temp Fan3     ");
    lcd.print(fan3);
    lcd.setCursor(0, 3);
    lcd.print(">Temp Fan4     ");
    lcd.print(fan4);
    if (ok)
    {
        delay(delayButton);
        state = setfan4;
        digitalWrite(ledPin4, LOW);
        lcd.clear();
    };
    if (cancel)
    {
        delay(delayButton);
        state = awal;
        digitalWrite(ledPin4, LOW);
        lcd.clear();
    };
    if (up)
    {
        delay(delayButton);
        state = menu3;
        digitalWrite(ledPin4, LOW);
        lcd.clear();
    };
    if (down)
    {
        delay(delayButton);
        state = menu5;
        digitalWrite(ledPin4, LOW);
        lcd.clear();
    };
    break;

case menu5:
    digitalWrite(ledPin5, HIGH);
    lcd.setCursor(0, 0);
    lcd.print(">Set Cooler   ");
    lcd.print(cooler1);
    lcd.setCursor(0, 1);
    lcd.print(" Set Heater   ");
    lcd.print(heater1);
    lcd.setCursor(0, 2);
    lcd.print(" Fan On       ");
    lcd.print(timeron);
    lcd.setCursor(0, 3);
    lcd.print(" Fan Off      ");
    lcd.print(timeroff);
    if (ok)
    {
        delay(delayButton);
        state = cooler;
        digitalWrite(ledPin5, LOW);
        lcd.clear();
    };
    if (cancel)
    {
        delay(delayButton);
        state = awal;
        digitalWrite(ledPin5, LOW);
        lcd.clear();
    };
    if (up)
    {
        delay(delayButton);
        state = menu4;
        digitalWrite(ledPin5, LOW);
        lcd.clear();
    };
    if (down)
    {
        delay(delayButton);
        state = menu6;
        digitalWrite(ledPin5, LOW);
        lcd.clear();
    };
    break;

case menu6:
    digitalWrite(ledPin6, HIGH);
    lcd.setCursor(0, 0);
    lcd.print(" Set Cooler   ");
    lcd.print(cooler1);
    lcd.setCursor(0, 1);
    lcd.print(">Set Heater   ");
    lcd.print(heater1);
    lcd.setCursor(0, 2);
    lcd.print(" Fan On       ");
    lcd.print(timeron);
    lcd.setCursor(0, 3);
    lcd.print(" Fan Off      ");
    lcd.print(timeroff);
    if (ok)
    {
        delay(delayButton);
        state = heater;
        digitalWrite(ledPin6, LOW);
        lcd.clear();
    };
    if (cancel)
    {
        delay(delayButton);
        state = awal;
        digitalWrite(ledPin6, LOW);
        lcd.clear();
    };
    if (up)
    {
        delay(delayButton);
        state = menu5;
        digitalWrite(ledPin6, LOW);
        lcd.clear();
    };
    if (down)
    {
        delay(delayButton);
        state = menu7;
        digitalWrite(ledPin6, LOW);
        lcd.clear();
    };
    break;

case menu7:
    digitalWrite(ledPin1, HIGH);
    lcd.setCursor(0, 0);
    lcd.print(" Set Cooler   ");
    lcd.print(cooler1);
    lcd.setCursor(0, 1);
    lcd.print(" Set Heater   ");
    lcd.print(heater1);
    lcd.setCursor(0, 2);
    lcd.print(">Fan On       ");
    lcd.print(timeron);
    lcd.setCursor(0, 3);
    lcd.print(" Fan Off      ");
    lcd.print(timeroff);
    if (ok)
    {
        delay(delayButton);
        state = timeon;
        digitalWrite(ledPin1, LOW);
        lcd.clear();
    };
    if (cancel)
    {
        delay(delayButton);
        state = awal;
        digitalWrite(ledPin1, LOW);
        lcd.clear();
    };
    if (up)
    {
        delay(delayButton);
        state = menu6;
        digitalWrite(ledPin1, LOW);
        lcd.clear();
    };
    if (down)
    {
        delay(delayButton);
        state = menu8;
        digitalWrite(ledPin1, LOW);
        lcd.clear();
    };
    break;

case menu8:
    digitalWrite(ledPin2, HIGH);
    lcd.setCursor(0, 0);
    lcd.print(" Set Cooler   ");
    lcd.print(cooler1);
    lcd.setCursor(0, 1);
    lcd.print(" Set Heater   ");
    lcd.print(heater1);
    lcd.setCursor(0, 2);
    lcd.print(" Fan On       ");
    lcd.print(timeron);
    lcd.setCursor(0, 3);
    lcd.print(">Fan Off      ");
    lcd.print(timeroff);
    if (ok)
    {
        delay(delayButton);
        state = timeoff;
        digitalWrite(ledPin2, LOW);
        lcd.clear();
    };
    if (cancel)
    {
        delay(delayButton);
        state = awal;
        digitalWrite(ledPin2, LOW);
        lcd.clear();
    };
    if (up)
    {
        delay(delayButton);
        state = menu7;
        digitalWrite(ledPin2, LOW);
        lcd.clear();
    };
    if (down)
    {
        delay(delayButton);
        state = menu9;
        digitalWrite(ledPin2, LOW);
        lcd.clear();
    };
    break;

case menu9:
    digitalWrite(ledPin3, HIGH);
    lcd.setCursor(0, 0);
    lcd.print(">Cool On      ");
    lcd.print(timeron1);
    lcd.setCursor(0, 1);
    lcd.print(" Cool Off     ");
    lcd.print(timeroff1);
    lcd.setCursor(0, 2);
    lcd.print(" Heat On      ");
    lcd.print(timeron2);
    lcd.setCursor(0, 3);
    lcd.print(" Heat Off     ");
    lcd.print(timeroff2);
    if (ok)
    {
        delay(delayButton);
        state = timeon1;
        digitalWrite(ledPin3, LOW);
        lcd.clear();
    };
    if (cancel)
    {
        delay(delayButton);
        state = awal;
        digitalWrite(ledPin3, LOW);
        lcd.clear();
    };
    if (up)
    {
        delay(delayButton);
        state = menu8;
        digitalWrite(ledPin3, LOW);
        lcd.clear();
    };
    if (down)
    {
        delay(delayButton);
        state = menu10;
        digitalWrite(ledPin3, LOW);
        lcd.clear();
    };
    break;

case menu10:
    digitalWrite(ledPin4, HIGH);
    lcd.setCursor(0, 0);
    lcd.print(" Cool On      ");
    lcd.print(timeron1);
    lcd.setCursor(0, 1);
    lcd.print(">Cool Off     ");
    lcd.print(timeroff1);
    lcd.setCursor(0, 2);
    lcd.print(" Heat On      ");
    lcd.print(timeron2);
    lcd.setCursor(0, 3);
    lcd.print(" Heat Off     ");
    lcd.print(timeroff2);
    if (ok)
    {
        delay(delayButton);
        state = timeoff1;
        digitalWrite(ledPin4, LOW);
        lcd.clear();
    };
    if (cancel)
    {
        delay(delayButton);
        state = awal;
        digitalWrite(ledPin4, LOW);
        lcd.clear();
    };
    if (up)
    {
        delay(delayButton);
        state = menu9;
        digitalWrite(ledPin4, LOW);
        lcd.clear();
    };
    if (down)
    {
        delay(delayButton);
        state = menu11;
        digitalWrite(ledPin4, LOW);
        lcd.clear();
    };
    break;

case menu11:
    digitalWrite(ledPin5, HIGH);
    lcd.setCursor(0, 0);
    lcd.print(" Cool On      ");
    lcd.print(timeron1);
    lcd.setCursor(0, 1);
    lcd.print(" Cool Off     ");
    lcd.print(timeroff1);
    lcd.setCursor(0, 2);
    lcd.print(">Heat On      ");
    lcd.print(timeron2);
    lcd.setCursor(0, 3);
    lcd.print(" Heat Off     ");
    lcd.print(timeroff2);
    if (ok)
    {
        delay(delayButton);
        state = timeon2;
        digitalWrite(ledPin5, LOW);
        lcd.clear();
    };
    if (cancel)
    {
        delay(delayButton);
        state = awal;
        digitalWrite(ledPin5, LOW);
        lcd.clear();
    };
    if (up)
    {
        delay(delayButton);
        state = menu10;
        digitalWrite(ledPin5, LOW);
        lcd.clear();
    };
    if (down)
    {
        delay(delayButton);
        state = menu12;
        digitalWrite(ledPin5, LOW);
        lcd.clear();
    };
    break;

case menu12:
    digitalWrite(ledPin6, HIGH);
    lcd.setCursor(0, 0);
    lcd.print(" Cool On      ");
    lcd.print(timeron1);
    lcd.setCursor(0, 1);
    lcd.print(" Cool Off     ");
    lcd.print(timeroff1);
    lcd.setCursor(0, 2);
    lcd.print(" Heat On      ");
    lcd.print(timeron2);
    lcd.setCursor(0, 3);
    lcd.print(">Heat Off     ");
    lcd.print(timeroff2);
    if (ok)
    {
        delay(delayButton);
        state = timeoff2;
        digitalWrite(ledPin6, LOW);
        lcd.clear();
    };
    if (cancel)
    {
        delay(delayButton);
        state = awal;
        digitalWrite(ledPin6, LOW);
        lcd.clear();
    };
    if (up)
    {
        delay(delayButton);
        state = menu11;
        digitalWrite(ledPin6, LOW);
        lcd.clear();
    };
    if (down)
    {
        delay(delayButton);
        state = menu13;
        digitalWrite(ledPin6, LOW);
        lcd.clear();
    };
    break;

case menu13:
    digitalWrite(ledPin6, HIGH);
    lcd.setCursor(0, 0);
    lcd.print("Reset ");
    if (ok)
    {
        delay(delayButton);
        state = tidak;
        digitalWrite(ledPin6, LOW);
        lcd.clear();
    };
    if (cancel)
    {
        delay(delayButton);
        state = awal;
        digitalWrite(ledPin6, LOW);
        lcd.clear();
    };
    if (up)
    {
        delay(delayButton);
        state = menu12;
        digitalWrite(ledPin6, LOW);
        lcd.clear();
    };
    if (down)
    {
        delay(delayButton);
        state = menu1;
        digitalWrite(ledPin6, LOW);
        lcd.clear();
    };
    break;

case setfan1:
    digitalWrite(ledPin1, HIGH);
    lcd.setCursor(0, 0);
    lcd.print(" Temp Fan1 = ");
    lcd.print(fan1);
    EEPROM.write(addrfan[0], fan1);
    if (ok)
    {
        delay(delayButton);
        state = setfan1;
        lcd.clear();
    };
    if (cancel)
    {
        delay(delayButton);
        state = menu1;
        lcd.clear();
    };
    if (up)
    {
        fan1 += 0.1;
        delay(delayButton / 100);
    };
    if (down)
    {
        fan1 -= 0.1;
        delay(delayButton / 100);
    };
    break;

case setfan2:
    digitalWrite(ledPin2, HIGH);
    lcd.setCursor(0, 0);
    lcd.print(" Temp Fan2 = ");
    lcd.print(fan2);
    EEPROM.write(addrfan[1], fan2);
    if (ok)
    {
        delay(delayButton);
        state = setfan2;
    };
    if (cancel)
    {
        delay(delayButton);
        state = menu2;
        lcd.clear();
    };
    if (up)
    {
        delay(delayButton / 100);
        fan2 += 0.1;
    };
    if (down)
    {
        delay(delayButton / 100);
        fan2 -= 0.1;
    };
    break;

case setfan3:
    digitalWrite(ledPin3, HIGH);
    lcd.setCursor(0, 0);
    lcd.print(" Temp Fan3 = ");
    lcd.print(fan3);
    EEPROM.write(addrfan[2], fan3);
    if (ok)
    {
        delay(delayButton);
        state = setfan3;
        lcd.clear();
    };
    if (cancel)
    {
        delay(delayButton);
        state = menu3;
    };
    if (up)
    {
        delay(delayButton / 100);
        fan3 += 0.1;
    };
    if (down)
    {
        delay(delayButton / 100);
        fan3 -= 0.1;
    };
    break;

case setfan4:
    digitalWrite(ledPin4, HIGH);
    lcd.setCursor(0, 0);
    lcd.print(" Temp Fan4 = ");
    lcd.print(fan4);
    EEPROM.write(addrfan[3], fan4);
    if (ok)
    {
        delay(delayButton);
        state = setfan4;
        lcd.clear();
    };
    if (cancel)
    {
        delay(delayButton);
        state = menu4;
        lcd.clear();
    };
    if (up)
    {
        delay(delayButton / 100);
        fan4 += 0.1;
    };
    if (down)
    {
        delay(delayButton / 100);
        fan4 -= 0.1;
    };
    break;

case cooler:
    digitalWrite(ledPin5, HIGH);
    lcd.setCursor(0, 0);
    lcd.print(" Cooler = ");
    lcd.print(cooler1);
    EEPROM.write(addrcool, cooler);
    if (ok)
    {
        delay(delayButton);
        state = cooler;
    };
    if (cancel)
    {
        delay(delayButton);
        state = menu5;
    };
    if (up)
    {
        delay(delayButton / 100);
        cooler1 += 0.1;
    };
    if (down)
    {
        delay(delayButton / 100);
        cooler1 -= 0.1;
    };
    break;

case heater:
    digitalWrite(ledPin6, HIGH);
    lcd.setCursor(0, 0);
    lcd.print(" Heater = ");
    lcd.print(heater1);
    EEPROM.write(addrheat, heater);
    if (ok)
    {
        delay(delayButton);
        state = heater;
    };
    if (cancel)
    {
        delay(delayButton);
        state = menu6;
    };
    if (up)
    {
        delay(delayButton / 100);
        heater1 += 0.1;
    };
    if (down)
    {
        delay(delayButton / 100);
        heater1 -= 0.1;
    };
    break;

case timeon:
    digitalWrite(ledPin1, HIGH);
    lcd.setCursor(0, 0);
    lcd.print(" Timer On = ");
    lcd.print(timeron);
    EEPROM.write(addrtn[0], timeron);
    if (ok)
    {
        delay(delayButton);
        state = timeon;
        lcd.clear();
    };
    if (cancel)
    {
        delay(delayButton);
        state = menu7;
    };
    if (up)
    {
        delay(delayButton / 100);
        timeron += 0.1;
    };
    if (down)
    {
        delay(delayButton / 100);
        timeron -= 0.1;
    };
    break;

case timeoff:
    digitalWrite(ledPin2, HIGH);
    lcd.setCursor(0, 0);
    lcd.print(" Timer Off = ");
    lcd.print(timeroff);
    EEPROM.write(addrtf[0], timeroff);
    if (ok)
    {
        delay(delayButton);
        state = timeoff;
        lcd.clear();
    };
    if (cancel)
    {
        delay(delayButton);
        state = menu8;
    };
    if (up)
    {
        delay(delayButton / 100);
        timeroff += 0.1;
    };
    if (down)
    {
        delay(delayButton / 100);
        timeroff -= 0.1;
    };
    break;

case timeon1:
    digitalWrite(ledPin3, HIGH);
    lcd.setCursor(0, 0);
    lcd.print(" Timer On = ");
    lcd.print(timeron1);
    EEPROM.write(addrtn[1], timeron1);
    if (ok)
    {
        delay(delayButton);
        state = timeon1;
        lcd.clear();
    };
    if (cancel)
    {
        delay(delayButton);
        state = menu9;
    };
    if (up)
    {
        delay(delayButton / 100);
        timeron1 += 0.1;
    };
    if (down)
    {
        delay(delayButton / 100);
        timeron1 -= 0.1;
    };
    break;

case timeoff1:
    digitalWrite(ledPin4, HIGH);
    lcd.setCursor(0, 0);
    lcd.print(" Timer Off = ");
    lcd.print(timeroff1);
    EEPROM.write(addrtf[1], timeroff1);
    if (ok)
    {
        delay(delayButton);
        state = timeoff1;
        lcd.clear();
    };
    if (cancel)
    {
        delay(delayButton);
        state = menu10;
    };
    if (up)
    {
        delay(delayButton / 100);
        timeroff1 += 0.1;
    };
    if (down)
    {
        delay(delayButton / 100);
        timeroff1 -= 0.1;
    };
    break;

case timeon2:
    digitalWrite(ledPin5, HIGH);
    lcd.setCursor(0, 0);
    lcd.print(" Timer On = ");
    lcd.print(timeron2);
    EEPROM.write(addrtn[2], timeron2);
    if (ok)
    {
        delay(delayButton);
        state = timeon2;
        lcd.clear();
    };
    if (cancel)
    {
        delay(delayButton);
        state = menu11;
    };
    if (up)
    {
        delay(delayButton / 100);
        timeron2 += 0.1;
    };
    if (down)
    {
        delay(delayButton / 100);
        timeron2 -= 0.1;
    };
    break;

case timeoff2:
    digitalWrite(ledPin6, HIGH);
    lcd.setCursor(0, 0);
    lcd.print(" Timer Off = ");
    lcd.print(timeroff2);
    EEPROM.write(addrtf[2], timeroff2);
    if (ok)
    {
        delay(delayButton);
        state = timeoff2;
        lcd.clear();
    };
    if (cancel)
    {
        delay(delayButton);
        state = menu12;
    };
    if (up)
    {
        delay(delayButton / 100);
        timeroff2 += 0.1;
    };
    if (down)
    {
        delay(delayButton / 100);
        timeroff2 -= 0.1;
    };
    EEPROM.commit();
    break;

case tidak:
    lcd.setCursor(1, 0);
    lcd.print("Apakah Anda Yakin ??");
    lcd.setCursor(0, 1);
    lcd.print(">Tidak     ");
    lcd.setCursor(0, 2);
    lcd.print(" Ya     ");
    if (ok)
    {
        delay(delayButton);
        state = menu13;
        lcd.clear();
    };
    if (up)
    {
        delay(delayButton / 100);
        state = ya;
        lcd.clear();
    };
    if (down)
    {
        delay(delayButton / 100);
        state = ya;
        lcd.clear();
    };
    break;

case ya:
    lcd.setCursor(1, 0);
    lcd.print("Apakah Anda Yakin ??");
    lcd.setCursor(0, 1);
    lcd.print(" Tidak     ");
    lcd.setCursor(0, 2);
    lcd.print(">Ya     ");
    if (ok)
    {
        delay(delayButton);
        fan1 = 0;
        fan2 = 0;
        fan3 = 0;
        fan4 = 0;
        cooler1 = 0;
        heater1 = 0;
        timeron = 0;
        timeroff = 0;
        timeron1 = 0;
        timeroff1 = 0;
        timeron2 = 0;
        timeroff2 = 0;
        EEPROM.write(addrfan[0], fan1);
        EEPROM.write(addrfan[1], fan2);
        EEPROM.write(addrfan[2], fan3);
        EEPROM.write(addrfan[3], fan4);
        EEPROM.write(addrcool, cooler1);
        EEPROM.write(addrheat, heater1);
        EEPROM.write(addrtn[0], timeron);
        EEPROM.write(addrtf[0], timeroff);
        EEPROM.write(addrtn[1], timeron1);
        EEPROM.write(addrtf[1], timeroff1);
        EEPROM.write(addrtn[2], timeron2);
        EEPROM.write(addrtf[2], timeroff2);
        state = menu1;
        lcd.clear();
    };
    if (up)
    {
        delay(delayButton / 100);
        state = tidak;
        lcd.clear();
    };
    if (down)
    {
        delay(delayButton / 100);
        state = tidak;
        lcd.clear();
    };
    break;
}

}


/* Functions */

int apiFan1(String command)
{
  // Get state from command
  int fan1State = command.toInt();
  EEPROM.write(addrfan[0], fan1State);
  EEPROM.commit();
  printMessage(fan1State);
  return 1;
}

int apiFan2(String command)
{
  // Get state from command
  int fan2State = command.toInt();
  EEPROM.write(addrfan[1], fan2State);
  EEPROM.commit();
  printMessage(fan2State);
  return 1;
}

int apiFan3(String command)
{
  // Get state from command
  int fan3State = command.toInt();
  EEPROM.write(addrfan[2], fan3State);
  EEPROM.commit();
  printMessage(fan3State);
  return 1;
}

int apiFan4(String command)
{
  // Get state from command
  int fan4State = command.toInt();
  EEPROM.write(addrfan[3], fan4State);
  EEPROM.commit();
  printMessage(fan4State);
  return 1;
}

int apiHeater(String command)
{
  // Get state from command
  int heaterState = command.toInt();
  EEPROM.write(addrheat, heaterState);
  EEPROM.commit();
  printMessage(heaterState);
  return 1;
}

int apiCooler(String command)
{
  // Get state from command
  int coolerState = command.toInt();
  EEPROM.write(addrcool, coolerState);
  EEPROM.commit();
  printMessage(coolerState);
  return 1;
}

int apiFTempMin(String command)
{
  // Get state from command
  int tempMinState = command.toInt();
  EEPROM.write(addrFanTempMin, tempMinState);
  EEPROM.commit();
  printMessage(tempMinState);
  return 1;
}

int apiFTempMax(String command)
{
  // Get state from command
  int tempMaxState = command.toInt();
  EEPROM.write(addrFanTempMax, tempMaxState);
  EEPROM.commit();
  printMessage(tempMaxState);
  return 1;
}

int apiHTempMin(String command)
{
  // Get state from command
  int hMinState = command.toInt();
  EEPROM.write(addrHeatMin, hMinState);
  EEPROM.commit();
  printMessage(hMinState);
  return 1;
}

int apiHTempMax(String command)
{
  // Get state from command
  int hMaxState = command.toInt();
  EEPROM.write(addrHeatMax, hMaxState);
  EEPROM.commit();
  printMessage(hMaxState);
  return 1;
}

int apiCTempMin(String command)
{
  // Get state from command
  int cMinState = command.toInt();
  EEPROM.write(addrCoolMin, cMinState);
  EEPROM.commit();
  printMessage(cMinState);
  return 1;
}

int apiCTempMax(String command)
{
  // Get state from command
  int cMaxState = command.toInt();
  EEPROM.write(addrCoolMax, cMaxState);
  EEPROM.commit();
  printMessage(cMaxState);
  return 1;
}

void printMessage(int value)
{
  Serial.println("");
  Serial.println("");
  Serial.print("Value Saved = ");
  Serial.println(value);
  Serial.println("");
  Serial.println("");
}

void rtc_writetovar()
{
  //rtc
  DateTime now = rtc.now();
  second = (now.second());
  minute = (now.minute());
  hour = (now.hour());
  //  dayOfWeek = bcdToDec(now.);
  dayOfMonth = (now.day());
  month = (now.month());
  year = (now.year());

  Serial.print(second);
  Serial.print(minute);
  Serial.print(hour);
  Serial.print(dayOfMonth);
  Serial.print(month);
  Serial.println(year);
}
byte bcdToDec(byte val)
{
  return ((val / 16 * 10) + (val % 16));
}

void kirim_data()
{
  if (current - previous >= interval)
  {
    String sn = "2019030011";
    //    String dgw = "2021-01-29";
    //    String tgw = "14:13:00";
    String sensor1 = "temperature"; //temp
    String sensor2 = "amonia";      //amonia
    String sensor3 = "humidity";    //humidity
    String sensor4 = "fan1";
    String sensor5 = "fan2";
    String sensor6 = "fan3";
    String sensor7 = "fan4";
    String sensor8 = "heater1";
    String sensor9 = "cooler1";
    String sensor10 = "mintemp";
    String sensor11 = "maxtemp";

    rtc_writetovar();
    //    int temp = 26;
    // float amon = 5.2;
    //    int humid = 60;
    //    int fan3 = 1500; fan4 = 1500; cooler1 = 500; heater1 = 600; timeron = 5; timeroff = 5; timeron1 = 10; timeroff1 = 10; timeron2 = 20; timeroff2 = 20;
    String postData = (String) "&sn=" + sn + "&dgw=" + year + "-" + month + "-" + dayOfMonth + "&tgw=" + hour + ":" + minute + ":" + second +
                      "&sensor=" + sensor1 + "x" + sensor2 + "x" + sensor3 + "x" + sensor4 + "x" + sensor5 + "x" + sensor6 + "x" + sensor7 + "x" + sensor8 + "x" + sensor9 + "x" + sensor10 + "x" + sensor11 + 
                      "&nilai=" + temp + "x" + ppm + "x" + hum + "x" + fan1 + "x" + fan2 + "x" + fan3 + "x" + fan4 + "x" + cooler1 + "x" + heater1 + "x" + fTempMin + "x" + fTempMax;

    HTTPClient http;
    http.begin("http://www.smart-gh.com/input2.php?sn=2019030011" + postData);
    //    http.begin("http://www.smart-gh.com/input.php");
    http.addHeader("Content-Type", "application/x-www-form-urlencoded");

    auto httpCode = http.POST(postData);
    String payload = http.getString();

    Serial.println(postData);
    //    Serial.println(payload);
    http.end();
    // rest.handle(client);
  }
}
