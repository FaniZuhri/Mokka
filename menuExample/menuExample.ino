//Lib EEPROM for saving variables
#include "EEPROM.h"

//define EEPROM size
#define EEPROM_SIZE 100

//Lib API HTTP and REST
#include <HTTPClient.h>
#include <aREST.h>
//#define HARDWARE "esp32"

// go to aREST.h lib to config variables, functions, and buffer size
#define OUTPUT_BUFFER_SIZE 500
#define NUMBER_VARIABLES 20
#define NUMBER_FUNCTIONS 20

// Create aREST instance
aREST rest = aREST();

//Lib RTC Module
#include "RTClib.h"
// define RTC
#define DS3231_I2C_ADDRESS 0x68
RTC_DS3231 rtc;

//RTC variables
char daysOfTheWeek[7][12] = {"Minggu", "Senin", "Selasa", "Rabu", "Kamis", "Jumat", "Sabtu"};
String second, minute, hour, dayOfWeek, dayOfMonth, month, year;

//Lib LCD i2c
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 20, 4); // set the LCD address to 0x27 for a 16 chars and 2 line display

//Lib button
#include <Button.h>

//Lib DS18B20
#include <OneWire.h>
#include <DallasTemperature.h>

//define DS18B20 PIN
#define ONE_WIRE_BUS 15
OneWire oneWire(ONE_WIRE_BUS);

//Lib SHT21 / SHT30
// #include <HTU21D.h>
#include "SHT2x.h"

//Redefining SHT21 lib
SHT2x SHT2x;

//Lib WiFi
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiAP.h>
WiFiClient client;
//set ssid and password ESP32 as Access Point
const char *ssidAP = "Mustika Controller";
const char *passwordAP = "mustikajaya";

//set ssid and password to WiFi/Hotspot
const char *ssid = "Zuhri";
const char *password = "12345678";

//Redefining for making ESP32 as Access Point
WiFiServer server(80);
IPAddress AP_IP;

//Lib MQ137
#include <MQUnifiedsensor.h>

/* MQ-137 INITIALIZATION */

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

//functions that used in REST
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
int apiPpmTres(String command);

//state menu in LCD
enum menuLcd
{
  mainMenu,
  chooseFan1,
  chooseFan2,
  chooseFan3,
  chooseFan4,
  setFan1State,
  setFan2State,
  setFan3State,
  setFan4State,
  chooseHeater,
  chooseCooler,
  chooseTemp,
  chooseTempFanMax,
  chooseTempFanMin,
  chooseHeaterMax,
  chooseHeaterMin,
  chooseCoolerMax,
  chooseCoolerMin,
  setHeaterState,
  setCoolerState,
  setFanMax,
  setFanMin,
  setHeaterMax,
  setHeaterMin,
  setCoolerMax,
  setCoolerMin,
  yes,
  no,
};

//Sensor Variables
float ppm, temp, hum, amo;

//EEPROM Address for saving variables
int addrfan[4] = {2, 4, 6, 8};
int addrHeat = 10, addrCool = 12, addrFanTempMin = 16, addrFanTempMax = 14,
    addrHeatMin = 16, addrHeatMax = 18, addrCoolMin = 20, addrCoolMax = 22,
    addrPpmTres = 24;
int addr = 0;

//Pin for LED while in menu
int ledMenuFan1 = 2, ledMenuFan2 = 4, ledMenuFan3 = 16, ledMenuFan4 = 17, ledMenuHeater = 5, ledMenuCooler = 26;

//Pin for Buttons
int okBtn = 12, upBtn = 18, downBtn = 19, cancelBtn = 13, arrowState = 1;

//Fan and heater variables
int fan1, fan2, fan3, fan4, cooler1, heater1, delayBtn = 100, hTempMax, hTempMin, cTempMax, cTempMin, ppmTres;
int state, stateFTMax, stateFTMin, stateHTMax, stateHTMin, stateCTMax, stateCTMin, count, timeSecond, adc, fTempMin, fTempMax;

int current, previous = 0, interval = 5000;

//Pin for relay Fan and Heater
const int relayFan1 = 2;    // 26
const int relayFan2 = 4;    // 25
const int relayFan3 = 25;   // 33
const int relayFan4 = 32;   //32
const int relayHeater = 33; // 2
const int relayCooler = 26; // 4

//defining read button
#define ok !digitalRead(okBtn)
#define cancel !digitalRead(cancelBtn)
#define up !digitalRead(upBtn)
#define down !digitalRead(downBtn)

void setup()
{
  //Start Hardware Serial
  Serial.begin(9600);

#ifndef ESP8266
  while (!Serial)
    ;
#endif

  //Start SHT21
  SHT2x.begin();

  //Start EEPROM
  EEPROM.begin(EEPROM_SIZE);

  //set relay pin as OUTPUT
  pinMode(relayFan1, OUTPUT);
  pinMode(relayFan2, OUTPUT);
  pinMode(relayFan3, OUTPUT);
  pinMode(relayFan4, OUTPUT);
  pinMode(relayHeater, OUTPUT);
  pinMode(relayCooler, OUTPUT);

  //set led menu pin as output
  pinMode(ledMenuFan1, OUTPUT);
  pinMode(ledMenuFan2, OUTPUT);
  pinMode(ledMenuFan3, OUTPUT);
  pinMode(ledMenuFan4, OUTPUT);
  pinMode(ledMenuHeater, OUTPUT);
  pinMode(ledMenuCooler, OUTPUT);

  //Set button as INPUT
  pinMode(okBtn, INPUT_PULLUP);
  pinMode(cancelBtn, INPUT_PULLUP);
  pinMode(downBtn, INPUT_PULLUP);
  pinMode(upBtn, INPUT_PULLUP);

  //read var value from eeprom
  readFromEEPROM();

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
  rest.variable("ppmtres", &ppmTres);

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
  rest.function("apiPpmTres", apiPpmTres);

  // initialize the lcd
  lcd.init();
  lcd.init();
  lcd.backlight();
  lcd.clear();

  //First state to go to void loop
  state = mainMenu;

  //relay state
  relay(not fan1, not fan2, not fan3, not fan4, not heater1, not cooler1);

  // Give name & ID to the device (ID should be 6 characters long)
  rest.set_id("1");
  rest.set_name("MUSTIKA_CONTROLLER");
  Serial.println("REST ID = 1 & NAME = MUSTIKA_CONTROLLER");

  //Connect ESP32 to WiFi
  // You can remove the password parameter if you want the AP to be open.
  WiFi.mode(WIFI_OFF);
  delay(1000);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to: ");
  Serial.print(ssid);
  Serial.print(" ");
  lcd.clear();
  homeDisplay();
  lcd.setCursor(0, 2);
  lcd.print("Connecting to WiFi..");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  lcd.setCursor(0, 3);
  lcd.print("Connected!");
  delay(1000);

  //Start ESP32 as Access Point
  WiFi.softAP(ssidAP, passwordAP);
  IPAddress myIP = WiFi.softAPIP();
  AP_IP = myIP;
  Serial.print("AP IP address: ");
  Serial.println(AP_IP);
  lcd.clear();
  homeDisplay();
  Serial.println("Server started");
  lcd.setCursor(0, 2);
  lcd.print("Server started");
  lcd.setCursor(0, 3);
  lcd.print(AP_IP);
  server.begin();
  delay(3000);
  lcd.clear();
}

void loop()
{
  current = millis();

  //REST Handler
  WiFiClient client = server.available();
  //  rest.handle(client);

  //Menu Case
  switch (state)
  {

  //Main Menu, this case comes first as defined in void setup
  case mainMenu:

    //REST Handler
    //    WiFiClient client = server.available();
    rest.handle(client);

    //Display Sensor values and RTC to LCD
    mainDisplay();

    //All led OFF
    ledMenu(0, 0, 0, 0, 0, 0);

    //relay state
    relay(not fan1, not fan2, not fan3, not fan4, not heater1, not cooler1);

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
    Serial.println(temp, 0);
    mainDisplay();
    delay(500);

    //amonia measurement
    MQ135.update();           // Update data, the arduino will be read the voltage on the analog pin
    ppm = MQ135.readSensor(); // Sensor will read PPM concentration using the model and a and b values setted before or in the setup
    mainDisplay();
    Serial.println(ppm);
    //  MQ135.serialDebug(); // Will print the table on the serial port
    delay(500); //Sampling frequency

    //Double check that mainMenu is active
    Serial.println("main");

    amo = 300 * 1000;
    count = timeSecond;

    if (timeSecond - count < amo)
    {
      count = timeSecond;
    }

    timeSecond = (esp_timer_get_time() * 0.000001);

    if (timeSecond - count < 5)

    {
      kirim_data();
    }

    //If ANY of pin pressed
    if (ok or up or down or cancel)
    {
      delay(delayBtn);
      state = chooseFan1;
      relay(1, 1, 1, 1, 1, 1);
      lcd.clear();
    }
    break;

  //This is first case in Setting Menu. You will arrived in chooseFan1 Menu
  case chooseFan1:
    lcd.setCursor(0, 0);
    lcd.print(">Fan 1 State   ");
    lcd.print(fan1);
    lcd.setCursor(0, 1);
    lcd.print(" Fan 2 State   ");
    lcd.print(fan2);
    lcd.setCursor(0, 2);
    lcd.print(" Fan 3 State   ");
    lcd.print(fan3);
    lcd.setCursor(0, 3);
    lcd.print(" Fan 4 State   ");
    lcd.print(fan4);
    Serial.println("choose fan 1");
    ledMenu(1, 0, 0, 0, 0, 0);

    //If button pressed
    if (ok)
    {
      delay(delayBtn);
      state = setFan1State;
      lcd.clear();
    }
    if (up)
    {
      delay(delayBtn);
      state = chooseTemp;
      lcd.clear();
    }
    if (down)
    {
      delay(delayBtn);
      state = chooseFan2;
      lcd.clear();
    }
    if (cancel)
    {
      delay(delayBtn);
      state = mainMenu;
      lcd.clear();
    }
    break;

  case chooseFan2:
    lcd.setCursor(0, 0);
    lcd.print(" Fan 1 State   ");
    lcd.print(fan1);
    lcd.setCursor(0, 1);
    lcd.print(">Fan 2 State   ");
    lcd.print(fan2);
    lcd.setCursor(0, 2);
    lcd.print(" Fan 3 State   ");
    lcd.print(fan3);
    lcd.setCursor(0, 3);
    lcd.print(" Fan 4 State   ");
    lcd.print(fan4);
    Serial.println("choose fan 2");
    ledMenu(0, 1, 0, 0, 0, 0);
    if (ok)
    {
      delay(delayBtn);
      state = setFan2State;
      lcd.clear();
    }
    if (up)
    {
      delay(delayBtn);
      state = chooseFan1;
      lcd.clear();
    }
    if (down)
    {
      delay(delayBtn);
      state = chooseFan3;
      lcd.clear();
    }
    if (cancel)
    {
      delay(delayBtn);
      state = mainMenu;
      lcd.clear();
    }
    break;

  case chooseFan3:
    lcd.setCursor(0, 0);
    lcd.print(" Fan 1 State   ");
    lcd.print(fan1);
    lcd.setCursor(0, 1);
    lcd.print(" Fan 2 State   ");
    lcd.print(fan2);
    lcd.setCursor(0, 2);
    lcd.print(">Fan 3 State   ");
    lcd.print(fan3);
    lcd.setCursor(0, 3);
    lcd.print(" Fan 4 State   ");
    lcd.print(fan4);
    Serial.println("choose fan 3");
    ledMenu(0, 0, 1, 0, 0, 0);
    if (ok)
    {
      delay(delayBtn);
      state = setFan3State;
      lcd.clear();
    }
    if (up)
    {
      delay(delayBtn);
      state = chooseFan2;
      lcd.clear();
    }
    if (down)
    {
      delay(delayBtn);
      state = chooseFan4;
      lcd.clear();
    }
    if (cancel)
    {
      delay(delayBtn);
      state = mainMenu;
      lcd.clear();
    }
    break;

  case chooseFan4:
    lcd.setCursor(0, 0);
    lcd.print(" Fan 1 State   ");
    lcd.print(fan1);
    lcd.setCursor(0, 1);
    lcd.print(" Fan 2 State   ");
    lcd.print(fan2);
    lcd.setCursor(0, 2);
    lcd.print(" Fan 3 State   ");
    lcd.print(fan3);
    lcd.setCursor(0, 3);
    lcd.print(">Fan 4 State   ");
    lcd.print(fan4);
    Serial.println("choose fan 4");
    ledMenu(0, 0, 0, 1, 0, 0);
    if (ok)
    {
      delay(delayBtn);
      state = setFan4State;
      lcd.clear();
    }
    if (up)
    {
      delay(delayBtn);
      state = chooseFan3;
      lcd.clear();
    }
    if (down)
    {
      delay(delayBtn);
      state = chooseHeater;
      lcd.clear();
    }
    if (cancel)
    {
      delay(delayBtn);
      state = mainMenu;
      lcd.clear();
    }
    break;

  case setFan1State:
    lcd.setCursor(0, 0);
    lcd.print(">Fan 1 State   ");
    lcd.print(fan1);
    Serial.println("set fan 1");
    ledMenu(1, 0, 0, 0, 0, 0);
    if (ok)
    {
      delay(delayBtn);
      EEPROM.write(addrfan[0], fan1);
      EEPROM.commit();
      state = chooseFan1;
      lcd.clear();
    }
    if (cancel)
    {
      delay(delayBtn);
      state = chooseFan1;
      lcd.clear();
    };
    if (up)
    {
      fan1 = not fan1;
      delay(delayBtn);
    };
    if (down)
    {
      fan1 = not fan1;
      delay(delayBtn);
    };
    break;

  case setFan2State:
    lcd.setCursor(0, 1);
    lcd.print(">Fan 2 State   ");
    lcd.print(fan2);
    Serial.println("set fan 2");
    ledMenu(0, 1, 0, 0, 0, 0);
    if (ok)
    {
      delay(delayBtn);
      EEPROM.write(addrfan[1], fan2);
      EEPROM.commit();
      state = chooseFan2;
      lcd.clear();
    }
    if (cancel)
    {
      delay(delayBtn);
      state = chooseFan2;
      lcd.clear();
    };
    if (up)
    {
      fan2 = not fan2;
      delay(delayBtn);
    };
    if (down)
    {
      fan2 = not fan2;
      delay(delayBtn);
    };
    break;

  case setFan3State:
    lcd.setCursor(0, 2);
    lcd.print(">Fan 3 State   ");
    lcd.print(fan3);
    Serial.println("set fan 3");
    ledMenu(0, 0, 1, 0, 0, 0);
    if (ok)
    {
      delay(delayBtn);
      EEPROM.write(addrfan[2], fan3);
      EEPROM.commit();
      state = chooseFan3;
      lcd.clear();
    }
    if (cancel)
    {
      delay(delayBtn);
      state = chooseFan3;
      lcd.clear();
    };
    if (up)
    {
      fan3 = not fan3;
      delay(delayBtn);
    };
    if (down)
    {
      fan3 = not fan3;
      delay(delayBtn);
    };
    break;

  case setFan4State:
    lcd.setCursor(0, 3);
    lcd.print(">Fan 4 State   ");
    lcd.print(fan4);
    Serial.println("set fan 4");
    ledMenu(0, 0, 0, 1, 0, 0);
    if (ok)
    {
      delay(delayBtn);
      EEPROM.write(addrfan[3], fan4);
      EEPROM.commit();
      state = chooseFan4;
      lcd.clear();
    }
    if (cancel)
    {
      delay(delayBtn);
      state = chooseFan4;
      lcd.clear();
    };
    if (up)
    {
      fan4 = not fan4;
      delay(delayBtn);
    };
    if (down)
    {
      fan4 = not fan4;
      delay(delayBtn);
    };
    break;

  case chooseHeater:
    lcd.setCursor(0, 0);
    lcd.print(">Heater State   ");
    lcd.print(heater1);
    lcd.setCursor(0, 1);
    lcd.print(" Cooler State   ");
    lcd.print(cooler1);
    lcd.setCursor(0, 2);
    lcd.print(" Set Temp");
    Serial.println("choose heater");
    ledMenu(0, 0, 0, 0, 1, 0);
    if (ok)
    {
      delay(delayBtn);
      state = setHeaterState;
      lcd.clear();
    }
    if (up)
    {
      delay(delayBtn);
      state = chooseFan4;
      lcd.clear();
    }
    if (down)
    {
      delay(delayBtn);
      state = chooseCooler;
      lcd.clear();
    }
    if (cancel)
    {
      delay(delayBtn);
      state = mainMenu;
      lcd.clear();
    }
    break;

  case chooseCooler:
    lcd.setCursor(0, 0);
    lcd.print(" Heater State   ");
    lcd.print(heater1);
    lcd.setCursor(0, 1);
    lcd.print(">Cooler State   ");
    lcd.print(cooler1);
    lcd.setCursor(0, 2);
    lcd.print(" Set Temp");
    Serial.println("choose cooler");
    ledMenu(0, 0, 0, 0, 0, 1);
    if (ok)
    {
      delay(delayBtn);
      state = setCoolerState;
      lcd.clear();
    }
    if (up)
    {
      delay(delayBtn);
      state = chooseHeater;
      lcd.clear();
    }
    if (down)
    {
      delay(delayBtn);
      state = chooseTemp;
      lcd.clear();
    }
    if (cancel)
    {
      delay(delayBtn);
      state = mainMenu;
      lcd.clear();
    }
    break;

  //This menu is for temperature setting. This contains fan, cooler, and heater temp max and temp min settings.
  case chooseTemp:
    lcd.setCursor(0, 0);
    lcd.print(" Heater State   ");
    lcd.print(heater1);
    lcd.setCursor(0, 1);
    lcd.print(" Cooler State   ");
    lcd.print(cooler1);
    lcd.setCursor(0, 2);
    lcd.print(">Set Temp");
    Serial.println("choose temp menu");
    ledMenu(0, 0, 0, 0, 0, 0);
    if (ok)
    {
      delay(delayBtn);
      state = chooseTempFanMax;
      lcd.clear();
    }
    if (up)
    {
      delay(delayBtn);
      state = chooseCooler;
      lcd.clear();
    }
    if (down)
    {
      delay(delayBtn);
      state = chooseFan1;
      lcd.clear();
    }
    if (cancel)
    {
      delay(delayBtn);
      state = mainMenu;
      lcd.clear();
    }
    break;

  case setHeaterState:
    lcd.setCursor(0, 0);
    lcd.print(">Heater State   ");
    lcd.print(heater1);
    Serial.println("set heater state");
    ledMenu(0, 0, 0, 0, 1, 0);
    if (ok)
    {
      delay(delayBtn);
      EEPROM.write(addrHeat, heater1);
      EEPROM.commit();
      state = chooseHeater;
      lcd.clear();
    }
    if (cancel)
    {
      delay(delayBtn);
      state = chooseHeater;
      lcd.clear();
    };
    if (up)
    {
      heater1 = not heater1;
      delay(delayBtn);
    };
    if (down)
    {
      heater1 = not heater1;
      delay(delayBtn);
    };
    break;

  case setCoolerState:
    lcd.setCursor(0, 1);
    lcd.print(">Cooler State   ");
    lcd.print(cooler1);
    Serial.println("set cooler state");
    ledMenu(0, 0, 0, 0, 0, 1);
    if (ok)
    {
      delay(delayBtn);
      EEPROM.write(addrCool, cooler1);
      EEPROM.commit();
      state = chooseCooler;
      lcd.clear();
    }
    if (cancel)
    {
      delay(delayBtn);
      state = chooseCooler;
      lcd.clear();
    };
    if (up)
    {
      cooler1 = not cooler1;
      delay(delayBtn);
    };
    if (down)
    {
      cooler1 = not cooler1;
      delay(delayBtn);
    };
    break;

  case chooseTempFanMax:
    lcd.setCursor(0, 0);
    lcd.print(">Temp Fan Max     ");
    lcd.print(fTempMax);
    lcd.setCursor(0, 1);
    lcd.print(" Temp Fan Min     ");
    lcd.print(fTempMin);
    lcd.setCursor(0, 2);
    lcd.print(" Temp Heater Max  ");
    lcd.print(hTempMax);
    lcd.setCursor(0, 3);
    lcd.print(" Temp Heater Min  ");
    lcd.print(hTempMin);
    Serial.println("choose temp fan max");
    ledMenu(1, 1, 1, 1, 0, 0);
    if (ok)
    {
      delay(delayBtn);
      state = setFanMax;
      lcd.clear();
    }
    if (cancel)
    {
      delay(delayBtn);
      state = chooseTemp;
      lcd.clear();
    };
    if (up)
    {
      delay(delayBtn);
      state = chooseCoolerMin;
      lcd.clear();
    };
    if (down)
    {
      delay(delayBtn);
      state = chooseTempFanMin;
      lcd.clear();
    };
    break;

  case chooseTempFanMin:
    lcd.setCursor(0, 0);
    lcd.print(" Temp Fan Max     ");
    lcd.print(fTempMax);
    lcd.setCursor(0, 1);
    lcd.print(">Temp Fan Min     ");
    lcd.print(fTempMin);
    lcd.setCursor(0, 2);
    lcd.print(" Temp Heater Max  ");
    lcd.print(hTempMax);
    lcd.setCursor(0, 3);
    lcd.print(" Temp Heater Min  ");
    lcd.print(hTempMin);
    Serial.println("choose temp fan min");
    ledMenu(1, 1, 1, 1, 0, 0);
    if (ok)
    {
      delay(delayBtn);
      state = setFanMin;
      lcd.clear();
    }
    if (cancel)
    {
      delay(delayBtn);
      state = chooseTemp;
      lcd.clear();
    };
    if (up)
    {
      delay(delayBtn);
      state = chooseTempFanMax;
      lcd.clear();
    };
    if (down)
    {
      delay(delayBtn);
      state = chooseHeaterMax;
      lcd.clear();
    };
    break;

  case chooseHeaterMax:
    lcd.setCursor(0, 0);
    lcd.print(" Temp Fan Max     ");
    lcd.print(fTempMax);
    lcd.setCursor(0, 1);
    lcd.print(" Temp Fan Min     ");
    lcd.print(fTempMin);
    lcd.setCursor(0, 2);
    lcd.print(">Temp Heater Max  ");
    lcd.print(hTempMax);
    lcd.setCursor(0, 3);
    lcd.print(" Temp Heater Min  ");
    lcd.print(hTempMin);
    Serial.println("choose heater temp max");
    ledMenu(0, 0, 0, 0, 1, 0);
    if (ok)
    {
      delay(delayBtn);
      state = setHeaterMax;
      lcd.clear();
    }
    if (cancel)
    {
      delay(delayBtn);
      state = chooseTemp;
      lcd.clear();
    };
    if (up)
    {
      delay(delayBtn);
      state = chooseTempFanMin;
      lcd.clear();
    };
    if (down)
    {
      delay(delayBtn);
      state = chooseHeaterMin;
      lcd.clear();
    };
    break;

  case chooseHeaterMin:
    lcd.setCursor(0, 0);
    lcd.print(" Temp Fan Max     ");
    lcd.print(fTempMax);
    lcd.setCursor(0, 1);
    lcd.print(" Temp Fan Min     ");
    lcd.print(fTempMin);
    lcd.setCursor(0, 2);
    lcd.print(" Temp Heater Max  ");
    lcd.print(hTempMax);
    lcd.setCursor(0, 3);
    lcd.print(">Temp Heater Min  ");
    lcd.print(hTempMin);
    Serial.println("choose heater temp min");
    ledMenu(0, 0, 0, 0, 1, 0);
    if (ok)
    {
      delay(delayBtn);
      state = setHeaterMin;
      lcd.clear();
    }
    if (cancel)
    {
      delay(delayBtn);
      state = chooseTemp;
      lcd.clear();
    };
    if (up)
    {
      delay(delayBtn);
      state = chooseHeaterMax;
      lcd.clear();
    };
    if (down)
    {
      delay(delayBtn);
      state = chooseCoolerMax;
      lcd.clear();
    };
    break;

  case chooseCoolerMax:
    lcd.setCursor(0, 0);
    lcd.print(">Temp Cooler Max  ");
    lcd.print(cTempMax);
    lcd.setCursor(0, 1);
    lcd.print(" Temp Cooler Min  ");
    lcd.print(cTempMin);
    Serial.println("choose cooler temp max");
    ledMenu(0, 0, 0, 0, 0, 1);
    if (ok)
    {
      delay(delayBtn);
      state = setCoolerMax;
      lcd.clear();
    }
    if (cancel)
    {
      delay(delayBtn);
      state = chooseTemp;
      lcd.clear();
    };
    if (up)
    {
      delay(delayBtn);
      state = chooseHeaterMax;
      lcd.clear();
    };
    if (down)
    {
      delay(delayBtn);
      state = chooseCoolerMin;
      lcd.clear();
    };
    break;

  case chooseCoolerMin:
    lcd.setCursor(0, 0);
    lcd.print(" Temp Cooler Max  ");
    lcd.print(cTempMax);
    lcd.setCursor(0, 1);
    lcd.print(">Temp Cooler Min  ");
    lcd.print(cTempMin);
    Serial.println("choose cooler temp min");
    ledMenu(0, 0, 0, 0, 0, 1);
    if (ok)
    {
      delay(delayBtn);
      state = setCoolerMin;
      lcd.clear();
    }
    if (cancel)
    {
      delay(delayBtn);
      state = chooseTemp;
      lcd.clear();
    };
    if (up)
    {
      delay(delayBtn);
      state = chooseCoolerMax;
      lcd.clear();
    };
    if (down)
    {
      delay(delayBtn);
      state = chooseTempFanMax;
      lcd.clear();
    };
    break;

  case setFanMax:
    lcd.setCursor(0, 0);
    lcd.print(">Temp Fan Max     ");
    lcd.print(fTempMax);
    Serial.println("set fan temp max");
    ledMenu(1, 1, 1, 1, 0, 0);
    if (ok)
    {
      delay(delayBtn);
      EEPROM.write(addrFanTempMax, fTempMax);
      EEPROM.commit();
      state = chooseTempFanMax;
      lcd.clear();
    }
    if (cancel)
    {
      delay(delayBtn);
      state = chooseTempFanMax;
      lcd.clear();
    };
    if (up)
    {
      fTempMax += 1;
      delay(delayBtn / 100);
    };
    if (down)
    {
      fTempMax -= 1;
      delay(delayBtn / 100);
    };
    break;

  case setFanMin:
    lcd.setCursor(0, 1);
    lcd.print(">Temp Fan Min     ");
    lcd.print(fTempMin);
    Serial.println("set fan temp min");
    ledMenu(1, 1, 1, 1, 0, 0);
    if (ok)
    {
      delay(delayBtn);
      EEPROM.write(addrFanTempMin, fTempMin);
      EEPROM.commit();
      state = chooseTempFanMin;
      lcd.clear();
    }
    if (cancel)
    {
      delay(delayBtn);
      state = chooseTempFanMin;
      lcd.clear();
    };
    if (up)
    {
      fTempMin += 1;
      delay(delayBtn / 100);
    };
    if (down)
    {
      fTempMin -= 1;
      delay(delayBtn / 100);
    };
    break;

  case setHeaterMax:
    lcd.setCursor(0, 2);
    lcd.print(">Temp Heater Max  ");
    lcd.print(hTempMax);
    Serial.println("set heater temp max");
    ledMenu(0, 0, 0, 0, 1, 0);
    if (ok)
    {
      delay(delayBtn);
      EEPROM.write(addrHeatMax, hTempMax);
      EEPROM.commit();
      state = chooseHeaterMax;
      lcd.clear();
    }
    if (cancel)
    {
      delay(delayBtn);
      state = chooseHeaterMax;
      lcd.clear();
    };
    if (up)
    {
      hTempMax += 1;
      delay(delayBtn / 100);
    };
    if (down)
    {
      hTempMax -= 1;
      delay(delayBtn / 100);
    };
    break;

  case setHeaterMin:
    lcd.setCursor(0, 3);
    lcd.print(">Temp Heater Min  ");
    lcd.print(hTempMin);
    Serial.println("set heater temp min");
    ledMenu(0, 0, 0, 0, 1, 0);
    if (ok)
    {
      delay(delayBtn);
      EEPROM.write(addrHeatMin, hTempMin);
      EEPROM.commit();
      state = chooseHeaterMin;
      lcd.clear();
    }
    if (cancel)
    {
      delay(delayBtn);
      state = chooseHeaterMin;
      lcd.clear();
    };
    if (up)
    {
      hTempMin += 1;
      delay(delayBtn / 100);
    };
    if (down)
    {
      hTempMin -= 1;
      delay(delayBtn / 100);
    };
    break;

  case setCoolerMax:
    lcd.setCursor(0, 0);
    lcd.print(">Temp Cooler Max  ");
    lcd.print(cTempMax);
    Serial.println("set cooler temp max");
    ledMenu(0, 0, 0, 0, 0, 1);
    if (ok)
    {
      delay(delayBtn);
      EEPROM.write(addrCoolMax, cTempMax);
      EEPROM.commit();
      state = chooseCoolerMax;
      lcd.clear();
    }
    if (cancel)
    {
      delay(delayBtn);
      state = chooseCoolerMax;
      lcd.clear();
    };
    if (up)
    {
      cTempMax += 1;
      delay(delayBtn / 100);
    };
    if (down)
    {
      cTempMax -= 1;
      delay(delayBtn / 100);
    };
    break;

  case setCoolerMin:
    lcd.setCursor(0, 1);
    lcd.print(">Temp Cooler Min  ");
    lcd.print(cTempMin);
    Serial.println("set cooler temp min");
    ledMenu(0, 0, 0, 0, 0, 1);
    if (ok)
    {
      delay(delayBtn);
      EEPROM.write(addrCoolMin, cTempMin);
      EEPROM.commit();
      state = chooseCoolerMin;
      lcd.clear();
    }
    if (cancel)
    {
      delay(delayBtn);
      state = chooseCoolerMin;
      lcd.clear();
    };
    if (up)
    {
      cTempMin += 1;
      delay(delayBtn / 100);
    };
    if (down)
    {
      cTempMin -= 1;
      delay(delayBtn / 100);
    };
    break;

  default:
    break;
  }
}

void rtcUpdate()
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
}
byte bcdToDec(byte val)
{
  return ((val / 16 * 10) + (val % 16));
}

void mainDisplay()
{
  //  readDS3231time(&second, &minute, &hour, &dayOfWeek, &dayOfMonth, &month, &year);
  rtcUpdate();
  lcd.setCursor(0, 0);
  lcd.print("  ");
  lcd.print(dayOfMonth);
  lcd.print("-");
  lcd.print(month);
  lcd.print("-");
  lcd.print(year);
  lcd.print(" ");
  lcd.print(hour);
  lcd.print(":");
  lcd.print(minute);
  lcd.print("  ");

  lcd.setCursor(0, 1);
  lcd.print("AMO:");
  lcd.print(ppm);
  lcd.print("  ppm");

  lcd.setCursor(0, 2);
  lcd.print("T/H:");
  lcd.print(temp);
  lcd.print(char(223));
  lcd.print("C/");
  lcd.print(hum);
  lcd.print("%");

  lcd.setCursor(0, 3);
  lcd.print("WS :");
  lcd.print("10.44");
  lcd.print("  m/s");
}

void relay(int relayFan1State, int relayFan2State, int relayFan3State, int relayFan4State, int relayHeaterState, int relayCoolerState)
{
  digitalWrite(relayFan1, relayFan1State);
  digitalWrite(relayFan2, relayFan2State);
  digitalWrite(relayFan3, relayFan3State);
  digitalWrite(relayFan4, relayFan4State);
  digitalWrite(relayHeater, relayHeaterState);
  digitalWrite(relayCooler, relayCoolerState);
}

void ledMenu(int ledMenuFan1State, int ledMenuFan2State, int ledMenuFan3State, int ledMenuFan4State, int ledMenuHeaterState, int ledMenuCoolerState)
{
  digitalWrite(ledMenuFan1, ledMenuFan1State);
  digitalWrite(ledMenuFan2, ledMenuFan2State);
  digitalWrite(ledMenuFan3, ledMenuFan3State);
  digitalWrite(ledMenuFan4, ledMenuFan4State);
  digitalWrite(ledMenuHeater, ledMenuHeaterState);
  digitalWrite(ledMenuCooler, ledMenuCoolerState);
}

void readFromEEPROM()
{
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
  cooler1 = EEPROM.read(addrCool);
  if (cooler1 > 1)
  {
    cooler1 = 1;
    EEPROM.write(addrCool, cooler1);
    EEPROM.commit();
  }
  heater1 = EEPROM.read(addrHeat);
  if (heater1 > 1)
  {
    heater1 = 0;
    EEPROM.write(addrHeat, heater1);
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
  hTempMin = EEPROM.read(addrHeatMin);
  if (hTempMin > 100)
  {
    hTempMin = 20;
    EEPROM.write(addrHeatMin, hTempMin);
    EEPROM.commit();
  }
  hTempMax = EEPROM.read(addrHeatMax);
  if (hTempMax > 100)
  {
    hTempMax = 50;
    EEPROM.write(addrHeatMax, hTempMax);
    EEPROM.commit();
  }
  cTempMin = EEPROM.read(addrCoolMin);
  if (cTempMin > 100)
  {
    cTempMin = 20;
    EEPROM.write(addrCoolMin, cTempMin);
    EEPROM.commit();
  }
  cTempMax = EEPROM.read(addrCoolMax);
  if (cTempMax > 100)
  {
    cTempMax = 50;
    EEPROM.write(addrCoolMax, cTempMax);
    EEPROM.commit();
  }
  ppmTres = EEPROM.read(addrPpmTres);
}

void homeDisplay()
{
  //  readDS3231time(&second, &minute, &hour, &dayOfWeek, &dayOfMonth, &month, &year);
  rtcUpdate();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("  Mokka PT.Mustika");
  lcd.setCursor(0, 1);
  lcd.print("  ");
  lcd.print(dayOfMonth);
  lcd.print("-");
  lcd.print(month);
  lcd.print("-");
  lcd.print(year);
  lcd.print(" ");
  lcd.print(hour);
  lcd.print(":");
  lcd.print(minute);
  lcd.print("  ");
}

/********************************* Functions For Controlling *****************************************/

int apiFan1(String command)
{
  // Get state from command
  fan1 = command.toInt();
  EEPROM.write(addrfan[0], fan1);
  EEPROM.commit();
  printMessage(fan1);
  return 1;
}

int apiFan2(String command)
{
  // Get state from command
  fan2 = command.toInt();
  EEPROM.write(addrfan[1], fan2);
  EEPROM.commit();
  printMessage(fan2);
  return 1;
}

int apiFan3(String command)
{
  // Get state from command
  fan3 = command.toInt();
  EEPROM.write(addrfan[2], fan3);
  EEPROM.commit();
  printMessage(fan3);
  return 1;
}

int apiFan4(String command)
{
  // Get state from command
  fan4 = command.toInt();
  EEPROM.write(addrfan[3], fan4);
  EEPROM.commit();
  printMessage(fan4);
  return 1;
}

int apiHeater(String command)
{
  // Get state from command
  heater1 = command.toInt();
  EEPROM.write(addrHeat, heater1);
  EEPROM.commit();
  printMessage(heater1);
  return 1;
}

int apiCooler(String command)
{
  // Get state from command
  cooler1 = command.toInt();
  EEPROM.write(addrCool, cooler1);
  EEPROM.commit();
  printMessage(cooler1);
  return 1;
}

int apiFTempMin(String command)
{
  // Get state from command
  fTempMin = command.toInt();
  EEPROM.write(addrFanTempMin, fTempMin);
  EEPROM.commit();
  printMessage(fTempMin);
  return 1;
}

int apiFTempMax(String command)
{
  // Get state from command
  fTempMax = command.toInt();
  EEPROM.write(addrFanTempMax, fTempMax);
  EEPROM.commit();
  printMessage(fTempMax);
  return 1;
}

int apiHTempMin(String command)
{
  // Get state from command
  hTempMin = command.toInt();
  EEPROM.write(addrHeatMin, hTempMin);
  EEPROM.commit();
  printMessage(hTempMin);
  return 1;
}

int apiHTempMax(String command)
{
  // Get state from command
  hTempMax = command.toInt();
  EEPROM.write(addrHeatMax, hTempMax);
  EEPROM.commit();
  printMessage(hTempMax);
  return 1;
}

int apiCTempMin(String command)
{
  // Get state from command
  cTempMin = command.toInt();
  EEPROM.write(addrCoolMin, cTempMin);
  EEPROM.commit();
  printMessage(cTempMin);
  return 1;
}

int apiCTempMax(String command)
{
  // Get state from command
  cTempMax = command.toInt();
  EEPROM.write(addrCoolMax, cTempMax);
  EEPROM.commit();
  printMessage(cTempMax);
  return 1;
}

int apiPpmTres(String command)
{
  //Get state from command
  ppmTres = command.toInt();
  EEPROM.write(addrPpmTres, ppmTres);
  EEPROM.commit();
  printMessage(ppmTres);
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

/********************* End Controlling **********************/

/*************************** SEND DATA TO SERVER **************************/
void kirim_data()
{
  if (current - previous >= interval)
  {
    String sn = "2019030011";
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

    rtcUpdate();
    String postData = (String) "&sn=" + sn + "&dgw=" + year + "-" + month + "-" + dayOfMonth + "&tgw=" + hour + ":" + minute + ":" + second +
                      "&sensor=" + sensor1 + "x" + sensor2 + "x" + sensor3 + "x" + sensor4 + "x" + sensor5 + "x" + sensor6 + "x" + sensor7 + "x" + sensor8 + "x" + sensor9 + "x" + sensor10 + "x" + sensor11 +
                      "&nilai=" + temp + "x" + ppm + "x" + hum + "x" + fan1 + "x" + fan2 + "x" + fan3 + "x" + fan4 + "x" + cooler1 + "x" + heater1 + "x" + fTempMin + "x" + fTempMax;

    HTTPClient http;
    http.begin("http://www.smart-gh.com/input2.php?sn=2019030011" + postData);
    http.addHeader("Content-Type", "application/x-www-form-urlencoded");

    auto httpCode = http.POST(postData);
    String payload = http.getString();

    Serial.println(postData);
    http.end();
  }
}
/********************** END SEND DATA ******************************/
