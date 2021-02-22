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
#define OUTPUT_BUFFER_SIZE 50000
#define NUMBER_VARIABLES 15
#define NUMBER_FUNCTIONS 15

int addrfan[4] = {2, 4, 6, 8};
int addrheat = 10, addrcool = 12, addrFanTempMin = 16, addrFanTempMax = 14,
    addrHeatMin = 16, addrHeatMax = 18, addrCoolMin = 20, addrCoolMax = 22,
    addrppmTres = 24;
int addr = 0;

#define EEPROM_SIZE 100
#define ONE_WIRE_BUS 15

const char *ssidAP = "Mustika Controller";
const char *passwordAP = "mustikajaya";

const char *ssid = "Mie Goyeng";
const char *password = "digodogsek";

int fan1, fan2, fan3, fan4, cooler1, heater1, delayButton = 100, hTempMax, hTempMin, cTempMax, cTempMin, ppmTres, arrowState;
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
int apiPpmTres(String command);

// Create aREST instance
aREST rest = aREST();

int relayON = LOW;   //relay nyala
int relayOFF = HIGH; //relay mati

long n, f, n1, f1, n2, f2, amo, timeSecond, timer1, timer2, timer3, timeNow;

enum sub
{
    homeMenu,
    setting1,
    setting2,
    fan1StateMenu,
    fan2StateMenu,
    fan3StateMenu,
    fan4StateMenu,
    heaterStateMenu,
    coolerStateMenu,
    setTempMaxMenu,
    setTempMinMenu,
    setFanMaxMenu,
    setFanMinMenu,
    setHeaterMaxMenu,
    setHeaterMinMenu,
    setCoolerMaxMenu,
    setCoolerMinMenu,
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
IPAddress AP_IP;

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

    state = homeMenu;
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

    // Give name & ID to the device (ID should be 6 characters long)
    rest.set_id("1");
    rest.set_name("MUSTIKA_CONTROLLER");
    Serial.println("REST ID = 1 & NAME = MUSTIKA_CONTROLLER");

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
    //Handle client rest, taruh di loop paling atas
    WiFiClient client = server.available();
    rest.handle(client);
    current = millis();
    //rtc
    DateTime now = rtc.now();

    uint32_t start = micros();

    mainDisplay();
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
    delay(1000);

    //amonia
    MQ135.update();           // Update data, the arduino will be read the voltage on the analog pin
    amo = MQ135.readSensor(); // Sensor will read PPM concentration using the model and a and b values setted before or in the setup
                              //   ppm = amo;
    mainDisplay();
    Serial.println(amo);
    //  MQ135.serialDebug(); // Will print the table on the serial port
    delay(500); //Sampling frequency
    //amonia
    //   amo = 300 * 1000;
    
//    kirim_data();
//    Serial.println(state);
    switch (state)
    {
    case homeMenu:
        mainDisplay();
       if (ok) {
           lcd.clear();
           state = setting1;
       }
       if (cancel)
       {
          lcd.setCursor(0,0);
          lcd.print("ampun bang jago"); 
       }
    break;

    case setting1:
        arrowState = 1;
        setting1Menu();
        if (up)
        {
            arrowState-=1;
            if (arrowState == 0)
            {
                arrowState = 1;
            }
            setting1Menu();
        }
        if (down)
        {
            arrowState+=1;
            setting1Menu();
            if (arrowState == 5)
            {
                state = setting2;
                // break;
            }
        }
        
    break;
    }
}

/********************************* Functions For Controlling *****************************************/

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

int apiPpmTres(String command)
{
    //Get state from command
    int ppmTresState = command.toInt();
    EEPROM.write(addrppmTres, ppmTresState);
    EEPROM.commit();
    printMessage(ppmTresState);
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

/************************* RTC *******************************/
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
}
byte bcdToDec(byte val)
{
    return ((val / 16 * 10) + (val % 16));
}

/************************************** END RTC *****************************/

/*************************** SEND DATA TO SERVER **************************/
void kirim_data()
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

        rtc_writetovar();
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
}

/********************** END SEND DATA ******************************/

//----------------------- HOME DISPLAY --------------------------//
void homeDisplay()
{
    rtc_writetovar();
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
//------------------END of HOME DISPLAY ----------------------//

//----------------------- MAIN DISPLAY --------------------------//
void mainDisplay()
{
//    readDS3231time(&second, &minute, &hour, &dayOfWeek, &dayOfMonth, &month, &year);
    rtc_writetovar();
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
    lcd.print(amo);
    lcd.print("  ppm");

    lcd.setCursor(0, 2);
    lcd.print("T/H:");
    lcd.print(temp,0);
    lcd.print(char(223));
    lcd.print("C/");
    lcd.print(hum,0);
    lcd.print("%");

    lcd.setCursor(0, 3);
    lcd.print("Settings");
}
//------------------END of MAIN DISPLAY ----------------------//

/******************** READ VARIABLES FORM EEPROM ************************/
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
}

/******************** END READING ************************/

void setting1Menu()
{   
    if (arrowState == 1)
    {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(">Fan 1 State");
        lcd.setCursor(0, 1);
        lcd.print(" Fan 2 State");
        lcd.setCursor(0, 2);
        lcd.print(" Fan 3 State");
        lcd.setCursor(0, 3);
        lcd.print(" Fan 4 State");
    }
    if (arrowState == 2)
    {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(" Fan 1 State");
        lcd.setCursor(0, 1);
        lcd.print(">Fan 2 State");
        lcd.setCursor(0, 2);
        lcd.print(" Fan 3 State");
        lcd.setCursor(0, 3);
        lcd.print(" Fan 4 State");
    }
    if (arrowState == 3)
    {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(" Fan 1 State");
        lcd.setCursor(0, 1);
        lcd.print(" Fan 2 State");
        lcd.setCursor(0, 2);
        lcd.print(">Fan 3 State");
        lcd.setCursor(0, 3);
        lcd.print(" Fan 4 State");
    }
    if (arrowState == 4)
    {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(" Fan 1 State");
        lcd.setCursor(0, 1);
        lcd.print(" Fan 2 State");
        lcd.setCursor(0, 2);
        lcd.print(" Fan 3 State");
        lcd.setCursor(0, 3);
        lcd.print(">Fan 4 State");
    }
}
