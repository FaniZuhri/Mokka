/*
  Button

  Turns on and off a light emitting diode(LED) connected to digital pin 13,
  when pressing a pushbutton attached to pin 2.

  The circuit:
  - LED attached from pin 13 to ground
  - pushbutton attached to pin 2 from +5V
  - 10K resistor attached to pin 2 from ground

  - Note: on most Arduinos there is already an LED on the board
    attached to pin 13.

  created 2005
  by DojoDave <http://www.0j0.org>
  modified 30 Aug 2011
  by Tom Igoe

  This example code is in the public domain.

  http://www.arduino.cc/en/Tutorial/Button
*/
#include "EEPROM.h"
#include <WiFi.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Button.h>
#include "RTClib.h"
LiquidCrystal_I2C lcd(0x27, 20, 4); // set the LCD address to 0x27 for a 16 chars and 2 line display
RTC_DS3231 rtc;

enum menuLcd
{
  mainMenu,
  chooseFan1,
  chooseFan2,
  chooseFan3,
  chooseFan4,
  setFan1,
  setFan2,
  setFan3,
  setFan4,
  chooseHeater,
  chooseCooler,
  chooseTemp,
  chooseTempFanMax,
  chooseTempFanMin,
  chooseHeaterMax,
  chooseHeaterMin,
  chooseCoolerMax,
  chooseCoolerMin,
  setHeater,
  setCooler,
  setFanMax,
  setFanMin,
  setHeaterMax,
  setHeaterMin,
  setCoolerMax,
  setCoolerMin,
  yes,
  no,
};

int addrfan[4] = {2, 4, 6, 8};
int addrheat = 10, addrcool = 12, addrFanTempMin = 16, addrFanTempMax = 14,
    addrHeatMin = 16, addrHeatMax = 18, addrCoolMin = 20, addrCoolMax = 22,
    addrppmTres = 24;
int addr = 0;

// constants won't change. They're used here to set pin numbers:
int okBtn = 12;
int upBtn = 18;
int downBtn = 19;
int cancelBtn = 13;
int arrowState = 1;
float ppm, temp, hum, amo;
int fan1, fan2, fan3, fan4, cooler1, heater1, delayBtn = 100, hTempMax, hTempMin, cTempMax, cTempMin, ppmTres;
int state, stateFTMax, stateFTMin, stateHTMax, stateHTMin, stateCTMax, stateCTMin, count, adc, fTempMin, fTempMax;
char daysOfTheWeek[7][12] = {"Minggu", "Senin", "Selasa", "Rabu", "Kamis", "Jumat", "Sabtu"};
String second, minute, hour, dayOfWeek, dayOfMonth, month, year;

#define ok !digitalRead(okBtn)
#define cancel !digitalRead(cancelBtn)
#define up !digitalRead(upBtn)
#define down !digitalRead(downBtn)

void setup()
{
  Serial.begin(9600);
  pinMode(okBtn, INPUT_PULLUP);
  pinMode(cancelBtn, INPUT_PULLUP);
  pinMode(downBtn, INPUT_PULLUP);
  pinMode(upBtn, INPUT_PULLUP);
  lcd.init(); // initialize the lcd
  lcd.init();
  // Print a message to the LCD.
  lcd.backlight();
  lcd.clear();
  //  Serial.println("setup");
  state = mainMenu;
}

void loop()
{
  // read the state of the pushbutton value:
  //  Serial.println("loop");
  DateTime now = rtc.now();
  switch (state)
  {
  case mainMenu:
    mainDisplay();
    Serial.println("main");
    if (ok or up or down or cancel)
    {
      delay(delayBtn);
      state = chooseFan1;
      lcd.clear();
    }
    break;

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
    if (ok)
    {
      delay(delayBtn);
      state = setFan1;
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
    if (ok)
    {
      delay(delayBtn);
      state = setFan2;
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
    if (ok)
    {
      delay(delayBtn);
      state = setFan3;
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
    if (ok)
    {
      delay(delayBtn);
      state = setFan4;
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
  
  case setFan1:
    lcd.setCursor(0, 0);
    lcd.print(">Fan 1 State   ");
    lcd.print(fan1);
    Serial.println("fan");
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

  case setFan2:
    lcd.setCursor(0, 1);
    lcd.print(">Fan 2 State   ");
    lcd.print(fan2);
    Serial.println("fan");
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

  case setFan3:
    lcd.setCursor(0, 2);
    lcd.print(">Fan 3 State   ");
    lcd.print(fan3);
    Serial.println("fan");
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

  case setFan4:
    lcd.setCursor(0, 3);
    lcd.print(">Fan 4 State   ");
    lcd.print(fan4);
    Serial.println("fan");
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
    if (ok)
    {
      delay(delayBtn);
      state = setHeater;
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
    Serial.println("choose heater");
    if (ok)
    {
      delay(delayBtn);
      state = setCooler;
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

  case chooseTemp:
    lcd.setCursor(0, 0);
    lcd.print(" Heater State   ");
    lcd.print(heater1);
    lcd.setCursor(0, 1);
    lcd.print(" Cooler State   ");
    lcd.print(cooler1);
    lcd.setCursor(0, 2);
    lcd.print(">Set Temp");
    Serial.println("choose heater");
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

  case setHeater:
    lcd.setCursor(0, 0);
    lcd.print(">Heater State   ");
    lcd.print(heater1);
    Serial.println("fan");
    if (ok)
    {
      delay(delayBtn);
      EEPROM.write(addrheat, heater1);
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

  case setCooler:
    lcd.setCursor(0, 1);
    lcd.print(">Cooler State   ");
    lcd.print(cooler1);
    Serial.println("fan");
    if (ok)
    {
      delay(delayBtn);
      EEPROM.write(addrcool, cooler1);
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
    Serial.println("fan");
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
    Serial.println("fan");
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
    Serial.println("fan");
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
    Serial.println("fan");
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
    Serial.println("fan");
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
    Serial.println("fan");
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
    Serial.println("fan");
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
      delay(delayBtn/100);
    };
    if (down)
    {
      fTempMax -= 1;
      delay(delayBtn/100);
    };
    break;

  case setFanMin:
    lcd.setCursor(0, 1);
    lcd.print(">Temp Fan Min     ");
    lcd.print(fTempMin);
    Serial.println("fan");
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
      delay(delayBtn/100);
    };
    if (down)
    {
      fTempMin -= 1;
      delay(delayBtn/100);
    };
    break;

  case setHeaterMax:
    lcd.setCursor(0, 2);
    lcd.print(">Temp Heater Max  ");
    lcd.print(hTempMax);
    Serial.println("fan");
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
      delay(delayBtn/100);
    };
    if (down)
    {
      hTempMax -= 1;
      delay(delayBtn/100);
    };
    break;

  case setHeaterMin:
    lcd.setCursor(0, 3);
    lcd.print(">Temp Heater Min  ");
    lcd.print(hTempMin);
    Serial.println("fan");
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
      delay(delayBtn/100);
    };
    if (down)
    {
      hTempMin -= 1;
      delay(delayBtn/100);
    };
    break;

  case setCoolerMax:
    lcd.setCursor(0, 0);
    lcd.print(">Temp Cooler Max  ");
    lcd.print(cTempMax);
    Serial.println("fan");
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
      delay(delayBtn/100);
    };
    if (down)
    {
      cTempMax -= 1;
      delay(delayBtn/100);
    };
    break;

  case setCoolerMin:
    lcd.setCursor(0, 1);
    lcd.print(">Temp Cooler Min  ");
    lcd.print(cTempMin);
    Serial.println("fan");
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
      delay(delayBtn/100);
    };
    if (down)
    {
      cTempMin -= 1;
      delay(delayBtn/100);
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
  lcd.print("20");
  lcd.print(year);
  lcd.print(" ");
  lcd.print(hour);
  lcd.print(":");
  lcd.print(minute);
  lcd.print("  ");

  lcd.setCursor(0,1);
  lcd.print("AMO:");
  lcd.print(amo);
  lcd.print("  ppm");

  lcd.setCursor(0,2);
  lcd.print("T/H:");
  lcd.print(temp);
  lcd.print(char(223));
  lcd.print("C/");
  lcd.print(hum);
  lcd.print("%");

  lcd.setCursor(0,3);
  lcd.print("WS :");
  lcd.print("10.44");
  lcd.print("  m/s");
}
