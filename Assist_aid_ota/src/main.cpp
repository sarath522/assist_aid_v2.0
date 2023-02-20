// ble logo
// limit switch gpio change
// battery value after removal of charger
// faster update of load cell values in display
// battery value previous and current comparison
// battery value stored in EEPROM
// battery value display after charging increased to 5seconds

// TODO : Add a battery bar instead of battery percentage

#include <Arduino.h>
#include <Wire.h>
#include "SSD1306Wire.h"
#include "HX711.h"
#include "EEPROM.h"
#include <NimBLEDevice.h>

#include <ArduinoBleOTA.h>

BLEServer *pServer = NULL;
BLECharacteristic *pTxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;
char char_received;

bool charger_connected = false;

int batt_range_bar;
 
float scale1_data;
float scale2_data;

byte count = 1;

float display_data;
float prev_display_data1;
float prev_data_sent = 1000;

int prev_battery_range;
int battery_range;

unsigned long previousMillis = 0;
unsigned long previousMillis_batt = 0;
bool start_sampling = false;

String disp = "";

bool device_on = true; // changed from false

#define OLED_SDA 14
#define OLED_SCL 13

#define LOADCELL1_DOUT_PIN 32
#define LOADCELL1_SCK_PIN 33

#define LOADCELL2_DOUT_PIN 21
#define LOADCELL2_SCK_PIN 22

#define LIMIT_SWITCH 16

#define LOAD_SWITCH 15

#define battery_enable_pin 2
#define battery_analog_pin 35

#define battery_status1_pin 17
#define battery_status2_pin 34

#define uS_TO_S_FACTOR 1000000ULL // Conversion factor for micro seconds to seconds /
#define TIME_TO_SLEEP 5

#define ON_OFF_RESET_PIN 4

// USER_BUTTON INTIALISATION
// const int user_button = 4;
int userbuttonState;
int lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 2500;
bool ota_flag = false;

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

SSD1306Wire display(0x3C, OLED_SDA, OLED_SCL);
HX711 scale1;
HX711 scale2;

class MyServerCallbacks : public BLEServerCallbacks
{
  void onConnect(BLEServer *pServer)
  {
    deviceConnected = true;
  };

  void onDisconnect(BLEServer *pServer)
  {
    deviceConnected = false;
  }
};

bool send_data_flag = false;
String init_data = "";
std::string tx_data = "";

bool reset_values = false;

void IRAM_ATTR ON_OFF_RESET() // changes made
{
  reset_values = true;
}

bool BLE_device_command(String inCmd)
{

  if (inCmd.indexOf("HS") >= 0)
  {
    init_data = "h\0";
    return 1;
  }
  else if (inCmd.indexOf("FS") >= 0)
  {
    init_data = "f\0";
    return 1;
  }
  else if (inCmd.indexOf("LS") >= 0)
  {
    init_data = "l\0";
    return 1;
  }
  else if (inCmd.indexOf("OS") >= 0)
  {
    init_data = "0\0";
    return 1;
  }
  else if (inCmd.indexOf("AS") >= 0)
  {
    init_data = "a\0";
    return 1;
  }
  else if (inCmd.indexOf("NS") >= 0)
  {
    init_data = "n\0";
    return 1;
  }
  else if (inCmd.indexOf("BS") >= 0)
  {
    init_data = "b\0";
    return 1;
  }
  else if (inCmd.indexOf("TS") >= 0)
  {
    init_data = "t\0";
    return 1;
  }
  else if (inCmd.indexOf("ZS") >= 0)
  {
    init_data = "z\0";
    return 1;
  }
  else
  {
    init_data = "invalidCmd";
    return 1;
  }
}

class MyCallbacks : public BLECharacteristicCallbacks
{
  void onWrite(BLECharacteristic *pCharacteristic)
  {
    std::string rxValue = pCharacteristic->getValue();
    String rx_data = "";

    if (rxValue.length() > 0)
    {
      for (int i = 0; i < rxValue.length(); i++)
        rx_data += rxValue[i];
    }
    Serial.println(rx_data);
    bool ret = BLE_device_command(rx_data);

    if (ret == 1)
      send_data_flag = true;
  }
};

void init_BLE()
{
  // Create the BLE Device
  BLEDevice::init("AssistAid");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pTxCharacteristic = pService->createCharacteristic(
      CHARACTERISTIC_UUID_TX,
      NIMBLE_PROPERTY::NOTIFY);

  BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(
      CHARACTERISTIC_UUID_RX,
      NIMBLE_PROPERTY::WRITE);

  pServer->setCallbacks(new MyServerCallbacks());
  pRxCharacteristic->setCallbacks(new MyCallbacks());

  // Start the service
  pService->start();

  // Start advertising
  pServer->getAdvertising()->start();
  Serial.println("Waiting a client connection to notify...");
}

void init_loadcell1()
{
  // init Load cell
  scale1.begin(LOADCELL1_DOUT_PIN, LOADCELL1_SCK_PIN);

  Serial.println("Before setting up the scale:");
  Serial.print("read: \t\t");
  Serial.println(scale1.read()); // print a raw reading from the ADC

  Serial.print("read average: \t\t");
  Serial.println(scale1.read_average(20)); // print the average of 20 readings from the ADC //actual value 20

  Serial.print("get value: \t\t");
  Serial.println(scale1.get_value(5)); // print the average of 5 readings from the ADC minus the tare weight (not set yet)//actual value 2

  Serial.print("get units: \t\t");
  Serial.println(scale1.get_units(5), 1); // print the average of 5 readings from the ADC minus tare weight (not set) divided
  // by the SCALE parameter (not set yet)

  scale1.set_scale(37640.f); // this value is obtained by calibrating the scale with known weights; see the README for details
  scale1.tare();             // reset the scale to 0

  Serial.println("After setting up the scale:");

  Serial.print("read: \t\t");
  Serial.println(scale1.read()); // print a raw reading from the ADC

  Serial.print("read average: \t\t");
  Serial.println(scale1.read_average(20)); // print the average of 20 readings from the ADC

  Serial.print("get value: \t\t");
  Serial.println(scale1.get_value(5)); // print the average of 5 readings from the ADC minus the tare weight, set with tare()

  Serial.print("get units: \t\t");
  Serial.println(scale1.get_units(5), 1); // print the average of 5 readings from the ADC minus tare weight, divided
  // by the SCALE parameter set with set_scale
}

void init_loadcell2()
{
  // init Load cell
  scale2.begin(LOADCELL2_DOUT_PIN, LOADCELL2_SCK_PIN);

  Serial.println("Before setting up the scale:");
  Serial.print("read: \t\t");
  Serial.println(scale2.read()); // print a raw reading from the ADC

  Serial.print("read average: \t\t");
  Serial.println(scale2.read_average(20)); // print the average of 20 readings from the ADC

  Serial.print("get value: \t\t");
  Serial.println(scale2.get_value(5)); // print the average of 5 readings from the ADC minus the tare weight (not set yet)

  Serial.print("get units: \t\t");
  Serial.println(scale2.get_units(5), 1); // print the average of 5 readings from the ADC minus tare weight (not set) divided
  // by the SCALE parameter (not set yet)


 // scale2.set_scale(-461050.f);
  
  // scale2.set_scale(-459750.f); // this value is obtained by calibrating the scale with known weights; see the README for details
  scale2.set_scale(-511150.f); // this value is obtained by calibrating the scale with known weights; see the README for details
  scale2.tare();               // reset the scale to 0

  Serial.println("After setting up the scale:");

  Serial.print("read: \t\t");
  Serial.println(scale2.read()); // print a raw reading from the ADC

  Serial.print("read average: \t\t");
  Serial.println(scale2.read_average(20)); // print the average of 20 readings from the ADC

  Serial.print("get value: \t\t");
  Serial.println(scale2.get_value(5)); // print the average of 5 readings from the ADC minus the tare weight, set with tare()

  Serial.print("get units: \t\t");
  Serial.println(scale2.get_units(5), 1); // print the average of 5 readings from the ADC minus tare weight, divided
  // by the SCALE parameter set with set_scale
}

void init_oled()
{

  // Initialising the UI will init the display too.
  display.init();
  display.flipScreenVertically();
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_16);
  display.drawString(0, 0, "Initializing");
  display.drawString(0, 20, "Please Wait..");
  display.display();
}

uint32_t battery_value = 0;

void display_low_battery()
{
  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_16); // 10
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(20, 25, "Battery LOW!!");
  display.display();
  delay(2000);
}

void display_ota_mode()
{
  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_16); // 10
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(15, 25, "OTA mode ON");
  display.display();
  delay(2000);
}

void check_battery_status()
{
  if (battery_value >= 4096)
    battery_value = 4095;
  // if ((battery_value > 3240) && (battery_value <= 3270))
  if (battery_value <= 3430)
  {
    if (device_on == false)
    {
      display_low_battery();
    }
    else
    {
      display_low_battery();
      display.displayOff();
      delay(500);
      device_on = false;
      digitalWrite(LOAD_SWITCH, LOW);
    }
  }
  else
  {
    if (device_on == false)
    {
      device_on = true;
      digitalWrite(LOAD_SWITCH, HIGH);
    }
  }
  battery_range = map(battery_value, 3400, 4095, 0, 100);
  battery_value = 0;
  if (prev_battery_range == 0)
    prev_battery_range = battery_range;

  if (battery_range < prev_battery_range)
    prev_battery_range = battery_range;
  else if (battery_range > prev_battery_range)
    battery_range = prev_battery_range;

  Serial.print("battery_range  ");
  Serial.print(battery_range);
  Serial.print('\t');
  Serial.print("prev_battery_range  ");
  Serial.println(prev_battery_range);
  Serial.printf("%d  %d\n", digitalRead(battery_status1_pin), digitalRead(battery_status2_pin));
}

void display_charging_status()
{
  
  int x = 105;
  int y = 0;
  display.drawRect(x + 0, y + 0, 19, 10);
  display.fillRect(x + 19, y + 2, 3, 6);
  x = 112;
  display.drawLine(x + 4, y + 0, x + 0, y + 4);
  display.drawLine(x + 0, y + 4, x + 4, y + 4);
  display.drawLine(x + 4, y + 4, x + 0, y + 8);
  
}

// ota update function
void OTA_UPDATE()
{
  ArduinoBleOTA.begin("ArduinoBleOTA", InternalStorage);

  ota_flag = true;

  while (ota_flag == 1)
  {
#if defined(BLE_PULL_REQUIRED)
    BLE.poll();
#endif
    ArduinoBleOTA.pull();
  }
}

void blu_logoon()
{
  display.drawLine(6, 0, 9, 3);
  display.drawLine(9, 3, 6, 6);
  display.drawLine(6, 6, 9, 9);
  display.drawLine(9, 9, 6, 12);
  display.drawLine(6, 0, 6, 12);
  display.drawLine(3, 3, 6, 6);
  display.drawLine(6, 6, 3, 9);
  display.display();
}

void blu_logooff()
{
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 2, " ");
  display.display();
  display.setFont(ArialMT_Plain_10);
  display.drawString(6, 1, " ");
  display.display();
  display.setFont(ArialMT_Plain_10);
  display.drawString(7, 0, " ");
  display.display();
  display.setFont(ArialMT_Plain_10);
  display.drawString(7, 4, " ");
}

int convert_range_to_bar()
{
  if ((battery_range > 75) && (battery_range <= 100))
    return 4;
  else if ((battery_range > 50) && (battery_range <= 75))
    return 3;
  else if ((battery_range > 25) && (battery_range <= 50))
    return 2;
  else if ((battery_range > 7) && (battery_range <= 25))
    return 1;
  else
    return 0;
}

void display_battery_bars(int bars)
{
  int x = 105;
  int y = 0;
  display.drawRect(x + 0, y + 0, 19, 10);
  display.fillRect(x + 19, y + 2, 3, 6);
  if (bars >= 1)
    display.fillRect(x + 2, y + 2, 3, 6);
  if (bars >= 2)
    display.fillRect(x + 6, y + 2, 3, 6);
  if (bars >= 3)
    display.fillRect(x + 10, y + 2, 3, 6);
  if (bars >= 4)
    display.fillRect(x + 14, y + 2, 3, 6);
  display.display();
}

bool batt_full = false;

void display_update()
{
  
  if ((digitalRead(battery_status1_pin) == LOW) && (digitalRead(battery_status2_pin) == HIGH))
  {
    charger_connected = true;
    prev_battery_range = 0;
    count = 1;
    EEPROM.write(0, 0);
    EEPROM.commit();
    delay(100);
  }
  else if ((digitalRead(battery_status1_pin) == HIGH) && (digitalRead(battery_status2_pin) == LOW))
  {
    batt_full = true;
    Serial.println("BT full");
    EEPROM.write(0, 4);
    EEPROM.commit();
    delay(100);
  }
  else
  {
    if (charger_connected == true)
    {
      digitalWrite(battery_enable_pin, HIGH);
      delay(3000);
      battery_value = 0;
      for (int i = 0; i < 5; i++)
        battery_value = battery_value + analogRead(battery_analog_pin);
      digitalWrite(battery_enable_pin, LOW);
      battery_value = battery_value / 5;
      battery_value = battery_value - 90;
      Serial.print("After charger :");
      Serial.println(battery_value);
      check_battery_status();
      batt_range_bar = convert_range_to_bar();
      charger_connected = false;
      EEPROM.write(0, batt_range_bar);
      EEPROM.commit();
      delay(100);
      batt_full=false;
      start_sampling = false;
      previousMillis = millis();
    }
  }

    if (deviceConnected)
    {
      blu_logoon();
    }
    else
    {
      blu_logooff();
    }
    
    if (charger_connected && !batt_full){

      display.clear();
      display_charging_status();
    }
    else if ( batt_full){
      display_battery_bars(4);
    }
    else
      display_battery_bars(batt_range_bar);

    display.display();
  }

void display_loadvalue()
{
if (reset_values == true)
  {

    prev_display_data1 = 0;
    display_data = 0;
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.setFont(ArialMT_Plain_24);
    display.drawString(25, 25, "0.00Kg");
    display.display();
    reset_values = false;
  }
    if (device_on == true)
  {
    if (prev_display_data1 <= display_data)
    {
      disp = "";
      disp += String(display_data);
      disp += "Kg";
      if (!deviceConnected)
        prev_display_data1 = display_data;
    }
    display.clear();
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.setFont(ArialMT_Plain_24);
    display.drawString(25, 25, String(disp));
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.setFont(ArialMT_Plain_10);

}
}

void setup()
{
  Serial.begin(115200);

  Serial.println("Firmware Version 2.0.0");
  delay(2500);
  pinMode(battery_analog_pin, INPUT);
  pinMode(LOAD_SWITCH, OUTPUT);
  pinMode(battery_enable_pin, OUTPUT);
  digitalWrite(LOAD_SWITCH, HIGH);
  digitalWrite(battery_enable_pin, HIGH);
  delay(250);

  EEPROM.begin(1);

  if (device_on == true)
  {
    init_oled();
    init_BLE();
    init_loadcell1();

    init_loadcell2();
    display.clear();

    pinMode(battery_status1_pin, INPUT);
    pinMode(battery_status2_pin, INPUT);

    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.setFont(ArialMT_Plain_16);
    display.drawString(0, 0, "Done");
    display.display();

    pinMode(LIMIT_SWITCH, INPUT);
    pinMode(ON_OFF_RESET_PIN, INPUT);                         // changes made
    attachInterrupt(ON_OFF_RESET_PIN, ON_OFF_RESET, FALLING); // changes made
  }
  int temp_value = 0;
  for (int i = 0; i < 500; i++)
    temp_value = temp_value + analogRead(battery_analog_pin);
  temp_value = temp_value / 500;
  battery_value = temp_value - 30;
  Serial.println(battery_value);
  check_battery_status();
  batt_range_bar = convert_range_to_bar();
  Serial.println(batt_range_bar);

  int battery_eeprom = EEPROM.read(0);
  Serial.println(battery_eeprom);
  if (batt_range_bar > battery_eeprom && battery_eeprom !=0)
  {
    batt_range_bar = battery_eeprom;
  }
  else
  {
    EEPROM.write(0, batt_range_bar);
    EEPROM.commit();
    delay(100);
  }
  digitalWrite(battery_enable_pin, LOW);
  previousMillis = millis();
}


void loop()
{
  unsigned long currentMillis = millis();

  if (((currentMillis - previousMillis) >= 20000) && !charger_connected)
  { 
    digitalWrite(battery_enable_pin, HIGH);
    if(!start_sampling)
    {
      start_sampling = true;
      previousMillis_batt = currentMillis;
    
    }
    if(currentMillis - previousMillis_batt > 2000)
    {  
      previousMillis = currentMillis;
      int temp_value = 0;
      for (int i = 0; i < 10; i++)
        temp_value = temp_value + analogRead(battery_analog_pin);
      temp_value = temp_value / 10;
      battery_value = temp_value;
      Serial.print(battery_value);
      Serial.print(",");
      check_battery_status();
      batt_range_bar = convert_range_to_bar();
      Serial.println(convert_range_to_bar());
      int battery_eeprom = EEPROM.read(0);
      if (batt_range_bar > battery_eeprom)
        batt_range_bar = battery_eeprom;
      EEPROM.write(0, batt_range_bar);
      EEPROM.commit();
      delay(100);
      digitalWrite(battery_enable_pin, LOW);
      start_sampling = false;
      Serial.print(battery_range);
      Serial.print(",");
      Serial.println(batt_range_bar);
    }
  }
  if (device_on == true)
  {
    // if limit switch is high then read 70kg load cell
    if (digitalRead(LIMIT_SWITCH) == LOW)
    {
      display_data = scale1.get_units(10);
      if (display_data < 0)
        display_data = 0.0;
    }
    // if limit switch is low then read 10kg load cell
    else if (digitalRead(LIMIT_SWITCH) == HIGH)
    {
      display_data = scale2.get_units(10);
      if (display_data < 0)
        display_data = 0.0;
    }
    // Read battery status and update in display

    // update display
   
    
  
       display_update();
    if(charger_connected == false){
      display_loadvalue();
    }
   
  
    if (deviceConnected)
    {
    
      if (send_data_flag)
      {
        if (init_data != "b\0")
        {
          if(init_data != "z\0")
          {
            if (display_data != prev_data_sent)
            {
              prev_data_sent = display_data;
              if (prev_display_data1 <= display_data)
              {
                prev_display_data1 = display_data;
                init_data.remove(1);
                init_data += display_data;
              }
            }
          }
          else
          {
              init_data.remove(1);
              init_data += battery_range;
          }
          for (int i = 0; i < init_data.length(); i++)
            tx_data += init_data[i];

          tx_data += display_data;
          pTxCharacteristic->setValue(tx_data); // Notify fromSerial.
          pTxCharacteristic->notify();
        }
        else
        {
          init_data.remove(1);
          init_data += batt_range_bar;
          for (int i = 0; i < init_data.length(); i++)
            tx_data += init_data[i];

          tx_data += display_data;
          pTxCharacteristic->setValue(tx_data); // Notify fromSerial.
          pTxCharacteristic->notify();
          send_data_flag = false;
        }
        Serial.println(init_data);

        tx_data = "";
      }
    }
    // disconnecting
    if (!deviceConnected && oldDeviceConnected)
    {
      delay(500);                  // give the bluetooth stack the chance to get things ready
      pServer->startAdvertising(); // restart advertising
      Serial.println("start advertising");
      oldDeviceConnected = deviceConnected;
    }
    // connecting
    if (deviceConnected && !oldDeviceConnected)
    {
      // do stuff here on connecting
      oldDeviceConnected = deviceConnected;
    }

    // long press OTA MODE
    int button_value = digitalRead(ON_OFF_RESET_PIN);

    if (button_value != lastButtonState)
    {
      lastDebounceTime = millis();
    }

    if ((millis() - lastDebounceTime) > debounceDelay)
    {
      if (button_value != userbuttonState)
      {
        userbuttonState = button_value;
        if (userbuttonState == LOW)
        {

          BLEDevice::deinit(true);

          Serial.println("OTA MODE ENTER");
          display_ota_mode();
          OTA_UPDATE(); // OTA update function

        }
      }
    }

    lastButtonState = button_value;
    // Long Press OTA mode over
  }
}
  //////// void loop finish
