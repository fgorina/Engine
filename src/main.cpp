
#include <EEPROM.h>
#include <ArduinoWebsockets.h>
#include "WiFi.h"
// #include <Time.h>
#include <ArduinoJson.h>
#include <ArduinoJson.hpp>
#include <HTTPClient.h>
#include <ESPmDNS.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define DEBUG true
#define DEBUG_1 true
#define DEBUG_ISR false
#define DEBUG_T true

#define EEPROM_SIZE 512

#define IN_GPIO 16
#define ONBOARD_LED 2

const int oneWireBus = 17;
int ntDevices;

typedef union
{
  DeviceAddress address;
  uint64_t id;
} DeviceId;

DeviceId tempDeviceAddress;

DeviceId tempDevices[5] = {
    {.id = (uint64_t)4198535122781495592},
    {.id = (uint64_t)11176943009740906792},
    {.id = (uint64_t)8501594867311599912},
    {.id = (uint64_t)369306671541149992},
    {.id = (uint64_t)13078325212068733224}};

const String paths[5] = {

    "propulsion.1.temperature",
    "propulsion.1.exhaustTemperature",
    "propulsion.1.engineRoomTemperature",
    "",
    ""

};

const float baseKelvin = 273.16;

float tempEngine = 0.0;     // Sensor 0
float tempExhaust = 0.0;    // Sensor 1
float tempEngineRoom = 0.0; // Sensor 2

#define AVG_SAMPLES 100

// Wifi and SignalK Connections

char ssid[20] = "Yamato";
char password[20] = "ailataN1991";
char device_name[20] = "engine";
char skserver[20] = "";
int skport = 0; // It is 4 bytes
char skpath[100] = "/signalk/v1/stream?subscribe=none";

// Frequency to rpme conversion
// it is freq * 60 / poles (cycles/s -> cycles / min * poles)

const double poles = 6.0;    // Number of poles in the alternatos
const double relation = 3.0; // Relation between engine speed and alternator speed

unsigned long last = micros();
int samples = 0;
unsigned long ac_period = 0;
double f = 0;
double rpm = 0;

char buffer[256];

// 1-Wire and Temperature

OneWire oneWire(oneWireBus);
DallasTemperature tSensors(&oneWire);

using namespace websockets;

bool mdnsDone = false; // Will be true when we have a server

int enabled = 0; // 0 Deshabilita les accions fins que s'ha rebut un command
WebsocketsClient client;
int socketState = -4; // Change to -4 if want connect -5 does not use WiFi, -4 -> Before connecting to WiFi, -3, -2.Connection authorized, 2-> Connected and authorized

String me = "vessels.self";
char token[256] = "";
char bigBuffer[1024] = "";

TaskHandle_t task_pres;
TaskHandle_t task_empty;
TaskHandle_t taskNetwork;

int ledState = 0;
int ledOn = 0;
int ledOff = 100;

// Function Prototypes

uint64_t tou64(DeviceAddress addr)
{
  uint64_t acum = 0;
  for (int i = 0; i < 8; i++)
  {
    acum = acum * 256 + addr[7 - i];
  }
  return acum;
}

int lookupDevice(DeviceId device)
{

  for (int i = 0; i < 5; i++)
  {
    if (device.id == tempDevices[i].id)
    {
      return (i);
    }
  }
  return -1;
}

void IRAM_ATTR ISR()
{
#if DEBUG_ISR
  Serial.print(".");
#endif

  unsigned long m = micros();
  unsigned long period = m - last;
  ac_period += period;
  samples += 1;
  last = m;
}

// LEDs

void clearLed()
{
  ledState = 0;
  digitalWrite(ONBOARD_LED, ledState);
}

void setLed()
{
  ledState = 1;

  digitalWrite(ONBOARD_LED, ledState);
}

void toggleLed()
{
  if (ledState == 0)
  {
    ledState = 1;
  }
  else
  {
    ledState = 0;
  }

  digitalWrite(ONBOARD_LED, ledState);
}

void ledTask(void *parameter)
{

  for (;;)
  {
    if (ledOn > 0)
    {
      setLed();
      vTaskDelay(ledOn);
    }

    if (ledOff > 0)
    {
      clearLed();
      vTaskDelay(ledOff);
    }
  }
}

#include "signalk.h"

// This writes the screen and computes frequency ajd angle.
void presentationTask(void *parameter)
{
  float f = 0.0;
  while (true)
  {
    if (samples >= AVG_SAMPLES)
    {
      int oldsamples = samples;
      unsigned long old_ac_period = ac_period;

      double period = 2.0 * double(ac_period) / double(samples);
      samples = 0;
      ac_period = 0;

      f = double(1000000) / double(period);
      rpm = f * 60.0 / poles * 2 / relation;

      if (DEBUG_1)
      {
        Serial.print("Ac Period ");
        Serial.print(old_ac_period);
        Serial.print(" Samples ");
        Serial.print(oldsamples);
        Serial.print(" f ");
        Serial.print(f);
        Serial.print(" RPM ");
        Serial.println(rpm);
      }

      sendData( f / poles  / relation);
    }
    else if ((micros() - last) > 1000000l)
    {
      f = 0.0;
      last = micros();
      sendData(0.0);
    }

    vTaskDelay(50); // May be adjusted for necessity Era 50changed for debug T
  }
}

// Runs the on board led to show connection status to signalk
void emptyTask(void *parameter)
{
  while (true)
  {

    if (ledOn > 0)
    {
      setLed();
      vTaskDelay(ledOn);
    }

    if (ledOff > 0)
    {
      clearLed();
      vTaskDelay(ledOff);
    }
  }
}

// Load Data from EEPROM
void loadEEPROM()
{

  float f1;
  float f2;
  char s20[20];

  EEPROM.begin(EEPROM_SIZE);

  EEPROM.get(0, f1);
  EEPROM.get(4, f2);

  if (isnan(f1) || isnan(f2) || f1 == 0.0 || f2 == 0.0)
  {

    f1 = 1.0;
    f2 = 1.0;
    // Init EEPROM Area
    EEPROM.put(0, f1);
    EEPROM.put(4, f2);
    EEPROM.put(8, ssid);
    EEPROM.put(28, password);
    EEPROM.put(48, device_name);
    EEPROM.put(68, skserver);
    EEPROM.put(88, skport);
    EEPROM.put(92, skpath);
    EEPROM.put(192, token);

    EEPROM.commit();
    if (DEBUG || true)
    {
      Serial.println();
    }
    if (DEBUG || true)
    {
      Serial.println("Written default data to EEPROM");
    }
  }
  else
  {

    if (DEBUG || true)
    {
      Serial.println("EEPROM already vith values");
    }
  }

  EEPROM.get(8, ssid);
  EEPROM.get(28, password);
  EEPROM.get(48, s20);

  if (strlen(s20) != 0 && false)
  {
    strcpy(device_name, s20);
  }

  EEPROM.get(68, skserver);
  EEPROM.get(88, skport);
  EEPROM.get(92, skpath);
  EEPROM.get(192, token);

  if (strlen(skserver) > 0)
  {
    Serial.println("Alreaddy have a server, no need to lookup by mDns");
    mdnsDone = true;
  }
  print_info();
}
void setup()
{

  Serial.begin(115200);

  pinMode(IN_GPIO, INPUT);

  loadEEPROM();

  tSensors.begin();

  ntDevices = tSensors.getDeviceCount();

  if (DEBUG_T)
  {
    Serial.print("Found ");
    ntDevices;
    Serial.println("devices.");
  }

  for (int i = 0; i < ntDevices; i++)
  {
    tSensors.getAddress(tempDeviceAddress.address, i);
    if (DEBUG_T)
    {
      Serial.print("    Device ");
      Serial.print(i);
      Serial.print(" ");

      Serial.print(tou64(tempDeviceAddress.address));
      Serial.print(" ");

      Serial.print(tempDeviceAddress.id);

      Serial.print("  ");

      int idev = lookupDevice(tempDeviceAddress);
      Serial.print(idev);
      Serial.println();
    }
  }

  xTaskCreatePinnedToCore(presentationTask, "Presentation", 4000, NULL, 1, &task_pres, 0);
  xTaskCreatePinnedToCore(emptyTask, "Empty", 4000, NULL, 1, &task_empty, 1);
  xTaskCreatePinnedToCore(networkTask, "TaskNetwork", 4000, NULL, 1, &taskNetwork, 0);

  attachInterrupt(IN_GPIO, ISR, CHANGE);
}
void loop()
{
  tSensors.requestTemperatures();
  DeviceAddress oldDeviceAddress;

  for (int i = 0; i < ntDevices; i++)
  {
    if (tSensors.getAddress(tempDeviceAddress.address, i))
    {
      int device = lookupDevice(tempDeviceAddress);

      if (device != -1)
      {

        float tc = tSensors.getTempC(tempDeviceAddress.address);

        if (device == 0)
        {
          tempEngine = baseKelvin + tc;
        }
        else if (device == 1)
        {
          tempExhaust = baseKelvin + tc;
        }
        else if (device == 2)
        {
          tempEngineRoom = baseKelvin + tc;
        }

        if (device == 0 || device == 1 || device == 2)
        {
          sendTemperature(tc, device);
        }

        if (DEBUG_T)
        {
          Serial.print("Device ");
          Serial.print(device);
          Serial.print(" (");
          Serial.print(tempDeviceAddress.id);
          Serial.print(") ");
          Serial.print(tc);
          Serial.println(" ºC");
        }
      }
    }
  }

  vTaskDelay(4000);
}
