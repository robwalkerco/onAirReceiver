#include <Arduino.h>

#include <sstream>
#include <string>

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

// Dimensions the buffer that the task being created will use as its stack.
// NOTE:  This is the number of bytes the stack will hold, not the number of
// words as found in vanilla FreeRTOS.
#define STACK_SIZE 200

// Structure that will hold the TCB of the task being created.
StaticTask_t xTaskBuffer;

// Buffer that the task being created will use as its stack.  Note this is
// an array of StackType_t variables.  The size of StackType_t is dependent on
// the RTOS port.
StackType_t xStack[STACK_SIZE];

#define uS_TO_S_FACTOR 1000000 /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP 1        /* Time ESP32 will go to sleep (in seconds) */

BLEScan *pBLEScan;

#define SCAN_TIME 5 // seconds

// use first channel of 16 channels (started from zero)
#define LEDC_CHANNEL_0 0
// #define LEDC_CHANNEL_1 1

// use 13 bit precission for LEDC timer
#define LEDC_TIMER_13_BIT 13

// use 5000 Hz as a LEDC base frequency
#define LEDC_BASE_FREQ 5000

// fade LED PIN (replace with LED_BUILTIN constant for built-in LED)
#define LED_PIN 27
// #define LED_PIN2 LED_BUILTIN

#define OFF_BRIGHTNESS 0
#define INITIAL_BRIGHTNESS 25
#define INITIAL_FADE_AMOUNT 5

int brightness = INITIAL_BRIGHTNESS;  // how bright the LED is
int fadeAmount = INITIAL_FADE_AMOUNT; // how many points to fade the LED by

bool onAir;
bool seenInScan;

class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks
{
  void onResult(BLEAdvertisedDevice advertisedDevice)
  {
    Serial.println(" ");

    Serial.println("Advertised Device:");
    Serial.println(advertisedDevice.toString().c_str());

    if (advertisedDevice.haveManufacturerData())
    {
      std::string md = advertisedDevice.getManufacturerData();

      Serial.println("Length: ");
      Serial.println(md.length());
      Serial.println("");
      if (md.length() == 25)
      {
        char *pHex = BLEUtils::buildHexData(nullptr, (uint8_t *)md.data(), md.length());

        std::string manufacturerDataStr = pHex;
        std::string uuid = manufacturerDataStr.substr(8, 32).c_str();
        std::string minor = manufacturerDataStr.substr(47, 1).c_str();
        free(pHex);

        if (
            uuid == "22d05fe6ed266b8d56471895e33c9791")
        {
          Serial.println("FOUND IT");
          Serial.println("Minor");
          Serial.println(minor.c_str());

          if (minor == "1")
          {
            onAir = true;
          }
          else
          {
            onAir = false;
          }

          seenInScan = true;

          pBLEScan->stop();
        }
      }
    }

    Serial.println(" ");
  }
};

// Arduino like analogWrite
// value has to be between 0 and valueMax
void ledcAnalogWrite(uint8_t channel, uint32_t value, uint32_t valueMax = 255)
{
  // calculate duty, 8191 from 2 ^ 13 - 1
  uint32_t duty = (8191 / valueMax) * min(value, valueMax);

  // write duty to LEDC
  ledcWrite(channel, duty);
}

void bleScan(void *parameter)
{
  for (;;)
  {
    seenInScan = false;

    pBLEScan = BLEDevice::getScan(); //create new scan
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
    pBLEScan->setActiveScan(true); //active scan uses more power, but get results faster
    pBLEScan->setInterval(0x50);
    pBLEScan->setWindow(0x30);

    Serial.println((String) "Start BLE scan for " + SCAN_TIME + " seconds...");

    pBLEScan->start(SCAN_TIME);

    Serial.println("Scan done!");

    if (!seenInScan)
    {
      onAir = false;
    }

    if (onAir)
    {
      // Wait for a bit
      vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    else
    {
      // Deep Sleep for some time
      esp_deep_sleep_start();
    }
    continue;
  }
}

void setup()
{
  Serial.begin(115200);

  Serial.println("ESP32 BLE Scanner");
  Serial.println("BLEDevice::init()");

  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);

  onAir = false;

  BLEDevice::init("");

  BLEScan *pBLEScan = BLEDevice::getScan(); //create new scan
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(false); //active scan uses more power, but get results faster
  pBLEScan->setInterval(0x50);
  pBLEScan->setWindow(0x30);

  // Setup timer and attach timer to a led pin
  ledcSetup(LEDC_CHANNEL_0, LEDC_BASE_FREQ, LEDC_TIMER_13_BIT);
  // ledcSetup(LEDC_CHANNEL_1, LEDC_BASE_FREQ, LEDC_TIMER_13_BIT);
  ledcAttachPin(LED_PIN, LEDC_CHANNEL_0);
  // ledcAttachPin(LED_PIN2, LEDC_CHANNEL_1);

  xTaskCreatePinnedToCore(
      bleScan,
      "bleScan", // Task name
      5000,      // Stack size (bytes)
      NULL,      // Parameter
      1,         // Task priority
      NULL,      // Task handle
      0);
}

void flash()
{
  if (onAir)
  {
    // digitalWrite(LED_PIN, HIGH);
    // brightness is set to 0 when the leds should be off, so turn them back on
    if (brightness == OFF_BRIGHTNESS)
    {
      brightness = INITIAL_BRIGHTNESS;
    }

    // set the brightness on LEDC channel 0
    // pinMode(LED_BUILTIN, OUTPUT);
    ledcAnalogWrite(LEDC_CHANNEL_0, brightness);
    // ledcAnalogWrite(LEDC_CHANNEL_1, brightness);

    // change the brightness for next time through the loop:
    brightness = brightness + fadeAmount;

    // reverse the direction of the fading at the ends of the fade:
    if (brightness <= INITIAL_BRIGHTNESS || brightness >= 255)
    {
      fadeAmount = -fadeAmount;
    }
    // wait for 30 milliseconds to see the dimming effect
    delay(20);
  }
  else
  {
    // Reset the led
    brightness = OFF_BRIGHTNESS;
    fadeAmount = INITIAL_FADE_AMOUNT;

    ledcAnalogWrite(LEDC_CHANNEL_0, brightness);
    // digitalWrite(LED_PIN, LOW);
    // digitalWrite(LED_PIN, HIGH);
    // ledcAnalogWrite(LEDC_CHANNEL_1, brightness);
  }
}

void loop()
{
  flash();
}