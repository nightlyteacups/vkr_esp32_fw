#include "soc/rtc_cntl_reg.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "driver/adc.h"
#include "esp32/ulp.h"
#include "esp_sleep.h"
#include "ulp_main.h"
#include "ulptool.h"
#include <WEMOS_SHT3X.h>
#include <Wire.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <WiFi.h>
#include <EEPROM.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Adafruit_NeoPixel.h>
#include <HTTPUpdate.h>
#include <WiFiClient.h>


#define SERVICE_UUID              "4fafc201-1fb5-459e-8fcc-c5c9c331914b"      // uuid of ble service
#define SSID_CHARACTERISTIC_UUID  "beb5483e-36e1-4688-b7f5-ea07361b26a8"      // uuid of wifi ssid
#define PASSW_CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a9"      // uuid of wifi password

#define SHT_30_ADRR 0x44       // I2C slave address of SHT30 sensor
#define MAX_44009_ADRR 0x4A    // I2C slave address of MAX44009 sensor
#define EEPROM_SIZE 512        // max size of EEPROM
#define MAX_ATTEMTPS 10        // limit of attempts to connect to wifi
#define BUTTON_TURN_BITMASK 0x400000000         // 2^34 in hex
#define BUTTON_TURN_RESET_BITMASK 0x400008000   // 2^34 + 2^15 in hex


// Unlike the esp-idf always use these binary blob names
extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t ulp_main_bin_end[]   asm("_binary_ulp_main_bin_end");

const char* root_ca_cert = "-----BEGIN CERTIFICATE-----\n"    // HTTPS certificate of server
                           "MIIDdTCCAl2gAwIBAgILBAAAAAABFUtaw5QwDQYJKoZIhvcNAQEFBQAwVzELMAkG\n"
                           "A1UEBhMCQkUxGTAXBgNVBAoTEEdsb2JhbFNpZ24gbnYtc2ExEDAOBgNVBAsTB1Jv\n"
                           "b3QgQ0ExGzAZBgNVBAMTEkdsb2JhbFNpZ24gUm9vdCBDQTAeFw05ODA5MDExMjAw\n"
                           "MDBaFw0yODAxMjgxMjAwMDBaMFcxCzAJBgNVBAYTAkJFMRkwFwYDVQQKExBHbG9i\n"
                           "YWxTaWduIG52LXNhMRAwDgYDVQQLEwdSb290IENBMRswGQYDVQQDExJHbG9iYWxT\n"
                           "aWduIFJvb3QgQ0EwggEiMA0GCSqGSIb3DQEBAQUAA4IBDwAwggEKAoIBAQDaDuaZ\n"
                           "jc6j40+Kfvvxi4Mla+pIH/EqsLmVEQS98GPR4mdmzxzdzxtIK+6NiY6arymAZavp\n"
                           "xy0Sy6scTHAHoT0KMM0VjU/43dSMUBUc71DuxC73/OlS8pF94G3VNTCOXkNz8kHp\n"
                           "1Wrjsok6Vjk4bwY8iGlbKk3Fp1S4bInMm/k8yuX9ifUSPJJ4ltbcdG6TRGHRjcdG\n"
                           "snUOhugZitVtbNV4FpWi6cgKOOvyJBNPc1STE4U6G7weNLWLBYy5d4ux2x8gkasJ\n"
                           "U26Qzns3dLlwR5EiUWMWea6xrkEmCMgZK9FGqkjWZCrXgzT/LCrBbBlDSgeF59N8\n"
                           "9iFo7+ryUp9/k5DPAgMBAAGjQjBAMA4GA1UdDwEB/wQEAwIBBjAPBgNVHRMBAf8E\n"
                           "BTADAQH/MB0GA1UdDgQWBBRge2YaRQ2XyolQL30EzTSo//z9SzANBgkqhkiG9w0B\n"
                           "AQUFAAOCAQEA1nPnfE920I2/7LqivjTFKDK1fPxsnCwrvQmeU79rXqoRSLblCKOz\n"
                           "yj1hTdNGCbM+w6DjY1Ub8rrvrTnhQ7k4o+YviiY776BQVvnGCv04zcQLcFGUl5gE\n"
                           "38NflNUVyRRBnMRddWQVDf9VMOyGj/8N7yy5Y0b2qvzfvGn9LhJIZJrglfCm7ymP\n"
                           "AbEVtQwdpf5pLGkkeB6zpxxxYu7KyJesF12KwvhHhm4qxFYxldBniYUr+WymXUad\n"
                           "DKqC5JlR3XC321Y9YeRq4VzW9v493kHMB65jUr9TU/Qr6cf9tveCX4XSQRjbgbME\n"
                           "HMUfpIBvFSDJ3gyICh3WZlXi/EjJKSZp4A==\n"
                           "-----END CERTIFICATE-----\n";

const gpio_num_t SENSORS_KEY = GPIO_NUM_33;  // GPIO connected to sensors key
const gpio_num_t MOTOR_KEY = GPIO_NUM_27;    // GPIO connected to motor key
const gpio_num_t LED_PIN = GPIO_NUM_17;      // GPIO connected to data pin of LED

const byte TURN_PIN = 34;          // num of GPIO connected to turn on/off button
const byte RESET_PIN = 15;         // num of GPIO connected to EEPROM reset button

const char* DEVICE_ID = "1234";              // ID of device for authentication on server
const uint32_t FW_VERSION = 2020051601;      // year_month_day_revision

const uint32_t IRRIGATION_PERIOD = 200;       // period of irrigation (seconds)
const uint32_t UPDATE_PERIOD = 4 * 60 * 60;   // period of update check (seconds)

bool wifi_ssid_set = false;        // set if wifi ssid was entered
bool wifi_pass_set = false;        // set if wifi password was entered
byte credentials_flag;     // set (184) if wifi credentials are in memory
std::string wifi_ssid;             // SSID of wifi hotspot
std::string wifi_passphrase;       // wifi password

// declare LED object
Adafruit_NeoPixel indicator(1, LED_PIN, NEO_GRB + NEO_KHZ800);

// define led colors for convenient use
uint32_t white = Adafruit_NeoPixel::Color(200, 200, 200);
uint32_t green = Adafruit_NeoPixel::Color(0, 200, 0);
uint32_t red = Adafruit_NeoPixel::Color(200, 0, 0);
uint32_t orange = Adafruit_NeoPixel::Color(200, 130, 0);
uint32_t yellow = Adafruit_NeoPixel::Color(200, 200, 0);
uint32_t violet = Adafruit_NeoPixel::Color(200, 0, 200);
uint32_t blue = Adafruit_NeoPixel::Color(0, 0, 200);
uint32_t no_color = Adafruit_NeoPixel::Color(0, 0, 0);

class InputCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {          // called when any characteristic is changed
      std::string uuid_of_changed = pCharacteristic->getUUID().toString();       // get uuid of changed characteristic
      if (uuid_of_changed == SSID_CHARACTERISTIC_UUID) {            // set flag and assign variable depending on the uuid
        wifi_ssid_set = true;
        wifi_ssid = pCharacteristic->getValue();
        ets_printf("WiFi SSID was entered and is: %s\n", wifi_ssid.c_str());
      }
      if (uuid_of_changed == PASSW_CHARACTERISTIC_UUID) {
        wifi_pass_set = true;
        wifi_passphrase = pCharacteristic->getValue();
        ets_printf("WiFi password was entered and is: %s\n", wifi_passphrase.c_str());
      }
    }
};

static void init_ulp_program() {  // init ULP program
  int res = ulp_reset_flag & 0xFFFF;
  int hib = ulp_hibernation_flag & 0xFFFF;

  esp_err_t err = ulptool_load_binary(0, ulp_main_bin_start, (ulp_main_bin_end - ulp_main_bin_start) / sizeof(uint32_t));
  ESP_ERROR_CHECK(err);

  ets_printf("Inside init: hib = %d\n", hib);
  ets_printf("Inside init: res = %d\n", res);

  ulp_adc_1_4r = 0;   // raw adc values
  ulp_adc_1_4q = 0;   // r - reminder after devision
  ulp_adc_1_7r = 0;   // q - quotient
  ulp_adc_1_7q = 0;
  ulp_adc_2_4r = 0;
  ulp_adc_2_4q = 0;
  ulp_adc_2_6r = 0;
  ulp_adc_2_6q = 0;
  ulp_analog_measurements_taken = 0; // set (2905) if analog measurements in deep sleep were taken

  // init adc1
  adc1_config_width(ADC_WIDTH_BIT_12);                         // set bit width of ADC1
  adc1_config_channel_atten(ADC1_CHANNEL_4, ADC_ATTEN_DB_11);  // GPIO 32
  adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_DB_11);  // GPIO 35
  adc1_ulp_enable();                                           // enable ADC1 for ULP

  // init adc2
  //adc2_config_width(ADC_WIDTH_BIT_12);                       // not purposed for ADC2
  adc2_config_channel_atten(ADC2_CHANNEL_4, ADC_ATTEN_DB_11);  // GPIO 13
  adc2_config_channel_atten(ADC2_CHANNEL_6, ADC_ATTEN_DB_11);  // GPIO 14
  //adc2_ulp_enable();                                         // not supported and not needed for ADC2

  rtc_gpio_init(SENSORS_KEY);  // GPIO_33 connected to sensors key
  rtc_gpio_set_direction(SENSORS_KEY, RTC_GPIO_MODE_OUTPUT_ONLY);  // GPIO_33 is output

  if (hib == 13) {  // if flag was set before reboot
    detachInterrupt(digitalPinToInterrupt(TURN_PIN));   // because interrupts and wakeups can override eachother for some reason
    delay(100);  // just to make sure no signal from push button is present, might be not neccessary
    ets_printf("Entering hibernation..\n\n");
    esp_sleep_enable_ext1_wakeup(BUTTON_TURN_BITMASK, ESP_EXT1_WAKEUP_ANY_HIGH);  // wakeup only by turn on/off button
    esp_deep_sleep_start();
  }
  if (res == 2502) { // if flag was set before reboot
    EEPROM.write(0, 0);  // EEPROM[0] = 0 (clear EEPROM)
    EEPROM.commit();
    ulp_reset_flag = 0;  // free flag
    byte cntrlByte = EEPROM.read(0);
    ets_printf("After clear --- Control byte = %d / 512\n", cntrlByte);
  }

  ulp_hibernation_flag = 0;   // initial state: 0 - on, 13 - off
}

void IRAM_ATTR esp_hibernation_start() {  // ISR to start hibernation
  ets_printf("Turn off button pressed, set flag and reboot\n");
  ulp_hibernation_flag = 13;  // set flag for hibernation
  indicator.setPixelColor(0, no_color);  // turn LED off
  indicator.show();   // apply changes to LED
  indicator.show();   // double because ESP32 has timing issues with neopixel-like leds
  ESP.restart();  // reboot ESP
}

void IRAM_ATTR active_clear_eeprom() {  // ISR to clear EEPROM
  ets_printf("Reset button pressed while active, set flag, reboot and clear EEPROM\n");
  byte cntrlByte = EEPROM.read(0);  // get size of used EEPROM
  ets_printf("Before clear --- Control byte = %d\n", cntrlByte);
  ulp_reset_flag = 2502;  // set flag for EEPROM clear
  ESP.restart();  // reboot ESP
}

void sleep_clear_eeprom() {    // clear EEPROM after wakeup from deep sleep
  ets_printf("Wakeup by reset button, clear EEPROM, then reboot\n");
  byte cntrlByte = EEPROM.read(0);
  ets_printf("Before clear --- Control byte = %d\n", cntrlByte);
  EEPROM.write(0, 0);  // EEPROM[0] = 0 (clear control byte)
  EEPROM.write(9, 0);  // EEPROM[9] = 0 (clear wifi credentials flag)
  EEPROM.commit();
  ulp_reset_flag = 0;  // free flag
  cntrlByte = EEPROM.read(0);
  ets_printf("After clear ---  Control byte = %d\n\n", cntrlByte);
  ESP.restart();  // reboot ESP
}

void write_String(byte add, std::string data) {  // write String data to EEPROM
  int _size = data.length();
  int i;
  for (i = 0; i < _size; i++) {
    EEPROM.write(add + i, data[i]);
  }
  EEPROM.write(add + _size, '\0');      // add termination null character for String Data
  EEPROM.commit();
}

String read_String(byte add) {  // read String data from EEPROM
  int i;
  char data[64];     // max 64 Bytes
  int len = 0;
  unsigned char k;
  k = EEPROM.read(add);
  while (k != '\0' && len < 65) {      // read until null character
    k = EEPROM.read(add + len);
    data[len] = k;
    len++;
  }
  data[len] = '\0';
  return String(data);
}

void write_UInt(byte add, uint32_t num) {      // write uint32_t to EEPROM
  byte data[4];   // split uint by array of 4 bytes
  data[0] = num & 0x000000FF;
  data[1] = (num & 0x0000FF00) >> 8;
  data[2] = (num & 0x00FF0000) >> 16;
  data[3] = (num & 0xFF000000) >> 24;
  for (int i = 0; i < 4; i++) {
    EEPROM.write(add + i, data[i]);  // write each byte individually
  }
  EEPROM.commit();
}

uint32_t read_UInt(byte add) {      // write uint32_t from EEPROM
  byte data[4];
  uint32_t num = 0;   // final 32-bit value
  data[0] = num & 0x000000FF;
  data[1] = (num & 0x0000FF00) >> 8;
  data[2] = (num & 0x00FF0000) >> 16;
  data[3] = (num & 0xFF000000) >> 24;
  for (int i = 0; i < 4; i++) {
    data[i] = EEPROM.read(add + i);  // read each byte
  }
  for (int i = 0; i < 4; i++) {
    num += data[i] << 8 * i;      // transform byte array to uint32
  }
  return num;
}

void blink_orange_led() {
  for (int i = 0; i <= 7; i++) {
    indicator.setPixelColor(0, no_color);  // turn LED off
    indicator.show();   // apply changes to LED
    indicator.show();   // double because ESP32 has timing issues with neopixel-like leds
    delay(500);
    indicator.setPixelColor(0, orange);  // turn LED orange
    indicator.show();   // apply changes to LED
    indicator.show();   // double because ESP32 has timing issues with neopixel-like leds
    delay(500);
  }
}

void blink_cur_led() {         // blink while sending sensor data to server
  byte last_led_color = EEPROM.read(1);
  uint32_t current_color;

  switch (last_led_color) {
    case 1:
      current_color = green;
      break;
    case 2:
      current_color = yellow;
      break;
    case 3:
      current_color = red;
      break;
  }
  for (int i = 0; i <= 3; i++) {
    indicator.setPixelColor(0, no_color);  // turn LED off
    indicator.show();   // apply changes to LED
    indicator.show();   // double because ESP32 has timing issues with neopixel-like leds
    delay(500);
    indicator.setPixelColor(0, current_color);  // turn LED orange
    indicator.show();   // apply changes to LED
    indicator.show();   // double because ESP32 has timing issues with neopixel-like leds
    delay(500);
  }
}

void set_last_led() {
  byte last_led_color = EEPROM.read(1);
  switch (last_led_color) {
    case 1:
      indicator.setPixelColor(0, green);  // turn LED green
      indicator.show();   // apply changes to LED
      indicator.show();   // double because ESP32 has timing issues with neopixel-like leds
      break;
    case 2:
      indicator.setPixelColor(0, yellow);  // turn LED yellow
      indicator.show();   // apply changes to LED
      indicator.show();   // double because ESP32 has timing issues with neopixel-like leds
      break;
    case 3:
      indicator.setPixelColor(0, red);  // turn LED red
      indicator.show();   // apply changes to LED
      indicator.show();   // double because ESP32 has timing issues with neopixel-like leds
      break;
    default:
      EEPROM.write(1, 1);           // set last_led_color green (EEPROM[1] = 1) - first boot
      EEPROM.commit();
      indicator.setPixelColor(0, green);  // turn LED green
      indicator.show();   // apply changes to LED
      indicator.show();   // double because ESP32 has timing issues with neopixel-like leds
      break;
  }
}

void save_cur_led(byte color) {      // save current led color in EEPROM
  EEPROM.write(1, color);
  EEPROM.commit();
}

bool connect_to_WiFi(String ssid, String passphrase) {    // establish connection with wifi hotspot
  ets_printf("\nAttempt to connect to Wi-Fi\n");

  WiFi.begin(ssid.c_str(), passphrase.c_str());

  byte atmpt = 1;
  ets_printf("Connecting");
  while (WiFi.status() != WL_CONNECTED) {       // wait for modem to connect
    ets_printf(".");
    delay(500);
    atmpt++;                     // increase number of attempts
    if (atmpt > MAX_ATTEMTPS) {  // if attempts exceed limit
      break;                     // stop
    }
  }
  if (WiFi.status() == WL_CONNECTED) {           // check status after all attempts
    ets_printf("\nWi-Fi connected\nLocal ip: ");
    ets_printf("%s\n", WiFi.localIP().toString().c_str());
    return true;
  }
  else {
    ets_printf("\nFailed to connect\n");
    return false;
  }
}

void wait_for_config() {         // idle while not all flags are set
  while (!(wifi_ssid_set && wifi_pass_set)) {
    delay(300);
  }
}

void configure_via_BLE() {
  BLEDevice::init("ESP32_VKR");             // name of BLE device
  BLEServer *pServer = BLEDevice::createServer();

  BLEService *pService = pServer->createService(SERVICE_UUID);       // create ble service on ble server

  BLECharacteristic *SSID_pCharacteristic = pService->createCharacteristic(     // add characteristic to service
        SSID_CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ |
        BLECharacteristic::PROPERTY_WRITE
      );
  BLECharacteristic *PASSW_pCharacteristic = pService->createCharacteristic(     // add characteristic to service
        PASSW_CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ |
        BLECharacteristic::PROPERTY_WRITE
      );

  SSID_pCharacteristic->setCallbacks(new InputCallbacks());
  PASSW_pCharacteristic->setCallbacks(new InputCallbacks());
  SSID_pCharacteristic->setValue("");
  PASSW_pCharacteristic->setValue("");
  pService->start();                   // start service on ble server

  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->start();                  // start ble advertising
  ets_printf("\nStarting BLE advertising..\n");
  delay(3000);
  ets_printf("Success\n");

  wait_for_config();

  ets_printf("\nFinally out of WHILE loop\n");
  ets_printf("Shutting down BLE advertising..\n");
  pAdvertising->stop();       // stop ble advertising

  delay(1700);
}

uint32_t get_now_time() {          // get current time in Unix format
  HTTPClient http;

  ets_printf("http GET:\n");

  http.begin("http://worldtimeapi.org/api/timezone/Etc/GMT");   // specify the URL
  int httpResponseCode = http.GET();                            // make the request

  if (httpResponseCode > 0) { // check for the returning code
    String response = http.getString();                     // get the response to the request
    ets_printf("Code: %d\n", httpResponseCode);          // print return code
    //Serial.printf("Response: %s\n", response.c_str());      // print response

    uint32_t now_time = parse_unix_time(response.c_str());
    ets_printf("UnixTime now: %d\n", now_time);

    http.end(); // free the resources
    return now_time;
  }
  else {
    ets_printf("Error on HTTP request (%d)\n", httpResponseCode);
    http.end(); // free the resources
    return 0;
  }
}

uint32_t parse_unix_time(const char reply[]) {    // parse time from JSON response
  StaticJsonDocument<512> doc;
  DeserializationError error = deserializeJson(doc, reply);
  if (!error) {
    int unix_time = doc["unixtime"];
    return unix_time;
  }
  else {
    ets_printf("Error while JSON unix_time parse (%s)\n", error.c_str());
    return 0;
  }
}

double calculate_volts(double raw, uint32_t adc) {  // calculate voltage from raw ADC bits
  if (adc == 1) {
    return (53134 * raw + 32768) / 65536.0 + 142;
  }
  else {
    return (53287 * raw + 32768) / 65536.0 + 128;
  }
}

float get_air_hum() {    // get air humidity from SHT30
  SHT3X sensor(SHT_30_ADRR);
  if (sensor.get() == 0) {  // if sensor is available
    return sensor.humidity;
  }
  else {
    ets_printf("Warning, SHT30 is unavailable!\n");
    return 0.0;
  }
}

float get_air_temp() {    // get air temperature from SHT30
  SHT3X sensor(SHT_30_ADRR);
  if (sensor.get() == 0) {  // if sensor is available
    return sensor.cTemp;
  }
  else {
    ets_printf("Warning, SHT30 is unavailable!\n");
    return 0.0;
  }
}

float get_luminance() {  // get luminance from MAX44009
  Wire.begin(21, 22); // init I2C: SDA = GPIO21 / SCL = GPIO22
  Wire.beginTransmission(MAX_44009_ADRR);
  Wire.write(0x02);  // add byte to queue
  Wire.write(0x40);  // add byte to queue
  Wire.endTransmission(); // send bytes from queue
  delay(300);   // wait for sensor to receive commands and
  unsigned int lum_data[2];  // array for aquired data
  Wire.beginTransmission(MAX_44009_ADRR);
  Wire.write(0x03);  // add byte to queue
  Wire.endTransmission(); // send bytes from queue
  // request 2 bytes of data
  Wire.requestFrom(MAX_44009_ADRR, 2);
  // read 2 bytes of data luminance msb, luminance lsb
  if (Wire.available() == 2) {
    lum_data[0] = Wire.read();
    lum_data[1] = Wire.read();
  }
  else {
    ets_printf("Warning, MAX44009 is unavailable!\n");
    return 0.0;
  }
  // convert the data to lux (all calculations based on sensor datasheet)
  int exponent = (lum_data[0] & 0xF0) >> 4;
  int mantissa = ((lum_data[0] & 0x0F) << 4) | (lum_data[1] & 0x0F);
  float luminance = pow(2, exponent) * mantissa * 0.045;
  return luminance;
}

double volt_to_hum(double volt) {  // convert voltage to soil humidity
  double hum = pow(volt / 1000, -2.152) * 69.943; // calculated for specific designed analog sensor
  if (hum > 100) {
    hum = 100.0;
  }
  if (hum < 0) {
    hum = 0.0;
  }
  return hum;
}

double volt_to_battery_percent(double volt) {  // convert voltage to percent of battery charge
  double battery_percent = (volt - 1350) / 750 * 100; // calculated for specific designed analog sensor
  if (battery_percent > 100) {
    battery_percent = 100.0;
  }
  if (battery_percent < 0) {
    battery_percent = 0.0;
  }
  return battery_percent;
}

bool volt_to_water_level(double volt) {
  if (volt > 1650) {
    return false;
  }
  else {
    return true;
  }
}

int check_if_device_linked(String deviceID) {
  if ((WiFi.status() == WL_CONNECTED)) {    // check the current connection status
    ets_printf("\nChecking if device is linked to user and plant..\n");

    // dID = String(deviceID);   // http client lib bug avoid
    delay(200);

    HTTPClient httpClient;
    httpClient.begin("https://nevokshonov.mastercode.me/Api/Device/" + deviceID, root_ca_cert);
    int httpResponseCode = httpClient.GET();
    if (httpResponseCode == 200) {
      ets_printf("Device ID %s is linked to user and plant\n", deviceID.c_str());
    }
    else if (httpResponseCode == 400) {
      ets_printf("Error, Device ID %s is absent from database: 400\n", deviceID.c_str());
      httpClient.end(); // free the resources
    }
    else if (httpResponseCode == 403) {
      ets_printf("Error, Device ID %s is not linked to the user and/or plant: 403\n", deviceID.c_str());
      httpClient.end(); // free the resources
    }
    else {
      ets_printf("Error while check if device linked: %d\n", httpResponseCode);
      httpClient.end(); // free the resources
    }
    return httpResponseCode;
  }
  else {
    ets_printf("Lost connection to WiFi hotspot\n");
    return -2;
  }
}

String https_post_data(const char* device_id, int light, int temp, int env_hum, int soil_hum, int soil_fert, int battery, int water_level) {    // post data from sensors to server
  if ((WiFi.status() == WL_CONNECTED)) {    // check the current connection status

    HTTPClient http;
    ets_printf("https POST data:\n");

    http.begin("https://nevokshonov.mastercode.me/Api/Device", root_ca_cert);  // specify destination for HTTPS request and sertificate
    http.addHeader("Content-Type", "application/json");                        // specify content-type header

    char request_body[200];  // buffer for formatted body of request
    sprintf(request_body, "{DeviceId: %s, Light: %d, Temp: %d, EnvHumid: %d, SoilMoist: %d, SoilEc: %d, Battery: %d, WaterRemained: %d}", device_id, light, temp, env_hum, soil_hum, soil_fert, battery, water_level);   // form the body of request
    int httpResponseCode = http.POST(request_body);   // send the actual POST request and get response code

    if (httpResponseCode == 200) {
      String response = http.getString();                  // get the response to the request
      ets_printf("Code: %d\n", httpResponseCode);          // print return code
      ets_printf("Response: %s\n", response.c_str());      // print response to request
      http.end(); // free the resources
      return response;
    }
    else if (httpResponseCode == 400) {
      ets_printf("Error, Device ID is absent from database: 400\n");
      http.end(); // free the resources
      return "400";
    }
    else if (httpResponseCode == 403) {
      ets_printf("Error, device is not linked to the user: 403\n");
      http.end(); // free the resources
      return "403";
    }
    else {
      ets_printf("Error while data POST: %d\n", httpResponseCode);
      http.end(); // free the resources
      return String(httpResponseCode);
    }
  }
  else {
    ets_printf("Lost connection to WiFi hotspot\n");
    return "-2";
  }
}

bool parse_sensor_data_check(const char response[]) {     // parse response to sensor data post and see if environmental params are ok
  StaticJsonDocument<512> doc;
  DeserializationError error = deserializeJson(doc, response);   // deserialize JSON response
  if (!error) {
    bool isLightOk = doc["isLightOk"];          // read bool field from response
    bool isTempOk = doc["isTempOk"];            // read bool field from response
    bool isEnvHumidOk = doc["isEnvHumidOk"];    // read bool field from response
    bool isSoilMoistOk = doc["isSoilMoistOk"];  // read bool field from response
    bool isSoilEcOk = doc["isSoilEcOk"];        // read bool field from response

    return (isLightOk & isTempOk & isEnvHumidOk & isSoilMoistOk & isSoilEcOk);  // true if all are true
  }
  else {
    ets_printf("\nError while JSON sensor data response parse (%s)\n", error.c_str());
    return false;
  }
}

void checkForUpdates() {         // check if there is new version of firmware on server
  if ((WiFi.status() == WL_CONNECTED)) {    // check the current connection status
    ets_printf("\nChecking for firmware updates..\n");

    WiFiClient Wclient;
    HTTPClient httpClient;
    httpClient.begin("https://nevokshonov.mastercode.me/Api/DevicePlant/GetFirmware", root_ca_cert);
    int httpResponseCode = httpClient.GET();
    if (httpResponseCode == 200) {
      String response = httpClient.getString();      // get the response to the request
      ets_printf("Current firmware version: %d\n", FW_VERSION);

      uint32_t newFWVersion = parse_fw_version(response.c_str());   // get up-to-date firmware version from reply
      ets_printf("Available firmware version: %d\n", newFWVersion);
      String fwImageURL = parse_fw_link(response.c_str());   // get firmware image link from reply
      ets_printf("Firmware image URL: %s\n", fwImageURL.c_str());

      if (newFWVersion > FW_VERSION) {
        ets_printf("Preparing to update\n");

        t_httpUpdate_return ret = httpUpdate.update(Wclient, fwImageURL); // download firmware image and try to update
        switch (ret) {
          case HTTP_UPDATE_FAILED:
            ets_printf("HTTP_UPDATE_FAILED Error (%d): %s", httpUpdate.getLastError(), httpUpdate.getLastErrorString().c_str()); /// FOR ESP32
            break;

          case HTTP_UPDATE_NO_UPDATES:
            ets_printf("HTTP_UPDATE_NO_UPDATES\n");
            break;

          case HTTP_UPDATE_OK:
            ets_printf("Update successfully completed. Rebooting\n\n");
            ESP.restart();
        }
      }
      else {
        ets_printf("Already on latest version (%d)\n", FW_VERSION);
      }
    }
    else {
      ets_printf("Firmware version check failed, got HTTP response code: %d\n", httpResponseCode);
    }
    httpClient.end();   // free resources
  }
  else {
    ets_printf("Lost connection to WiFi hotspot\n");
  }
}

uint32_t parse_fw_version(const char reply[]) {   // parse up-to-date version of firmware from response
  StaticJsonDocument<512> doc;
  DeserializationError error = deserializeJson(doc, reply);
  if (!error) {
    uint32_t fw_version = doc["version"];
    return fw_version;
  }
  else {
    ets_printf("Error while JSON fw_version parse (%s)\n", error.c_str());
    return 0;
  }
}

String parse_fw_link(const char reply[]) {   // parse link to firmware image from response
  StaticJsonDocument<512> doc;
  DeserializationError error = deserializeJson(doc, reply);
  if (!error) {
    String fw_link = doc["link"];
    return fw_link;
  }
  else {
    ets_printf("Error while JSON fw_link parse (%s)\n", error.c_str());
    return String('\0');
  }
}

void setup() {
  rtc_gpio_deinit(GPIO_NUM_15);  // because ext1 overrides GPIO
  rtc_gpio_deinit(GPIO_NUM_34);  // because ext1 overrides GPIO
  delay(1000);                     // wait for device boot properly just in case
  EEPROM.begin(EEPROM_SIZE);       // init EEPROM
  indicator.begin();               // init LED
  pinMode(TURN_PIN, INPUT);        // set GPIO_34 as input
  pinMode(RESET_PIN, INPUT);       // set GPIO_15 as input
  pinMode(SENSORS_KEY, OUTPUT);    // set GPIO_33 as output
  pinMode(MOTOR_KEY, OUTPUT);      // set GPIO_27 as output
  //Serial.begin(115200);          // sometimes usage of serial and radio module causes conflicts on Core 0, use ets_printf instead

  attachInterrupt(TURN_PIN, esp_hibernation_start, RISING);  // start hibernation from active mode if RISISNG on GPIO
  attachInterrupt(RESET_PIN, active_clear_eeprom, RISING);   // clear eeprom if reset button pressed in active mode

  ets_printf("on -- hibernation flag = %d\n", ulp_hibernation_flag & 0xFFFF);
  ets_printf("on -- reset flag = %d\n", ulp_reset_flag & 0xFFFF);

  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();        // get cause of main processor wakeup

  if (cause == ESP_SLEEP_WAKEUP_ULP) {  // wakeup by the end of ULP program
    ets_printf("Wakeup by ULP\n");
  }

  else if (cause == ESP_SLEEP_WAKEUP_EXT1) {  // wakeup by (multiple) external GPIO
    int GPIO = log(esp_sleep_get_ext1_wakeup_status()) / log(2); // get num of GPIO that caused wakeup
    ets_printf("Wakeup by EXT1, pin was: %d\n", GPIO);
    if (GPIO == RESET_PIN) {   // if reset button was pressed
      ets_printf("Reset button was pressed, ");
      if ((ulp_hibernation_flag & 0xFFFF) == 13 || (ulp_hibernation_flag & 0xFFFF) == 1) {
        ets_printf("but hibernation flag is set, how's that even possible??\n");
        esp_hibernation_start(); // go hibernating
      }
      else {
        ets_printf("and wakeup actually from deep sleep to clear EEPROM\n");
        sleep_clear_eeprom(); // clear EEPROM from deep sleep mode (before reboot)
      }
    }
    else if (GPIO == TURN_PIN) {   // if turn on/off button was pressed
      ets_printf("Turn on/off button was pressed, ");
      if ((ulp_hibernation_flag & 0xFFFF) == 13 || (ulp_hibernation_flag & 0xFFFF) == 1) { // if was hibernating
        ets_printf("and hibernation flag is set, so turn on and reset flag\n");
        ulp_hibernation_flag = 0;  // clear hibernation flag
      }
      else {
        ets_printf("but hibernation flag not set, therefore was sleeping, so start hibernation\n");
        esp_hibernation_start();
      }
    }
    else {   // should not go here
      ets_printf("Unknown pin was used as wakeup cause, something is very wrong!");
    }
  }
  else {   // wakeup by power on or other reasons
    ets_printf("Wakeup by power on or reset, initializing ULP\n");
    init_ulp_program();
  }

  set_last_led();

  byte cntrlByte = EEPROM.read(0);
  ets_printf("Control byte = %d\n", cntrlByte);
  if (cntrlByte) {
    credentials_flag = EEPROM.read(9);
    if (credentials_flag == 184) {          // if credentials are in memory
      ets_printf("credentials_flag is set, read from EEPROM\n");
      String ssid = read_String(10);
      ets_printf("SSID: %s\n", ssid.c_str());
      String passphrase = read_String(43);
      ets_printf("password: %s\n", passphrase.c_str());

      bool success = connect_to_WiFi(ssid, passphrase);    // attempt to connect to WiFi
      if (success) { // if credentials are valid
        ets_printf("WiFi credentials from EEPROM are valid\n");
      }
      else {
        ets_printf("Credentials from EEPROM are invaild and/or AP is unreachable. Please wipe the EEPROM and try again\n");
        indicator.setPixelColor(0, blue);  // turn LED blue
        indicator.show();   // apply changes to LED
        indicator.show();   // double because ESP32 has timing issues with neopixel-like leds
        delay(10 * 1000);
        esp_hibernation_start();  // turn off device
      }
    }

    else {   // ask for input credentials
      ets_printf("credentials_flag NOT set, asking for input\n");

      indicator.setPixelColor(0, white);  // turn LED white
      indicator.show();   // apply changes to LED
      indicator.show();   // double because ESP32 has timing issues with neopixel-like leds

      configure_via_BLE();

      bool success = connect_to_WiFi(String(wifi_ssid.c_str()), String(wifi_passphrase.c_str()));    // attempt to connect to WiFi
      if (success) {        // if credentials are valid
        write_String(10, wifi_ssid);         // save wifi SSID to EEPROM
        write_String(43, wifi_passphrase);   // save wifi password to EEPROM
        EEPROM.write(9, 184);           // set credentials flag (EEPROM[9] = 184)
        EEPROM.write(0, 255);           // set control byte (EEPROM[0] = 255)
        EEPROM.commit();
        ets_printf("WiFi credentials are valid and saved\n");
        set_last_led();
      }
      else {
        ets_printf("\nInput credentials are invaild and/or WiFi AP is unreachable. Please reconfigure the device\n");
        indicator.setPixelColor(0, blue);  // turn LED blue
        indicator.show();   // apply changes to LED
        indicator.show();   // double because ESP32 has timing issues with neopixel-like leds
        delay(10 * 1000);
        set_last_led();
        esp_hibernation_start();  // turn off device
      }
    }
  }
  else {
    ets_printf("EEPROM is clear, first configuration required\n");

    indicator.setPixelColor(0, white);  // turn LED white
    indicator.show();   // apply changes to LED
    indicator.show();   // double because ESP32 has timing issues with neopixel-like leds

    configure_via_BLE();

    bool success = connect_to_WiFi(String(wifi_ssid.c_str()), String(wifi_passphrase.c_str()));    // attempt to connect to WiFi
    if (success) {        // if credentials are valid
      write_String(10, wifi_ssid);         // save wifi SSID to EEPROM
      write_String(43, wifi_passphrase);   // save wifi password to EEPROM
      EEPROM.write(9, 184);           // set credentials flag (EEPROM[9] = 184)
      EEPROM.write(0, 255);           // set control byte (EEPROM[0] = 255)
      EEPROM.commit();

      ets_printf("WiFi credentials are valid and saved\n");
    }
    else {
      ets_printf("\nInput credentials are invaild and/or WiFi AP is unreachable. Please reconfigure the device\n");
      indicator.setPixelColor(0, blue);  // turn LED blue
      indicator.show();   // apply changes to LED
      indicator.show();   // double because ESP32 has timing issues with neopixel-like leds
      delay(10 * 1000);
      set_last_led();
      esp_hibernation_start();  // turn off device
    }
  }

  // check if linked, if not linked blink orange
  int device_linked = check_if_device_linked("1234");
  if (device_linked == -2) {   // DEvice ID not in DB
    indicator.setPixelColor(0, blue);  // turn LED blue
    indicator.show();   // apply changes to LED
    indicator.show();   // double because ESP32 has timing issues with neopixel-like leds
    delay(10 * 1000);
    // maybe turn off
    esp_hibernation_start();  // turn off device
  }
  else if (device_linked == 400 || device_linked == 403) {  // not linked
    blink_orange_led();  // blink orange
    set_last_led();
    // maybe turn off
    esp_hibernation_start();  // turn off device
  }
  else if (device_linked == 200) {
    set_last_led();
  }

  if ((ulp_analog_measurements_taken & 0xFFFF) == 2905) {  // if analog measurements were taken

    // calculate voltage from raw bits
    double voltage14 = calculate_volts((ulp_adc_1_4q & 0xFFFF) + (ulp_adc_1_4r & 0xFFFF) / 16.0, 1);
    double voltage17 = calculate_volts((ulp_adc_1_7q & 0xFFFF) + (ulp_adc_1_7r & 0xFFFF) / 16.0, 1);
    double voltage24 = calculate_volts((ulp_adc_2_4q & 0xFFFF) + (ulp_adc_2_4r & 0xFFFF) / 16.0, 2);
    double voltage26 = calculate_volts((ulp_adc_2_6q & 0xFFFF) + (ulp_adc_2_6r & 0xFFFF) / 16.0, 2);

    char char_buffer[80];  // buffer for format output, used because ets_printf does not support float format (%f)

    sprintf(char_buffer, "Value @ ADC1 CH 4 (humidity sensor): %f (%f mV)\n", (ulp_adc_1_4q & 0xFFFF) + (ulp_adc_1_4r & 0xFFFF) / 16.0, voltage14);  // format floats
    ets_printf("%s", char_buffer);
    sprintf(char_buffer, "Value @ ADC1 CH 7 (fertility sensor): %f (%f mV)\n", (ulp_adc_1_7q & 0xFFFF) + (ulp_adc_1_7r & 0xFFFF) / 16.0, voltage17);  // format floats
    ets_printf("%s", char_buffer);
    sprintf(char_buffer, "Value @ ADC2 CH 4 (water level sensor): %f (%f mV)\n", (ulp_adc_2_4q & 0xFFFF) + (ulp_adc_2_4r & 0xFFFF) / 16.0, voltage24);  // format floats
    ets_printf("%s", char_buffer);
    sprintf(char_buffer, "Value @ ADC2 CH 6 (battery charge sensor): %f (%f mV)\n", (ulp_adc_2_6q & 0xFFFF) + (ulp_adc_2_6r & 0xFFFF) / 16.0, voltage26);  // format floats
    ets_printf("%s", char_buffer);

    // convert voltage to values
    double soil_humidity = volt_to_hum(voltage14);                // calculate soil humidity
    double battery_percent = volt_to_battery_percent(voltage26);  // calculate battery percent
    bool water_level_ok = volt_to_water_level(voltage24);         // calculate if water level is fine
    //double soil_fertility = volt_to_fert(voltage17);            // calculate soil fertility
    double soil_fertility = 1715.3;                               // just decoy

    ets_printf("Analog measurements were taken, turn on i2c sensors\n");
    ets_printf("Lock the sensors key\n");
    digitalWrite(SENSORS_KEY, HIGH);  // set GPIO_33 HIGH to lock the sensors key
    delay(1200);

    float luminance = get_luminance();       // get luminance from I2C sensor
    float air_temperature = get_air_temp();  // get air temperature from I2C sensor
    float air_humidity = get_air_hum();      // get air humidity from I2C sensor

    ets_printf("Unlock the sensors key\n");
    digitalWrite(SENSORS_KEY, LOW); // set GPIO_33 LOW to unlock the sensors key

    sprintf(char_buffer, "\nLuminance: %f lux\n", luminance);  // format floats
    ets_printf("%s", char_buffer);
    sprintf(char_buffer, "Air temperature: %f C\n", air_temperature);  // format floats
    ets_printf("%s", char_buffer);
    sprintf(char_buffer, "Air humidity: %f %%\n", air_humidity);  // format floats
    ets_printf("%s", char_buffer);
    ets_printf("__\n");
    sprintf(char_buffer, "Soil humidity: %f %%\n", soil_humidity);  // format floats
    ets_printf("%s", char_buffer);
    sprintf(char_buffer, "Soil fertility: %f S\n", soil_fertility);  // format floats
    ets_printf("%s", char_buffer);
    sprintf(char_buffer, "Battery charge: %f %%\n", battery_percent);  // format floats
    ets_printf("%s", char_buffer);
    const char* water_level = (water_level_ok) ? "fine" : "low";
    ets_printf("Waterlevel: %s\n\n", water_level);

    int Lum = int(luminance + 0.5);  // round float values
    int Temp = int(air_temperature + 0.5);
    int Air_Hum = int(air_humidity + 0.5);
    int Soil_Hum = int(soil_humidity + 0.5);
    int Soil_Fert = int(soil_fertility + 0.5);
    int Battery = int(battery_percent + 0.5);
    int Water_Left = (water_level_ok) ? 100 : 0;

    String response = https_post_data(DEVICE_ID, Lum, Temp, Air_Hum, Soil_Hum, Soil_Fert, Battery, Water_Left);  // post sensors data to server
    blink_cur_led();
    bool params_ok = parse_sensor_data_check(response.c_str());  // see if environmental params are ok

    ets_printf("\nEnvironmental params are OK: %s\n", params_ok ? "true" : "false");

    if (!params_ok) {  // if params are not normal
      ets_printf("\nSome params are critical");
      if (Battery < 35) {
        ets_printf("\nLow battery charge, red led!\n");
        save_cur_led(3);   // led will be red
        set_last_led;
      }
      else {
        ets_printf(", led going yellow!\n");
        save_cur_led(2);   // led will be yellow
        set_last_led;
      }
    }
    else {
      if (Battery < 35) {
        ets_printf("\nLow battery charge, red led!\n");
        save_cur_led(3);   // led will be red
        set_last_led;
      }
      else {
        ets_printf("\nAll params and battery fine, green led\n");
        save_cur_led(1);   // led will be red
        set_last_led;
      }
    }

    ulp_analog_measurements_taken = 0;  // clear flag to take analog measurements again

    // check if irrigation required
    uint32_t last_irrigation_time = read_UInt(107);    // read last time of irrigation from EEPROM
    ets_printf("Last irrigation time: %d\n", last_irrigation_time);
    uint32_t time_now = get_now_time();
    uint32_t time_since = time_now - last_irrigation_time;
    ets_printf("Time since last irrigation: %d sec\n", time_since);

    if (time_since > IRRIGATION_PERIOD) {
      ets_printf("Time since irrigation > period, Start irrigation\n");
      digitalWrite(MOTOR_KEY, HIGH);  // set GPIO_27 HIGH to start the motor
      delay(5000);

      ets_printf("Stop irrigation\n");
      digitalWrite(MOTOR_KEY, LOW); // set GPIO_27 LOW to stop the motor
      delay(400);
      write_UInt(107, time_now);
    }

    // check if update required
    uint32_t last_upd_check_time = read_UInt(111);    // read last time of update check from EEPROM
    ets_printf("\nLast check for update time: %d\n", last_upd_check_time);
    time_now = get_now_time();
    time_since = time_now - last_upd_check_time;
    ets_printf("Time since last check for update: %d sec\n", time_since);
    if (time_since > UPDATE_PERIOD) {
      ets_printf("Time since check for update > period\n");
      write_UInt(111, time_now);  // save new time of update check in EEPROM
      checkForUpdates(); // check for update and update if required
    }

    rtc_gpio_deinit(SENSORS_KEY);    // to be able to use it again by ulp
    rtc_gpio_init(SENSORS_KEY);      // init GPIO_33 for co-processor
    rtc_gpio_set_direction(SENSORS_KEY, RTC_GPIO_MODE_OUTPUT_ONLY);  // GPIO_33 is output

  }
  else {
    ets_printf("Analog measurements not taken yet (probably first boot)\n");
  }

  ets_printf("Entering deep sleep\n\n");

  detachInterrupt(digitalPinToInterrupt(TURN_PIN));  // detach interrupts from main processor to be able to use pins as wakeup by ULP
  detachInterrupt(digitalPinToInterrupt(RESET_PIN));
  esp_sleep_enable_ext1_wakeup(BUTTON_TURN_RESET_BITMASK, ESP_EXT1_WAKEUP_ANY_HIGH);  // wake up by either reset or turn on/off button

  // start the ULP program
  ESP_ERROR_CHECK( ulp_run((&ulp_entry - RTC_SLOW_MEM) / sizeof(uint32_t)));
  ESP_ERROR_CHECK( esp_sleep_enable_ulp_wakeup() );
  esp_deep_sleep_start();
}

void loop() {
  // not used due to deep sleep wakeup shenanigans
}
