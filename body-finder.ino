#include "Arduino.h"
#include "GPS_Air530.h"
#include <Wire.h>
#include "cubecell_SSD1306Wire.h"

#define MIN_DELAY_BETWEEN_TRANSMISSIONS 20
#define RF_FREQUENCY 868000000   // Hz
#define TX_OUTPUT_POWER 22       // dBm
#define LORA_BANDWIDTH 0         // [0: 125 kHz, \
                                //  1: 250 kHz, \
                                //  2: 500 kHz, \
                                //  3: Reserved]
#define LORA_SPREADING_FACTOR 12 // [SF7..SF12]
#define LORA_CODINGRATE 4        // [1: 4/5, \
                                //  2: 4/6, \
                                //  3: 4/7, \
                                //  4: 4/8]
#define TX_TIMEOUT_VALUE 3000
#define LORA_PREAMBLE_LENGTH 8 // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT 0  // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false

#define RX_TIMEOUT_VALUE 1000

typedef enum
{
  WAIT,
  RX,
  TX
} RadioStates_t;

RadioStates_t radioState;
void OnTxDone(void);
void OnTxTimeout(void);
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);

double buddyLattitudeData, buddyLongitudeData;
uint32_t nextTransmissionTime = 0;

#define BUFFER_SIZE 30 // Define the payload size here

static RadioEvents_t RadioEvents;

#define VBAT_ADC_CTL P3_3
#define ADC1 P2_0
#define ADC ADC1

typedef enum eLoRaMacBatteryLevel
{
  /*!
     * External power source
     */
  BAT_LEVEL_EXT_SRC = 0x00,
  /*!
     * Battery level empty
     */
  BAT_LEVEL_EMPTY = 0x01,
  /*!
     * Battery level full
     */
  BAT_LEVEL_FULL = 0xFE,
  /*!
     * Battery level - no measurement available
     */
  BAT_LEVEL_NO_MEASURE = 0xFF,
} LoRaMacBatteryLevel_t;

uint16_t getBatteryVoltage(void)
{
  pinMode(VBAT_ADC_CTL, OUTPUT);
  digitalWrite(VBAT_ADC_CTL, LOW);
  uint16_t volt = analogRead(ADC) * 2;

  /*
	 * Board, BoardPlus, Capsule, GPS and HalfAA variants
	 * have external 10K VDD pullup resistor
	 * connected to GPIO7 (USER_KEY / VBAT_ADC_CTL) pin
	 */
  pinMode(VBAT_ADC_CTL, INPUT);
  return volt;
}

uint8_t getBatteryLevel()
{
  // 5.5 End-Device Status (DevStatusReq, DevStatusAns)
  // 0      The end-device is connected to an external power source.
  // 1..254 The battery level, 1 being at minimum and 254 being at maximum
  // 255    The end-device was not able to measure the battery level.
  const double maxBattery = 4.212;
  const double minBattery = 3.7;
  const double batVoltage = fmax(minBattery, fmin(maxBattery, getBatteryVoltage() / 1000.0));
  const uint8_t batlevel = BAT_LEVEL_EMPTY + ((batVoltage - minBattery) / (maxBattery - minBattery)) * (BAT_LEVEL_FULL - BAT_LEVEL_EMPTY);
  return batlevel;
}

SSD1306Wire display(0x3c, 500000, I2C_NUM_0, GEOMETRY_128_64, GPIO10); // addr , freq , i2c group , resolution , rst

int fracPart(double val, int n)
{
  return abs((int)((val - (int)(val)) * pow(10, n)));
}

void setup()
{
  delay(10);

  boardInitMcu();
  display.init();
  display.clear();
  display.display();

  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.setFont(ArialMT_Plain_16);
  display.drawString(64, 32 - 16 / 2, "BODY FINDER");
  display.display();

  RadioEvents.TxDone = OnTxDone;
  RadioEvents.TxTimeout = OnTxTimeout;
  RadioEvents.RxDone = OnRxDone;

  Radio.Init(&RadioEvents);
  Radio.SetChannel(RF_FREQUENCY);
  Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                    LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                    LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                    true /*CRC*/, 0 /*frequency hopping*/, 0 /*hop period*/, LORA_IQ_INVERSION_ON, TX_TIMEOUT_VALUE);
  Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                    LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                    LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                    0, true /*CRC*/, 0 /*frequency hopping*/, 0 /*hop period*/, LORA_IQ_INVERSION_ON, true /*continuous*/);

  Serial.begin(115200);
  Air530.begin();
  radioState = WAIT;
}

void render()
{
  char str[30];
  display.clear();
  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_LEFT);

  int index = sprintf(str, "%02d:%02d:%02d", Air530.time.hour(), Air530.time.minute(), Air530.time.second());
  str[index] = 0;
  display.drawString(40, 0, str);

  double batteryPercent = (double)getBatteryLevel() / 254 * 100;
  if (batteryPercent < 100)
  {
    index = sprintf(str, "%02d%%", (int)batteryPercent);
  }
  else
  {
    index = sprintf(str, "%03d%%", (int)batteryPercent);
  }
  str[index] = 0;
  display.drawString(98, 0, str);

  index = sprintf(str, "sats: %02d", (int)Air530.satellites.value());
  str[index] = 0;
  display.drawString(0, 16, str);

  if (radioState == TX)
  {
    index = sprintf(str, "TX");
    str[index] = 0;
    display.drawString(98, 16, str);
  }

  if (radioState == RX)
  {
    index = sprintf(str, "RX");
    str[index] = 0;
    display.drawString(98, 16, str);
  }

  buddyLattitudeData = 47.5305;
  buddyLongitudeData = -122.1427;

  if (buddyLattitudeData != 0)
  {
    index = sprintf(str, "V");
    display.setTextAlignment(TEXT_ALIGN_CENTER);
    display.setFont(ArialMT_Plain_16);
    display.drawString(64, 32 - 16 / 2, str);

    double lattitude = Air530.location.lat();
    double longitude = Air530.location.lng();
    // double TinyGPSPlus::courseTo(double lat1, double long1, double lat2, double long2)
    double distance = TinyGPSPlus::distanceBetween(lattitude, longitude, buddyLattitudeData, buddyLongitudeData);

    display.setFont(ArialMT_Plain_10);
    index = sprintf(str, "%d m", (int)distance);
    str[index] = 0;
    display.drawString(64, 48 - 10 / 2, str);
  }

  display.display();
}

void loop()
{
  Radio.IrqProcess();
  uint32_t starttime = millis();
  while ((millis() - starttime) < 1000)
  {
    while (Air530.available() > 0)
    {
      Air530.encode(Air530.read());
    }
  }
  uint32_t currentTime = Air530.time.value();
  if (Air530.location.isValid() && currentTime > nextTransmissionTime)
  {
    if (radioState != TX)
    {
      radioState = TX;
      uint8_t data[BUFFER_SIZE];
      double lattitude = Air530.location.lat();
      double longitude = Air530.location.lng();
      for (size_t i = 0; i < sizeof lattitude; i++)
      {
        data[i] = ((byte *)&lattitude)[i];
      }
      for (size_t i = 0; i < sizeof longitude; i++)
      {
        data[i + sizeof lattitude] = ((byte *)&longitude)[i];
      }
      nextTransmissionTime = currentTime + (random(20) + MIN_DELAY_BETWEEN_TRANSMISSIONS) * 1000;
      Radio.Send(data, sizeof lattitude + sizeof longitude);
    }
  }
  else
  {
    if (radioState != RX)
    {
      radioState = RX;
      Serial.println("into RX mode");
      Radio.Rx(0);
    }
  }

  render();
}

void OnTxDone(void)
{
  Serial.print("TX done!");
  Radio.Rx(0);
  radioState = RX;
}

void OnTxTimeout(void)
{
  Serial.print("TX Timeout......");
  radioState = WAIT;
}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
  for (size_t i = 0; i < sizeof buddyLattitudeData; i++)
  {
    ((byte *)&buddyLattitudeData)[i] = payload[i];
  }
  for (size_t i = 0; i < sizeof buddyLongitudeData; i++)
  {
    ((byte *)&buddyLongitudeData)[i] = payload[i + sizeof buddyLattitudeData];
  }
  Radio.Sleep();
  radioState = WAIT;
}
