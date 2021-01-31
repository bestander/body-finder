//the GPS module used is Air530.
#include "Arduino.h"
#include "GPS_Air530.h"
#include <Wire.h>
#include "cubecell_SSD1306Wire.h"

#define RF_FREQUENCY 868000000 // Hz

#define TX_OUTPUT_POWER 20 // dBm
#define PARTICPANT 0       // 0 - 5

#define LORA_BANDWIDTH 0        // [0: 125 kHz, \
                                //  1: 250 kHz, \
                                //  2: 500 kHz, \
                                //  3: Reserved]
#define LORA_SPREADING_FACTOR 8 // [SF7..SF12]
#define LORA_CODINGRATE 4       // [1: 4/5, \
                                //  2: 4/6, \
                                //  3: 4/7, \
                                //  4: 4/8]
#define LORA_PREAMBLE_LENGTH 8  // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT 0   // Symbols
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
int16_t txNumber = 0;

#define BUFFER_SIZE 30 // Define the payload size here

static RadioEvents_t RadioEvents;

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
  display.drawString(64, 32 - 16 / 2, "GPS starting...");
  display.display();

  RadioEvents.TxDone = OnTxDone;
  RadioEvents.TxTimeout = OnTxTimeout;
  RadioEvents.RxDone = OnRxDone;

  Radio.Init(&RadioEvents);
  Radio.SetChannel(RF_FREQUENCY);
  Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                    LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                    LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                    true, 0, 0, LORA_IQ_INVERSION_ON, 3000);
  Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                    LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                    LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                    0, true, 0, 0, LORA_IQ_INVERSION_ON, true);

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

  int index = sprintf(str, "Me: sats: %d", (int)Air530.satellites.value());
  str[index] = 0;
  display.drawString(0, 0, str);

  index = sprintf(str, "%02d:%02d:%02d", Air530.time.hour(), Air530.time.minute(), Air530.time.second());
  str[index] = 0;
  display.drawString(65, 0, str);

  index = sprintf(str, "lat:%d.%d lon:%d.%d",
                  (int)Air530.location.lat(),
                  fracPart(Air530.location.lat(), 4),
                  (int)Air530.location.lng(),
                  fracPart(Air530.location.lng(), 4));
  str[index] = 0;
  display.drawString(0, 16, str);

  if (buddyLattitudeData != 0)
  {
    index = sprintf(str, "Buddy: distance: %dm", 0);
    str[index] = 0;
    display.drawString(0, 32, str);

    index = sprintf(str, "lat:%d.%d lon:%d.%d",
                    (int)buddyLattitudeData,
                    fracPart(buddyLattitudeData, 4),
                    0,
                    0);
    display.drawString(0, 48, str);
  }
  if (radioState == TX)
  {
    index = sprintf(str, "TX");
    str[index] = 0;
    display.drawString(112, 0, str);
  }
  if (radioState == RX)
  {
    index = sprintf(str, "RX");
    str[index] = 0;
    display.drawString(112, 0, str);
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
  uint8_t minTxSeconds = PARTICPANT * 10;
  uint8_t maxTxSeconds = minTxSeconds + 10;
  uint8_t seconds = Air530.time.second();
  if (seconds > minTxSeconds && seconds < maxTxSeconds)
  {
    if (radioState != TX)
    {
      radioState = TX;
      txNumber++;
      // TODO serialize Air530.location.rawLat() and Air530.location.rawLon() into data[]
      uint8_t data[4];
      data[0] = 0;
      data[1] = 1;
      data[2] = 2;
      data[3] = 3;
      Radio.Send((uint8_t *)data, sizeof(data));
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
  // TODO parse payload
  //  buddyLattitudeData = rawLatData.deg + rawLatData.billionths / 1000000000.0;
  //  buddyLongitudeData = rawLonData.deg + rawLonData.billionths / 1000000000.0;
  char rxpacket[BUFFER_SIZE];
  memcpy(rxpacket, payload, size);
  rxpacket[size] = '\0';
  Radio.Sleep();
  Serial.printf("\r\nreceived packet \"%s\" with rssi %d , length %d\r\n", rxpacket, rssi, size);
  radioState = WAIT;
}
