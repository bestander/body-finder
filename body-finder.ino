#include "Arduino.h"
#include "GPS_Air530.h"
#include <Wire.h>
#include "cubecell_SSD1306Wire.h"
#include <QMC5883LCompass.h>

// TODO
// - support multiple buddies
// - display time last seen
// - make nice transitions
// - add calibration instructions

/***
 * Radio section START 
 */
typedef struct
{
  uint64_t id;
  char *name;
} UsersDictionary_t;

const UsersDictionary_t KNOWN_USERS[]{
    {0xA6A70E30, "USER1"},
    {0xA6A70F27, "USER2"}};

#define MIN_DELAY_BETWEEN_TRANSMISSIONS_SEC 10
#define RF_FREQUENCY 915000000   // Hz
#define TX_OUTPUT_POWER 22       // dBm
#define LORA_BANDWIDTH 0         // [0: 125 kHz, \
                                //  1: 250 kHz, \
                                //  2: 500 kHz, \
                                //  3: Reserved]
#define LORA_SPREADING_FACTOR 10 // [SF7..SF12]
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

typedef struct
{
  uint64_t id;
  double longitude;
  double lattitude;
  uint32_t lastSeen;
} BuddiesDictionary_t;
BuddiesDictionary_t buddies[5];

typedef enum
{
  WAIT,
  RX,
  TX
} RadioStates_t;
RadioStates_t radioState;
uint32_t nextTransmissionTime = 0;
uint32_t transmissionEndTime = 0;

void OnRxTimeout(void)
{
  Serial.println("OnRxTimeout");
}

void OnRxError(void)
{
  Serial.println("OnRxError");
}

void OnTxDone(void)
{
  Serial.println("OnTxDone");
  transmissionEndTime = millis();
  radioState = WAIT;
}

void OnTxTimeout(void)
{
  Serial.println("OnTxTimeout");
  radioState = WAIT;
}

typedef struct
{
  double lattitude;
  double longitude;
  uint64_t id;
} PositionMessage_t;

typedef union
{
  PositionMessage_t structure;
  uint8_t byteArray[sizeof(PositionMessage_t)];
} TransmitPositionMessage_t;

void printByteArrayAsHex(uint8_t *payload, uint16_t size)
{
  for (int i = 0; i < size; i++)
  {
    Serial.printf("%02X", payload[i]);
  }
  Serial.println("");
}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
  Serial.print("OnRxDone: ");
  printByteArrayAsHex(payload, size);

  TransmitPositionMessage_t message;
  memcpy(message.byteArray, payload, size);
  buddies[0].id = message.structure.id;
  buddies[0].lattitude = message.structure.lattitude;
  buddies[0].longitude = message.structure.longitude;
  radioState = WAIT;
}

void transmitReceivePosition()
{
  uint32_t currentTime = millis();
  if (radioState != TX && currentTime > nextTransmissionTime)
  {
    radioState = TX;
    TransmitPositionMessage_t message;
    message.structure.lattitude = Air530.location.lat();
    message.structure.longitude = Air530.location.lng();
    message.structure.id = getID();
    nextTransmissionTime = currentTime + (random(MIN_DELAY_BETWEEN_TRANSMISSIONS_SEC) + MIN_DELAY_BETWEEN_TRANSMISSIONS_SEC) * 1000;
    transmissionEndTime = currentTime + TX_TIMEOUT_VALUE;
    byte len = sizeof(PositionMessage_t);
    uint8_t rxpacket[len];
    memcpy(rxpacket, message.byteArray, len);
    Serial.print("Transmitting: ");
    printByteArrayAsHex(rxpacket, len);
    Radio.Send(rxpacket, len);
  }
  if (radioState != RX && currentTime > transmissionEndTime)
  {
    radioState = RX;
    Radio.Rx(0);
  }
}

/***
 * COMPASS SECTION START 
 */
#define DISPLAY_I2C_ADDRESS 0x0C
#define COMPASS_I2C_ADDRESS 0x0D
QMC5883LCompass compass;
int16_t myAzimuth;

void readCompass()
{
  // stop display
  Wire.endTransmission();
  Wire.beginTransmission(COMPASS_I2C_ADDRESS);
  compass.read();
  myAzimuth = compass.getAzimuth();
  Wire.endTransmission();
  Wire.beginTransmission(DISPLAY_I2C_ADDRESS);
}

/***
 * BATTERY SECTION START 
 */
#define VBAT_ADC_CTL P3_3
#define ADC P2_0
typedef enum eLoRaMacBatteryLevel
{
  /***
   * External power source
   */
  BAT_LEVEL_EXT_SRC = 0x00,
  /**
   * Battery level empty
   */
  BAT_LEVEL_EMPTY = 0x01,
  /***
    * Battery level full
    */
  BAT_LEVEL_FULL = 0xFE,
  /***
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

/***
 * DISPLAY SECTION START 
 */
SSD1306Wire display(0x3c, 500000, I2C_NUM_0, GEOMETRY_128_64, GPIO10); // addr , freq , i2c group , resolution , rst

void drawString(int16_t x, int16_t y, String str)
{
  display.drawString(x, y, str);
  Serial.println(str);
}

int fracPart(double val, int n)
{
  return abs((int)((val - (int)(val)) * pow(10, n)));
  return abs((int)((val - (int)(val)) * pow(10, n)));
}

struct Button
{
  const uint8_t PIN;
  unsigned long lastFire = 0;
  uint32_t numberKeyPresses;
};

Button button = {GPIO7, 0, false};

void onButtonPress()
{
  if ((millis() - button.lastFire) > 200)
  {
    button.lastFire = millis();
    button.numberKeyPresses += 1;
  }
}

void renderDirection(double courseToBuddy)
{
  int azimuthOfBuddy = myAzimuth + (int)courseToBuddy;
  double radians = azimuthOfBuddy * 1000 / 57296;
  double bearingX = -sin(radians) * 40 + 65;
  double bearingY = -cos(radians) * 20 + 40;
  display.drawCircle((int)bearingX, (int)bearingY, 3);
}

void render()
{
  char str[30];
  display.clear();
  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_LEFT);

  int index = sprintf(str, "sats: %02d", (int)Air530.satellites.value());
  str[index] = 0;
  drawString(0, 0, str);

  index = sprintf(str, radioState == TX ? "TX" : "RX");
  str[index] = 0;
  drawString(60, 0, str);

  double batteryPercent = (double)getBatteryLevel() / 254 * 100;
  index = sprintf(str, "%d%%", (int)batteryPercent);
  str[index] = 0;
  drawString(98, 0, str);

  if (button.numberKeyPresses % 2 == 1)
  {
    index = sprintf(str, "%02d:%02d:%02d", Air530.time.hour(), Air530.time.minute(), Air530.time.second());
    str[index] = 0;
    drawString(30, 16, str);
    double lattitude = Air530.location.lat();
    double longitude = Air530.location.lng();
    index = sprintf(str, "lat:%d.%d lon:%d.%d",
                    (int)lattitude,
                    fracPart(lattitude, 4),
                    (int)longitude,
                    fracPart(longitude, 4));
    drawString(0, 32, str);
  }
  else if (button.numberKeyPresses % 2 == 0)
  {
    double buddyLattitudeData = buddies[0].lattitude;
    double buddyLongitudeData = buddies[0].longitude;
    uint64_t buddyId = buddies[0].id;
    if (buddyLattitudeData != 0)
    {
      char *name;
      bool found = false;
      for (uint8_t i = 0; i < sizeof(KNOWN_USERS) / sizeof(UsersDictionary_t); ++i)
      {
        int diff = KNOWN_USERS[i].id - buddyId;
        if (diff == 0)
        {
          name = KNOWN_USERS[i].name;
          found = true;
          break;
        }
      }
      if (!found)
      {
        char id[8];
        sprintf(id, "%X", buddyId);
        name = id;
      }

      index = sprintf(str, "%s", name);
      display.setTextAlignment(TEXT_ALIGN_CENTER);
      display.setFont(ArialMT_Plain_16);
      drawString(64, 24, str);

      double lattitude = Air530.location.lat();
      double longitude = Air530.location.lng();
      double distance = TinyGPSPlus::distanceBetween(lattitude, longitude, buddyLattitudeData, buddyLongitudeData);

      display.setFont(ArialMT_Plain_10);
      index = sprintf(str, "%d m", (int)distance);
      str[index] = 0;
      drawString(64, 42, str);

      double courseToBuddy = TinyGPSPlus::courseTo(lattitude, longitude, buddyLattitudeData, buddyLongitudeData);
      renderDirection(courseToBuddy);
    }
  }

  display.display();
}

/***
 * SETUP AND LOOP
 */
static RadioEvents_t RadioEvents;
void setup()
{
  Serial.begin(115200);
  delay(10);
  boardInitMcu();
  display.init();
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.setFont(ArialMT_Plain_16);
  drawString(64, 32 - 16 / 2, "BODY FINDER");
  display.display();

  RadioEvents.TxDone = OnTxDone;
  RadioEvents.TxTimeout = OnTxTimeout;
  RadioEvents.RxDone = OnRxDone;
  RadioEvents.RxTimeout = OnRxTimeout;
  RadioEvents.RxError = OnRxError;
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
  radioState = WAIT;
  Air530.begin();
  compass.init();
  compass.setCalibration(-2057, 1285, -2148, 1220, -1757, 1462);
  pinMode(button.PIN, INPUT_PULLUP);
  attachInterrupt(button.PIN, onButtonPress, FALLING);
}

void loop()
{
  Radio.IrqProcess();
  readCompass();
  uint32_t starttime = millis();
  while ((millis() - starttime) < 1000)
  {
    while (Air530.available() > 0)
    {
      Air530.encode(Air530.read());
    }
  }
  transmitReceivePosition();
  render();
}
