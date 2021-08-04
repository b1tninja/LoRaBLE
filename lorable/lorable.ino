#include "Arduino.h"
#include "heltec.h"
#include "WiFi.h"
#include "logo.h"

#include <Wire.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
     
#define VEXT_PIN 37

#define SERVICE_NAME "LoRaBLE"
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
#define BatteryService BLEUUID((uint16_t)0x180F)

#define BAND    915E6  //you can set band here directly,e.g. 868E6,915E6

BLEServer *pServer = NULL;

BLECharacteristic *pTxCharacteristic;
BLECharacteristic *pRxCharacteristic;
BLECharacteristic BatteryLevelCharacteristic(BLEUUID((uint16_t) 0x2A19),
                                             BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);

BLEDescriptor BatteryLevelDescriptor(BLEUUID((uint16_t) 0x2901));

BLECharacteristic SerialNoCharacteristic(BLEUUID((uint16_t) 0x2A25),
                                         BLECharacteristic::PROPERTY_READ);
//BLEDescriptor SerialNoDescriptor(BLEUUID((uint16_t) 0x2A25));


char chip_id[13] = {0};

bool deviceConnected = false;
bool oldDeviceConnected = false;
uint8_t txValue = 0;
String rssi = "RSSI --";
String packSize = "--";
String packet;
unsigned int counter = 0;
bool receiveflag = false; // software flag for LoRa receiver, received data makes it true.
long lastSendTime = 0;        // last send time
int interval = 1000;          // interval between sends
bool resendflag = false;
bool deepsleepflag = false;

void logo() {
    Heltec.display->clear();
    Heltec.display->drawXbm(0, 5, logo_width, logo_height, (const unsigned char *) logo_bits);
    Heltec.display->display();
}

uint8_t estimateBatteryPercentage() {
    uint8_t level;
    uint16_t mv = analogRead(VEXT_PIN) * 0.00225 * 1000;
    level = ceil((float) (mv - 3000) / 7.5) + 50;
    Serial.printf("Batt mV: %i (%i%%)", mv, level);
    Serial.println();

    return level;
    // TODO: voltage divider
    // https://github.com/G6EJD/LiPo_Battery_Capacity_Estimator/blob/master/ReadBatteryCapacity_LIPO.ino
    float voltage = analogRead(39) / 4096.0 * 7.23;    // NODEMCU ESP32 with 100K+100K voltage divider added
    Serial.println("Voltage = " + String(voltage));
    if (voltage > 4.19) {
        return 100;
    } else if (voltage <= 3.50) {
        return 0;
    } else {
        return 2808.3808 * pow(voltage, 4) - 43560.9157 * pow(voltage, 3) + 252848.5888 * pow(voltage, 2) -
               650767.4615 * voltage + 626532.5703;
    }
}

void interrupt_GPIO0() {
    delay(10);
    if (digitalRead(0) == 0) {
        if (digitalRead(LED) == LOW) { resendflag = true; }
        else {
            deepsleepflag = true;
        }
    }
}

void send() {
    LoRa.beginPacket(false);
    char buffer[9] = {0};
    sprintf(buffer, "%08X", esp_random());
    LoRa.print(buffer);
    LoRa.endPacket();
}

void displaySendReceive() {
    Heltec.display->clear();
    Heltec.display->drawString(0, 50, "Packet " + (String)(counter) + " received");
    Heltec.display->drawString(0, 0, "Size:  " + packSize + " packages:");
    Heltec.display->drawString(0, 10, packet);
    Heltec.display->drawString(0, 20, "With " + rssi + "db");
    Heltec.display->display();
}

void onReceive(int packetSize)//LoRa receiver interrupt service
{
    //if (packetSize == 0) return;
    packet = "";
    packSize = String(packetSize, DEC);
    
    while (LoRa.available()) {
        packet += (char) LoRa.read();
    }
    
    Serial.println(packet);
    rssi = "RSSI: " + String(LoRa.packetRssi(), DEC);
    counter++;
    receiveflag = true;
    

    if (deviceConnected) {
      // SET / NOTIFY Bluetooth
      pTxCharacteristic->setValue(packet.c_str());
      pTxCharacteristic->notify();
    }
}


class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer *pServer) {
        deviceConnected = true;
    };

    void onDisconnect(BLEServer *pServer) {
        deviceConnected = false;
    }
};


class MyCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        std::string rxValue = pCharacteristic->getValue();

        if (rxValue.length() > 0) {
            Serial.println(rxValue.c_str());

            LoRa.beginPacket();
            LoRa.print(rxValue.c_str());
            LoRa.endPacket();
        }
    }
};

void setupLoRa() {
    attachInterrupt(0, interrupt_GPIO0, FALLING);

    LoRa.setSyncWord(0x23);

    LoRa.onReceive(onReceive);
    send();
    LoRa.receive();
}

void setupBLE() {

    // Create the BLE Device
    BLEDevice::init(SERVICE_NAME);

    // Create the BLE Server
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    // Create the BLE Service
    BLEService *pService = pServer->createService(SERVICE_UUID);

    // Create a BLE Characteristic
    pTxCharacteristic = pService->createCharacteristic(
            CHARACTERISTIC_UUID_TX,
            BLECharacteristic::PROPERTY_NOTIFY
    );

    pTxCharacteristic->addDescriptor(new BLE2902());

    pRxCharacteristic = pService->createCharacteristic(
            CHARACTERISTIC_UUID_RX,
            BLECharacteristic::PROPERTY_WRITE
    );

    pRxCharacteristic->setCallbacks(new MyCallbacks());

    // TODO: additional characterists
    // band
    // sync word
    // spread factor
    // PABOOST

    BLEService *pBattery = pServer->createService(BatteryService);

    pBattery->addCharacteristic(&BatteryLevelCharacteristic);
    BatteryLevelDescriptor.setValue("Percentage 0 - 100");
    BatteryLevelCharacteristic.addDescriptor(&BatteryLevelDescriptor);
    BatteryLevelCharacteristic.addDescriptor(new BLE2902());


    pServer->getAdvertising()->addServiceUUID(BatteryService);

    pBattery->start();
    // Start advertising
    pServer->getAdvertising()->start();



    // Start the service
    pService->start();

    // Start advertising
    pServer->getAdvertising()->start();


}

void setupADC()
{
    adcAttachPin(13);
    analogSetClockDiv(255); // 1338mS
}

void setupSerial()
{
    Serial.begin(115200);
}

void setChipID()
{
    uint64_t chipid;
    chipid = ESP.getEfuseMac();//The chip ID is essentially its MAC address(length: 6 bytes).
    sprintf(chip_id, "%04X%08X", (uint16_t)(chipid >> 32), (uint32_t) chipid);
    Serial.printf("ESP32 chip ID: %s", chip_id);
    Serial.println();

    // TODO:
    // pService->addCharacteristic(&SerialNoCharacteristic);
    // SerialNoCharacteristic->setValue(chip_id);
}

void setup() {
    Heltec.begin(true, true, true, false, BAND);
    logo();
    setupSerial();
    setChipID();
    setupADC();
    setupBLE();
    setupLoRa();
}


void loop() {
    /*
    uint8_t percentage = estimateBatteryPercentage();
    BatteryLevelCharacteristic.setValue(&percentage, 1);
    BatteryLevelCharacteristic.notify();
    //*/

    if (deepsleepflag) {
        LoRa.end();
        LoRa.sleep();
        delay(100);
        pinMode(4, INPUT);
        pinMode(5, INPUT);
        pinMode(14, INPUT);
        pinMode(15, INPUT);
        pinMode(16, INPUT);
        pinMode(17, INPUT);
        pinMode(18, INPUT);
        pinMode(19, INPUT);
        pinMode(26, INPUT);
        pinMode(27, INPUT);
        digitalWrite(Vext, HIGH);
        delay(2);
        esp_deep_sleep_start();
    }

    if (resendflag) {
        resendflag = false;
        send();
        LoRa.receive();
        displaySendReceive();
    }
    
    if (receiveflag) {
        digitalWrite(25, HIGH);
        displaySendReceive();
        delay(1000);
        receiveflag = false;
        send();
        LoRa.receive();
        // TODO: relay over bluetooth
        displaySendReceive();
    }

    // disconnecting
    if (!deviceConnected && oldDeviceConnected) {
        Serial.println("Disconnecting...");
        delay(500); // give the bluetooth stack the chance to get things ready
        pServer->startAdvertising(); // restart advertising
        Serial.println("start advertising");
        oldDeviceConnected = deviceConnected;
    }

    // connecting
    if (deviceConnected && !oldDeviceConnected) {
        Serial.println("Connecting...");
        // do stuff here on connecting
        oldDeviceConnected = deviceConnected;
    }
}
