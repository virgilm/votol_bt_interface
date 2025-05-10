/*
  Votol Tuner Hardware Module
  (c) 2025 Metahack

  Licensed under Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International (CC BY-NC-SA 4.0).

  You are free to share and adapt this sketch for non-commercial purposes,
  as long as you credit the author and license your new creations under the same terms.
  Full license details: https://creativecommons.org/licenses/by-nc-sa/4.0/

  Support the project: https://www.buymeacoffee.com/metahack
*/

// Overall, adding delays in the send/receive tight loops CAN make the devioce drop packets SILENTLY!
// VERY TRICKY TO DEBUG!
// For any debugging, use the highest possible baud rate for the serial (debug) port

#include <CAN.h>
// Multiple versions available, Originally used https://github.com/sandeepmistry/arduino-CAN
// Use https://github.com/avlasic/arduino-CAN/tree/patch-1 (fork from https://github.com/sandeepmistry/arduino-CAN) due to a bug (as of Sept 2024)

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include "esp_wifi.h"
#include <esp_task_wdt.h>
#define WRITE_MSG_LENGTH 136
typedef struct {
    uint8_t data[WRITE_MSG_LENGTH];  // Max message size
    size_t length;
} BTMessage;
QueueHandle_t xQueue;
// Pay EXTREMELY close attention to the size of the sketch, enabling other modules/libraries might increate the memory requirements above the ESP32 available memory!
// Currently, Sketch uses 1214449 bytes (92%) of program storage space. Maximum is 1310720 bytes.

//Watchdog timeout
#define WDT_TIMEOUT 10

// #define FAKE // uncomment this to generate fake messages for app debugging
// #define DEBUG // uncomment this to print debug messages
// Enabling DEBUG makes things unstable/WD crashes/lost messages, use SPARINGLY!
// Absoultely no shipping code with DEBUG enabled!
#define LED_ON

// normal ESP32
const int LED_BUILTIN = 2; // BLUE LED
// LOLIN32 - NOT WORKING YET
// const int LED_BUILTIN = 5;

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID        "CAFE"
#define CHARACTERISTIC_UUID "BABE"
#define MAX_BT_SIZE 20

#define FROM_CONTROLLER_CAN_ID 0x3fe
#define TO_CONTROLLER_CAN_ID   0x3ff
#define SPEEDO_MSG_ID 0x1026105A

#define SPEEDO_MULTI 4 // send every 4 messages, each 1 sec
int speedo_count = 0;
int id;

#define MAX_DATA_SIZE 8
byte cdata[MAX_DATA_SIZE] = {0};

BLEServer *pServer;
BLEService *pService;
BLECharacteristic *pCharacteristic;

#define HDR_SIZE 4
unsigned char speedo_hdr[HDR_SIZE] = {0x0E, 0x55, 0xAA, 0xAA};

unsigned char prbuf[WRITE_MSG_LENGTH * 4]; // extra padding, WRITE_MSG_LENGTH *3 should be enough

bool deviceConnected = false;

bool can_received = false;
int can_packet_size = 0;

int last_core0 = millis();

// Initialization portion
int last = millis();
int state = LOW;
int min_voltage = 690; // 69.0 V
int max_voltage = 850; // 85.5 V
int delta_v = 1; //0.1V
int odometer = 0;
bool sent = false;

#define DISPLAY_MSG_LENGTH 24
#define SPEEDO_MSG_LENGTH 9

// Fake data, to be used when FAKE is defined
unsigned char display_data[DISPLAY_MSG_LENGTH] = {0x09, 0x55, 0xAA, 0xAA, 0x00, 0x00, 0x00, 0x02,
                                  0xE2, 0xFF, 0xF9, 0x00, 0x00, 0x00, 0x00, 0x00,
                                  0x01, 0x00, 0x46, 0x5D, 0x00, 0x00, 0xC9, 0x00};
unsigned char speedo_data[SPEEDO_MSG_LENGTH] = {0x0E, 0x55, 0xAA, 0xAA, 0x00, 0x01, 0x00, 0x00, 0x00};
unsigned char hello_data[WRITE_MSG_LENGTH] = {0x07, 0x55, 0xAA, 0xAA, 0x00, 0xAA, 0x00, 0x00, 
                                0x37, 0x5A, 0x00, 0x00, 0x2E, 0x02, 0x5C, 0x03, 
                                0x3A, 0x02, 0x0A, 0x78, 0x00, 0x60, 0x22, 0x26,
                                0x02, 0x00, 0x00, 0x00, 0x00, 0xDC, 0x00, 0x50,
                                0x4b, 0x64, 0xb8, 0x0b, 0xe0, 0x2e, 0x70, 0x17,
                                0x02, 0x25, 0x08, 0x78, 0x1E, 0x0A, 0x78, 0x00, 
                                0x8C, 0xaf, 0xb4, 0x69, 0x00, 0x54, 0x24, 0xd4,
                                0x24, 0x0A, 0x30, 0x10, 0x1E, 0x00, 0x00, 0x04, 
                                0xcc, 0x0c, 0xe8, 0x03, 0xb0, 0x04, 0x51, 0x44,
                                0x14, 0x2c, 0x01, 0x88, 0x13, 0x28, 0x00, 0x0a,
                                0x00, 0xc0, 0x00, 0xc0, 0x80, 0xc8, 0x0b, 0xc0,
                                0x25, 0xC0, 0x11, 0xC3, 0x00, 0xC0, 0x00, 0x58, 
                                0x00, 0x00, 0xc0, 0x0f, 0xc0, 0x8a, 0xc0, 0x09,
                                0xc2, 0x10, 0xc1, 0x07, 0xc2, 0x00, 0xc0, 0x92,
                                0xcb, 0x00, 0x3a, 0xc7, 0x0f, 0xdf, 0x52, 0xff,
                                0x00, 0x00, 0x00, 0x00, 0x51, 0x00, 0xd2, 0x05,
                                0x00, 0x00, 0x00, 0xc3, 0xcd, 0x00, 0x00, 0x00};

// used only for FAKE messages
void IRAM_ATTR sendLongBTMessage(unsigned char* body, int length) {
  while (length > 0) {
    if (length > MAX_BT_SIZE) {
      pCharacteristic->setValue(body, MAX_BT_SIZE);
      pCharacteristic->notify();
      length -= MAX_BT_SIZE;
      body += MAX_BT_SIZE;
    } else {
      pCharacteristic->setValue(body, length);
      pCharacteristic->notify();
      length = 0;
    }
  }
}

// debug function, DO NOT USE under normal circumstances!
void printBuffer8(unsigned char *print_buffer, int length) {
    int k, m = 0, j = 0;
    // print the buffer for debugging
    while (length > 0) {
      for (k = 0; k < 8; k++) {
          m += sprintf((char *)prbuf + m, "%02hhX ", print_buffer[k+j*8]);
      }
      Serial.write((char *) prbuf, 8*3);
      Serial.println("");
      m = 0;
      j++;
      length = length - 8;
    }
}

// send to CAN bus
void sendCanMessage(unsigned char* body, int length) {
  int sent_bytes = 0;
  while (length > 0) {
    if (length > MAX_DATA_SIZE) {
      CAN.beginPacket(TO_CONTROLLER_CAN_ID);
      sent_bytes = CAN.write(body, MAX_DATA_SIZE);
      CAN.endPacket();
      length -= MAX_DATA_SIZE;
      body += MAX_DATA_SIZE;
    } else {
      CAN.beginPacket(TO_CONTROLLER_CAN_ID);
      sent_bytes = CAN.write(body, length);
      CAN.endPacket();
      length = 0;
    }
    #ifdef DEBUG
        ESP_DRAM_LOGE("sendCanMessage", " %d", sent_bytes);
    //        printBuffer8((unsigned char *)(msg.c_str()), msg.length());
    #endif
  }
}

// task
void sendDataToCAN(void* arg) {
  esp_task_wdt_add(NULL);
  BTMessage msg;
  for (;;) {
    unsigned long now = millis();
    // Keep the watchdog happy
    if (now - last_core0 >= 500) {
      esp_task_wdt_reset();
#ifdef DEBUG
      ESP_DRAM_LOGE("WD0", "Reset WD0 %ld", now);
#endif
      last_core0 = now;
    }

    // Wait for next BLE -> CAN message
    if (xQueueReceive(xQueue, &msg, pdMS_TO_TICKS(10) ) == pdPASS) {
      #ifdef DEBUG
        ESP_DRAM_LOGE("BT Task", "<- %d", (int)msg.length);
        //        printBuffer8((unsigned char *)(msg.data), msg.length);
      #endif

      sendCanMessage(msg.data, msg.length);
    } else {
        vTaskDelay(1); // prevent starving IDLE tasks and WDT
    }
  }
}

class BTCallback: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) override {
    std::string incoming = pCharacteristic->getValue();
    if (incoming.length() == 0) return; // no data

    // Prepare a queue message
    BTMessage msg;
    size_t copyLength = min(incoming.length(), (size_t)WRITE_MSG_LENGTH); // prevent overflow
    memcpy(msg.data, incoming.c_str(), copyLength);
    msg.length = copyLength;

#ifndef FAKE
    // Send to queue (blocks if queue is full — you can also use 0 timeout if you want non-blocking)
    if (xQueueSend(xQueue, &msg, portMAX_DELAY) != pdTRUE) {
      #ifdef DEBUG
      Serial.println("Queue full — BLE message dropped!");
      #endif
    }

    #ifdef DEBUG
    Serial.printf("Enqueued BLE msg length %d\n", (int)msg.length);
    #endif
#endif

#ifdef FAKE // check this out!
    if (incoming[0] == 0x07) {
      // this is a write
      sendLongBTMessage(hello_data, WRITE_MSG_LENGTH);
    } else if (incoming[0] == 0x0A) {
      // this is a read
      sendLongBTMessage(hello_data, WRITE_MSG_LENGTH);
    } else if (incoming[0] == 0x09) {
      // send fake display response back
      if ((min_voltage + delta_v) > max_voltage) {
        delta_v = 0;
      } else {
        delta_v = delta_v + 1;
      }
      display_data[7] = highByte(min_voltage + delta_v);
      display_data[8] = lowByte(min_voltage + delta_v);
      display_data[9] = 10;
      display_data[10] = random(128);
      display_data[17] = random(255);
      display_data[18] = random(175) + 50;
      display_data[19] = random(175) + 50;
      sendLongBTMessage(display_data, DISPLAY_MSG_LENGTH);
    }
#endif
  }
};

void IRAM_ATTR CANCallback(int packetSize) {
  // this gets called when we get CAN data
  can_received = true;
  can_packet_size = packetSize;
}

// main execution loop
void loop() {
  unsigned long now = millis();
  int result = 0;
  if (now - last >= 500) {
#ifdef LED_ON
      if (deviceConnected) {
        digitalWrite(LED_BUILTIN, state);
        if (state == HIGH) {
          state = LOW;
        } else {
          state = HIGH;
        }
      }
#endif
      result = esp_task_wdt_reset();
      last = now;
#ifdef DEBUG
      ESP_DRAM_LOGE("WD", "Reset WD %ld, result %d", now, result);
#endif
#ifdef FAKE
      if (deviceConnected) {
        speedo_data[7] = odometer;
        speedo_data[4] = odometer/10;
        memcpy(prbuf, speedo_data, SPEEDO_MSG_LENGTH);
        pCharacteristic->setValue(prbuf, SPEEDO_MSG_LENGTH);
        pCharacteristic->notify();
        odometer += 1;
      }
#endif
  } // end of LED blinking and periodic WD reset

  if (can_received) {
    if (can_packet_size != 0) {
      id = CAN.packetId();
      if (id == SPEEDO_MSG_ID) {
        speedo_count += 1;
        if (speedo_count == SPEEDO_MULTI) {
          // send one copy every SPEEDO_MULTI intervals
          // attach the header
          CAN.readBytes(cdata, can_packet_size);
#ifdef DEBUG
          ESP_DRAM_LOGE("CANCallback", " S-> %d", can_packet_size);
#endif
          memcpy(prbuf, speedo_hdr, HDR_SIZE);
          memcpy(prbuf + HDR_SIZE, cdata, can_packet_size);
          pCharacteristic->setValue(prbuf, HDR_SIZE + can_packet_size);
          pCharacteristic->notify();
          speedo_count = 0;
        }
      } else if (id == FROM_CONTROLLER_CAN_ID) {
        CAN.readBytes(cdata, can_packet_size);
#ifdef DEBUG
        ESP_DRAM_LOGE("CANCallback", " C-> %d", can_packet_size);
#endif
        // printBuffer8(cdata, packetSize);
        pCharacteristic->setValue(cdata, can_packet_size);
        pCharacteristic->notify();
      } else if (id == TO_CONTROLLER_CAN_ID) {
#ifdef DEBUG
          ESP_DRAM_LOGE("CANCallback", " C <- %d", can_packet_size);
#endif
    //    Serial.println("Only in debug mode");
    //    CAN.readBytes(cdata, packetSize);
    //    printBuffer8(cdata, packetSize);
      } else {
    //    Serial.print(id);
    //    Serial.println(" Should never happen!");
      }
      // xTaskCreate(sendDataToBT, "SendDataToBT", 4096, static_cast<void*>(&packetSize), tskIDLE_PRIORITY, NULL);
      can_received = false;
    }
  }
}

// Main BLE Server Callbacks
class BLEServerCB : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer, esp_ble_gatts_cb_param_t *param) override {
    // keep advertising even after connection - helps with reconnecting!
    pServer->startAdvertising();
    deviceConnected = true;
  }

  void onDisconnect(BLEServer* pServer) {
#ifdef DEBUG
    ESP_DRAM_LOGE("BT", "Disconnect");
#endif
    pServer->startAdvertising(); // restart advertising
    deviceConnected = false;
  }
} bleServerCB;

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(250000); // should match the serial port monitor to enable debugging/boot messages
  Serial.setTimeout(100);
  while(!Serial);

  delay(500);

  esp_task_wdt_init(WDT_TIMEOUT, true); //enable panic so ESP32 restarts
  esp_task_wdt_add(NULL); //add current thread to WDT watch

  Serial.println(F("Serial init ok!"));

  esp_wifi_set_mode(WIFI_MODE_NULL);

  Serial.println(F("WiFi disabled!"));

  xQueue = xQueueCreate(20, sizeof(BTMessage));
  if(xQueue != NULL) {
    xTaskCreatePinnedToCore(
      sendDataToCAN, // callback
      "SendDataToCAN", // description
      4096, // stack size
      NULL, // params
      tskIDLE_PRIORITY | portPRIVILEGE_BIT, // priority
      NULL,
      0 // core 0
      );
  } else {
    Serial.println(F("Waiting for BT Queue Init ..."));
    while (1);
  }

// Normal ESP32 board
   CAN.setPins(GPIO_NUM_25, GPIO_NUM_26);
// LOLIN32 board? - NOT WORKING YET
//  CAN.setPins(GPIO_NUM_22, GPIO_NUM_21);

  if (!CAN.begin(500E3)) {
    Serial.println("Starting CAN failed!");
    while (1);
  }

// CAN filtering does not work as expected. Ideally, we should only filter for 3 messages (FROM_CONTROLLER_CAN_ID, TO_CONTROLLER_CAN_ID & SPEEDO_MSG_ID)
// in practice that does not seem to work properly!
// as a workaoround, we filter out via filters as much as possible, and then we do further filtering in software.
// properly working filters would be a HUGE improvment.

//  CAN.filter(FROM_CONTROLLER_CAN_ID, 0x3FF); // filter messages *from* controller
//  CAN.filter(TO_CONTROLLER_CAN_ID);   // filter messages *to* controller
// adding this line breaks everything!!
//  CAN.filterExtended(SPEEDO_MSG_ID, 0x1FFFFFFF);    // filter display messages

  CAN.filter(0x8, 0x8);
  // this is a manual OR between the SPPEDO & FROM_CONTROLLER_CAN_ID
  // might accept other packets too, as it only tests 1 bit
  // still better than nothing
  // https://esp32.com/viewtopic.php?t=16575
  // https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/twai.html#overview
  // http://media3.evtv.me/ESP32CANDue.pdf

  CAN.onReceive(CANCallback);

  Serial.println(F("CAN init ok!"));

  BLEDevice::init("MetaHack");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(&bleServerCB);

  pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_NOTIFY |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );

  // create CCC descriptor and add it to the characteristic
  BLEDescriptor *pClientCharacteristicConfigDescriptor = new BLEDescriptor((uint16_t)0x2902);
  pCharacteristic->addDescriptor(pClientCharacteristicConfigDescriptor);
  pCharacteristic->setValue("MetaHack!");
  pCharacteristic->setCallbacks(new BTCallback());

  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  // functions that help with Android connections issue
  pAdvertising->setMinPreferred(0x6);
  pAdvertising->setMaxPreferred(0x20);
  BLEDevice::startAdvertising();

  Serial.println(F("BT init ok!"));

  digitalWrite(LED_BUILTIN, LOW);

  // esp_log_level_set("*", ESP_LOG_ERROR);
  // set all components to ERROR level
}
