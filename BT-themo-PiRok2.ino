#include <Arduino.h>
#include <NimBLEDevice.h>

// ============================================================
// Configuration
// ============================================================

// Beanconqueror teste le nom en uppercase et cherche :
// "THERMAQBLUE" ou "THERMAQ BLUE"
#define DEVICE_NAME "THERMAQ BLUE"

// Mets 1 pour debug série, 0 pour silence
#define DEBUG_SERIAL2 0

#define TEMPERATURE_NOTIFY_INTERVAL_MS 1000

#define MIN_TEMP_C -50.0f
#define MAX_TEMP_C 250.0f

// ============================================================
// UUIDs ETI Ltd / ThermaQ Blue utilisés par Beanconqueror
// ============================================================

#define ETI_SERVICE_UUID                 "45544942-4c55-4554-4845-524db87ad700"
#define ETI_CHANNEL_1_TEMP_CHAR_UUID     "45544942-4c55-4554-4845-524db87ad701"
#define ETI_CHANNEL_2_TEMP_CHAR_UUID     "45544942-4c55-4554-4845-524db87ad703"

#define ETI_CHANNEL_1_CONFIG_CHAR_UUID   "45544942-4c55-4554-4845-524db87ad707"
#define ETI_CHANNEL_2_CONFIG_CHAR_UUID   "45544942-4c55-4554-4845-524db87ad708"
#define ETI_DEVICE_CONFIG_CHAR_UUID      "45544942-4c55-4554-4845-524db87ad709"
#define ETI_TRIM_CHAR_UUID               "45544942-4c55-4554-4845-524db87ad70a"

// Batterie standard
#define BATTERY_SERVICE_UUID             "180F"
#define BATTERY_CHAR_UUID                "2A19"

// ============================================================
// BLE reconnection / anti-ghost connection patch n°2
// ============================================================

NimBLEServer* bleServer = nullptr;

bool bleClientConnected = false;
bool restartAdvertisingRequested = false;

unsigned long restartAdvertisingAtMs = 0;
unsigned long lastAdvertisingKickMs = 0;

#define NO_CONN_HANDLE 0xFFFF
uint16_t activeConnHandle = NO_CONN_HANDLE;

// Relance régulière de l'advertising.
// 2000 ms = toutes les 2 secondes.
#define ADVERTISING_KICK_INTERVAL_MS 2000

// Tenter de continuer à advertiser même connecté.
// Utile si une ancienne connexion reste fantôme.
#define ALWAYS_ADVERTISE_WHILE_CONNECTED 1

// ============================================================
// Globals
// ============================================================

NimBLECharacteristic* tempChannel1Characteristic = nullptr;
NimBLECharacteristic* tempChannel2Characteristic = nullptr;
NimBLECharacteristic* batteryCharacteristic = nullptr;

NimBLECharacteristic* channel1ConfigCharacteristic = nullptr;
NimBLECharacteristic* channel2ConfigCharacteristic = nullptr;
NimBLECharacteristic* deviceConfigCharacteristic = nullptr;
NimBLECharacteristic* trimCharacteristic = nullptr;

// Température par défaut
float currentTemperatureC = 93.5f;

String serialLine = "";

unsigned long lastTemperatureNotifyMs = 0;

// ============================================================
// Debug helpers
// ============================================================

void debugPrint(const String& msg) {
#if DEBUG_SERIAL2
  Serial.print(msg);
#endif
}

void debugPrintln(const String& msg) {
#if DEBUG_SERIAL2
  Serial.println(msg);
#endif
}

// ============================================================
// BLE advertising helpers - patch n°2
// ============================================================

void startAdvertisingSafely() {
  NimBLEAdvertising* advertising = NimBLEDevice::getAdvertising();

  if (advertising == nullptr) {
    return;
  }

#if DEBUG_SERIAL2
  Serial.println("Starting/restarting BLE advertising...");
#endif

  advertising->start();
  lastAdvertisingKickMs = millis();
}

void scheduleAdvertisingRestart(unsigned long delayMs) {
  restartAdvertisingRequested = true;
  restartAdvertisingAtMs = millis() + delayMs;
}

void handleBleAdvertisingRestart() {
  unsigned long now = millis();

  // Redémarrage demandé après déconnexion.
  if (restartAdvertisingRequested) {
    if ((long)(now - restartAdvertisingAtMs) >= 0) {
      startAdvertisingSafely();
      restartAdvertisingRequested = false;

#if DEBUG_SERIAL2
      Serial.println("BLE advertising restarted after disconnect");
#endif
    }
  }

  // Kick périodique : tente de garder/rétablir l'advertising.
  // Si l'ESP32 est déjà advertising, ce n'est pas grave.
  // Si la stack refuse pendant une connexion, ce n'est pas grave non plus.
  if (now - lastAdvertisingKickMs >= ADVERTISING_KICK_INTERVAL_MS) {
    if (!bleClientConnected || ALWAYS_ADVERTISE_WHILE_CONNECTED) {
      startAdvertisingSafely();
    }
  }
}

// ============================================================
// BLE callbacks
// ============================================================

class ServerCallbacks : public NimBLEServerCallbacks {
  // Variante simple, selon versions NimBLE
  void onConnect(NimBLEServer* pServer) {
    bleServer = pServer;
    bleClientConnected = true;
    restartAdvertisingRequested = false;

#if DEBUG_SERIAL2
    Serial.println("BLE client connected");
#endif

    // Patch n°2 :
    // essayer de continuer à advertiser même connecté.
    startAdvertisingSafely();
  }

  // Variante avec handle de connexion, selon versions NimBLE
  void onConnect(NimBLEServer* pServer, ble_gap_conn_desc* desc) {
    bleServer = pServer;
    bleClientConnected = true;
    restartAdvertisingRequested = false;

#if DEBUG_SERIAL2
    Serial.println("BLE client connected with conn handle");
#endif

    if (desc != nullptr) {
      uint16_t newConnHandle = desc->conn_handle;

      // Si une ancienne connexion existe encore et qu'une nouvelle arrive,
      // on coupe l'ancienne.
      if (
        activeConnHandle != NO_CONN_HANDLE &&
        activeConnHandle != newConnHandle
      ) {
#if DEBUG_SERIAL2
        Serial.print("Disconnecting previous ghost connection: ");
        Serial.println(activeConnHandle);
#endif
        pServer->disconnect(activeConnHandle);
      }

      activeConnHandle = newConnHandle;
    }

    // Patch n°2 :
    // rester visible pour permettre une reconnexion.
    startAdvertisingSafely();
  }

  void onDisconnect(NimBLEServer* pServer) {
    bleClientConnected = false;
    activeConnHandle = NO_CONN_HANDLE;

#if DEBUG_SERIAL2
    Serial.println("BLE client disconnected");
#endif

    // Ne pas relancer directement ici :
    // on programme un restart dans loop().
    scheduleAdvertisingRestart(500);
  }

  // Variante avec desc, selon versions NimBLE
  void onDisconnect(NimBLEServer* pServer, ble_gap_conn_desc* desc) {
    bleClientConnected = false;
    activeConnHandle = NO_CONN_HANDLE;

#if DEBUG_SERIAL2
    Serial.println("BLE client disconnected with conn handle");
#endif

    scheduleAdvertisingRestart(500);
  }
};

// ============================================================
// ETI temperature encoding
// ============================================================

void floatToLittleEndianBytes(float value, uint8_t* data) {
  // ESP32 est little-endian, mais on le fait explicitement.
  union {
    float f;
    uint8_t b[4];
  } converter;

  converter.f = value;

  data[0] = converter.b[0];
  data[1] = converter.b[1];
  data[2] = converter.b[2];
  data[3] = converter.b[3];
}

void setTemperatureValue() {
  if (tempChannel1Characteristic == nullptr) return;

  uint8_t data[4];
  floatToLittleEndianBytes(currentTemperatureC, data);

  tempChannel1Characteristic->setValue(data, 4);

  // On met aussi le canal 2 à la même valeur pour être gentil avec les apps,
  // même si Beanconqueror lit surtout le canal 1.
  if (tempChannel2Characteristic != nullptr) {
    tempChannel2Characteristic->setValue(data, 4);
  }
}

void notifyTemperature() {
  if (tempChannel1Characteristic == nullptr) return;

  setTemperatureValue();

  tempChannel1Characteristic->notify();

  if (tempChannel2Characteristic != nullptr) {
    tempChannel2Characteristic->notify();
  }

#if DEBUG_SERIAL2
  Serial.print("ETI temperature notify: ");
  Serial.print(currentTemperatureC, 2);
  Serial.println(" C");
#endif
}

// ============================================================
// Serial parsing
// ============================================================

bool parseSerialLine(String line) {
  line.trim();

  if (line.length() == 0) {
    return false;
  }

  line.replace(",", ".");
  line.toUpperCase();

  if (line == "WHO?" || line == "ID?") {
    Serial.println("ID:PIROK_THERMO");
    return true;
  }

  // Formats acceptés :
  // T:93.5
  // TEMP:93.5
  // 93.5

  if (line.startsWith("T:")) {
    line = line.substring(2);
  } else if (line.startsWith("TEMP:")) {
    line = line.substring(5);
  }

  float tempC = line.toFloat();

  if (tempC < MIN_TEMP_C) tempC = MIN_TEMP_C;
  if (tempC > MAX_TEMP_C) tempC = MAX_TEMP_C;

  currentTemperatureC = tempC;

  setTemperatureValue();
  notifyTemperature();

#if DEBUG_SERIAL2
  Serial.print("Received temperature: ");
  Serial.print(currentTemperatureC, 2);
  Serial.println(" C");
#endif

  return true;
}

void readSerialInput() {
  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      if (serialLine.length() > 0) {
        parseSerialLine(serialLine);
        serialLine = "";
      }
    } else {
      if (serialLine.length() < 64) {
        serialLine += c;
      } else {
        serialLine = "";
      }
    }
  }
}

// ============================================================
// BLE setup
// ============================================================

void setupBLE() {
  NimBLEDevice::init(DEVICE_NAME);
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);

  NimBLEServer* server = NimBLEDevice::createServer();
  bleServer = server;
  server->setCallbacks(new ServerCallbacks());

  // ----------------------------------------------------------
  // ETI temperature service
  // ----------------------------------------------------------

  NimBLEService* etiService = server->createService(ETI_SERVICE_UUID);

  tempChannel1Characteristic = etiService->createCharacteristic(
    ETI_CHANNEL_1_TEMP_CHAR_UUID,
    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
  );

  tempChannel2Characteristic = etiService->createCharacteristic(
    ETI_CHANNEL_2_TEMP_CHAR_UUID,
    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
  );

  // Caractéristiques de config factices.
  // Beanconqueror ne semble pas les lire pour afficher la température,
  // mais elles rendent l'émulation plus proche d'un vrai ETI.
  channel1ConfigCharacteristic = etiService->createCharacteristic(
    ETI_CHANNEL_1_CONFIG_CHAR_UUID,
    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE
  );

  channel2ConfigCharacteristic = etiService->createCharacteristic(
    ETI_CHANNEL_2_CONFIG_CHAR_UUID,
    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE
  );

  deviceConfigCharacteristic = etiService->createCharacteristic(
    ETI_DEVICE_CONFIG_CHAR_UUID,
    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE
  );

  trimCharacteristic = etiService->createCharacteristic(
    ETI_TRIM_CHAR_UUID,
    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE
  );

  uint8_t dummyConfig[1] = {0x00};

  channel1ConfigCharacteristic->setValue(dummyConfig, 1);
  channel2ConfigCharacteristic->setValue(dummyConfig, 1);
  deviceConfigCharacteristic->setValue(dummyConfig, 1);
  trimCharacteristic->setValue(dummyConfig, 1);

  setTemperatureValue();

  etiService->start();

  // ----------------------------------------------------------
  // Battery service
  // ----------------------------------------------------------

  NimBLEService* batteryService = server->createService(BATTERY_SERVICE_UUID);

  batteryCharacteristic = batteryService->createCharacteristic(
    BATTERY_CHAR_UUID,
    NIMBLE_PROPERTY::READ
  );

  uint8_t batteryLevel = 100;
  batteryCharacteristic->setValue(&batteryLevel, 1);

  batteryService->start();

  // ----------------------------------------------------------
  // Advertising
  // ----------------------------------------------------------

  NimBLEAdvertising* advertising = NimBLEDevice::getAdvertising();

  NimBLEAdvertisementData advData;
  advData.setFlags(0x06);
  advData.setName(DEVICE_NAME);

  NimBLEAdvertisementData scanData;
  scanData.addServiceUUID(ETI_SERVICE_UUID);
  scanData.addServiceUUID(BATTERY_SERVICE_UUID);

  advertising->setAdvertisementData(advData);
  advertising->setScanResponseData(scanData);

  // Intervalles d'advertising raisonnables.
  // 160 = 100 ms environ ; 320 = 200 ms environ.
  advertising->setMinInterval(160);
  advertising->setMaxInterval(320);

  advertising->start();
  lastAdvertisingKickMs = millis();
}

// ============================================================
// Arduino lifecycle
// ============================================================

void setup() {
  Serial.begin(115200);
  delay(500);

#if DEBUG_SERIAL2
  Serial.println();
  Serial.println("ESP32 ETI ThermaQ Blue emulator starting...");
  Serial.print("Advertising as: ");
  Serial.println(DEVICE_NAME);
  Serial.println("Accepted serial formats:");
  Serial.println("  T:93.5");
  Serial.println("  TEMP:93.5");
  Serial.println("  93.5");
#endif

  setupBLE();
}

void loop() {
  handleBleAdvertisingRestart();

  readSerialInput();

  unsigned long now = millis();

  if (now - lastTemperatureNotifyMs >= TEMPERATURE_NOTIFY_INTERVAL_MS) {
    lastTemperatureNotifyMs = now;
    notifyTemperature();
  }

  delay(1);
}