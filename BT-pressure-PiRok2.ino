#include <Arduino.h>
#include <NimBLEDevice.h>

// ============================================================
// Configuration
// ============================================================

#define DEVICE_NAME "PRS-PI"

// Mets à 1 pour debug série, 0 pour silence quasi complet
#define DEBUG_SERIAL 0

// Pression max machine : 15 bar = 15000 mbar
#define MAX_PRESSURE_MBAR 15000

// Intervalle de notification BLE.
// 100 ms = 10 Hz, proche du comportement capteur temps réel.
#define NOTIFY_INTERVAL_MS 100

// Envoyer la température toutes les 16 notifications, protocole PRS.
#define TEMP_EVERY_N_NOTIFICATIONS 16

// ============================================================
// UUIDs PRS / Pressensor
// ============================================================

#define PRESSURE_SERVICE_UUID   "873ae82a-4c5a-4342-b539-9d900bf7ebd0"
#define PRESSURE_CHAR_UUID      "873ae82b-4c5a-4342-b539-9d900bf7ebd0"
#define ZERO_PRESSURE_CHAR_UUID "873ae82c-4c5a-4342-b539-9d900bf7ebd0"

#define BATTERY_SERVICE_UUID    "180F"
#define BATTERY_CHAR_UUID       "2A19"

#define LOG_SERVICE_UUID        "873ae828-4c5a-4342-b539-9d900bf7ebd0"
#define LOG_CHAR_UUID           "873ae829-4c5a-4342-b539-9d900bf7ebd0"

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

NimBLECharacteristic* pressureCharacteristic = nullptr;
NimBLECharacteristic* batteryCharacteristic = nullptr;
NimBLECharacteristic* logCharacteristic = nullptr;

volatile int16_t currentPressureMbar = 0;
volatile int16_t zeroOffsetMbar = 0;

// Température par défaut : 93.5 °C = 935 dixièmes de degré
volatile int16_t currentTemperatureTenthDeg = 935;

bool pressureNotificationsEnabled = false;

String serialLine = "";
uint16_t notifyCounter = 0;
unsigned long lastNotifyMs = 0;

// ============================================================
// Debug helper
// ============================================================

void debugPrint(const String& msg) {
#if DEBUG_SERIAL
  Serial.print(msg);
#endif
}

void debugPrintln(const String& msg) {
#if DEBUG_SERIAL
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

#if DEBUG_SERIAL
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

#if DEBUG_SERIAL
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
// Encoding helpers
// ============================================================

void int16ToBigEndian(int16_t value, uint8_t* data) {
  data[0] = (value >> 8) & 0xFF;
  data[1] = value & 0xFF;
}

int16_t getCorrectedPressureMbar() {
  int32_t corrected = (int32_t)currentPressureMbar - (int32_t)zeroOffsetMbar;

  if (corrected > 32767) corrected = 32767;
  if (corrected < -32768) corrected = -32768;

  return (int16_t)corrected;
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

#if DEBUG_SERIAL
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

#if DEBUG_SERIAL
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
#if DEBUG_SERIAL
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
    pressureNotificationsEnabled = false;

#if DEBUG_SERIAL
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
    pressureNotificationsEnabled = false;

#if DEBUG_SERIAL
    Serial.println("BLE client disconnected with conn handle");
#endif

    scheduleAdvertisingRestart(500);
  }
};

class PressureCallbacks : public NimBLECharacteristicCallbacks {
  void onSubscribe(NimBLECharacteristic* pCharacteristic, ble_gap_conn_desc* desc, uint16_t subValue) {
    pressureNotificationsEnabled = subValue > 0;

#if DEBUG_SERIAL
    Serial.print("Pressure notifications: ");
    Serial.println(pressureNotificationsEnabled ? "ON" : "OFF");
#endif
  }
};

class ZeroPressureCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* pCharacteristic) {
    zeroOffsetMbar = currentPressureMbar;

#if DEBUG_SERIAL
    Serial.print("Zero pressure set to ");
    Serial.print(zeroOffsetMbar);
    Serial.println(" mbar");
#endif
  }
};

// ============================================================
// BLE notification
// ============================================================

void updatePressureCharacteristicValue() {
  if (pressureCharacteristic == nullptr) return;

  int16_t pressureValue = getCorrectedPressureMbar();

  uint8_t data[2];
  int16ToBigEndian(pressureValue, data);

  pressureCharacteristic->setValue(data, 2);
}

void notifyPressure() {
  if (pressureCharacteristic == nullptr) return;

  int16_t pressureValue = getCorrectedPressureMbar();
  notifyCounter++;

  if (notifyCounter % TEMP_EVERY_N_NOTIFICATIONS == 0) {
    uint8_t data[4];

    int16ToBigEndian(pressureValue, data);
    int16ToBigEndian(currentTemperatureTenthDeg, data + 2);

    pressureCharacteristic->setValue(data, 4);
    pressureCharacteristic->notify();

#if DEBUG_SERIAL
    Serial.print("Notify P+T: ");
    Serial.print(pressureValue);
    Serial.print(" mbar, ");
    Serial.print(currentTemperatureTenthDeg / 10.0f);
    Serial.println(" C");
#endif

  } else {
    uint8_t data[2];

    int16ToBigEndian(pressureValue, data);

    pressureCharacteristic->setValue(data, 2);
    pressureCharacteristic->notify();

#if DEBUG_SERIAL
    Serial.print("Notify P: ");
    Serial.print(pressureValue);
    Serial.println(" mbar");
#endif
  }
}

void notifyLog(const char* msg) {
  if (logCharacteristic == nullptr) return;

  logCharacteristic->setValue((uint8_t*)msg, strlen(msg) + 1);
  logCharacteristic->notify();
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
    Serial.println("ID:PIROK_PRESSURE");
    return true;
  }

  // Formats acceptés :
  // P:9000       pression en mbar
  // MBAR:9000   pression en mbar
  // BAR:9.0     pression en bar
  // B:9.0       pression en bar
  // T:93.5      température en °C
  // TEMP:93.5   température en °C
  // 9000        pression en mbar
  // 9.0         pression en bar

  if (line.startsWith("T:")) {
    float tempC = line.substring(2).toFloat();
    currentTemperatureTenthDeg = (int16_t)round(tempC * 10.0f);

#if DEBUG_SERIAL
    Serial.print("Temperature: ");
    Serial.print(tempC);
    Serial.println(" C");
#endif

    return true;
  }

  if (line.startsWith("TEMP:")) {
    float tempC = line.substring(5).toFloat();
    currentTemperatureTenthDeg = (int16_t)round(tempC * 10.0f);

#if DEBUG_SERIAL
    Serial.print("Temperature: ");
    Serial.print(tempC);
    Serial.println(" C");
#endif

    return true;
  }

  bool isBar = false;
  bool isMbar = false;

  if (line.startsWith("P:")) {
    line = line.substring(2);
    isMbar = true;
  } else if (line.startsWith("MBAR:")) {
    line = line.substring(5);
    isMbar = true;
  } else if (line.startsWith("BAR:")) {
    line = line.substring(4);
    isBar = true;
  } else if (line.startsWith("B:")) {
    line = line.substring(2);
    isBar = true;
  } else {
    if (line.indexOf('.') >= 0) {
      isBar = true;
    } else {
      isMbar = true;
    }
  }

  float value = line.toFloat();
  int32_t pressureMbar = 0;

  if (isBar) {
    pressureMbar = (int32_t)round(value * 1000.0f);
  } else if (isMbar) {
    pressureMbar = (int32_t)round(value);
  }

  if (pressureMbar < 0) pressureMbar = 0;
  if (pressureMbar > MAX_PRESSURE_MBAR) pressureMbar = MAX_PRESSURE_MBAR;

  currentPressureMbar = (int16_t)pressureMbar;

  updatePressureCharacteristicValue();

  // Important : on notifie toujours.
  // C'est ce comportement qui a permis aux apps de voir la valeur.
  notifyPressure();

#if DEBUG_SERIAL
  Serial.print("Pressure: ");
  Serial.print(currentPressureMbar);
  Serial.print(" mbar / ");
  Serial.print(currentPressureMbar / 1000.0f, 2);
  Serial.println(" bar");
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
  // Pressure service
  // ----------------------------------------------------------

  NimBLEService* pressureService = server->createService(PRESSURE_SERVICE_UUID);

  pressureCharacteristic = pressureService->createCharacteristic(
    PRESSURE_CHAR_UUID,
    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
  );

  pressureCharacteristic->setCallbacks(new PressureCallbacks());

  uint8_t initialPressure[2] = {0x00, 0x00};
  pressureCharacteristic->setValue(initialPressure, 2);

  NimBLECharacteristic* zeroCharacteristic = pressureService->createCharacteristic(
    ZERO_PRESSURE_CHAR_UUID,
    NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR
  );

  zeroCharacteristic->setCallbacks(new ZeroPressureCallbacks());

  pressureService->start();

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
  // Log service
  // ----------------------------------------------------------

  NimBLEService* logService = server->createService(LOG_SERVICE_UUID);

  logCharacteristic = logService->createCharacteristic(
    LOG_CHAR_UUID,
    NIMBLE_PROPERTY::NOTIFY
  );

  const char* initialLog = "PRS bridge ready";
  logCharacteristic->setValue((uint8_t*)initialLog, strlen(initialLog) + 1);

  logService->start();

  // ----------------------------------------------------------
  // Advertising
  // ----------------------------------------------------------

  NimBLEAdvertising* advertising = NimBLEDevice::getAdvertising();

  NimBLEAdvertisementData advData;
  advData.setFlags(0x06);
  advData.setName(DEVICE_NAME);

  NimBLEAdvertisementData scanData;
  scanData.addServiceUUID(PRESSURE_SERVICE_UUID);
  scanData.addServiceUUID(BATTERY_SERVICE_UUID);
  scanData.addServiceUUID(LOG_SERVICE_UUID);

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

#if DEBUG_SERIAL
  Serial.println();
  Serial.println("ESP32 PRS pressure bridge starting...");
#endif

  setupBLE();

#if DEBUG_SERIAL
  Serial.print("Advertising as ");
  Serial.println(DEVICE_NAME);
  Serial.println("Accepted serial formats: P:9000, BAR:9.0, T:93.5");
#endif
}

void loop() {
  handleBleAdvertisingRestart();

  readSerialInput();

  unsigned long now = millis();

  // Notification périodique.
  // On garde l'envoi continu parce que c'est ce qui fonctionne avec les apps.
  if (now - lastNotifyMs >= NOTIFY_INTERVAL_MS) {
    lastNotifyMs = now;
    notifyPressure();
  }

  delay(1);
}