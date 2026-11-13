/*
  SD-CANbus-Logger.ino
  Firmware: V02

  Project: SD-CANbus-Logger
  Target: ESP32-D0WD-V3-based CAN logger hardware
  Framework target: Arduino-ESP32 3.3.x / ESP-IDF 5.x

  V02 purpose:
    - Preserve the V01 SD-card read/write verification.
    - Add infrastructure Wi-Fi station connectivity while retaining the AP.
    - Store infrastructure Wi-Fi credentials in ESP32 NVS (Preferences).
    - Add shared diagnostics for Serial, browser status, and JSON endpoints.
    - Add boot memory checkpoints, runtime/HTTP/SD metrics, and event history.
    - Keep CAN/TWAI inactive; no CAN frames are transmitted or received.

  SD pin assignments:
    CS   = GPIO13
    SCK  = GPIO14
    MOSI = GPIO15
    MISO = GPIO2

  CAN pin assignments reserved for a later revision:
    RX = GPIO26
    TX = GPIO27

  Serial commands (115200 baud):
    help
    status
    events
    checkpoints
    sdtest
    wifi status
    wifi connect SSID|PASSWORD
    wifi disconnect
    wifi forget

  Wi-Fi:
    AP SSID: SD-CANbus-Logger
    AP password: CANlogger01
    AP address: 192.168.4.1
    mDNS: http://sd-canbus-logger.local/
    Infrastructure credentials are entered over USB serial and saved in NVS.

  Credential command syntax:
    wifi connect MyNetwork|MyPassword
    The pipe character separates SSID and password and cannot be part of either.
    SSID must be 1-32 characters; password must be 8-63 characters, or empty
    for an open network. Passwords are never printed in diagnostics.

  Browser endpoints:
    /                 Dashboard
    /api/status       System diagnostic snapshot
    /api/events       Recent diagnostic events
    /api/test         Run SD write/append/readback verification
    /api/client-error Browser-side fetch error telemetry

  Notes:
    - Remove the microSD card while flashing if required by the board.
    - Select the appropriate ESP32 Dev Module board definition.
    - CAN/TWAI is intentionally not initialized in V02.
*/

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <FS.h>
#include <SD.h>
#include <SPI.h>
#include <Preferences.h>
#include <ESPmDNS.h>
#include <esp_system.h>
#include <esp_heap_caps.h>

// -----------------------------------------------------------------------------
// Firmware identity and project configuration
// -----------------------------------------------------------------------------

static const char PROJECT_NAME[] = "SD-CANbus-Logger";
static const char FIRMWARE_VERSION[] = "V02";
static const char BUILD_LABEL[] = __DATE__ " " __TIME__;

static const char WIFI_AP_SSID[] = "SD-CANbus-Logger";
static const char WIFI_AP_PASSWORD[] = "CANlogger01";
static const char MDNS_HOSTNAME[] = "sd-canbus-logger";

static const uint32_t WIFI_CONNECT_TIMEOUT_MS = 20000;
static const uint32_t WIFI_RETRY_INTERVAL_MS = 30000;
static const uint32_t MDNS_RETRY_INTERVAL_MS = 10000;

// -----------------------------------------------------------------------------
// Hardware pin assignments
// -----------------------------------------------------------------------------

static const int SD_CS_PIN   = 13;
static const int SD_SCK_PIN  = 14;
static const int SD_MOSI_PIN = 15;
static const int SD_MISO_PIN = 2;

static const int CAN_RX_PIN = 26;  // Reserved; not initialized in V02.
static const int CAN_TX_PIN = 27;  // Reserved; not initialized in V02.

static const uint32_t SD_SPI_FREQUENCY = 4000000;
static const char SD_TEST_DIRECTORY[] = "/diag";
static const char SD_TEST_FILE[] = "/diag/SDTEST.TXT";

// -----------------------------------------------------------------------------
// Diagnostic data model
// -----------------------------------------------------------------------------

static const uint8_t EVENT_HISTORY_SIZE = 32;
static const uint8_t MAX_BOOT_CHECKPOINTS = 12;

struct DiagnosticEvent {
  uint32_t uptimeMs;
  char severity[6];
  char message[64];
};

struct BootCheckpoint {
  char name[24];
  uint32_t freeHeap;
  uint32_t minimumHeap;
  uint32_t largestBlock;
  uint32_t internalFree;
};

DiagnosticEvent eventHistory[EVENT_HISTORY_SIZE];
uint8_t eventHead = 0;   // Next position to write.
uint8_t eventCount = 0;

BootCheckpoint bootCheckpoints[MAX_BOOT_CHECKPOINTS];
uint8_t bootCheckpointCount = 0;

// -----------------------------------------------------------------------------
// Runtime objects
// -----------------------------------------------------------------------------

WebServer server(80);
Preferences preferences;

bool preferencesReady = false;
bool wifiApStarted = false;
bool wifiStaConnected = false;
bool wifiStaWasConnected = false;
bool wifiStaConnecting = false;
bool wifiStaManuallyDisconnected = false;
bool mdnsStarted = false;

bool sdReady = false;
bool sdTestPassed = false;
bool sdTestRunning = false;

String configuredStaSsid;
String configuredStaPassword;  // Never printed or returned by an endpoint.

String sdLastResult = "Not run";
String sdCardTypeText = "Unknown";
uint64_t sdCardCapacityBytes = 0;
uint64_t sdTotalBytes = 0;
uint64_t sdUsedBytes = 0;

uint32_t bootMillis = 0;
uint32_t staAttemptStartedMs = 0;
uint32_t nextStaRetryMs = 0;
uint32_t lastMdnsAttemptMs = 0;
uint32_t staConnectedSinceMs = 0;
uint32_t lastStaConnectionDurationMs = 0;
uint32_t staConnectAttempts = 0;
uint32_t staDisconnectCount = 0;
int lastStaDisconnectReason = -1;

uint32_t sdMountAttempts = 0;
uint32_t sdMountFailures = 0;
uint32_t sdTestCount = 0;
uint32_t sdTestDurationMs = 0;
uint32_t sdWriteOperations = 0;
uint32_t sdWriteFailures = 0;
uint32_t sdReadOperations = 0;
uint32_t sdReadFailures = 0;
uint32_t sdLastWriteDurationMs = 0;
uint32_t sdMaxWriteDurationMs = 0;
uint64_t sdTotalWriteDurationMs = 0;
uint32_t sdLastReadDurationMs = 0;
uint32_t sdMaxReadDurationMs = 0;
uint64_t sdTotalReadDurationMs = 0;

uint32_t httpRequests = 0;
uint32_t httpRootRequests = 0;
uint32_t httpStatusRequests = 0;
uint32_t httpTestRequests = 0;
uint32_t httpEventsRequests = 0;
uint32_t httpClientErrorReports = 0;
uint32_t httpNotFoundRequests = 0;
uint32_t httpErrorResponses = 0;
uint32_t httpLastDurationUs = 0;
uint32_t httpMaxDurationUs = 0;
char httpLastRoute[32] = "none";
uint16_t httpLastStatusCode = 0;

uint32_t loopCount = 0;
uint32_t loopLastDurationUs = 0;
uint32_t loopMaxDurationUs = 0;
uint32_t loopLastGapUs = 0;
uint32_t loopMaxGapUs = 0;
uint32_t loopPreviousStartUs = 0;

char serialCommandBuffer[160];
size_t serialCommandLength = 0;
bool serialCommandOverflow = false;

// -----------------------------------------------------------------------------
// Forward declarations
// -----------------------------------------------------------------------------

void addEvent(const char *severity, const char *message);
void captureBootCheckpoint(const char *name);
void printBootCheckpoints();
void printEvents();

void startWiFi();
void startStationConnection(const String &ssid, const String &password, bool saveCredentials);
void updateWiFiState();
void startMDNSIfNeeded();
void printWiFiStatus();

void initializeSDCard();
void refreshSDInformation();
bool runSDTest();

void configureWebServer();
void handleRoot();
void handleStatus();
void handleEvents();
void handleSDTest();
void handleClientError();
void handleNotFound();

String makeStatusJson();
String makeEventsJson();
String htmlPage();

void serviceSerial();
void processSerialCommand(char *command);
void printHelp();
void printStatus();

String formatUptime(uint32_t milliseconds);
String getResetReasonText();
String getArduinoCoreVersion();
String jsonEscape(const String &value);
String getSystemHealth();
void recordHttp(const char *route, uint32_t startUs, uint16_t statusCode);
void sendHttpResponse(int statusCode, const char *contentType, const String &body);

// -----------------------------------------------------------------------------
// Setup
// -----------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(500);

  bootMillis = millis();

  Serial.println();
  Serial.println(F("================================================"));
  Serial.println(F("SD-CANbus-Logger"));
  Serial.println(F("System initialization"));
  Serial.println(F("================================================"));
  Serial.print(F("Firmware: "));
  Serial.println(FIRMWARE_VERSION);
  Serial.print(F("Build:    "));
  Serial.println(BUILD_LABEL);
  Serial.print(F("Reset:    "));
  Serial.println(getResetReasonText());
  Serial.println();

  addEvent("INFO", "System startup");
  captureBootCheckpoint("Startup");

  // Load saved infrastructure credentials.
  preferencesReady = preferences.begin("netcfg", false);
  if (preferencesReady) {
    configuredStaSsid = preferences.getString("ssid", "");
    configuredStaPassword = preferences.getString("pass", "");
    addEvent("INFO", "Network settings loaded");
  } else {
    addEvent("WARN", "Preferences initialization failed");
  }
  captureBootCheckpoint("Settings loaded");

  // Start AP first so a recovery/configuration interface is always available.
  startWiFi();
  captureBootCheckpoint("Wi-Fi AP started");

WiFi.onEvent(
  [](WiFiEvent_t event, WiFiEventInfo_t info) {
    if (event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED) {
      lastStaDisconnectReason =
          (int)info.wifi_sta_disconnected.reason;
    }
  },
  ARDUINO_EVENT_WIFI_STA_DISCONNECTED
);

  // Initialize SD using the board's documented SPI pin assignments.
  initializeSDCard();
  captureBootCheckpoint("SD initialized");

  configureWebServer();
  server.begin();
  addEvent("INFO", "HTTP server started");
  captureBootCheckpoint("HTTP started");

  startMDNSIfNeeded();
  captureBootCheckpoint("mDNS initialized");

  Serial.println();
  Serial.println(F("[HTTP] Dashboard and API are ready."));
  Serial.print(F("[HTTP] AP: http://"));
  Serial.print(WiFi.softAPIP());
  Serial.println(F("/"));
  Serial.print(F("[HTTP] mDNS: http://"));
  Serial.print(MDNS_HOSTNAME);
  Serial.println(F(".local/"));
  Serial.println();

  printHelp();
  printStatus();
  printBootCheckpoints();
}

// -----------------------------------------------------------------------------
// Main loop and runtime timing
// -----------------------------------------------------------------------------

void loop() {
  uint32_t loopStartUs = micros();

  if (loopPreviousStartUs != 0) {
    loopLastGapUs = loopStartUs - loopPreviousStartUs;
    if (loopLastGapUs > loopMaxGapUs) loopMaxGapUs = loopLastGapUs;
  }
  loopPreviousStartUs = loopStartUs;

  server.handleClient();
  serviceSerial();
  updateWiFiState();
  startMDNSIfNeeded();

  delay(2);

  loopLastDurationUs = micros() - loopStartUs;
  if (loopLastDurationUs > loopMaxDurationUs) {
    loopMaxDurationUs = loopLastDurationUs;
  }
  loopCount++;
}

// -----------------------------------------------------------------------------
// Diagnostic event history
// -----------------------------------------------------------------------------

void addEvent(const char *severity, const char *message) {
  DiagnosticEvent &event = eventHistory[eventHead];

  event.uptimeMs = millis() - bootMillis;
  snprintf(event.severity, sizeof(event.severity), "%s", severity);
  snprintf(event.message, sizeof(event.message), "%s", message);

  eventHead = (eventHead + 1) % EVENT_HISTORY_SIZE;
  if (eventCount < EVENT_HISTORY_SIZE) eventCount++;
}

void printEvents() {
  Serial.println();
  Serial.println(F("RECENT DIAGNOSTIC EVENTS"));
  Serial.println(F("------------------------"));

  if (eventCount == 0) {
    Serial.println(F("No events recorded."));
    return;
  }

  uint8_t first = (eventHead + EVENT_HISTORY_SIZE - eventCount) % EVENT_HISTORY_SIZE;

  for (uint8_t i = 0; i < eventCount; i++) {
    uint8_t index = (first + i) % EVENT_HISTORY_SIZE;
    const DiagnosticEvent &event = eventHistory[index];

    Serial.print(formatUptime(event.uptimeMs));
    Serial.print(F("  "));
    Serial.print(event.severity);
    Serial.print(F("  "));
    Serial.println(event.message);
  }
  Serial.println();
}

void captureBootCheckpoint(const char *name) {
  if (bootCheckpointCount >= MAX_BOOT_CHECKPOINTS) return;

  BootCheckpoint &point = bootCheckpoints[bootCheckpointCount++];
  snprintf(point.name, sizeof(point.name), "%s", name);
  point.freeHeap = ESP.getFreeHeap();
  point.minimumHeap = ESP.getMinFreeHeap();
  point.largestBlock = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
  point.internalFree = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
}

void printBootCheckpoints() {
  Serial.println();
  Serial.println(F("BOOT MEMORY CHECKPOINTS"));
  Serial.println(F("-----------------------"));
  Serial.println(F("Checkpoint             Free       Minimum   Largest   Internal"));

  for (uint8_t i = 0; i < bootCheckpointCount; i++) {
    const BootCheckpoint &p = bootCheckpoints[i];

    char line[128];
    snprintf(line, sizeof(line), "%-22s %-10lu %-9lu %-9lu %lu",
             p.name,
             (unsigned long)p.freeHeap,
             (unsigned long)p.minimumHeap,
             (unsigned long)p.largestBlock,
             (unsigned long)p.internalFree);
    Serial.println(line);
  }
  Serial.println(F("All memory values are bytes."));
  Serial.println();
}

// -----------------------------------------------------------------------------
// Wi-Fi AP + infrastructure station
// -----------------------------------------------------------------------------

void startWiFi() {
  Serial.println(F("[Wi-Fi] Starting AP+STA mode..."));

  WiFi.mode(WIFI_AP_STA);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);

  wifiApStarted = WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASSWORD);

  if (wifiApStarted) {
    addEvent("INFO", "Wi-Fi access point started");
    Serial.println(F("[Wi-Fi] AP: PASS"));
    Serial.print(F("[Wi-Fi] AP SSID: "));
    Serial.println(WIFI_AP_SSID);
    Serial.print(F("[Wi-Fi] AP IP:   "));
    Serial.println(WiFi.softAPIP());
  } else {
    addEvent("ERROR", "Wi-Fi access point failed to start");
    Serial.println(F("[Wi-Fi] AP: FAIL"));
  }

  if (configuredStaSsid.length() > 0) {
    Serial.println(F("[Wi-Fi] Saved infrastructure network found."));
    startStationConnection(configuredStaSsid, configuredStaPassword, false);
  } else {
    Serial.println(F("[Wi-Fi] No saved infrastructure credentials."));
  }
}

void startStationConnection(const String &ssid, const String &password, bool saveCredentials) {
  if (ssid.length() == 0 || ssid.length() > 32 || password.length() > 63 ||
      (password.length() > 0 && password.length() < 8)) {
    Serial.println(F("[Wi-Fi] Invalid credentials. SSID: 1-32 chars; password: empty or 8-63 chars."));
    addEvent("WARN", "Rejected invalid Wi-Fi credentials");
    return;
  }

  configuredStaSsid = ssid;
  configuredStaPassword = password;

  if (saveCredentials && preferencesReady) {
    preferences.putString("ssid", configuredStaSsid);
    preferences.putString("pass", configuredStaPassword);
    addEvent("INFO", "Infrastructure credentials saved");
  }

  wifiStaManuallyDisconnected = false;
  wifiStaConnecting = true;
  wifiStaConnected = false;
  wifiStaWasConnected = false;
  staAttemptStartedMs = millis();
  nextStaRetryMs = 0;
  staConnectAttempts++;

  Serial.print(F("[Wi-Fi] Connecting to infrastructure SSID: "));
  Serial.println(configuredStaSsid);
  Serial.println(F("[Wi-Fi] Password not displayed."));
  Serial.print(F("[Wi-Fi] Connection attempt: "));
  Serial.println(staConnectAttempts);

  addEvent("INFO", "Infrastructure connection started");

  WiFi.mode(WIFI_AP_STA);
  WiFi.disconnect(false, false);
  delay(50);
  WiFi.begin(configuredStaSsid.c_str(), configuredStaPassword.c_str());
}

void updateWiFiState() {
  wl_status_t currentStatus = WiFi.status();

  if (currentStatus == WL_CONNECTED) {
    wifiStaConnected = true;

    if (!wifiStaWasConnected) {
      wifiStaWasConnected = true;
      wifiStaConnecting = false;
      staConnectedSinceMs = millis();

      char message[64];
      snprintf(message, sizeof(message), "STA connected; IP %s",
               WiFi.localIP().toString().c_str());
      addEvent("INFO", message);

      Serial.print(F("[Wi-Fi] STA connected. IP: "));
      Serial.println(WiFi.localIP());

      captureBootCheckpoint("STA connected");
    }
    return;
  }

  wifiStaConnected = false;

  if (wifiStaWasConnected) {
    wifiStaWasConnected = false;
    lastStaConnectionDurationMs = millis() - staConnectedSinceMs;
    staDisconnectCount++;


    addEvent("WARN", "Infrastructure Wi-Fi disconnected");

    Serial.println(F("[Wi-Fi] STA disconnected."));
    nextStaRetryMs = millis() + WIFI_RETRY_INTERVAL_MS;
  }

  if (wifiStaManuallyDisconnected || configuredStaSsid.length() == 0) {
    wifiStaConnecting = false;
    return;
  }

  if (wifiStaConnecting) {
    if (millis() - staAttemptStartedMs >= WIFI_CONNECT_TIMEOUT_MS) {
      wifiStaConnecting = false;
      nextStaRetryMs = millis() + WIFI_RETRY_INTERVAL_MS;
      addEvent("WARN", "Infrastructure connection timed out");
      Serial.println(F("[Wi-Fi] Connection timed out; retry scheduled."));
    }
    return;
  }

  if (nextStaRetryMs != 0 && (int32_t)(millis() - nextStaRetryMs) >= 0) {
    startStationConnection(configuredStaSsid, configuredStaPassword, false);
  }
}

void startMDNSIfNeeded() {
  if (mdnsStarted) return;
  if (!wifiApStarted && !wifiStaConnected) return;

  if (lastMdnsAttemptMs != 0 &&
      millis() - lastMdnsAttemptMs < MDNS_RETRY_INTERVAL_MS) return;

  lastMdnsAttemptMs = millis();

  if (MDNS.begin(MDNS_HOSTNAME)) {
    MDNS.addService("http", "tcp", 80);
    mdnsStarted = true;
    addEvent("INFO", "mDNS responder started");
    Serial.print(F("[mDNS] Available as http://"));
    Serial.print(MDNS_HOSTNAME);
    Serial.println(F(".local/"));
  } else {
    addEvent("WARN", "mDNS start failed; retry scheduled");
    Serial.println(F("[mDNS] Start failed; will retry."));
  }
}

void printWiFiStatus() {
  Serial.println();
  Serial.println(F("WI-FI STATUS"));
  Serial.println(F("------------"));
  Serial.print(F("Mode:                 "));
  Serial.println(F("AP+STA"));
  Serial.print(F("AP running:           "));
  Serial.println(wifiApStarted ? F("YES") : F("NO"));
  Serial.print(F("AP SSID:              "));
  Serial.println(WIFI_AP_SSID);
  Serial.print(F("AP IP:                "));
  Serial.println(WiFi.softAPIP());
  Serial.print(F("AP clients:           "));
  Serial.println(WiFi.softAPgetStationNum());
  Serial.print(F("AP MAC:               "));
  Serial.println(WiFi.softAPmacAddress());
  Serial.print(F("STA configured:       "));
  Serial.println(configuredStaSsid.length() ? F("YES") : F("NO"));
  Serial.print(F("STA SSID:             "));
  Serial.println(configuredStaSsid.length() ? configuredStaSsid : String("(not configured)"));
  Serial.print(F("STA connected:        "));
  Serial.println(wifiStaConnected ? F("YES") : F("NO"));
  Serial.print(F("STA IP:               "));
  Serial.println(wifiStaConnected ? WiFi.localIP().toString() : String("0.0.0.0"));
  Serial.print(F("STA RSSI:             "));
  if (wifiStaConnected) {
    Serial.print(WiFi.RSSI());
    Serial.println(F(" dBm"));
  } else {
    Serial.println(F("N/A"));
  }
  Serial.print(F("STA MAC:              "));
  Serial.println(WiFi.macAddress());
  Serial.print(F("STA attempts:         "));
  Serial.println(staConnectAttempts);
  Serial.print(F("STA disconnects:      "));
  Serial.println(staDisconnectCount);
  Serial.print(F("Last disconnect code: "));
  Serial.println(lastStaDisconnectReason);
  Serial.print(F("mDNS:                 "));
  Serial.println(mdnsStarted ? F("RUNNING") : F("NOT RUNNING"));
  Serial.print(F("Hostname:             http://"));
  Serial.print(MDNS_HOSTNAME);
  Serial.println(F(".local/"));
  Serial.println();
}

// -----------------------------------------------------------------------------
// SD initialization and information
// -----------------------------------------------------------------------------

void initializeSDCard() {
  sdMountAttempts++;

  Serial.println(F("[SD] Starting SPI and mounting card..."));
  SPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);

  if (SD.begin(SD_CS_PIN, SPI, SD_SPI_FREQUENCY)) {
    sdReady = true;
    addEvent("INFO", "SD card mounted");
    Serial.println(F("[SD] Mount: PASS"));
    refreshSDInformation();

    if (!SD.exists(SD_TEST_DIRECTORY)) {
      if (SD.mkdir(SD_TEST_DIRECTORY)) {
        Serial.println(F("[SD] Created /diag directory."));
      } else {
        addEvent("WARN", "Could not create SD diagnostic directory");
        Serial.println(F("[SD] WARNING: Could not create /diag directory."));
      }
    }

    runSDTest();
  } else {
    sdReady = false;
    sdMountFailures++;
    sdLastResult = "Card mount failed";
    addEvent("ERROR", "SD mount failed");
    Serial.println(F("[SD] Mount: FAIL"));
    Serial.println(F("[SD] Check card insertion, format, and pinout."));
  }
}

void refreshSDInformation() {
  if (!sdReady) return;

  uint8_t cardType = SD.cardType();

  switch (cardType) {
    case CARD_MMC:  sdCardTypeText = "MMC"; break;
    case CARD_SD:   sdCardTypeText = "SDSC"; break;
    case CARD_SDHC: sdCardTypeText = "SDHC/SDXC"; break;
    case CARD_NONE: sdCardTypeText = "No card"; break;
    default:        sdCardTypeText = "Unknown"; break;
  }

  sdCardCapacityBytes = SD.cardSize();
  sdTotalBytes = SD.totalBytes();
  sdUsedBytes = SD.usedBytes();
}

// -----------------------------------------------------------------------------
// SD write/append/readback diagnostic
// -----------------------------------------------------------------------------

bool runSDTest() {
  if (!sdReady) {
    sdTestPassed = false;
    sdLastResult = "SD card is not mounted";
    addEvent("ERROR", "SD test requested while card unavailable");
    Serial.println(F("[SD TEST] Cannot run: SD is not mounted."));
    return false;
  }

  if (sdTestRunning) {
    Serial.println(F("[SD TEST] Test already in progress."));
    return false;
  }

  sdTestRunning = true;
  sdTestCount++;
  uint32_t testStartMs = millis();

  char firstRecord[96];
  char secondRecord[96];

  snprintf(firstRecord, sizeof(firstRecord),
           "SD-CANbus-Logger V02 - TEST %lu - WRITE PASS",
           (unsigned long)sdTestCount);
  snprintf(secondRecord, sizeof(secondRecord),
           "SD-CANbus-Logger V02 - TEST %lu - APPEND PASS",
           (unsigned long)sdTestCount);

  Serial.println();
  Serial.println(F("[SD TEST] Starting write/append/readback verification..."));

  auto failTest = [&](const char *reason) -> bool {
    sdTestPassed = false;
    sdLastResult = reason;
    sdTestDurationMs = millis() - testStartMs;
    sdTestRunning = false;
    addEvent("ERROR", reason);
    Serial.print(F("[SD TEST] FAIL: "));
    Serial.println(reason);
    return false;
  };

  if (SD.exists(SD_TEST_FILE) && !SD.remove(SD_TEST_FILE)) {
    return failTest("Could not remove previous test file");
  }

  // First write.
  uint32_t operationStartMs = millis();
  File file = SD.open(SD_TEST_FILE, FILE_WRITE);
  if (!file) {
    sdWriteFailures++;
    return failTest("Open for write failed");
  }

  size_t firstBytes = file.println(firstRecord);
  file.flush();
  file.close();

  sdLastWriteDurationMs = millis() - operationStartMs;
  sdWriteOperations++;
  sdTotalWriteDurationMs += sdLastWriteDurationMs;
  if (sdLastWriteDurationMs > sdMaxWriteDurationMs) {
    sdMaxWriteDurationMs = sdLastWriteDurationMs;
  }

  if (firstBytes == 0) {
    sdWriteFailures++;
    return failTest("Initial write failed");
  }

  // Append second record.
  operationStartMs = millis();
  file = SD.open(SD_TEST_FILE, FILE_APPEND);
  if (!file) {
    sdWriteFailures++;
    return failTest("Open for append failed");
  }

  size_t secondBytes = file.println(secondRecord);
  file.flush();
  file.close();

  uint32_t appendDurationMs = millis() - operationStartMs;
  sdLastWriteDurationMs = appendDurationMs;
  sdWriteOperations++;
  sdTotalWriteDurationMs += appendDurationMs;
  if (appendDurationMs > sdMaxWriteDurationMs) {
    sdMaxWriteDurationMs = appendDurationMs;
  }

  if (secondBytes == 0) {
    sdWriteFailures++;
    return failTest("Append failed");
  }

  // Read and verify both records.
  operationStartMs = millis();
  file = SD.open(SD_TEST_FILE, FILE_READ);
  if (!file) {
    sdReadFailures++;
    return failTest("Open for read failed");
  }

  String contents;
  contents.reserve(256);

  while (file.available()) {
    contents += (char)file.read();
    if (contents.length() > 512) break;
  }
  file.close();

  sdLastReadDurationMs = millis() - operationStartMs;
  sdReadOperations++;
  sdTotalReadDurationMs += sdLastReadDurationMs;
  if (sdLastReadDurationMs > sdMaxReadDurationMs) {
    sdMaxReadDurationMs = sdLastReadDurationMs;
  }

  bool firstFound = contents.indexOf(firstRecord) >= 0;
  bool secondFound = contents.indexOf(secondRecord) >= 0;

  sdTestDurationMs = millis() - testStartMs;
  sdTestPassed = firstFound && secondFound;
  sdTestRunning = false;

  if (sdTestPassed) {
    sdLastResult = "PASS - write, append, readback verified";
    addEvent("INFO", "SD write/append/readback test passed");
    Serial.println(F("[SD TEST] Initial write: PASS"));
    Serial.println(F("[SD TEST] Append:        PASS"));
    Serial.println(F("[SD TEST] Readback:      PASS"));
  } else {
    sdLastResult = "FAIL - readback content mismatch";
    addEvent("ERROR", "SD readback content mismatch");
    Serial.println(F("[SD TEST] FAIL: readback content mismatch."));
    Serial.println(contents);
  }

  refreshSDInformation();

  Serial.print(F("[SD TEST] Duration: "));
  Serial.print(sdTestDurationMs);
  Serial.println(F(" ms"));

  Serial.print(F("[SD TEST] File size: "));
  File checkFile = SD.open(SD_TEST_FILE, FILE_READ);
  if (checkFile) {
    Serial.print(checkFile.size());
    Serial.println(F(" bytes"));
    checkFile.close();
  } else {
    Serial.println(F("unavailable"));
  }

  Serial.print(F("[SD TEST] Result: "));
  Serial.println(sdTestPassed ? F("PASS") : F("FAIL"));
  Serial.println();

  return sdTestPassed;
}

// -----------------------------------------------------------------------------
// HTTP server and instrumentation
// -----------------------------------------------------------------------------

void configureWebServer() {
  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/status", HTTP_GET, handleStatus);
  server.on("/api/events", HTTP_GET, handleEvents);
  server.on("/api/test", HTTP_GET, handleSDTest);
  server.on("/api/client-error", HTTP_POST, handleClientError);
  server.onNotFound(handleNotFound);
}

void sendHttpResponse(int statusCode, const char *contentType, const String &body) {
  server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate");
  server.sendHeader("Connection", "close");
  server.send(statusCode, contentType, body);
}

void recordHttp(const char *route, uint32_t startUs, uint16_t statusCode) {
  uint32_t duration = micros() - startUs;

  httpRequests++;
  httpLastDurationUs = duration;
  if (duration > httpMaxDurationUs) httpMaxDurationUs = duration;

  snprintf(httpLastRoute, sizeof(httpLastRoute), "%s", route);
  httpLastStatusCode = statusCode;

  if (statusCode >= 400) httpErrorResponses++;
}

void handleRoot() {
  uint32_t startUs = micros();
  httpRootRequests++;

  String page = htmlPage();
  sendHttpResponse(200, "text/html", page);
  recordHttp("GET /", startUs, 200);
}

void handleStatus() {
  uint32_t startUs = micros();
  httpStatusRequests++;

  refreshSDInformation();
  String json = makeStatusJson();
  sendHttpResponse(200, "application/json", json);
  recordHttp("GET /api/status", startUs, 200);
}

void handleEvents() {
  uint32_t startUs = micros();
  httpEventsRequests++;

  String json = makeEventsJson();
  sendHttpResponse(200, "application/json", json);
  recordHttp("GET /api/events", startUs, 200);
}

void handleSDTest() {
  uint32_t startUs = micros();
  httpTestRequests++;

  bool result = runSDTest();
  String json = makeStatusJson();
  uint16_t code = result ? 200 : 500;

  sendHttpResponse(code, "application/json", json);
  recordHttp("GET /api/test", startUs, code);
}

void handleClientError() {
  uint32_t startUs = micros();
  httpClientErrorReports++;

  String report = server.arg("plain");
  report.replace("\r", " ");
  report.replace("\n", " ");
  if (report.length() > 60) report.remove(60);

  char message[80];
  snprintf(message, sizeof(message), "Browser fetch failure: %s", report.c_str());
  addEvent("WARN", message);

  sendHttpResponse(204, "text/plain", "");
  recordHttp("POST /api/client-error", startUs, 204);
}

void handleNotFound() {
  uint32_t startUs = micros();
  httpNotFoundRequests++;

  String route = server.uri();
  String response = "Not found: " + route;
  sendHttpResponse(404, "text/plain", response);
  recordHttp("404", startUs, 404);
}

// -----------------------------------------------------------------------------
// System status, health, JSON helpers
// -----------------------------------------------------------------------------

String formatUptime(uint32_t milliseconds) {
  uint32_t seconds = milliseconds / 1000;
  uint32_t days = seconds / 86400;
  seconds %= 86400;
  uint32_t hours = seconds / 3600;
  seconds %= 3600;
  uint32_t minutes = seconds / 60;
  seconds %= 60;

  char buffer[40];
  snprintf(buffer, sizeof(buffer), "%lu d %02lu:%02lu:%02lu",
           (unsigned long)days, (unsigned long)hours,
           (unsigned long)minutes, (unsigned long)seconds);
  return String(buffer);
}

String getResetReasonText() {
  switch (esp_reset_reason()) {
    case ESP_RST_UNKNOWN:   return "Unknown";
    case ESP_RST_POWERON:   return "Power-on";
    case ESP_RST_EXT:       return "External reset";
    case ESP_RST_SW:        return "Software reset";
    case ESP_RST_PANIC:     return "Exception/panic";
    case ESP_RST_INT_WDT:   return "Interrupt watchdog";
    case ESP_RST_TASK_WDT:  return "Task watchdog";
    case ESP_RST_WDT:       return "Other watchdog";
    case ESP_RST_DEEPSLEEP: return "Deep-sleep wake";
    case ESP_RST_BROWNOUT:  return "Brownout";
    case ESP_RST_SDIO:      return "SDIO reset";
    default:                return "Other";
  }
}

String getArduinoCoreVersion() {
#ifdef ESP_ARDUINO_VERSION_STR
  return String(ESP_ARDUINO_VERSION_STR);
#elif defined(ESP_ARDUINO_VERSION_MAJOR)
  return String(ESP_ARDUINO_VERSION_MAJOR) + "." +
         String(ESP_ARDUINO_VERSION_MINOR) + "." +
         String(ESP_ARDUINO_VERSION_PATCH);
#else
  return "Not exposed by core";
#endif
}

String jsonEscape(const String &value) {
  String escaped;
  escaped.reserve(value.length() + 8);

  for (size_t i = 0; i < value.length(); i++) {
    char c = value[i];
    if (c == '"' || c == '\\') {
      escaped += '\\';
      escaped += c;
    } else if (c == '\n') {
      escaped += "\\n";
    } else if (c == '\r') {
      escaped += "\\r";
    } else if ((uint8_t)c >= 0x20) {
      escaped += c;
    }
  }
  return escaped;
}

String getSystemHealth() {
  if (!wifiApStarted || !sdReady || !sdTestPassed) return "FAIL";
  if (sdWriteFailures || sdReadFailures || sdMountFailures || httpErrorResponses) return "WARN";
  if (configuredStaSsid.length() > 0 && !wifiStaConnected) return "WARN";
  return "PASS";
}

String makeStatusJson() {
  uint32_t freeHeap = ESP.getFreeHeap();
  uint32_t largestBlock = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
  uint32_t internalFree = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  float fragmentation = freeHeap ? 100.0f * (1.0f - ((float)largestBlock / (float)freeHeap)) : 0.0f;

  String json;
  json.reserve(3600);

  json += "{";
  json += "\"project\":\"" + String(PROJECT_NAME) + "\",";
  json += "\"firmware\":\"" + String(FIRMWARE_VERSION) + "\",";
  json += "\"build\":\"" + String(BUILD_LABEL) + "\",";
  json += "\"arduinoCore\":\"" + jsonEscape(getArduinoCoreVersion()) + "\",";
  json += "\"espIdf\":\"" + jsonEscape(ESP.getSdkVersion()) + "\",";
  json += "\"chip\":\"" + jsonEscape(ESP.getChipModel()) + "\",";
  json += "\"chipRevision\":" + String(ESP.getChipRevision()) + ",";
  json += "\"cpuMHz\":" + String(ESP.getCpuFreqMHz()) + ",";
  json += "\"flashMB\":" + String((double)ESP.getFlashChipSize() / 1048576.0, 1) + ",";
  json += "\"sketchKB\":" + String((double)ESP.getSketchSize() / 1024.0, 1) + ",";
  json += "\"freeSketchKB\":" + String((double)ESP.getFreeSketchSpace() / 1024.0, 1) + ",";
  json += "\"uptimeMs\":" + String(millis() - bootMillis) + ",";
  json += "\"uptime\":\"" + formatUptime(millis() - bootMillis) + "\",";
  json += "\"resetReason\":\"" + getResetReasonText() + "\",";
  json += "\"health\":\"" + getSystemHealth() + "\",";

  json += "\"heapFree\":" + String(freeHeap) + ",";
  json += "\"heapMinimum\":" + String(ESP.getMinFreeHeap()) + ",";
  json += "\"heapLargestBlock\":" + String(largestBlock) + ",";
  json += "\"heapInternalFree\":" + String(internalFree) + ",";
  json += "\"heapFragmentationPercent\":" + String(fragmentation, 1) + ",";

  json += "\"wifiMode\":\"AP+STA\",";
  json += "\"apStarted\":";
  json += (wifiApStarted ? "true" : "false");
  json += ",\"apSsid\":\"" + String(WIFI_AP_SSID) + "\",";
  json += "\"apIp\":\"" + WiFi.softAPIP().toString() + "\",";
  json += "\"apMac\":\"" + WiFi.softAPmacAddress() + "\",";
  json += "\"apClients\":" + String(WiFi.softAPgetStationNum()) + ",";
  json += "\"staConfigured\":";
  json += (configuredStaSsid.length() ? "true" : "false");
  json += ",\"staSsid\":\"" + jsonEscape(configuredStaSsid) + "\",";
  json += "\"staConnected\":";
  json += (wifiStaConnected ? "true" : "false");
  json += ",\"staIp\":\"" + (wifiStaConnected ? WiFi.localIP().toString() : String("0.0.0.0")) + "\",";
  json += "\"staRssi\":" + String(wifiStaConnected ? WiFi.RSSI() : 0) + ",";
  json += "\"staMac\":\"" + WiFi.macAddress() + "\",";
  json += "\"staAttempts\":" + String(staConnectAttempts) + ",";
  json += "\"staDisconnects\":" + String(staDisconnectCount) + ",";
  json += "\"staLastDisconnectReason\":" + String(lastStaDisconnectReason) + ",";
  json += "\"staLastConnectionDurationMs\":" + String(lastStaConnectionDurationMs) + ",";
  json += "\"mdnsStarted\":";
  json += (mdnsStarted ? "true" : "false");
  json += ",\"mdnsHostname\":\"" + String(MDNS_HOSTNAME) + ".local\",";

  json += "\"sdReady\":";
  json += (sdReady ? "true" : "false");
  json += ",\"sdCardType\":\"" + sdCardTypeText + "\",";
  json += "\"sdCardMB\":" + String((double)sdCardCapacityBytes / 1048576.0, 1) + ",";
  json += "\"sdTotalMB\":" + String((double)sdTotalBytes / 1048576.0, 1) + ",";
  json += "\"sdUsedMB\":" + String((double)sdUsedBytes / 1048576.0, 1) + ",";
  json += "\"sdFreeMB\":" + String((double)(sdTotalBytes - sdUsedBytes) / 1048576.0, 1) + ",";
  json += "\"sdMountAttempts\":" + String(sdMountAttempts) + ",";
  json += "\"sdMountFailures\":" + String(sdMountFailures) + ",";
  json += "\"sdTestCount\":" + String(sdTestCount) + ",";
  json += "\"sdTestPassed\":";
  json += (sdTestPassed ? "true" : "false");
  json += ",\"sdTestDurationMs\":" + String(sdTestDurationMs) + ",";
  json += "\"sdTestResult\":\"" + jsonEscape(sdLastResult) + "\",";
  json += "\"sdWriteOperations\":" + String(sdWriteOperations) + ",";
  json += "\"sdWriteFailures\":" + String(sdWriteFailures) + ",";
  json += "\"sdReadOperations\":" + String(sdReadOperations) + ",";
  json += "\"sdReadFailures\":" + String(sdReadFailures) + ",";
  json += "\"sdLastWriteMs\":" + String(sdLastWriteDurationMs) + ",";
  json += "\"sdMaxWriteMs\":" + String(sdMaxWriteDurationMs) + ",";
  json += "\"sdAverageWriteMs\":" + String(sdWriteOperations ? (double)sdTotalWriteDurationMs / sdWriteOperations : 0.0, 2) + ",";
  json += "\"sdLastReadMs\":" + String(sdLastReadDurationMs) + ",";
  json += "\"sdMaxReadMs\":" + String(sdMaxReadDurationMs) + ",";
  json += "\"sdAverageReadMs\":" + String(sdReadOperations ? (double)sdTotalReadDurationMs / sdReadOperations : 0.0, 2) + ",";

  json += "\"httpRequests\":" + String(httpRequests) + ",";
  json += "\"httpRootRequests\":" + String(httpRootRequests) + ",";
  json += "\"httpStatusRequests\":" + String(httpStatusRequests) + ",";
  json += "\"httpTestRequests\":" + String(httpTestRequests) + ",";
  json += "\"httpEventsRequests\":" + String(httpEventsRequests) + ",";
  json += "\"httpClientErrorReports\":" + String(httpClientErrorReports) + ",";
  json += "\"httpNotFoundRequests\":" + String(httpNotFoundRequests) + ",";
  json += "\"httpErrorResponses\":" + String(httpErrorResponses) + ",";
  json += "\"httpLastRoute\":\"" + String(httpLastRoute) + "\",";
  json += "\"httpLastStatusCode\":" + String(httpLastStatusCode) + ",";
  json += "\"httpLastDurationUs\":" + String(httpLastDurationUs) + ",";
  json += "\"httpMaxDurationUs\":" + String(httpMaxDurationUs) + ",";

  json += "\"loopCount\":" + String(loopCount) + ",";
  json += "\"loopLastDurationUs\":" + String(loopLastDurationUs) + ",";
  json += "\"loopMaxDurationUs\":" + String(loopMaxDurationUs) + ",";
  json += "\"loopLastGapUs\":" + String(loopLastGapUs) + ",";
  json += "\"loopMaxGapUs\":" + String(loopMaxGapUs) + ",";
  json += "\"eventCount\":" + String(eventCount) + ",";
  json += "\"bootCheckpointCount\":" + String(bootCheckpointCount) + ",";
  json += "\"canInitialized\":false";
  json += "}";

  return json;
}

String makeEventsJson() {
  String json;
  json.reserve(2500);
  json = "{\"events\":[";

  uint8_t first = (eventHead + EVENT_HISTORY_SIZE - eventCount) % EVENT_HISTORY_SIZE;

  for (uint8_t i = 0; i < eventCount; i++) {
    uint8_t index = (first + i) % EVENT_HISTORY_SIZE;
    const DiagnosticEvent &event = eventHistory[index];

    if (i) json += ",";
    json += "{\"uptimeMs\":" + String(event.uptimeMs);
    json += ",\"uptime\":\"" + formatUptime(event.uptimeMs) + "\"";
    json += ",\"severity\":\"" + String(event.severity) + "\"";
    json += ",\"message\":\"" + jsonEscape(String(event.message)) + "\"}";
  }

  json += "]}";
  return json;
}

// -----------------------------------------------------------------------------
// Browser dashboard
// -----------------------------------------------------------------------------

String htmlPage() {
  String page;
  page.reserve(12500);

  page += R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>SD-CANbus-Logger</title>
<style>
:root{font-family:Arial,sans-serif;color:#202833;background:#f2f4f7}
body{margin:0;padding:18px}
main{max-width:1050px;margin:auto}
h1{font-size:1.55rem;margin:0 0 4px}
h2{font-size:1.12rem;margin:0 0 12px}
h3{font-size:1rem;margin:0 0 9px}
.sub,.muted{color:#667085}
.sub{margin:0 0 16px}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(270px,1fr));gap:12px}
.card{background:#fff;border:1px solid #d9dee7;border-radius:10px;padding:15px;margin:0 0 12px}
.row{display:flex;justify-content:space-between;gap:12px;padding:6px 0;border-bottom:1px solid #edf0f4}
.row:last-child{border-bottom:0}
.value{font-weight:600;text-align:right;overflow-wrap:anywhere}
.health{font-size:1.05rem;font-weight:700}
button{background:#155eef;color:#fff;border:0;border-radius:7px;padding:10px 14px;font-size:.95rem;margin:5px 5px 5px 0;cursor:pointer}
button.secondary{background:#475467}
table{width:100%;border-collapse:collapse;font-size:.86rem}
td,th{text-align:left;padding:7px;border-bottom:1px solid #edf0f4;vertical-align:top}
th{color:#667085}
#message{min-height:1.2em;color:#344054}
.small{font-size:.83rem}
pre{white-space:pre-wrap;overflow-wrap:anywhere;background:#f8fafc;padding:10px;border-radius:6px}
</style>
</head>
<body>
<main>
<h1>SD-CANbus-Logger</h1>
<p class="sub">System dashboard - V02</p>

<div class="grid">
  <section class="card">
    <h2>System health</h2>
    <div class="row"><span>Overall</span><span class="health" id="health">--</span></div>
    <div class="row"><span>Firmware</span><span class="value" id="firmware">--</span></div>
    <div class="row"><span>Build</span><span class="value" id="build">--</span></div>
    <div class="row"><span>Uptime</span><span class="value" id="uptime">--</span></div>
    <div class="row"><span>Reset reason</span><span class="value" id="resetReason">--</span></div>
  </section>

  <section class="card">
    <h2>Device identity</h2>
    <div class="row"><span>Chip</span><span class="value" id="chip">--</span></div>
    <div class="row"><span>Revision</span><span class="value" id="chipRevision">--</span></div>
    <div class="row"><span>CPU</span><span class="value" id="cpu">--</span></div>
    <div class="row"><span>Flash</span><span class="value" id="flash">--</span></div>
    <div class="row"><span>Arduino core</span><span class="value" id="arduinoCore">--</span></div>
    <div class="row"><span>ESP-IDF</span><span class="value" id="espIdf">--</span></div>
  </section>

  <section class="card">
    <h2>Memory</h2>
    <div class="row"><span>Free heap</span><span class="value" id="heapFree">--</span></div>
    <div class="row"><span>Minimum free heap</span><span class="value" id="heapMinimum">--</span></div>
    <div class="row"><span>Largest free block</span><span class="value" id="heapLargestBlock">--</span></div>
    <div class="row"><span>Internal free heap</span><span class="value" id="heapInternalFree">--</span></div>
    <div class="row"><span>Fragmentation indicator</span><span class="value" id="heapFragmentationPercent">--</span></div>
  </section>

  <section class="card">
    <h2>Wi-Fi access point</h2>
    <div class="row"><span>Status</span><span class="value" id="apStarted">--</span></div>
    <div class="row"><span>SSID</span><span class="value" id="apSsid">--</span></div>
    <div class="row"><span>IP</span><span class="value" id="apIp">--</span></div>
    <div class="row"><span>Clients</span><span class="value" id="apClients">--</span></div>
  </section>

  <section class="card">
    <h2>Infrastructure Wi-Fi</h2>
    <div class="row"><span>Configured SSID</span><span class="value" id="staSsid">--</span></div>
    <div class="row"><span>Connection</span><span class="value" id="staConnected">--</span></div>
    <div class="row"><span>IP address</span><span class="value" id="staIp">--</span></div>
    <div class="row"><span>RSSI</span><span class="value" id="staRssi">--</span></div>
    <div class="row"><span>Connect attempts</span><span class="value" id="staAttempts">--</span></div>
    <div class="row"><span>Disconnects</span><span class="value" id="staDisconnects">--</span></div>
    <div class="row"><span>mDNS</span><span class="value" id="mdnsStarted">--</span></div>
    <p class="small muted">Configure infrastructure Wi-Fi through the USB serial command interface.</p>
  </section>

  <section class="card">
    <h2>microSD</h2>
    <div class="row"><span>Mount status</span><span class="value" id="sdReady">--</span></div>
    <div class="row"><span>Card type</span><span class="value" id="sdCardType">--</span></div>
    <div class="row"><span>Card capacity</span><span class="value" id="sdCardMB">--</span></div>
    <div class="row"><span>Filesystem total</span><span class="value" id="sdTotalMB">--</span></div>
    <div class="row"><span>Filesystem used</span><span class="value" id="sdUsedMB">--</span></div>
    <div class="row"><span>Filesystem free</span><span class="value" id="sdFreeMB">--</span></div>
    <div class="row"><span>Mount failures</span><span class="value" id="sdMountFailures">--</span></div>
  </section>

  <section class="card">
    <h2>SD operation diagnostics</h2>
    <div class="row"><span>Test result</span><span class="value" id="sdTestResult">--</span></div>
    <div class="row"><span>Test count</span><span class="value" id="sdTestCount">--</span></div>
    <div class="row"><span>Test duration</span><span class="value" id="sdTestDurationMs">--</span></div>
    <div class="row"><span>Write operations / failures</span><span class="value" id="sdWrites">--</span></div>
    <div class="row"><span>Last / max write</span><span class="value" id="sdWriteTiming">--</span></div>
    <div class="row"><span>Average write</span><span class="value" id="sdAverageWriteMs">--</span></div>
    <div class="row"><span>Read operations / failures</span><span class="value" id="sdReads">--</span></div>
    <div class="row"><span>Last / max read</span><span class="value" id="sdReadTiming">--</span></div>
  </section>

  <section class="card">
    <h2>HTTP diagnostics</h2>
    <div class="row"><span>Total requests</span><span class="value" id="httpRequests">--</span></div>
    <div class="row"><span>Status requests</span><span class="value" id="httpStatusRequests">--</span></div>
    <div class="row"><span>Browser fetch reports</span><span class="value" id="httpClientErrorReports">--</span></div>
    <div class="row"><span>HTTP error responses</span><span class="value" id="httpErrorResponses">--</span></div>
    <div class="row"><span>Last route / status</span><span class="value" id="httpLastRoute">--</span></div>
    <div class="row"><span>Last / max handler time</span><span class="value" id="httpTiming">--</span></div>
  </section>

  <section class="card">
    <h2>Main-loop diagnostics</h2>
    <div class="row"><span>Loop iterations</span><span class="value" id="loopCount">--</span></div>
    <div class="row"><span>Last / max loop duration</span><span class="value" id="loopDuration">--</span></div>
    <div class="row"><span>Last / max loop gap</span><span class="value" id="loopGap">--</span></div>
    <div class="row"><span>Diagnostic events retained</span><span class="value" id="eventCount">--</span></div>
    <div class="row"><span>CAN</span><span class="value">Not initialized</span></div>
  </section>
</div>

<section class="card">
  <h2>SD verification</h2>
  <button onclick="runTest()">Run SD write/read test</button>
  <button class="secondary" onclick="refreshStatus()">Refresh status</button>
  <p id="message"></p>
</section>

<section class="card">
  <h2>Boot memory checkpoints</h2>
  <p class="small muted">Captured at subsystem initialization boundaries. Values are bytes.</p>
  <div id="checkpoints">Checkpoint values are available in Serial output for this revision.</div>
</section>

<section class="card">
  <h2>Recent diagnostic events</h2>
  <button class="secondary" onclick="refreshEvents()">Refresh events</button>
  <div id="events" class="small muted">Loading...</div>
</section>

<p class="small muted">Status refresh interval: 5 seconds. CAN/TWAI is intentionally inactive in V02.</p>
</main>

<script>
function setText(id,value){document.getElementById(id).textContent=value;}
function mb(v){return Number(v).toFixed(1)+' MB';}
function kb(v){return (Number(v)/1024).toFixed(1)+' KB';}
function reportBrowserError(endpoint,error){
  const detail=endpoint+': '+(error&&error.message?error.message:String(error));
  try{
    const blob=new Blob([detail],{type:'text/plain'});
    if(navigator.sendBeacon) navigator.sendBeacon('/api/client-error',blob);
  }catch(e){}
}
async function fetchJson(url){
  const controller=new AbortController();
  const timeout=setTimeout(()=>controller.abort(),5000);
  try{
    const response=await fetch(url,{cache:'no-store',signal:controller.signal});
    if(!response.ok) throw new Error('HTTP '+response.status);
    return await response.json();
  }finally{clearTimeout(timeout);}
}
async function refreshStatus(){
  try{
    const s=await fetchJson('/api/status');
    setText('health',s.health);
    setText('firmware',s.firmware);
    setText('build',s.build);
    setText('uptime',s.uptime);
    setText('resetReason',s.resetReason);
    setText('chip',s.chip);
    setText('chipRevision',s.chipRevision);
    setText('cpu',s.cpuMHz+' MHz');
    setText('flash',s.flashMB+' MB');
    setText('arduinoCore',s.arduinoCore);
    setText('espIdf',s.espIdf);
    setText('heapFree',kb(s.heapFree));
    setText('heapMinimum',kb(s.heapMinimum));
    setText('heapLargestBlock',kb(s.heapLargestBlock));
    setText('heapInternalFree',kb(s.heapInternalFree));
    setText('heapFragmentationPercent',s.heapFragmentationPercent+'%');
    setText('apStarted',s.apStarted?'RUNNING':'FAILED');
    setText('apSsid',s.apSsid);
    setText('apIp',s.apIp);
    setText('apClients',s.apClients);
    setText('staSsid',s.staConfigured?s.staSsid:'Not configured');
    setText('staConnected',s.staConnected?'CONNECTED':'DISCONNECTED');
    setText('staIp',s.staIp);
    setText('staRssi',s.staConnected?s.staRssi+' dBm':'N/A');
    setText('staAttempts',s.staAttempts);
    setText('staDisconnects',s.staDisconnects);
    setText('mdnsStarted',s.mdnsStarted?s.mdnsHostname:'Not running');
    setText('sdReady',s.sdReady?'MOUNTED':'NOT MOUNTED');
    setText('sdCardType',s.sdCardType);
    setText('sdCardMB',mb(s.sdCardMB));
    setText('sdTotalMB',mb(s.sdTotalMB));
    setText('sdUsedMB',mb(s.sdUsedMB));
    setText('sdFreeMB',mb(s.sdFreeMB));
    setText('sdMountFailures',s.sdMountFailures);
    setText('sdTestResult',s.sdTestResult);
    setText('sdTestCount',s.sdTestCount);
    setText('sdTestDurationMs',s.sdTestDurationMs+' ms');
    setText('sdWrites',s.sdWriteOperations+' / '+s.sdWriteFailures);
    setText('sdWriteTiming',s.sdLastWriteMs+' / '+s.sdMaxWriteMs+' ms');
    setText('sdAverageWriteMs',s.sdAverageWriteMs+' ms');
    setText('sdReads',s.sdReadOperations+' / '+s.sdReadFailures);
    setText('sdReadTiming',s.sdLastReadMs+' / '+s.sdMaxReadMs+' ms');
    setText('httpRequests',s.httpRequests);
    setText('httpStatusRequests',s.httpStatusRequests);
    setText('httpClientErrorReports',s.httpClientErrorReports);
    setText('httpErrorResponses',s.httpErrorResponses);
    setText('httpLastRoute',s.httpLastRoute+' / '+s.httpLastStatusCode);
    setText('httpTiming',s.httpLastDurationUs+' / '+s.httpMaxDurationUs+' us');
    setText('loopCount',s.loopCount);
    setText('loopDuration',s.loopLastDurationUs+' / '+s.loopMaxDurationUs+' us');
    setText('loopGap',s.loopLastGapUs+' / '+s.loopMaxGapUs+' us');
    setText('eventCount',s.eventCount);
    setText('message','');
  }catch(e){
    setText('message','Status refresh failed: '+e.message);
    reportBrowserError('/api/status',e);
  }
}
async function refreshEvents(){
  try{
    const data=await fetchJson('/api/events');
    const host=document.getElementById('events');
    if(!data.events.length){host.textContent='No events recorded.';return;}
    const table=document.createElement('table');
    const head=document.createElement('tr');
    ['Uptime','Severity','Event'].forEach(v=>{const th=document.createElement('th');th.textContent=v;head.appendChild(th);});
    table.appendChild(head);
    data.events.forEach(ev=>{
      const tr=document.createElement('tr');
      [ev.uptime,ev.severity,ev.message].forEach(v=>{const td=document.createElement('td');td.textContent=v;tr.appendChild(td);});
      table.appendChild(tr);
    });
    host.replaceChildren(table);
  }catch(e){document.getElementById('events').textContent='Event retrieval failed: '+e.message;reportBrowserError('/api/events',e);}
}
async function runTest(){
  setText('message','Running SD test...');
  try{
    const s=await fetchJson('/api/test');
    setText('message',s.sdTestResult);
    await refreshStatus();
    await refreshEvents();
  }catch(e){setText('message','SD test request failed: '+e.message);reportBrowserError('/api/test',e);}
}
refreshStatus();
refreshEvents();
setInterval(refreshStatus,5000);
setInterval(refreshEvents,15000);
</script>
</body>
</html>
)rawliteral";

  return page;
}

// -----------------------------------------------------------------------------
// Serial command interface
// -----------------------------------------------------------------------------

void serviceSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\r') continue;

    if (c == '\n') {
      if (!serialCommandOverflow) {
        serialCommandBuffer[serialCommandLength] = '\0';
        if (serialCommandLength > 0) processSerialCommand(serialCommandBuffer);
      } else {
        Serial.println(F("[SERIAL] Command too long; discarded."));
      }

      serialCommandLength = 0;
      serialCommandOverflow = false;
      continue;
    }

    if (!serialCommandOverflow) {
      if (serialCommandLength < sizeof(serialCommandBuffer) - 1) {
        serialCommandBuffer[serialCommandLength++] = c;
      } else {
        serialCommandOverflow = true;
      }
    }
  }
}

void processSerialCommand(char *command) {
  if (!strcmp(command, "help")) {
    printHelp();
  } else if (!strcmp(command, "status")) {
    printStatus();
  } else if (!strcmp(command, "events")) {
    printEvents();
  } else if (!strcmp(command, "checkpoints")) {
    printBootCheckpoints();
  } else if (!strcmp(command, "sdtest")) {
    runSDTest();
  } else if (!strcmp(command, "wifi status")) {
    printWiFiStatus();
  } else if (!strcmp(command, "wifi disconnect")) {
    WiFi.disconnect(false, false);
    wifiStaManuallyDisconnected = true;
    wifiStaConnecting = false;
    wifiStaConnected = false;
    wifiStaWasConnected = false;
    nextStaRetryMs = 0;
    addEvent("INFO", "Infrastructure Wi-Fi manually disconnected");
    Serial.println(F("[Wi-Fi] Disconnected. Saved credentials retained."));
  } else if (!strcmp(command, "wifi forget")) {
    WiFi.disconnect(false, false);
    wifiStaManuallyDisconnected = true;
    wifiStaConnecting = false;
    wifiStaConnected = false;
    wifiStaWasConnected = false;
    configuredStaSsid = "";
    configuredStaPassword = "";
    if (preferencesReady) {
      preferences.remove("ssid");
      preferences.remove("pass");
    }
    addEvent("INFO", "Saved infrastructure credentials erased");
    Serial.println(F("[Wi-Fi] Saved credentials erased."));
  } else if (!strncmp(command, "wifi connect ", 13)) {
    char *credentials = command + 13;
    char *separator = strchr(credentials, '|');

    if (!separator) {
      Serial.println(F("Syntax: wifi connect SSID|PASSWORD"));
      return;
    }

    *separator = '\0';
    String ssid(credentials);
    String password(separator + 1);
    startStationConnection(ssid, password, true);
  } else {
    Serial.print(F("Unknown command: "));
    Serial.println(command);
    printHelp();
  }
}

void printHelp() {
  Serial.println();
  Serial.println(F("SD-CANbus-Logger SERIAL COMMANDS"));
  Serial.println(F("--------------------------------"));
  Serial.println(F("help                         Show commands"));
  Serial.println(F("status                       Full system status"));
  Serial.println(F("events                       Recent diagnostic event history"));
  Serial.println(F("checkpoints                  Boot memory checkpoints"));
  Serial.println(F("sdtest                       Run SD write/append/readback test"));
  Serial.println(F("wifi status                  Show AP and infrastructure status"));
  Serial.println(F("wifi connect SSID|PASSWORD  Save credentials and connect"));
  Serial.println(F("wifi disconnect              Disconnect; retain saved credentials"));
  Serial.println(F("wifi forget                  Erase saved infrastructure credentials"));
  Serial.println();
  Serial.println(F("AP remains enabled as a recovery interface."));
  Serial.println(F("Wi-Fi password is never printed."));
  Serial.println();
}

void printStatus() {
  refreshSDInformation();

  uint32_t freeHeap = ESP.getFreeHeap();
  uint32_t largestBlock = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
  uint32_t internalFree = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  float fragmentation = freeHeap ? 100.0f * (1.0f - ((float)largestBlock / (float)freeHeap)) : 0.0f;

  Serial.println();
  Serial.println(F("SD-CANbus-Logger SYSTEM STATUS"));
  Serial.println(F("=============================="));
  Serial.print(F("Project:              ")); Serial.println(PROJECT_NAME);
  Serial.print(F("Firmware:             ")); Serial.println(FIRMWARE_VERSION);
  Serial.print(F("Build:                ")); Serial.println(BUILD_LABEL);
  Serial.print(F("Arduino core:         ")); Serial.println(getArduinoCoreVersion());
  Serial.print(F("ESP-IDF:              ")); Serial.println(ESP.getSdkVersion());
  Serial.print(F("Chip:                 ")); Serial.println(ESP.getChipModel());
  Serial.print(F("Chip revision:        ")); Serial.println(ESP.getChipRevision());
  Serial.print(F("CPU frequency:        ")); Serial.print(ESP.getCpuFreqMHz()); Serial.println(F(" MHz"));
  Serial.print(F("Flash capacity:       ")); Serial.print((double)ESP.getFlashChipSize() / 1048576.0, 1); Serial.println(F(" MB"));
  Serial.print(F("Sketch size:          ")); Serial.print((double)ESP.getSketchSize() / 1024.0, 1); Serial.println(F(" KB"));
  Serial.print(F("Free app space:       ")); Serial.print((double)ESP.getFreeSketchSpace() / 1024.0, 1); Serial.println(F(" KB"));
  Serial.print(F("Uptime:               ")); Serial.println(formatUptime(millis() - bootMillis));
  Serial.print(F("Reset reason:         ")); Serial.println(getResetReasonText());
  Serial.print(F("System health:        ")); Serial.println(getSystemHealth());

  Serial.println();
  Serial.println(F("MEMORY"));
  Serial.println(F("------"));
  Serial.print(F("Free heap:            ")); Serial.print(freeHeap); Serial.println(F(" bytes"));
  Serial.print(F("Minimum free heap:    ")); Serial.print(ESP.getMinFreeHeap()); Serial.println(F(" bytes"));
  Serial.print(F("Largest free block:   ")); Serial.print(largestBlock); Serial.println(F(" bytes"));
  Serial.print(F("Internal free heap:   ")); Serial.print(internalFree); Serial.println(F(" bytes"));
  Serial.print(F("Fragmentation index:  ")); Serial.print(fragmentation, 1); Serial.println(F("%"));

  printWiFiStatus();

  Serial.println(F("SD STORAGE"));
  Serial.println(F("-----------"));
  Serial.print(F("Mounted:              ")); Serial.println(sdReady ? F("YES") : F("NO"));
  Serial.print(F("Card type:            ")); Serial.println(sdCardTypeText);
  Serial.print(F("Card capacity:        ")); Serial.print((double)sdCardCapacityBytes / 1048576.0, 1); Serial.println(F(" MB"));
  Serial.print(F("Filesystem total:     ")); Serial.print((double)sdTotalBytes / 1048576.0, 1); Serial.println(F(" MB"));
  Serial.print(F("Filesystem used:      ")); Serial.print((double)sdUsedBytes / 1048576.0, 1); Serial.println(F(" MB"));
  Serial.print(F("Filesystem free:      ")); Serial.print((double)(sdTotalBytes - sdUsedBytes) / 1048576.0, 1); Serial.println(F(" MB"));
  Serial.print(F("Mount attempts/fails: ")); Serial.print(sdMountAttempts); Serial.print(F(" / ")); Serial.println(sdMountFailures);
  Serial.print(F("SD tests:             ")); Serial.println(sdTestCount);
  Serial.print(F("Last test:            ")); Serial.println(sdLastResult);
  Serial.print(F("Test duration:        ")); Serial.print(sdTestDurationMs); Serial.println(F(" ms"));
  Serial.print(F("Write ops/failures:   ")); Serial.print(sdWriteOperations); Serial.print(F(" / ")); Serial.println(sdWriteFailures);
  Serial.print(F("Last/max write:       ")); Serial.print(sdLastWriteDurationMs); Serial.print(F(" / ")); Serial.print(sdMaxWriteDurationMs); Serial.println(F(" ms"));
  Serial.print(F("Average write:        ")); Serial.print(sdWriteOperations ? (double)sdTotalWriteDurationMs / sdWriteOperations : 0.0, 2); Serial.println(F(" ms"));
  Serial.print(F("Read ops/failures:    ")); Serial.print(sdReadOperations); Serial.print(F(" / ")); Serial.println(sdReadFailures);
  Serial.print(F("Last/max read:        ")); Serial.print(sdLastReadDurationMs); Serial.print(F(" / ")); Serial.print(sdMaxReadDurationMs); Serial.println(F(" ms"));

  Serial.println();
  Serial.println(F("HTTP SERVER"));
  Serial.println(F("-----------"));
  Serial.print(F("Total requests:       ")); Serial.println(httpRequests);
  Serial.print(F("Status requests:      ")); Serial.println(httpStatusRequests);
  Serial.print(F("Test requests:        ")); Serial.println(httpTestRequests);
  Serial.print(F("Browser error reports:")); Serial.print(F(" ")); Serial.println(httpClientErrorReports);
  Serial.print(F("HTTP error responses: ")); Serial.println(httpErrorResponses);
  Serial.print(F("404 requests:         ")); Serial.println(httpNotFoundRequests);
  Serial.print(F("Last route:           ")); Serial.println(httpLastRoute);
  Serial.print(F("Last status:          ")); Serial.println(httpLastStatusCode);
  Serial.print(F("Last/max handler:     ")); Serial.print(httpLastDurationUs); Serial.print(F(" / ")); Serial.print(httpMaxDurationUs); Serial.println(F(" us"));

  Serial.println();
  Serial.println(F("MAIN LOOP"));
  Serial.println(F("---------"));
  Serial.print(F("Loop iterations:      ")); Serial.println(loopCount);
  Serial.print(F("Last/max duration:    ")); Serial.print(loopLastDurationUs); Serial.print(F(" / ")); Serial.print(loopMaxDurationUs); Serial.println(F(" us"));
  Serial.print(F("Last/max loop gap:    ")); Serial.print(loopLastGapUs); Serial.print(F(" / ")); Serial.print(loopMaxGapUs); Serial.println(F(" us"));

  Serial.println();
  Serial.print(F("Diagnostic events:    ")); Serial.println(eventCount);
  Serial.print(F("Boot checkpoints:     ")); Serial.println(bootCheckpointCount);
  Serial.println(F("CAN/TWAI:             NOT INITIALIZED"));
  Serial.println();
}
