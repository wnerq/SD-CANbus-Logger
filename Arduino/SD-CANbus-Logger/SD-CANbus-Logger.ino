/*
  SD-CANbus-Logger.ino
  Firmware: V04

  Project: SD-CANbus-Logger
  Target: ESP32-D0WD-V3-based CAN logger hardware
  Framework target: Arduino-ESP32 3.3.x / ESP-IDF 5.x

  V04 purpose:
    - Preserve the V03 Wi-Fi, SD, HTTP, and diagnostic foundation.
    - Distinguish Wi-Fi CONNECTING from an actual disconnected fault.
    - Drive the onboard WS2812-compatible RGB LED on GPIO4 using the
      Arduino-ESP32 RGB LED/RMT support.
    - Provide a subtle 10-second healthy heartbeat and subsystem fault codes.
    - Maintain bounded fault-event history and a bounded SD-write queue.
    - Append fault and recovery events to /diag/FAULTS.CSV.
    - Keep CAN/TWAI inactive; CAN fault states are reserved for later versions.

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
    faults
    faults clear
    httperrors
    loopevents
    checkpoints
    sdtest
    sd remount
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
    /api/http-errors  Recent HTTP 4xx/5xx requests
    /api/loop-events  Recent loop timing anomalies
    /api/faults       Active faults and recent fault/recovery events
    /api/test         Run SD write/append/readback verification
    /api/client-error Browser-side fetch error telemetry

  Notes:
    - Remove the microSD card while flashing if required by the board.
    - Select the appropriate ESP32 Dev Module board definition.
    - CAN/TWAI is intentionally not initialized in V04.
    - GPIO4 drives the onboard WS2812-compatible addressable RGB LED.
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
#include <esp_timer.h>
#include <esp32-hal-rgb-led.h>

// -----------------------------------------------------------------------------
// Firmware identity and project configuration
// -----------------------------------------------------------------------------

static const char PROJECT_NAME[] = "SD-CANbus-Logger";
static const char FIRMWARE_VERSION[] = "V04";
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

static const int CAN_RX_PIN = 26;  // Reserved; not initialized in V04.
static const int CAN_TX_PIN = 27;  // Reserved; not initialized in V04.

// Onboard addressable RGB LED: GPIO4 is the WS2812 data input.
static const int RGB_LED_DATA_PIN = 4;
static const uint32_t LED_HEARTBEAT_INTERVAL_MS = 10000;
static const uint16_t LED_HEARTBEAT_PULSE_MS = 25;

static const uint32_t SD_SPI_FREQUENCY = 4000000;
static const char SD_TEST_DIRECTORY[] = "/diag";
static const char SD_TEST_FILE[] = "/diag/SDTEST.TXT";
static const char FAULT_LOG_FILE[] = "/diag/FAULTS.CSV";

// -----------------------------------------------------------------------------
// Diagnostic data model
// -----------------------------------------------------------------------------

static const uint8_t EVENT_HISTORY_SIZE = 32;
static const uint8_t MAX_BOOT_CHECKPOINTS = 12;
static const uint8_t HTTP_ERROR_HISTORY_SIZE = 8;
static const uint8_t LOOP_EVENT_HISTORY_SIZE = 8;
static const uint32_t LOOP_DIAGNOSTIC_THRESHOLD_US = 100000; // 100 ms

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

struct HttpErrorRecord {
  uint32_t uptimeMs;
  uint16_t statusCode;
  char method[8];
  char uri[72];
};

struct LoopTimingRecord {
  uint32_t uptimeMs;
  uint32_t gapUs;
  uint32_t durationUs;
  uint16_t relatedHttpStatus;
  char relatedHttpRoute[48];
};

// Fault IDs represent independent fault conditions. Several conditions can be
// active simultaneously within the same subsystem.
enum FaultId : uint8_t {
  FAULT_WIFI_AP = 0,
  FAULT_WIFI_STA,
  FAULT_SD_MOUNT,
  FAULT_SD_TEST,
  FAULT_SD_LOG,
  FAULT_CAN_DRIVER,
  FAULT_ID_COUNT
};

struct ActiveFault {
  bool active;
  uint16_t code;
  char subsystem[8];
  char description[72];
};

struct FaultEventRecord {
  uint64_t timestampUs;
  uint32_t bootId;
  uint32_t sequence;
  uint16_t code;
  char subsystem[8];
  char event[20];
  char description[72];
};

static const uint8_t FAULT_EVENT_QUEUE_SIZE = 16;
static const uint8_t FAULT_HISTORY_SIZE = 16;
static const uint32_t FAULT_LOG_RETRY_INTERVAL_MS = 10000;

ActiveFault activeFaults[FAULT_ID_COUNT];
FaultEventRecord faultEventQueue[FAULT_EVENT_QUEUE_SIZE];
uint8_t faultQueueHead = 0;
uint8_t faultQueueTail = 0;
uint8_t faultQueueCount = 0;
uint32_t faultQueueDropCount = 0;

FaultEventRecord faultEventHistory[FAULT_HISTORY_SIZE];
uint8_t faultHistoryHead = 0;
uint8_t faultHistoryCount = 0;

uint32_t faultBootId = 0;
uint32_t faultEventSequence = 0;
bool faultBootIdPersistent = false;
bool faultLogReady = false;
uint32_t faultLogWriteCount = 0;
uint32_t faultLogWriteFailures = 0;
uint32_t nextFaultLogRetryMs = 0;

// LED patterns are advanced by updateLed() without delay().
enum LedPattern : uint8_t {
  LED_PATTERN_STARTUP = 0,
  LED_PATTERN_IDLE,
  LED_PATTERN_HEARTBEAT,
  LED_PATTERN_TRANSITION,
  LED_PATTERN_FAULT
};

LedPattern ledPattern = LED_PATTERN_STARTUP;
bool ledInitialized = false;
bool ledTransitionPending = false;
uint8_t ledStartupStep = 0;
uint8_t ledFaultSubsystem = 0;
uint8_t ledFaultFlashesRemaining = 0;
bool ledFaultGap = false;
uint32_t ledNextActionMs = 0;
uint32_t ledLastHeartbeatMs = 0;

DiagnosticEvent eventHistory[EVENT_HISTORY_SIZE];
uint8_t eventHead = 0;   // Next position to write.
uint8_t eventCount = 0;

BootCheckpoint bootCheckpoints[MAX_BOOT_CHECKPOINTS];
uint8_t bootCheckpointCount = 0;

HttpErrorRecord httpErrorHistory[HTTP_ERROR_HISTORY_SIZE];
uint8_t httpErrorHead = 0;
uint8_t httpErrorCount = 0;

LoopTimingRecord loopEventHistory[LOOP_EVENT_HISTORY_SIZE];
uint8_t loopEventHead = 0;
uint8_t loopEventCount = 0;
uint32_t loopTimingEventCount = 0;

// -----------------------------------------------------------------------------
// Runtime objects
// -----------------------------------------------------------------------------

WebServer server(80);
Preferences preferences;
Preferences diagnosticPreferences;

bool preferencesReady = false;
bool diagnosticPreferencesReady = false;
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
char httpLastRoute[80] = "none";
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
void printHttpErrors();
void printLoopEvents();
void addHttpError(const char *method, const char *uri, uint16_t statusCode);
void addLoopTimingEvent();

void initializeFaultSystem();
void enqueueFaultEvent(const char *subsystem, const char *eventName,
                       uint16_t code, const char *description);
void setFault(FaultId id, const char *subsystem, uint16_t code, const char *description);
void clearFault(FaultId id, const char *description);
bool isFaultActive(FaultId id);
bool isSubsystemFaultActive(const char *subsystem);
uint8_t getActiveFaultCount();
void serviceFaultLog();
bool initializeFaultLogFile();
bool writeFaultEvent(const FaultEventRecord &record);
void printFaults();
void handleFaults();

void initializeLed();
void updateLed();
void setLedRed(uint8_t brightness);
void requestLedTransition();
const char *getLedPatternText();
void startLedTransitionPattern();
void startLedFaultPattern(uint8_t subsystemIndex);
int findNextFaultSubsystem(int startIndex);

void startWiFi();
void startStationConnection(const String &ssid, const String &password, bool saveCredentials);
void updateWiFiState();
void startMDNSIfNeeded();
void printWiFiStatus();

void initializeSDCard();
void remountSDCard();
void refreshSDInformation();
bool runSDTest();

void configureWebServer();
void handleRoot();
void handleStatus();
void handleEvents();
void handleSDTest();
void handleClientError();
void handleHttpErrors();
void handleLoopEvents();
void handleNotFound();

String makeStatusJson();
String makeEventsJson();
String makeHttpErrorsJson();
String makeLoopEventsJson();
String makeFaultsJson();
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
String getSystemHealthDetails();
void printSystemHealthDetails();
void recordHttp(const char *method, const char *route, uint32_t startUs, uint16_t statusCode);
void printCommandEcho(const char *command);
void sendHttpResponse(int statusCode, const char *contentType, const String &body);

// -----------------------------------------------------------------------------
// Setup
// -----------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  bootMillis = millis();
  initializeLed();
  delay(500);

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

  initializeFaultSystem();
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
  serviceFaultLog();

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

    // At this point loopLastDurationUs belongs to the preceding loop, so the
    // gap and duration describe the same start-to-start interval.
    if (loopLastGapUs >= LOOP_DIAGNOSTIC_THRESHOLD_US ||
        loopLastDurationUs >= LOOP_DIAGNOSTIC_THRESHOLD_US) {
      addLoopTimingEvent();
    }
  }
  loopPreviousStartUs = loopStartUs;

  server.handleClient();
  serviceSerial();
  updateWiFiState();
  startMDNSIfNeeded();
  serviceFaultLog();
  updateLed();

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

void addHttpError(const char *method, const char *uri, uint16_t statusCode) {
  HttpErrorRecord &record = httpErrorHistory[httpErrorHead];
  record.uptimeMs = millis() - bootMillis;
  record.statusCode = statusCode;
  snprintf(record.method, sizeof(record.method), "%s", method);
  snprintf(record.uri, sizeof(record.uri), "%s", uri);
  httpErrorHead = (httpErrorHead + 1) % HTTP_ERROR_HISTORY_SIZE;
  if (httpErrorCount < HTTP_ERROR_HISTORY_SIZE) httpErrorCount++;
}

void printHttpErrors() {
  Serial.println();
  Serial.println(F("RECENT HTTP ERROR HISTORY"));
  Serial.println(F("-------------------------"));
  if (!httpErrorCount) {
    Serial.println(F("No HTTP error responses recorded."));
    return;
  }
  uint8_t first = (httpErrorHead + HTTP_ERROR_HISTORY_SIZE - httpErrorCount) % HTTP_ERROR_HISTORY_SIZE;
  for (uint8_t i = 0; i < httpErrorCount; i++) {
    const HttpErrorRecord &r = httpErrorHistory[(first + i) % HTTP_ERROR_HISTORY_SIZE];
    Serial.print(formatUptime(r.uptimeMs)); Serial.print(F("  "));
    Serial.print(r.method); Serial.print(' '); Serial.print(r.uri);
    Serial.print(F("  HTTP ")); Serial.println(r.statusCode);
  }
  Serial.println();
}

void addLoopTimingEvent() {
  LoopTimingRecord &record = loopEventHistory[loopEventHead];
  record.uptimeMs = millis() - bootMillis;
  record.gapUs = loopLastGapUs;
  record.durationUs = loopLastDurationUs;
  record.relatedHttpStatus = httpLastStatusCode;
  snprintf(record.relatedHttpRoute, sizeof(record.relatedHttpRoute), "%s", httpLastRoute);
  loopEventHead = (loopEventHead + 1) % LOOP_EVENT_HISTORY_SIZE;
  if (loopEventCount < LOOP_EVENT_HISTORY_SIZE) loopEventCount++;
  loopTimingEventCount++;
}

void printLoopEvents() {
  Serial.println();
  Serial.println(F("RECENT LOOP TIMING EVENTS (threshold 100 ms)"));
  Serial.println(F("--------------------------------------------"));
  if (!loopEventCount) {
    Serial.println(F("No loop timing events recorded."));
    return;
  }
  uint8_t first = (loopEventHead + LOOP_EVENT_HISTORY_SIZE - loopEventCount) % LOOP_EVENT_HISTORY_SIZE;
  for (uint8_t i = 0; i < loopEventCount; i++) {
    const LoopTimingRecord &r = loopEventHistory[(first + i) % LOOP_EVENT_HISTORY_SIZE];
    Serial.print(formatUptime(r.uptimeMs));
    Serial.print(F("  gap=")); Serial.print(r.gapUs);
    Serial.print(F(" us  duration=")); Serial.print(r.durationUs);
    Serial.print(F(" us  preceding HTTP context: "));
    Serial.print(r.relatedHttpRoute); Serial.print(F(" / "));
    Serial.println(r.relatedHttpStatus);
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
// Fault-event infrastructure and persistent fault log
// -----------------------------------------------------------------------------

void initializeFaultSystem() {
  diagnosticPreferencesReady = diagnosticPreferences.begin("diagcfg", false);
  if (diagnosticPreferencesReady) {
    uint32_t previousBootId = diagnosticPreferences.getUInt("bootId", 0);
    faultBootId = previousBootId + 1;
    if (faultBootId == 0) faultBootId = 1;
    faultBootIdPersistent = diagnosticPreferences.putUInt("bootId", faultBootId) > 0;
    diagnosticPreferences.end();
  }

  if (!faultBootIdPersistent) {
    // A random session identifier still distinguishes this boot when NVS is
    // unavailable, although it is not a persistent sequential boot counter.
    faultBootId = esp_random();
    if (faultBootId == 0) faultBootId = 1;
    addEvent("WARN", "Persistent diagnostic boot ID unavailable; using random session ID");
  }

  enqueueFaultEvent("SYSTEM", "BOOT", 0, "Power-on startup");
}

void enqueueFaultEvent(const char *subsystem, const char *eventName,
                       uint16_t code, const char *description) {
  FaultEventRecord record;
  memset(&record, 0, sizeof(record));
  record.timestampUs = (uint64_t)esp_timer_get_time();
  record.bootId = faultBootId;
  record.sequence = ++faultEventSequence;
  record.code = code;
  snprintf(record.subsystem, sizeof(record.subsystem), "%s", subsystem);
  snprintf(record.event, sizeof(record.event), "%s", eventName);
  snprintf(record.description, sizeof(record.description), "%s", description);

  faultEventHistory[faultHistoryHead] = record;
  faultHistoryHead = (faultHistoryHead + 1) % FAULT_HISTORY_SIZE;
  if (faultHistoryCount < FAULT_HISTORY_SIZE) faultHistoryCount++;

  if (faultQueueCount >= FAULT_EVENT_QUEUE_SIZE) {
    faultQueueDropCount++;
    return;
  }

  faultEventQueue[faultQueueHead] = record;
  faultQueueHead = (faultQueueHead + 1) % FAULT_EVENT_QUEUE_SIZE;
  faultQueueCount++;
}

bool isFaultActive(FaultId id) {
  return id < FAULT_ID_COUNT && activeFaults[id].active;
}

bool isSubsystemFaultActive(const char *subsystem) {
  for (uint8_t i = 0; i < FAULT_ID_COUNT; i++) {
    if (activeFaults[i].active && strcmp(activeFaults[i].subsystem, subsystem) == 0) {
      return true;
    }
  }
  return false;
}

uint8_t getActiveFaultCount() {
  uint8_t count = 0;
  for (uint8_t i = 0; i < FAULT_ID_COUNT; i++) {
    if (activeFaults[i].active) count++;
  }
  return count;
}

void setFault(FaultId id, const char *subsystem, uint16_t code, const char *description) {
  if (id >= FAULT_ID_COUNT) return;
  ActiveFault &fault = activeFaults[id];
  if (fault.active && fault.code == code &&
      strncmp(fault.description, description, sizeof(fault.description)) == 0) return;

  fault.active = true;
  fault.code = code;
  snprintf(fault.subsystem, sizeof(fault.subsystem), "%s", subsystem);
  snprintf(fault.description, sizeof(fault.description), "%s", description);
  enqueueFaultEvent(subsystem, "FAULT_ACTIVE", code, description);
  addEvent("ERROR", description);
}

void clearFault(FaultId id, const char *description) {
  if (id >= FAULT_ID_COUNT || !activeFaults[id].active) return;

  ActiveFault &fault = activeFaults[id];
  enqueueFaultEvent(fault.subsystem, "RECOVERED", fault.code, description);
  addEvent("INFO", description);
  fault.active = false;
  fault.code = 0;
  fault.description[0] = '\0';
  ledTransitionPending = true;
}

bool writeCsvQuoted(File &file, const char *value) {
  if (file.print('"') == 0) return false;
  for (const char *p = value; *p; ++p) {
    if (*p == '"' && file.print('"') == 0) return false;
    if (file.write((uint8_t)*p) != 1) return false;
  }
  return file.print('"') > 0;
}

bool initializeFaultLogFile() {
  if (!sdReady) return false;

  if (!SD.exists(SD_TEST_DIRECTORY) && !SD.mkdir(SD_TEST_DIRECTORY)) return false;

  File file = SD.open(FAULT_LOG_FILE, FILE_APPEND);
  if (!file) return false;

  bool ok = true;
  if (file.size() == 0) {
    const char header[] = "timestamp_us,boot_id,event_sequence,subsystem,event,code,description";
    ok = file.println(header) == (strlen(header) + 2); // CRLF terminator
    file.flush();
  }
  file.close();
  return ok;
}

bool writeFaultEvent(const FaultEventRecord &record) {
  File file = SD.open(FAULT_LOG_FILE, FILE_APPEND);
  if (!file) return false;

  char timestamp[24];
  snprintf(timestamp, sizeof(timestamp), "%llu", (unsigned long long)record.timestampUs);

  bool ok = file.print(timestamp) > 0;
  ok = ok && file.print(',') > 0;
  ok = ok && file.print(record.bootId) > 0;
  ok = ok && file.print(',') > 0;
  ok = ok && file.print(record.sequence) > 0;
  ok = ok && file.print(',') > 0;
  ok = ok && writeCsvQuoted(file, record.subsystem);
  ok = ok && file.print(',') > 0;
  ok = ok && writeCsvQuoted(file, record.event);
  ok = ok && file.print(',') > 0;
  ok = ok && file.print(record.code) > 0;
  ok = ok && file.print(',') > 0;
  ok = ok && writeCsvQuoted(file, record.description);
  size_t newlineBytes = file.println();
  ok = ok && newlineBytes > 0;
  file.flush();
  file.close();
  return ok;
}

void serviceFaultLog() {
  if (!sdReady) return;
  uint32_t now = millis();
  if (nextFaultLogRetryMs != 0 && (int32_t)(now - nextFaultLogRetryMs) < 0) return;

  if (!faultLogReady) {
    if (!initializeFaultLogFile()) {
      faultLogWriteFailures++;
      nextFaultLogRetryMs = now + FAULT_LOG_RETRY_INTERVAL_MS;
      setFault(FAULT_SD_LOG, "SD", 3, "Fault history file could not be initialized or opened");
      return;
    }
    faultLogReady = true;
    nextFaultLogRetryMs = 0;
  }

  if (faultQueueCount == 0) return;

  const FaultEventRecord &record = faultEventQueue[faultQueueTail];
  if (writeFaultEvent(record)) {
    faultQueueTail = (faultQueueTail + 1) % FAULT_EVENT_QUEUE_SIZE;
    faultQueueCount--;
    faultLogWriteCount++;
    nextFaultLogRetryMs = 0;
    if (isFaultActive(FAULT_SD_LOG) && sdTestPassed) {
      clearFault(FAULT_SD_LOG, "SD fault-event file write recovered");
    }
  } else {
    faultLogWriteFailures++;
    faultLogReady = false;
    nextFaultLogRetryMs = now + FAULT_LOG_RETRY_INTERVAL_MS;
    setFault(FAULT_SD_LOG, "SD", 3, "Fault history file write failed");
  }
}

void printFaults() {
  Serial.println();
  Serial.println(F("FAULT STATUS"));
  Serial.println(F("------------"));
  Serial.print(F("Boot/session ID:      ")); Serial.println(faultBootId);
  Serial.print(F("Boot ID persistent:   ")); Serial.println(faultBootIdPersistent ? F("YES") : F("NO - RANDOM ID"));
  Serial.print(F("Active faults:        ")); Serial.println(getActiveFaultCount());

  bool anyActive = false;
  for (uint8_t i = 0; i < FAULT_ID_COUNT; i++) {
    if (!activeFaults[i].active) continue;
    anyActive = true;
    Serial.print(F("  ")); Serial.print(activeFaults[i].subsystem);
    Serial.print(F(" code ")); Serial.print(activeFaults[i].code);
    Serial.print(F(": ")); Serial.println(activeFaults[i].description);
  }
  if (!anyActive) Serial.println(F("  None"));

Serial.print(F("Fault log:            "));
if (faultLogReady) {
  Serial.println(FAULT_LOG_FILE);
} else {
  Serial.println(F("NOT READY"));
}

  Serial.print(F("Queued events:        ")); Serial.println(faultQueueCount);
  Serial.print(F("Queue drops:          ")); Serial.println(faultQueueDropCount);
  Serial.print(F("Events written:       ")); Serial.println(faultLogWriteCount);
  Serial.print(F("Write failures:       ")); Serial.println(faultLogWriteFailures);

  Serial.println();
  Serial.println(F("RECENT FAULT / RECOVERY EVENTS"));
  Serial.println(F("------------------------------"));
  if (!faultHistoryCount) {
    Serial.println(F("No fault events retained."));
  } else {
    uint8_t first = (faultHistoryHead + FAULT_HISTORY_SIZE - faultHistoryCount) % FAULT_HISTORY_SIZE;
    for (uint8_t i = 0; i < faultHistoryCount; i++) {
      const FaultEventRecord &r = faultEventHistory[(first + i) % FAULT_HISTORY_SIZE];
      char timestampText[24];
      snprintf(timestampText, sizeof(timestampText), "%llu", (unsigned long long)r.timestampUs);
      Serial.print(timestampText); Serial.print(F(" us  boot="));
      Serial.print(r.bootId); Serial.print(F(" seq=")); Serial.print(r.sequence);
      Serial.print(' '); Serial.print(r.subsystem); Serial.print(' ');
      Serial.print(r.event); Serial.print(F(" code=")); Serial.print(r.code);
      Serial.print(F("  ")); Serial.println(r.description);
    }
  }
  Serial.println();
}

String makeFaultsJson() {
  String json;
  json.reserve(3000);
  json = "{\"bootId\":" + String(faultBootId);
  json += ",\"bootIdPersistent\":" + String(faultBootIdPersistent ? "true" : "false");
  json += ",\"activeCount\":" + String(getActiveFaultCount());
  json += ",\"queueCount\":" + String(faultQueueCount);
  json += ",\"queueDrops\":" + String(faultQueueDropCount);
  json += ",\"logReady\":" + String(faultLogReady ? "true" : "false");
  json += ",\"logWrites\":" + String(faultLogWriteCount);
  json += ",\"logWriteFailures\":" + String(faultLogWriteFailures);
  json += ",\"ledState\":\"" + String(getLedPatternText()) + "\"";
  json += ",\"active\":[";
  bool firstActive = true;
  for (uint8_t i = 0; i < FAULT_ID_COUNT; i++) {
    if (!activeFaults[i].active) continue;
    if (!firstActive) json += ',';
    firstActive = false;
    json += "{\"subsystem\":\"" + String(activeFaults[i].subsystem) + "\"";
    json += ",\"code\":" + String(activeFaults[i].code);
    json += ",\"description\":\"" + jsonEscape(String(activeFaults[i].description)) + "\"}";
  }
  json += "],\"events\":[";
  uint8_t first = (faultHistoryHead + FAULT_HISTORY_SIZE - faultHistoryCount) % FAULT_HISTORY_SIZE;
  for (uint8_t i = 0; i < faultHistoryCount; i++) {
    const FaultEventRecord &r = faultEventHistory[(first + i) % FAULT_HISTORY_SIZE];
    if (i) json += ',';
    char timestampText[24];
    snprintf(timestampText, sizeof(timestampText), "%llu", (unsigned long long)r.timestampUs);
    json += "{\"timestampUs\":" + String(timestampText);
    json += ",\"bootId\":" + String(r.bootId);
    json += ",\"sequence\":" + String(r.sequence);
    json += ",\"subsystem\":\"" + String(r.subsystem) + "\"";
    json += ",\"event\":\"" + String(r.event) + "\"";
    json += ",\"code\":" + String(r.code);
    json += ",\"description\":\"" + jsonEscape(String(r.description)) + "\"}";
  }
  json += "]}";
  return json;
}

void handleFaults() {
  uint32_t startUs = micros();
  String json = makeFaultsJson();
  sendHttpResponse(200, "application/json", json);
  recordHttp("GET", "/api/faults", startUs, 200);
}

// -----------------------------------------------------------------------------
// Nonblocking WS2812 LED diagnostics
// -----------------------------------------------------------------------------

void setLedRed(uint8_t brightness) {
  // Arduino-ESP32 3.3.x provides this RMT-backed WS2812 helper. Explicit GRB
  // order matches the standard WS2812B data order.
  // A one-pixel RMT frame is transmitted synchronously; the application
  // pattern scheduler itself remains nonblocking and calls this only on edges.
  rgbLedWriteOrdered(RGB_LED_DATA_PIN, LED_COLOR_ORDER_GRB, brightness, 0, 0);
}

void initializeLed() {
  setLedRed(0);
  ledInitialized = true;
  ledPattern = LED_PATTERN_STARTUP;
  ledStartupStep = 0;
  ledNextActionMs = millis();
  ledLastHeartbeatMs = millis();
}

void requestLedTransition() {
  ledTransitionPending = true;
}

const char *getLedPatternText() {
  if (!ledInitialized) return "NOT INITIALIZED";
  switch (ledPattern) {
    case LED_PATTERN_STARTUP: return "STARTUP";
    case LED_PATTERN_HEARTBEAT: return "HEARTBEAT";
    case LED_PATTERN_TRANSITION: return "TRANSITION";
    case LED_PATTERN_FAULT: return "FAULT";
    default: return getActiveFaultCount() ? "FAULT WAIT" : "HEALTHY IDLE";
  }
}

int findNextFaultSubsystem(int startIndex) {
  for (int offset = 0; offset < 3; offset++) {
    int index = (startIndex + offset) % 3;
    const char *name = index == 0 ? "WIFI" : (index == 1 ? "SD" : "CAN");
    if (isSubsystemFaultActive(name)) return index;
  }
  return -1;
}

void startLedTransitionPattern() {
  ledPattern = LED_PATTERN_TRANSITION;
  setLedRed(12);
  ledNextActionMs = millis() + 180;
}

void startLedFaultPattern(uint8_t subsystemIndex) {
  ledPattern = LED_PATTERN_FAULT;
  ledFaultSubsystem = subsystemIndex;
  ledFaultFlashesRemaining = subsystemIndex + 1;
  ledFaultGap = false;
  setLedRed(32);
  ledNextActionMs = millis() + 85;
}

void updateLed() {
  if (!ledInitialized) return;
  uint32_t now = millis();
  if ((int32_t)(now - ledNextActionMs) < 0) return;

  // Startup is a distinct 1-long + 2-short red sequence.
  if (ledPattern == LED_PATTERN_STARTUP) {
    static const uint16_t durations[] = {130, 100, 65, 90, 65, 100};
    if ((ledStartupStep % 2) == 0) setLedRed(20);
    else setLedRed(0);
    ledNextActionMs = now + durations[ledStartupStep];
    ledStartupStep++;
    if (ledStartupStep >= 6) {
      setLedRed(0);
      ledPattern = LED_PATTERN_IDLE;
      ledLastHeartbeatMs = now;
      ledNextActionMs = now;
    }
    return;
  }

  if (ledPattern == LED_PATTERN_HEARTBEAT) {
    setLedRed(0);
    ledPattern = LED_PATTERN_IDLE;
    ledLastHeartbeatMs = now;
    ledNextActionMs = now;
    return;
  }

  if (ledPattern == LED_PATTERN_TRANSITION) {
    setLedRed(0);
    ledPattern = LED_PATTERN_IDLE;
    ledTransitionPending = false;
    ledLastHeartbeatMs = now;
    ledNextActionMs = now;
    return;
  }

  if (ledPattern == LED_PATTERN_FAULT) {
    if (ledFaultSubsystem == 0xFF) {
      ledPattern = LED_PATTERN_IDLE;
      ledFaultSubsystem = 0;
      ledNextActionMs = now;
      return;
    }
    if (!ledFaultGap) {
      setLedRed(0);
      ledFaultFlashesRemaining--;
      if (ledFaultFlashesRemaining > 0) {
        ledFaultGap = true;
        ledNextActionMs = now + 125;
      } else {
        ledFaultGap = false;
        ledNextActionMs = now + 700;
        // A completed subsystem code is followed by the next active code.
        int next = findNextFaultSubsystem((ledFaultSubsystem + 1) % 3);
        if (next >= 0 && next != ledFaultSubsystem) {
          ledFaultSubsystem = (uint8_t)next;
          ledFaultFlashesRemaining = ledFaultSubsystem + 1;
          ledFaultGap = true;
        } else {
          ledFaultSubsystem = 0xFF;
        }
      }
    } else {
      if (ledFaultSubsystem == 0xFF) {
        ledPattern = LED_PATTERN_IDLE;
        ledFaultSubsystem = 0;
        ledNextActionMs = now;
      } else {
        setLedRed(32);
        ledFaultGap = false;
        ledNextActionMs = now + 85;
      }
    }
    return;
  }

  // IDLE: faults take priority, then a queued transition indication, then the
  // very short healthy heartbeat every ten seconds.
  int firstFault = findNextFaultSubsystem(0);
  if (firstFault >= 0) {
    startLedFaultPattern((uint8_t)firstFault);
    return;
  }

  if (ledTransitionPending) {
    startLedTransitionPattern();
    return;
  }

  if (now - ledLastHeartbeatMs >= LED_HEARTBEAT_INTERVAL_MS) {
    ledPattern = LED_PATTERN_HEARTBEAT;
    setLedRed(6);
    ledNextActionMs = now + LED_HEARTBEAT_PULSE_MS;
  }
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
    clearFault(FAULT_WIFI_AP, "Wi-Fi access point started successfully");
    addEvent("INFO", "Wi-Fi access point started");
    Serial.println(F("[Wi-Fi] AP: PASS"));
    Serial.print(F("[Wi-Fi] AP SSID: "));
    Serial.println(WIFI_AP_SSID);
    Serial.print(F("[Wi-Fi] AP IP:   "));
    Serial.println(WiFi.softAPIP());
  } else {
    setFault(FAULT_WIFI_AP, "WIFI", 1, "Wi-Fi access point failed to start");
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
  if (isFaultActive(FAULT_WIFI_STA)) {
    enqueueFaultEvent("WIFI", "RECOVERY_ATTEMPT", activeFaults[FAULT_WIFI_STA].code,
                      "Retrying infrastructure Wi-Fi connection");
  }
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

      clearFault(FAULT_WIFI_STA, "Infrastructure Wi-Fi connected");
      requestLedTransition();
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
    if (!wifiStaManuallyDisconnected) {
      setFault(FAULT_WIFI_STA, "WIFI", 2, "Configured infrastructure Wi-Fi disconnected");
    }

    Serial.println(F("[Wi-Fi] STA disconnected."));
    nextStaRetryMs = millis() + WIFI_RETRY_INTERVAL_MS;
  }

  if (wifiStaManuallyDisconnected || configuredStaSsid.length() == 0) {
    wifiStaConnecting = false;
    if (wifiStaManuallyDisconnected || configuredStaSsid.length() == 0) {
      clearFault(FAULT_WIFI_STA, "Infrastructure Wi-Fi is no longer expected to be connected");
    }
    return;
  }

  if (wifiStaConnecting) {
    if (millis() - staAttemptStartedMs >= WIFI_CONNECT_TIMEOUT_MS) {
      wifiStaConnecting = false;
      nextStaRetryMs = millis() + WIFI_RETRY_INTERVAL_MS;
      addEvent("WARN", "Infrastructure connection timed out");
      setFault(FAULT_WIFI_STA, "WIFI", 3, "Infrastructure Wi-Fi connection timed out");
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
  Serial.print(F("STA state:            "));
  if (wifiStaConnected) Serial.println(F("CONNECTED"));
  else if (wifiStaConnecting) Serial.println(F("CONNECTING"));
  else if (wifiStaManuallyDisconnected) Serial.println(F("MANUALLY DISCONNECTED"));
  else if (configuredStaSsid.length() == 0) Serial.println(F("NOT CONFIGURED"));
  else Serial.println(F("DISCONNECTED"));
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
    clearFault(FAULT_SD_MOUNT, "SD card mounted successfully");
    addEvent("INFO", "SD card mounted");
    Serial.println(F("[SD] Mount: PASS"));
    refreshSDInformation();

    if (!SD.exists(SD_TEST_DIRECTORY)) {
      if (SD.mkdir(SD_TEST_DIRECTORY)) {
        Serial.println(F("[SD] Created /diag directory."));
      } else {
        setFault(FAULT_SD_LOG, "SD", 2, "Could not create SD diagnostic directory");
        addEvent("WARN", "Could not create SD diagnostic directory");
        Serial.println(F("[SD] WARNING: Could not create /diag directory."));
      }
    }

    runSDTest();
  } else {
    sdReady = false;
    sdMountFailures++;
    sdLastResult = "Card mount failed";
    setFault(FAULT_SD_MOUNT, "SD", 1, "SD card mount failed");
    addEvent("ERROR", "SD mount failed");
    Serial.println(F("[SD] Mount: FAIL"));
    Serial.println(F("[SD] Check card insertion, format, and pinout."));
  }
}

void remountSDCard() {
  if (sdTestRunning) {
    Serial.println(F("[SD] Remount rejected: SD diagnostic test is running."));
    return;
  }

  Serial.println(F("[SD] Unmounting and retrying card initialization..."));
  if (isFaultActive(FAULT_SD_MOUNT)) {
    enqueueFaultEvent("SD", "RECOVERY_ATTEMPT", activeFaults[FAULT_SD_MOUNT].code,
                      "Manual SD remount requested");
  }
  if (isFaultActive(FAULT_SD_TEST)) {
    enqueueFaultEvent("SD", "RECOVERY_ATTEMPT", activeFaults[FAULT_SD_TEST].code,
                      "Manual SD verification retry requested");
  }
  if (isFaultActive(FAULT_SD_LOG)) {
    enqueueFaultEvent("SD", "RECOVERY_ATTEMPT", activeFaults[FAULT_SD_LOG].code,
                      "Manual SD fault-log recovery requested");
  }
  SD.end();
  sdReady = false;
  sdTestPassed = false;
  faultLogReady = false;
  sdCardCapacityBytes = 0;
  sdTotalBytes = 0;
  sdUsedBytes = 0;

  initializeSDCard();
  serviceFaultLog();
  Serial.println(sdReady && sdTestPassed ? F("[SD] Remount and verification: PASS")
                                         : F("[SD] Remount and verification: FAIL"));
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
           "SD-CANbus-Logger V04 - TEST %lu - WRITE PASS",
           (unsigned long)sdTestCount);
  snprintf(secondRecord, sizeof(secondRecord),
           "SD-CANbus-Logger V04 - TEST %lu - APPEND PASS",
           (unsigned long)sdTestCount);

  Serial.println();
  Serial.println(F("[SD TEST] Starting write/append/readback verification..."));

  auto failTest = [&](const char *reason) -> bool {
    sdTestPassed = false;
    sdLastResult = reason;
    sdTestDurationMs = millis() - testStartMs;
    sdTestRunning = false;
    setFault(FAULT_SD_TEST, "SD", 2, reason);
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
    clearFault(FAULT_SD_TEST, "SD write/append/readback test passed");
    requestLedTransition();
    addEvent("INFO", "SD write/append/readback test passed");
    Serial.println(F("[SD TEST] Initial write: PASS"));
    Serial.println(F("[SD TEST] Append:        PASS"));
    Serial.println(F("[SD TEST] Readback:      PASS"));
  } else {
    sdLastResult = "FAIL - readback content mismatch";
    setFault(FAULT_SD_TEST, "SD", 2, "SD readback content mismatch");
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
  server.on("/api/http-errors", HTTP_GET, handleHttpErrors);
  server.on("/api/loop-events", HTTP_GET, handleLoopEvents);
  server.on("/api/faults", HTTP_GET, handleFaults);
  server.on("/api/test", HTTP_GET, handleSDTest);
  server.on("/api/client-error", HTTP_POST, handleClientError);
  server.onNotFound(handleNotFound);
}

void sendHttpResponse(int statusCode, const char *contentType, const String &body) {
  server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate");
  server.sendHeader("Connection", "close");
  server.send(statusCode, contentType, body);
}

void recordHttp(const char *method, const char *route, uint32_t startUs, uint16_t statusCode) {
  uint32_t duration = micros() - startUs;

  httpRequests++;
  httpLastDurationUs = duration;
  if (duration > httpMaxDurationUs) httpMaxDurationUs = duration;

  snprintf(httpLastRoute, sizeof(httpLastRoute), "%s", route);
  httpLastStatusCode = statusCode;

  if (statusCode >= 400) {
    httpErrorResponses++;
    addHttpError(method, route, statusCode);
  }
}

void handleRoot() {
  uint32_t startUs = micros();
  httpRootRequests++;

  String page = htmlPage();
  sendHttpResponse(200, "text/html", page);
  recordHttp("GET", "/", startUs, 200);
}

void handleStatus() {
  uint32_t startUs = micros();
  httpStatusRequests++;

  refreshSDInformation();
  String json = makeStatusJson();
  sendHttpResponse(200, "application/json", json);
  recordHttp("GET", "/api/status", startUs, 200);
}

void handleEvents() {
  uint32_t startUs = micros();
  httpEventsRequests++;

  String json = makeEventsJson();
  sendHttpResponse(200, "application/json", json);
  recordHttp("GET", "/api/events", startUs, 200);
}

void handleSDTest() {
  uint32_t startUs = micros();
  httpTestRequests++;

  bool result = runSDTest();
  String json = makeStatusJson();
  uint16_t code = result ? 200 : 500;

  sendHttpResponse(code, "application/json", json);
  recordHttp("GET", "/api/test", startUs, code);
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
  recordHttp("POST", "/api/client-error", startUs, 204);
}

void handleNotFound() {
  uint32_t startUs = micros();
  httpNotFoundRequests++;

  String route = server.uri();
  String method = server.method() == HTTP_POST ? "POST" :
                  server.method() == HTTP_PUT ? "PUT" :
                  server.method() == HTTP_DELETE ? "DELETE" :
                  server.method() == HTTP_HEAD ? "HEAD" : "GET/OTHER";
  String response = "Not found: " + route;
  sendHttpResponse(404, "text/plain", response);
  recordHttp(method.c_str(), route.c_str(), startUs, 404);
}

void handleHttpErrors() {
  uint32_t startUs = micros();
  String json = makeHttpErrorsJson();
  sendHttpResponse(200, "application/json", json);
  recordHttp("GET", "/api/http-errors", startUs, 200);
}

void handleLoopEvents() {
  uint32_t startUs = micros();
  String json = makeLoopEventsJson();
  sendHttpResponse(200, "application/json", json);
  recordHttp("GET", "/api/loop-events", startUs, 200);
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
  if (!preferencesReady || !diagnosticPreferencesReady || !faultBootIdPersistent ||
      !mdnsStarted || sdWriteFailures || sdReadFailures || sdMountFailures ||
      httpErrorResponses || httpClientErrorReports || getActiveFaultCount() ||
      faultQueueDropCount || faultLogWriteFailures ||
      (configuredStaSsid.length() > 0 && !wifiStaConnected &&
       !wifiStaConnecting && !wifiStaManuallyDisconnected)) return "WARN";
  return "PASS";
}

String getSystemHealthDetails() {
  String details;
  details.reserve(420);
  if (!wifiApStarted) details += "Wi-Fi access point failed to start; ";
  if (!sdReady) details += "SD card is not mounted; ";
  if (!sdTestPassed) details += "Latest SD verification has not passed; ";
  if (!preferencesReady) details += "Network Preferences unavailable; ";
  if (!diagnosticPreferencesReady || !faultBootIdPersistent) details += "Persistent diagnostic boot ID unavailable; ";
  if (!mdnsStarted) details += "mDNS responder is not running; ";
  if (sdMountFailures) details += "SD mount failures=" + String(sdMountFailures) + "; ";
  if (sdWriteFailures) details += "SD write failures=" + String(sdWriteFailures) + "; ";
  if (sdReadFailures) details += "SD read failures=" + String(sdReadFailures) + "; ";
  if (httpErrorResponses) details += "HTTP 4xx/5xx responses=" + String(httpErrorResponses) + "; ";
  if (httpClientErrorReports) details += "Browser-reported fetch errors=" + String(httpClientErrorReports) + "; ";
  if (configuredStaSsid.length() > 0 && !wifiStaConnected && !wifiStaConnecting &&
      !wifiStaManuallyDisconnected) details += "Configured infrastructure Wi-Fi is disconnected; ";
  for (uint8_t i = 0; i < FAULT_ID_COUNT; i++) {
    if (activeFaults[i].active) {
      details += String(activeFaults[i].subsystem) + " fault: " +
                 String(activeFaults[i].description) + "; ";
    }
  }
  if (faultQueueDropCount) details += "Fault-event queue drops=" + String(faultQueueDropCount) + "; ";
  if (faultLogWriteFailures) details += "Fault-file write failures=" + String(faultLogWriteFailures) + "; ";
  if (details.length() == 0) return "No active health warnings.";
  details.remove(details.length() - 2);
  return details;
}

void printSystemHealthDetails() {
  Serial.print(F("Health detail:        "));
  Serial.println(getSystemHealthDetails());
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
  json += "\"healthDetails\":\"" + jsonEscape(getSystemHealthDetails()) + "\",";

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
  const char *staState = wifiStaConnected ? "CONNECTED" :
                         (wifiStaConnecting ? "CONNECTING" :
                         (wifiStaManuallyDisconnected ? "MANUALLY_DISCONNECTED" :
                         (configuredStaSsid.length() == 0 ? "NOT_CONFIGURED" : "DISCONNECTED")));
  json += ",\"staState\":\"" + String(staState) + "\",";
  json += "\"staConnecting\":" + String(wifiStaConnecting ? "true" : "false");
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
  json += "\"httpErrorHistoryCount\":" + String(httpErrorCount) + ",";

  json += "\"loopCount\":" + String(loopCount) + ",";
  json += "\"loopLastDurationUs\":" + String(loopLastDurationUs) + ",";
  json += "\"loopMaxDurationUs\":" + String(loopMaxDurationUs) + ",";
  json += "\"loopLastGapUs\":" + String(loopLastGapUs) + ",";
  json += "\"loopMaxGapUs\":" + String(loopMaxGapUs) + ",";
  json += "\"loopTimingEventCount\":" + String(loopTimingEventCount) + ",";
  json += "\"loopEventHistoryCount\":" + String(loopEventCount) + ",";
  json += "\"eventCount\":" + String(eventCount) + ",";
  json += "\"bootCheckpointCount\":" + String(bootCheckpointCount) + ",";
  json += "\"faultActiveCount\":" + String(getActiveFaultCount()) + ",";
  json += "\"faultQueueCount\":" + String(faultQueueCount) + ",";
  json += "\"faultQueueDrops\":" + String(faultQueueDropCount) + ",";
  json += "\"faultLogReady\":" + String(faultLogReady ? "true" : "false") + ",";
  json += "\"faultLogWriteFailures\":" + String(faultLogWriteFailures) + ",";
  json += "\"ledState\":\"" + String(getLedPatternText()) + "\",";
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

String makeHttpErrorsJson() {
  String json;
  json.reserve(1200);
  json = "{\"errors\":[";
  uint8_t first = (httpErrorHead + HTTP_ERROR_HISTORY_SIZE - httpErrorCount) % HTTP_ERROR_HISTORY_SIZE;
  for (uint8_t i = 0; i < httpErrorCount; i++) {
    const HttpErrorRecord &r = httpErrorHistory[(first + i) % HTTP_ERROR_HISTORY_SIZE];
    if (i) json += ",";
    json += "{\"uptimeMs\":" + String(r.uptimeMs);
    json += ",\"uptime\":\"" + formatUptime(r.uptimeMs) + "\"";
    json += ",\"method\":\"" + String(r.method) + "\"";
    json += ",\"uri\":\"" + jsonEscape(String(r.uri)) + "\"";
    json += ",\"status\":" + String(r.statusCode) + "}";
  }
  json += "]}";
  return json;
}

String makeLoopEventsJson() {
  String json;
  json.reserve(1500);
  json = "{\"events\":[";
  uint8_t first = (loopEventHead + LOOP_EVENT_HISTORY_SIZE - loopEventCount) % LOOP_EVENT_HISTORY_SIZE;
  for (uint8_t i = 0; i < loopEventCount; i++) {
    const LoopTimingRecord &r = loopEventHistory[(first + i) % LOOP_EVENT_HISTORY_SIZE];
    if (i) json += ",";
    json += "{\"uptimeMs\":" + String(r.uptimeMs);
    json += ",\"uptime\":\"" + formatUptime(r.uptimeMs) + "\"";
    json += ",\"gapUs\":" + String(r.gapUs);
    json += ",\"durationUs\":" + String(r.durationUs);
    json += ",\"httpStatus\":" + String(r.relatedHttpStatus);
    json += ",\"httpRoute\":\"" + jsonEscape(String(r.relatedHttpRoute)) + "\"}";
  }
  json += "]}";
  return json;
}

// -----------------------------------------------------------------------------
// Browser dashboard
// -----------------------------------------------------------------------------

String htmlPage() {
  String page;
  page.reserve(18000);

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
<p class="sub">System dashboard - V04</p>

<div class="grid">
  <section class="card">
    <h2>System health</h2>
    <div class="row"><span>Overall</span><span class="health" id="health">--</span></div>
    <div class="row"><span>Assessment</span><span class="value" id="healthDetails">--</span></div>
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
    <h2>Fault diagnostics</h2>
    <div class="row"><span>Active faults</span><span class="value" id="faultActiveCount">--</span></div>
    <div class="row"><span>LED indication</span><span class="value" id="ledState">--</span></div>
    <div class="row"><span>Fault log</span><span class="value" id="faultLogReady">--</span></div>
    <div class="row"><span>Queued events / drops</span><span class="value" id="faultQueue">--</span></div>
    <div id="activeFaults" class="small muted">Loading active faults...</div>
    <div id="faultEvents" class="small muted">Loading fault history...</div>
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
    <div class="row"><span>Browser-reported fetch errors</span><span class="value" id="httpClientErrorReports">--</span></div>
    <div class="row"><span>HTTP error responses</span><span class="value" id="httpErrorResponses">--</span></div>
    <div class="row"><span>Last route / status</span><span class="value" id="httpLastRoute">--</span></div>
    <div class="row"><span>Last / max handler time</span><span class="value" id="httpTiming">--</span></div>
    <div id="httpErrors" class="small muted">No HTTP error history loaded.</div>
  </section>

  <section class="card">
    <h2>Main-loop diagnostics</h2>
    <div class="row"><span>Loop iterations</span><span class="value" id="loopCount">--</span></div>
    <div class="row"><span>Last / max loop duration</span><span class="value" id="loopDuration">--</span></div>
    <div class="row"><span>Last / max loop gap</span><span class="value" id="loopGap">--</span></div>
    <div class="row"><span>Timing events captured</span><span class="value" id="loopTimingEvents">--</span></div>
    <div id="loopEvents" class="small muted">No loop timing history loaded.</div>
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

<p class="small muted">Status refresh interval: 5 seconds. Loop timing events are retained when a loop gap or duration reaches 100 ms. CAN/TWAI is intentionally inactive in V04.</p>
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
    setText('healthDetails',s.healthDetails);
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
    setText('staConnected',s.staState|| (s.staConnected?'CONNECTED':'DISCONNECTED'));
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
    setText('loopTimingEvents',s.loopTimingEventCount);
    setText('loopCount',s.loopCount);
    setText('loopDuration',s.loopLastDurationUs+' / '+s.loopMaxDurationUs+' us');
    setText('loopGap',s.loopLastGapUs+' / '+s.loopMaxGapUs+' us');
    setText('eventCount',s.eventCount);
    setText('faultActiveCount',s.faultActiveCount);
    setText('ledState',s.ledState);
    setText('faultLogReady',s.faultLogReady?'READY':'NOT READY');
    setText('faultQueue',s.faultQueueCount+' / '+s.faultQueueDrops);
    setText('message','');
  }catch(e){
    setText('message','Status refresh failed: '+e.message);
    reportBrowserError('/api/status',e);
  }
}
async function refreshFaults(){
  try{
    const data=await fetchJson('/api/faults');
    const active=document.getElementById('activeFaults');
    if(!data.active.length){active.textContent='No active subsystem faults.';}
    else{active.textContent=data.active.map(f=>f.subsystem+' code '+f.code+': '+f.description).join('\n');active.style.whiteSpace='pre-wrap';}
    const history=document.getElementById('faultEvents');
    if(!data.events.length){history.textContent='No fault/recovery events retained.';}
    else{history.textContent=data.events.map(e=>e.timestampUs+' us  '+e.subsystem+' '+e.event+' ['+e.code+'] '+e.description).join('\n');history.style.whiteSpace='pre-wrap';}
    setText('faultActiveCount',data.activeCount);
    setText('ledState',data.ledState);
    setText('faultLogReady',data.logReady?'READY':'NOT READY');
    setText('faultQueue',data.queueCount+' / '+data.queueDrops);
  }catch(e){document.getElementById('faultEvents').textContent='Fault history unavailable: '+e.message;}
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
async function refreshHttpErrors(){
  try{
    const data=await fetchJson('/api/http-errors');
    const host=document.getElementById('httpErrors');
    if(!data.errors.length){host.textContent='No HTTP error responses recorded.';return;}
    host.textContent=data.errors.map(e=>e.uptime+'  '+e.method+' '+e.uri+'  HTTP '+e.status).join('\n');
    host.style.whiteSpace='pre-wrap';
  }catch(e){document.getElementById('httpErrors').textContent='HTTP history unavailable: '+e.message;}
}
async function refreshLoopEvents(){
  try{
    const data=await fetchJson('/api/loop-events');
    const host=document.getElementById('loopEvents');
    if(!data.events.length){host.textContent='No loop timing events recorded.';return;}
    host.textContent=data.events.map(e=>e.uptime+'  gap '+e.gapUs+' us; duration '+e.durationUs+' us; HTTP '+e.httpRoute+' / '+e.httpStatus).join('\n');
    host.style.whiteSpace='pre-wrap';
  }catch(e){document.getElementById('loopEvents').textContent='Loop history unavailable: '+e.message;}
}
refreshStatus();
refreshFaults();
refreshEvents();
refreshHttpErrors();
refreshLoopEvents();
setInterval(refreshStatus,5000);
setInterval(refreshFaults,15000);
setInterval(refreshEvents,15000);
setInterval(refreshHttpErrors,15000);
setInterval(refreshLoopEvents,15000);
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
        if (serialCommandLength > 0) {
          printCommandEcho(serialCommandBuffer);
          processSerialCommand(serialCommandBuffer);
        }
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

void printCommandEcho(const char *command) {
  Serial.print(F("> "));
  if (!strncmp(command, "wifi connect ", 13)) {
    const char *credentials = command + 13;
    const char *separator = strchr(credentials, '|');
    if (separator) {
      Serial.print(F("wifi connect "));
      for (const char *p = credentials; p < separator; ++p) Serial.write(*p);
      Serial.println(F("|[PASSWORD HIDDEN]"));
    } else {
      Serial.println(F("wifi connect [CREDENTIALS HIDDEN]"));
    }
  } else {
    Serial.println(command);
  }
}

void processSerialCommand(char *command) {
  if (!strcmp(command, "help")) {
    printHelp();
  } else if (!strcmp(command, "status")) {
    printStatus();
  } else if (!strcmp(command, "events")) {
    printEvents();
  } else if (!strcmp(command, "faults")) {
    printFaults();
  } else if (!strcmp(command, "faults clear")) {
    faultHistoryHead = 0;
    faultHistoryCount = 0;
    Serial.println(F("[FAULTS] RAM event history cleared. Active faults, queued SD records, and /diag/FAULTS.CSV preserved."));
  } else if (!strcmp(command, "httperrors")) {
    printHttpErrors();
  } else if (!strcmp(command, "loopevents")) {
    printLoopEvents();
  } else if (!strcmp(command, "checkpoints")) {
    printBootCheckpoints();
  } else if (!strcmp(command, "sdtest")) {
    runSDTest();
  } else if (!strcmp(command, "sd remount")) {
    remountSDCard();
  } else if (!strcmp(command, "wifi status")) {
    printWiFiStatus();
  } else if (!strcmp(command, "wifi disconnect")) {
    WiFi.disconnect(false, false);
    wifiStaManuallyDisconnected = true;
    wifiStaConnecting = false;
    wifiStaConnected = false;
    wifiStaWasConnected = false;
    nextStaRetryMs = 0;
    clearFault(FAULT_WIFI_STA, "Infrastructure Wi-Fi manually disconnected by user");
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
    clearFault(FAULT_WIFI_STA, "Infrastructure Wi-Fi credentials removed by user");
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
    Serial.println(F("Unknown command."));
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
  Serial.println(F("faults                       Active faults and recent fault/recovery events"));
  Serial.println(F("faults clear                 Clear RAM history; preserve active/pending/persistent faults"));
  Serial.println(F("httperrors                   Recent HTTP 4xx/5xx requests"));
  Serial.println(F("loopevents                   Recent loop timing anomalies (>=100 ms)"));
  Serial.println(F("checkpoints                  Boot memory checkpoints"));
  Serial.println(F("sdtest                       Run SD write/append/readback test"));
  Serial.println(F("sd remount                    Unmount, remount, and verify the SD card"));
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
  printSystemHealthDetails();

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
  Serial.print(F("HTTP error history:   ")); Serial.print(httpErrorCount); Serial.print(F(" retained; ")); Serial.println(httpErrorResponses);

  Serial.println();
  Serial.println(F("MAIN LOOP"));
  Serial.println(F("---------"));
  Serial.print(F("Loop iterations:      ")); Serial.println(loopCount);
  Serial.print(F("Last/max duration:    ")); Serial.print(loopLastDurationUs); Serial.print(F(" / ")); Serial.print(loopMaxDurationUs); Serial.println(F(" us"));
  Serial.print(F("Last/max loop gap:    ")); Serial.print(loopLastGapUs); Serial.print(F(" / ")); Serial.print(loopMaxGapUs); Serial.println(F(" us"));
  Serial.print(F("Loop timing events:   ")); Serial.print(loopTimingEventCount); Serial.print(F(" total; ")); Serial.print(loopEventCount); Serial.println(F(" retained"));

  Serial.println();
  Serial.print(F("Diagnostic events:    ")); Serial.println(eventCount);
  Serial.print(F("Boot checkpoints:     ")); Serial.println(bootCheckpointCount);
  Serial.print(F("Active faults:        ")); Serial.println(getActiveFaultCount());
  for (uint8_t i = 0; i < FAULT_ID_COUNT; i++) {
    if (activeFaults[i].active) {
      Serial.print(F("  ")); Serial.print(activeFaults[i].subsystem);
      Serial.print(F(": ")); Serial.println(activeFaults[i].description);
    }
  }
  Serial.print(F("Fault event queue:    ")); Serial.print(faultQueueCount); Serial.print(F(" queued; ")); Serial.print(faultQueueDropCount); Serial.println(F(" dropped"));
 Serial.print(F("Fault file:           "));
if (faultLogReady) {
  Serial.println(FAULT_LOG_FILE);
} else {
  Serial.println(F("NOT READY"));
}
  Serial.print(F("Fault file writes:    ")); Serial.print(faultLogWriteCount); Serial.print(F(" pass; ")); Serial.println(faultLogWriteFailures);
  if (faultHistoryCount) {
    uint8_t lastIndex = (faultHistoryHead + FAULT_HISTORY_SIZE - 1) % FAULT_HISTORY_SIZE;
    const FaultEventRecord &lastFaultEvent = faultEventHistory[lastIndex];
    Serial.print(F("Last fault event:     "));
    Serial.print(lastFaultEvent.subsystem); Serial.print(' ');
    Serial.print(lastFaultEvent.event); Serial.print(F(" - "));
    Serial.println(lastFaultEvent.description);
  } else {
    Serial.println(F("Last fault event:     None retained"));
  }
  Serial.print(F("Diagnostic LED:       ")); Serial.print(getLedPatternText()); Serial.print(F(" on GPIO")); Serial.println(RGB_LED_DATA_PIN);
  Serial.println(F("CAN/TWAI:             NOT INITIALIZED"));
  Serial.println();
}
