/*
  SD-CANbus-Logger.ino
  Firmware: V10

  Project: SD-CANbus-Logger
  Target: ESP32-D0WD-V3-based CAN logger hardware
  Framework target: Arduino-ESP32 3.3.x / ESP-IDF 5.x

  V10 purpose:
    - Release the diagnostic test lock after LED operator confirmation.
    - Exercise red, green, and blue LED channels in a sequential visual test.
    - Add a common user-initiated hardware diagnostic test registry and result model.
    - Expose the same test services through serial commands and browser APIs/UI.
    - Provide LED visual confirmation, SD read/write verification, CAN activity
      observation, and AP/STA Wi-Fi checks.
    - Organize the browser interface into focused pages with page-aware polling.
    - Shorten the RGB diagnostic sequence and identify the network test as Wi-Fi.
    - Finalize an active CAN capture before a controlled software reset.
    - Keep tests bounded and prevent SD diagnostics from interfering with CAN capture.

  V08c purpose (retained in V08e):
    - Cache immutable device/build information at startup; remove expensive
      device-information queries from recurring status requests.
    - Cache SD card/filesystem metadata on mount and after SD mutations or
      capture finalization rather than refreshing it on every status request.
    - Add a bounded, versioned RAM diagnostic recorder independent of browser navigation.
    - Record periodic subsystem samples and significant diagnostic events.
    - Provide serial and browser controls, status, and CSV export.
    - Make the loop timing event threshold configurable.
    - Preserve the V07 CAN acquisition and binary capture architecture.

  V07 foundation:
    - Preserve the V05 Wi-Fi, SD, HTTP, and diagnostic foundation.
    - Distinguish Wi-Fi CONNECTING from an actual disconnected fault.
    - Drive the onboard WS2812-compatible RGB LED on GPIO4 using the
      Arduino-ESP32 RGB LED/RMT support.
    - Provide a subtle 10-second healthy heartbeat and subsystem fault codes.
    - Maintain bounded fault-event history and a bounded SD-write queue.
    - Append fault and recovery events to /diag/FAULTS.CSV.
    - Initialize native ESP32 TWAI at 250 kbit/s in listen-only mode.
    - Receive CAN frames in a dedicated high-priority FreeRTOS task.
    - Timestamp frames, count traffic, retain a bounded recent-frame history,
      and expose TWAI queue/error instrumentation.
    - Provide controlled software reset and read-only SD file inspection.

  SD pin assignments:
    CS   = GPIO13
    SCK  = GPIO14
    MOSI = GPIO15
    MISO = GPIO2

  CAN pin assignments:
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
    diag list|status
    diag run <test>
    diag confirm pass|fail
    verbose status|on|off|freeze|resume|clear
    verbose interval <ms>
    diag threshold [ms]
    sdtest
    sd remount
    reset
    ls [path]
    cat <path>
    head <path> [lines]
    tail <path> [lines]
    wifi status
    wifi connect [SSID|PASSWORD]
    wifi scan
    wifi disconnect
    wifi forget

  Wi-Fi:
    AP SSID: SD-CANbus-Logger
    AP password: CANlogger01
    AP address: 192.168.4.1
    mDNS: http://sd-canbus-logger.local/
    Infrastructure credentials are entered over USB serial and saved in NVS.

  Credential command syntax:
    wifi connect
    Reconnect using the saved infrastructure credentials.
    wifi connect MyNetwork|MyPassword
    The pipe character separates SSID and password and cannot be part of either.
    SSID must be 1-32 characters; password must be 8-63 characters, or empty
    for an open network. Passwords are never printed in diagnostics.

  Browser endpoints:
    /                 Dashboard
    /api/status       System diagnostic snapshot
    /api/can-frames   Recent CAN frame history
    /api/logging      CAN logging status
    /api/logging/start Start a CAN capture
    /api/logging/stop  Stop and finalize a CAN capture
    /api/events       Recent diagnostic events
    /api/http-errors  Recent HTTP 4xx/5xx requests
    /api/loop-events  Recent loop timing anomalies
    /api/diagnostics  Hardware diagnostic test results
    /api/diagnostics/status  Hardware diagnostic test results (explicit status route)
    /api/diagnostics/run  Start a hardware diagnostic (POST test=<id>)
    /api/diagnostics/confirm Confirm LED visual test (POST result=pass|fail)
    /api/verbose      Verbose recorder status and controls
    /download-verbose Download RAM diagnostic records as CSV
    /api/faults       Active faults and recent fault/recovery events
    /api/files         List files in an approved SD directory
    /api/file          View an SD file as text
    /download          Download an SD file
    /api/reset         Request a controlled software restart
    /api/test         Run SD write/append/readback verification
    /api/client-error Browser-side fetch error telemetry

  Notes:
    - Remove the microSD card while flashing if required by the board.
    - Select the appropriate ESP32 Dev Module board definition.
    - CAN/TWAI starts at 250 kbit/s in listen-only mode. A dedicated task drains
      the driver RX queue and stores a bounded in-RAM frame history.
    - The CAN logger writes received frames to versioned binary files using a bounded
      FreeRTOS queue; the CAN receive task never performs SD operations.
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
#include <driver/twai.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

// -----------------------------------------------------------------------------
// Firmware identity and project configuration
// -----------------------------------------------------------------------------

static const char PROJECT_NAME[] = "SD-CANbus-Logger";
static const char FIRMWARE_VERSION[] = "V10";
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

static const int CAN_RX_PIN = 26;
static const int CAN_TX_PIN = 27;
static const uint32_t CAN_BITRATE = 250000;
static const uint16_t CAN_RX_DRIVER_QUEUE_LENGTH = 64;
static const uint8_t CAN_FRAME_HISTORY_SIZE = 32;
static const uint16_t CAN_LOG_QUEUE_LENGTH = 512;
static const uint8_t CAN_LOG_BATCH_RECORDS = 32;
static const uint16_t CAN_LOG_FORMAT_VERSION = 1;
static const uint16_t CAN_RX_TASK_STACK_BYTES = 4096;
static const UBaseType_t CAN_RX_TASK_PRIORITY = 3;
static const BaseType_t CAN_RX_TASK_CORE = 1;

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
static uint32_t loopDiagnosticThresholdMs = 100; // Default 100 ms; configurable 10-1000 ms

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

struct CanFrameRecord {
  uint64_t timestampUs;       // esp_timer_get_time(): microseconds since boot.
  uint32_t identifier;
  uint8_t dataLengthCode;
  uint8_t data[8];
  bool extended;
  bool remote;
};

// On-card binary format. All fields are written in the ESP32's little-endian
// representation. Header and record sizes are fixed and versioned.
#pragma pack(push, 1)
struct CanLogFileHeader {
  char magic[4];              // "SCNL"
  uint16_t formatVersion;
  uint16_t headerSize;
  uint16_t recordSize;
  uint16_t reserved;
  uint32_t bitrate;
  uint32_t bootId;
  uint64_t startTimestampUs;
  uint32_t reserved2;
};
struct CanLogRecord {
  uint64_t timestampUs;
  uint32_t identifier;
  uint8_t flags;              // bit 0 extended, bit 1 remote
  uint8_t dlc;
  uint8_t data[8];
};
#pragma pack(pop)
static_assert(sizeof(CanLogFileHeader) == 32, "CAN log header size changed");
static_assert(sizeof(CanLogRecord) == 22, "CAN log record size changed");

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
// V08e cached system information and V08c verbose diagnostic recorder
// -----------------------------------------------------------------------------

static const uint16_t DIAG_RECORD_CAPACITY = 512;
static const uint16_t DIAG_RECORD_FORMAT_VERSION = 1;
static const uint32_t DIAG_DEFAULT_SAMPLE_INTERVAL_MS = 1000;
static const uint32_t DIAG_SLOW_SAMPLE_INTERVAL_MS = 5000;
static const uint32_t DIAG_MIN_SAMPLE_INTERVAL_MS = 250;
static const uint32_t DIAG_MAX_SAMPLE_INTERVAL_MS = 10000;
static const uint32_t DIAG_MIN_LOOP_THRESHOLD_MS = 10;
static const uint32_t DIAG_MAX_LOOP_THRESHOLD_MS = 1000;

enum DiagnosticRecordType : uint8_t {
  DIAG_TYPE_U32 = 1,
  DIAG_TYPE_I32 = 2,
  DIAG_TYPE_BOOL = 3,
  DIAG_TYPE_EVENT = 4
};

enum DiagnosticRecordId : uint16_t {
  DIAG_ID_BOOT = 1,
  DIAG_ID_EVENT_HASH,
  DIAG_ID_FAULT_STATE,
  DIAG_ID_LOOP_GAP_US,
  DIAG_ID_LOOP_DURATION_US,
  DIAG_ID_HTTP_HANDLER_US,
  DIAG_ID_HTTP_ROUTE_HASH,
  DIAG_ID_HEAP_FREE_BYTES,
  DIAG_ID_HEAP_LARGEST_BYTES,
  DIAG_ID_LOOP_LAST_GAP_US,
  DIAG_ID_LOOP_LAST_DURATION_US,
  DIAG_ID_WIFI_RSSI_DBM,
  DIAG_ID_CAN_FRAMES_RECEIVED,
  DIAG_ID_CAN_RECEIVE_ERRORS,
  DIAG_ID_CAN_RX_QUEUE_DEPTH,
  DIAG_ID_CAN_LOG_QUEUE_DEPTH,
  DIAG_ID_CAN_LOG_QUEUE_DROPS,
  DIAG_ID_SD_WRITE_FAILURES,
  DIAG_ID_SD_READ_FAILURES,
  DIAG_ID_SD_WRITE_DURATION_MS,
  DIAG_ID_SD_READ_DURATION_MS,
  DIAG_ID_CAN_LOG_WRITE_MS,
  DIAG_ID_CAN_LOG_BATCH_RECORDS,
  DIAG_ID_ACTIVE_FAULT_COUNT,
  DIAG_ID_HEAP_MIN_FREE_BYTES,
  DIAG_ID_HEAP_INTERNAL_FREE_BYTES,
  DIAG_ID_WIFI_AP_CLIENTS,
  DIAG_ID_WIFI_STA_STATE,
  DIAG_ID_WIFI_DISCONNECTS,
  DIAG_ID_SD_MOUNTED,
  DIAG_ID_SD_MOUNT_FAILURES,
  DIAG_ID_SD_WRITE_OPERATIONS,
  DIAG_ID_SD_READ_OPERATIONS,
  DIAG_ID_SD_LAST_WRITE_MS,
  DIAG_ID_SD_LAST_READ_MS,
  DIAG_ID_HTTP_REQUESTS,
  DIAG_ID_HTTP_ERRORS,
  DIAG_ID_HTTP_CLIENT_ERRORS,
  DIAG_ID_LOOP_ITERATIONS,
  DIAG_ID_LOOP_MAX_DURATION_US,
  DIAG_ID_LOOP_MAX_GAP_US,
  DIAG_ID_CAN_BUS_OFF_COUNT,
  DIAG_ID_CAN_RECOVERY_COUNT,
  DIAG_ID_CAN_BUS_ERRORS,
  DIAG_ID_CAN_DRIVER_MISSED,
  DIAG_ID_CAN_LOG_STATE,
  DIAG_ID_CAN_LOG_FRAMES_WRITTEN,
  DIAG_ID_CAN_LOG_WRITE_FAILURES,
  DIAG_ID_CAN_LOG_LAST_WRITE_MS,
  DIAG_ID_FAULT_QUEUE_DROPS,
  DIAG_ID_STATUS_SD_REFRESH_US,
  DIAG_ID_STATUS_SD_CARD_TYPE_US,
  DIAG_ID_STATUS_SD_CARD_SIZE_US,
  DIAG_ID_STATUS_SD_TOTAL_BYTES_US,
  DIAG_ID_STATUS_SD_USED_BYTES_US,
  DIAG_ID_STATUS_SYSTEM_MEMORY_US,
  DIAG_ID_STATUS_WIFI_US,
  DIAG_ID_STATUS_SD_JSON_US,
  DIAG_ID_STATUS_HTTP_LOOP_FAULT_US,
  DIAG_ID_STATUS_CAN_SNAPSHOT_US,
  DIAG_ID_STATUS_CAN_JSON_US,
  DIAG_ID_STATUS_JSON_TOTAL_US,
  DIAG_ID_STATUS_RESPONSE_SEND_US,
  DIAG_ID_STATUS_SYS_HEAP_US,
  DIAG_ID_STATUS_SYS_DEVICE_US,
  DIAG_ID_STATUS_SYS_RUNTIME_US,
  DIAG_ID_STATUS_SYS_HEALTH_US,
  DIAG_ID_STATUS_SYS_HEALTH_DETAILS_US,
  DIAG_ID_STATUS_SYS_OTHER_US,
  DIAG_ID_STATUS_SYS_CHIP_MODEL_US,
  DIAG_ID_STATUS_SYS_CHIP_REVISION_US,
  DIAG_ID_STATUS_SYS_CPU_FREQ_US,
  DIAG_ID_STATUS_SYS_FLASH_SIZE_US,
  DIAG_ID_STATUS_SYS_SKETCH_SIZE_US,
  DIAG_ID_STATUS_SYS_FREE_SKETCH_US
};

#pragma pack(push, 1)
struct DiagnosticRecord {
  uint64_t timestampUs;
  uint16_t recordId;
  uint8_t dataType;
  uint8_t flags;
  uint32_t value;
  uint32_t context;
};
#pragma pack(pop)
static_assert(sizeof(DiagnosticRecord) == 20, "Diagnostic record size changed");

DiagnosticRecord diagnosticRecords[DIAG_RECORD_CAPACITY];
uint16_t diagnosticRecordHead = 0;
uint16_t diagnosticRecordCount = 0;
uint32_t diagnosticRecordsTotal = 0;
uint32_t diagnosticRecordsOverwritten = 0;
uint32_t diagnosticClearCount = 0;
uint32_t diagnosticSampleIntervalMs = DIAG_DEFAULT_SAMPLE_INTERVAL_MS;
uint32_t diagnosticLastSampleMs = 0;
uint32_t diagnosticLastSlowSampleMs = 0;
uint32_t diagnosticLastRecordCostUs = 0;
uint32_t diagnosticMaxRecordCostUs = 0;
uint32_t diagnosticLastSampleCostUs = 0;
uint32_t diagnosticMaxSampleCostUs = 0;
bool verboseLoggingEnabled = false;
bool verboseLoggingFrozen = false;
portMUX_TYPE diagnosticMux = portMUX_INITIALIZER_UNLOCKED;


// -----------------------------------------------------------------------------
// V10 hardware diagnostic test framework
// -----------------------------------------------------------------------------

enum DiagnosticTestId : int8_t {
  DIAG_TEST_LED = 0,
  DIAG_TEST_SD,
  DIAG_TEST_CAN,
  DIAG_TEST_WIFI,
  DIAG_TEST_COUNT
};

struct DiagnosticTestResult {
  const char *id;
  const char *name;
  char state[24];       // NOT TESTED, RUNNING, AWAITING_CONFIRMATION, COMPLETE
  char result[16];      // NOT TESTED, PASS, WARN, FAIL
  char detail[112];
  uint32_t lastRunMs;
  uint32_t durationMs;
};

DiagnosticTestResult diagnosticTests[DIAG_TEST_COUNT] = {
  {"led", "LED RGB visual check", "NOT TESTED", "NOT TESTED", "Not run", 0, 0},
  {"sd", "SD write/readback", "NOT TESTED", "NOT TESTED", "Not run", 0, 0},
  {"can", "CAN activity", "NOT TESTED", "NOT TESTED", "Not run", 0, 0},
  {"wifi", "Wi-Fi AP/STA status", "NOT TESTED", "NOT TESTED", "Not run", 0, 0}
};

int8_t diagnosticTestActive = -1;
uint32_t diagnosticTestStartedMs = 0;
uint32_t diagnosticCanStartFrames = 0;
uint32_t diagnosticCanStartMs = 0;
bool diagnosticLedActive = false;
bool diagnosticLedAwaitingConfirmation = false;
uint8_t diagnosticLedPhase = 0; // 0-17: three pulses each for red, green, blue
uint32_t diagnosticLedNextMs = 0;
static const uint32_t DIAGNOSTIC_LED_PHASE_MS = 100;

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
bool wifiScanInProgress = false;

bool sdReady = false;
bool sdTestPassed = false;
bool sdTestRunning = false;
bool sdOperationActive = false;
uint32_t sdFileListRequests = 0;
uint32_t sdFileReadRequests = 0;
uint32_t sdFileDownloadRequests = 0;
uint32_t sdFileOperationFailures = 0;

bool canDriverInstalled = false;
bool canInitialized = false;
QueueHandle_t canLogQueue = nullptr;
File canLogFile;
volatile bool canLoggingActive = false;
uint32_t canLogEnqueueInFlight = 0; // Protected by canMux.
bool canLoggingStopping = false;
bool canLogFileOpen = false;
char canLogCurrentPath[40] = "";
uint32_t canLogFileSequence = 0;
uint32_t canLogFilesCreated = 0;
uint32_t canLogFramesQueued = 0;
uint32_t canLogFramesWritten = 0;
uint32_t canLogQueueDrops = 0;
uint32_t canLogWriteFailures = 0;
uint32_t canLogWriteOperations = 0;
uint32_t canLogLastWriteMs = 0;
uint32_t canLogMaxWriteMs = 0;
uint32_t canLogLastBatchRecords = 0;
uint32_t canLogQueueHighWater = 0;
uint32_t canLogStartTimestampUs = 0;
uint32_t canLogLastFlushMs = 0;
bool canRecoveryRequested = false;
bool canRxTaskStarted = false;
bool canRxTaskRunning = false;
TaskHandle_t canRxTaskHandle = nullptr;
String canStateText = "NOT INITIALIZED";
uint32_t canInitAttempts = 0;
uint32_t canInitFailures = 0;
uint32_t canBusOffCount = 0;
bool canBusOffLatched = false;
uint32_t canRecoveryCount = 0;
uint32_t canRecoveryAttempts = 0;
uint32_t canRxQueueFullCount = 0;
uint32_t canReceiveErrors = 0;
uint32_t canFramesReceived = 0;
uint32_t canStandardFrames = 0;
uint32_t canExtendedFrames = 0;
uint32_t canRemoteFrames = 0;
uint32_t canHistoryOverwrites = 0;
uint32_t canDriverRxQueueDepth = 0;
uint32_t canDriverRxQueueMaxDepth = 0;
uint32_t canDriverRxMissedCount = 0;
uint32_t canDriverRxOverrunCount = 0;
uint32_t canDriverRxErrorCounter = 0;
uint32_t canDriverTxErrorCounter = 0;
uint32_t canDriverBusErrorCount = 0;
uint32_t canDriverArbitrationLostCount = 0;
uint32_t canDriverTxFailedCount = 0;
uint64_t canLastFrameTimestampUs = 0;
uint32_t canLastError = 0;
uint32_t canLastStatePollMs = 0;
CanFrameRecord canFrameHistory[CAN_FRAME_HISTORY_SIZE];
uint8_t canFrameHistoryHead = 0;
uint8_t canFrameHistoryCount = 0;
portMUX_TYPE canMux = portMUX_INITIALIZER_UNLOCKED;

String configuredStaSsid;
String configuredStaPassword;  // Never printed or returned by an endpoint.

String sdLastResult = "Not run";
String sdCardTypeText = "Unknown";
uint64_t sdCardCapacityBytes = 0;
uint64_t sdTotalBytes = 0;
uint64_t sdUsedBytes = 0;

// Immutable or boot-stable system information. Initialized once during setup.
struct SystemInfoCache {
  bool initialized = false;
  String arduinoCoreVersion;
  String espIdfVersion;
  String chipModel;
  String resetReason;
  String apMac;
  String staMac;
  uint32_t chipRevision = 0;
  uint32_t cpuMHz = 0;
  uint32_t flashChipSize = 0;
  uint32_t sketchSize = 0;
  uint32_t freeSketchSpace = 0;
};
SystemInfoCache systemInfo;

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
char httpMaxRoute[80] = "none";
uint16_t httpMaxStatusCode = 0;

uint32_t loopCount = 0;
uint32_t loopLastDurationUs = 0;
uint32_t loopMaxDurationUs = 0;
uint32_t loopLastGapUs = 0;
uint32_t loopMaxGapUs = 0;
uint32_t loopPreviousStartUs = 0;

char serialCommandBuffer[160];
size_t serialCommandLength = 0;
bool serialCommandOverflow = false;

// Per-request timing breakdown for GET /api/status. Values are microseconds.
struct StatusJsonTimings {
  uint32_t sdRefreshUs = 0;
  uint32_t sdCardTypeUs = 0;
  uint32_t sdCardSizeUs = 0;
  uint32_t sdTotalBytesUs = 0;
  uint32_t sdUsedBytesUs = 0;
  uint32_t systemMemoryUs = 0;
  uint32_t wifiUs = 0;
  uint32_t sdJsonUs = 0;
  uint32_t httpLoopFaultUs = 0;
  uint32_t canSnapshotUs = 0;
  uint32_t canJsonUs = 0;
  uint32_t jsonTotalUs = 0;
  uint32_t responseSendUs = 0;
  uint32_t systemHeapUs = 0;
  uint32_t systemDeviceUs = 0;
  uint32_t systemRuntimeUs = 0;
  uint32_t systemHealthUs = 0;
  uint32_t systemHealthDetailsUs = 0;
  uint32_t systemOtherUs = 0;
  uint32_t systemChipModelUs = 0;
  uint32_t systemChipRevisionUs = 0;
  uint32_t systemCpuFreqUs = 0;
  uint32_t systemFlashSizeUs = 0;
  uint32_t systemSketchSizeUs = 0;
  uint32_t systemFreeSketchUs = 0;
};

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
void recordDiagnostic(uint16_t recordId, uint8_t dataType, uint32_t value,
                      uint32_t context = 0, uint8_t flags = 0);
uint32_t diagnosticStringHash(const char *text);
const char *diagnosticRecordName(uint16_t recordId);
const char *diagnosticRecordTypeName(uint8_t dataType);
const char *diagnosticRecordDetail(const DiagnosticRecord &record);
void serviceVerboseDiagnostics();
void printVerboseStatus();
void setVerboseLogging(bool enabled);
void clearVerboseRecords();
String makeVerboseStatusJson();
void handleVerboseStatus();
void handleVerboseStart();
void handleVerboseStop();
void handleVerboseClear();
void handleVerboseFreeze();
void handleVerboseResume();
void handleVerboseDownload();

bool startDiagnosticTest(const char *testId);
void serviceDiagnosticTests();
String makeDiagnosticsJson();
void printDiagnosticTests();
void printDiagnosticTestResult(uint8_t index);
void handleDiagnosticsStatus();
void handleDiagnosticsRun();
void handleDiagnosticsConfirm();

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
void setLedColor(uint8_t red, uint8_t green, uint8_t blue);
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
void startWiFiScan();
void serviceWiFiScan();
void printWiFiScanResults(int16_t count);

void initializeSDCard();
void remountSDCard();
bool isAllowedSDPath(const String &path, bool allowDirectory);
bool isTextDiagnosticFile(const String &path);
void printSDDirectory(const char *path);
void printSDFile(const char *path, const char *mode, uint16_t lineCount);
void handleFileList();
void handleFileView();
void handleFileDownload();
void handleResetRequest();
bool finalizeCaptureForReset(uint32_t timeoutMs = 5000);
void requestSoftwareReset(const char *source);
void flushPendingFaultEvents(uint32_t timeoutMs);
void initializeCANController();
void serviceCANLogger();
bool startCANLogging();
bool stopCANLogging(bool reportResult = true);
void printCANLoggingStatus();
String makeCANLoggingJson();
void handleCANLoggingStatus();
void handleCANLoggingStart();
void handleCANLoggingStop();
void updateCANState();
void canReceiveTask(void *parameter);
void printCANFrames(uint8_t requestedCount);
String makeCANFramesJson();
const char *getCANStateText();
void refreshSDInformation(StatusJsonTimings *timings = nullptr);
void refreshSDUsageInformation();
bool runSDTest();

void configureWebServer();
void handleRoot();
void handleStatus();
void handleCANFrames();
void handleEvents();
void handleSDTest();
void handleClientError();
void handleHttpErrors();
void handleLoopEvents();
void handleNotFound();

String makeStatusJson(StatusJsonTimings *timings = nullptr);

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
void initializeSystemInfoCache();
void initializeSystemNetworkIdentityCache();
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
  initializeSystemInfoCache();
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
  Serial.println(systemInfo.resetReason);
  Serial.println();

  initializeFaultSystem();
  recordDiagnostic(DIAG_ID_BOOT, DIAG_TYPE_EVENT, 9, systemInfo.chipRevision);
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
  initializeSystemNetworkIdentityCache();
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

  initializeCANController();
  captureBootCheckpoint("CAN initialized");

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
    const uint32_t thresholdUs = loopDiagnosticThresholdMs * 1000UL;
    if (loopLastGapUs >= thresholdUs || loopLastDurationUs >= thresholdUs) {
      addLoopTimingEvent();
    }
  }
  loopPreviousStartUs = loopStartUs;

  server.handleClient();
  serviceSerial();
  updateWiFiState();
  serviceWiFiScan();
  updateCANState();
  startMDNSIfNeeded();
  serviceFaultLog();  // Diagnostic fault records retain priority over bulk CAN capture writes.
  serviceCANLogger();
  serviceDiagnosticTests();
  updateLed();

  delay(2);

  loopLastDurationUs = micros() - loopStartUs;
  if (loopLastDurationUs > loopMaxDurationUs) {
    loopMaxDurationUs = loopLastDurationUs;
  }

  loopCount++;
  serviceVerboseDiagnostics();
}

// -----------------------------------------------------------------------------
// V08c verbose diagnostic recorder
// -----------------------------------------------------------------------------

uint32_t diagnosticStringHash(const char *text) {
  // FNV-1a: stable compact identifier for route/event strings without storing
  // dynamically allocated text in the recorder.
  uint32_t hash = 2166136261UL;
  if (!text) return hash;
  while (*text) {
    hash ^= (uint8_t)*text++;
    hash *= 16777619UL;
  }
  return hash;
}

const char *diagnosticRecordName(uint16_t recordId) {
  switch (recordId) {
    case DIAG_ID_BOOT: return "BOOT";
    case DIAG_ID_EVENT_HASH: return "EVENT_HASH";
    case DIAG_ID_FAULT_STATE: return "FAULT_STATE";
    case DIAG_ID_LOOP_GAP_US: return "LOOP_GAP_US";
    case DIAG_ID_LOOP_DURATION_US: return "LOOP_DURATION_US";
    case DIAG_ID_HTTP_HANDLER_US: return "HTTP_HANDLER_US";
    case DIAG_ID_HTTP_ROUTE_HASH: return "HTTP_ROUTE_HASH";
    case DIAG_ID_HEAP_FREE_BYTES: return "HEAP_FREE_BYTES";
    case DIAG_ID_HEAP_LARGEST_BYTES: return "HEAP_LARGEST_BYTES";
    case DIAG_ID_LOOP_LAST_GAP_US: return "LOOP_LAST_GAP_US";
    case DIAG_ID_LOOP_LAST_DURATION_US: return "LOOP_LAST_DURATION_US";
    case DIAG_ID_WIFI_RSSI_DBM: return "WIFI_RSSI_DBM";
    case DIAG_ID_CAN_FRAMES_RECEIVED: return "CAN_FRAMES_RECEIVED";
    case DIAG_ID_CAN_RECEIVE_ERRORS: return "CAN_RECEIVE_ERRORS";
    case DIAG_ID_CAN_RX_QUEUE_DEPTH: return "CAN_RX_QUEUE_DEPTH";
    case DIAG_ID_CAN_LOG_QUEUE_DEPTH: return "CAN_LOG_QUEUE_DEPTH";
    case DIAG_ID_CAN_LOG_QUEUE_DROPS: return "CAN_LOG_QUEUE_DROPS";
    case DIAG_ID_SD_WRITE_FAILURES: return "SD_WRITE_FAILURES";
    case DIAG_ID_SD_READ_FAILURES: return "SD_READ_FAILURES";
    case DIAG_ID_SD_WRITE_DURATION_MS: return "SD_WRITE_DURATION_MS";
    case DIAG_ID_SD_READ_DURATION_MS: return "SD_READ_DURATION_MS";
    case DIAG_ID_CAN_LOG_WRITE_MS: return "CAN_LOG_WRITE_MS";
    case DIAG_ID_CAN_LOG_BATCH_RECORDS: return "CAN_LOG_BATCH_RECORDS";
    case DIAG_ID_ACTIVE_FAULT_COUNT: return "ACTIVE_FAULT_COUNT";
    case DIAG_ID_HEAP_MIN_FREE_BYTES: return "HEAP_MIN_FREE_BYTES";
    case DIAG_ID_HEAP_INTERNAL_FREE_BYTES: return "HEAP_INTERNAL_FREE_BYTES";
    case DIAG_ID_WIFI_AP_CLIENTS: return "WIFI_AP_CLIENTS";
    case DIAG_ID_WIFI_STA_STATE: return "WIFI_STA_STATE";
    case DIAG_ID_WIFI_DISCONNECTS: return "WIFI_DISCONNECTS";
    case DIAG_ID_SD_MOUNTED: return "SD_MOUNTED";
    case DIAG_ID_SD_MOUNT_FAILURES: return "SD_MOUNT_FAILURES";
    case DIAG_ID_SD_WRITE_OPERATIONS: return "SD_WRITE_OPERATIONS";
    case DIAG_ID_SD_READ_OPERATIONS: return "SD_READ_OPERATIONS";
    case DIAG_ID_SD_LAST_WRITE_MS: return "SD_LAST_WRITE_MS";
    case DIAG_ID_SD_LAST_READ_MS: return "SD_LAST_READ_MS";
    case DIAG_ID_HTTP_REQUESTS: return "HTTP_REQUESTS";
    case DIAG_ID_HTTP_ERRORS: return "HTTP_ERRORS";
    case DIAG_ID_HTTP_CLIENT_ERRORS: return "HTTP_CLIENT_ERRORS";
    case DIAG_ID_LOOP_ITERATIONS: return "LOOP_ITERATIONS";
    case DIAG_ID_LOOP_MAX_DURATION_US: return "LOOP_MAX_DURATION_US";
    case DIAG_ID_LOOP_MAX_GAP_US: return "LOOP_MAX_GAP_US";
    case DIAG_ID_CAN_BUS_OFF_COUNT: return "CAN_BUS_OFF_COUNT";
    case DIAG_ID_CAN_RECOVERY_COUNT: return "CAN_RECOVERY_COUNT";
    case DIAG_ID_CAN_BUS_ERRORS: return "CAN_BUS_ERRORS";
    case DIAG_ID_CAN_DRIVER_MISSED: return "CAN_DRIVER_MISSED";
    case DIAG_ID_CAN_LOG_STATE: return "CAN_LOG_STATE";
    case DIAG_ID_CAN_LOG_FRAMES_WRITTEN: return "CAN_LOG_FRAMES_WRITTEN";
    case DIAG_ID_CAN_LOG_WRITE_FAILURES: return "CAN_LOG_WRITE_FAILURES";
    case DIAG_ID_CAN_LOG_LAST_WRITE_MS: return "CAN_LOG_LAST_WRITE_MS";
    case DIAG_ID_FAULT_QUEUE_DROPS: return "FAULT_QUEUE_DROPS";
    case DIAG_ID_STATUS_SD_REFRESH_US: return "STATUS_SD_REFRESH_US";
    case DIAG_ID_STATUS_SD_CARD_TYPE_US: return "STATUS_SD_CARD_TYPE_US";
    case DIAG_ID_STATUS_SD_CARD_SIZE_US: return "STATUS_SD_CARD_SIZE_US";
    case DIAG_ID_STATUS_SD_TOTAL_BYTES_US: return "STATUS_SD_TOTAL_BYTES_US";
    case DIAG_ID_STATUS_SD_USED_BYTES_US: return "STATUS_SD_USED_BYTES_US";
    case DIAG_ID_STATUS_SYSTEM_MEMORY_US: return "STATUS_SYSTEM_MEMORY_US";
    case DIAG_ID_STATUS_WIFI_US: return "STATUS_WIFI_US";
    case DIAG_ID_STATUS_SD_JSON_US: return "STATUS_SD_JSON_US";
    case DIAG_ID_STATUS_HTTP_LOOP_FAULT_US: return "STATUS_HTTP_LOOP_FAULT_US";
    case DIAG_ID_STATUS_CAN_SNAPSHOT_US: return "STATUS_CAN_SNAPSHOT_US";
    case DIAG_ID_STATUS_CAN_JSON_US: return "STATUS_CAN_JSON_US";
    case DIAG_ID_STATUS_JSON_TOTAL_US: return "STATUS_JSON_TOTAL_US";
    case DIAG_ID_STATUS_RESPONSE_SEND_US: return "STATUS_RESPONSE_SEND_US";
    case DIAG_ID_STATUS_SYS_HEAP_US: return "STATUS_SYS_HEAP_US";
    case DIAG_ID_STATUS_SYS_DEVICE_US: return "STATUS_SYS_DEVICE_US";
    case DIAG_ID_STATUS_SYS_RUNTIME_US: return "STATUS_SYS_RUNTIME_US";
    case DIAG_ID_STATUS_SYS_HEALTH_US: return "STATUS_SYS_HEALTH_US";
    case DIAG_ID_STATUS_SYS_HEALTH_DETAILS_US: return "STATUS_SYS_HEALTH_DETAILS_US";
    case DIAG_ID_STATUS_SYS_OTHER_US: return "STATUS_SYS_OTHER_US";
    case DIAG_ID_STATUS_SYS_CHIP_MODEL_US: return "STATUS_SYS_CHIP_MODEL_US";
    case DIAG_ID_STATUS_SYS_CHIP_REVISION_US: return "STATUS_SYS_CHIP_REVISION_US";
    case DIAG_ID_STATUS_SYS_CPU_FREQ_US: return "STATUS_SYS_CPU_FREQ_US";
    case DIAG_ID_STATUS_SYS_FLASH_SIZE_US: return "STATUS_SYS_FLASH_SIZE_US";
    case DIAG_ID_STATUS_SYS_SKETCH_SIZE_US: return "STATUS_SYS_SKETCH_SIZE_US";
    case DIAG_ID_STATUS_SYS_FREE_SKETCH_US: return "STATUS_SYS_FREE_SKETCH_US";
    default: return "UNKNOWN";
  }
}

const char *diagnosticRecordTypeName(uint8_t dataType) {
  switch (dataType) {
    case DIAG_TYPE_U32: return "U32";
    case DIAG_TYPE_I32: return "I32";
    case DIAG_TYPE_BOOL: return "BOOL";
    case DIAG_TYPE_EVENT: return "EVENT";
    default: return "UNKNOWN";
  }
}

const char *diagnosticRecordDetail(const DiagnosticRecord &record) {
  if (record.recordId == DIAG_ID_HTTP_ROUTE_HASH) {
    static const char *routes[] = {
      "/", "/api/status", "/api/can-frames", "/api/logging",
      "/api/logging/start", "/api/logging/stop", "/api/events",
      "/api/http-errors", "/api/loop-events", "/api/verbose",
      "/api/verbose/start", "/api/verbose/stop", "/api/verbose/clear",
      "/api/verbose/freeze", "/api/verbose/resume", "/download-verbose",
      "/api/faults", "/api/files", "/api/file", "/download",
      "/api/reset", "/api/test", "/api/client-error"
    };
    for (size_t i = 0; i < sizeof(routes) / sizeof(routes[0]); i++) {
      if (diagnosticStringHash(routes[i]) == record.value) return routes[i];
    }
    return "unknown route hash";
  }
  if (record.recordId == DIAG_ID_EVENT_HASH) return "event message hash; correlate with event history";
  if (record.recordId == DIAG_ID_FAULT_STATE) return record.flags ? "fault active" : "fault recovered";
  if (record.recordId == DIAG_ID_WIFI_RSSI_DBM) return record.context ? "STA connected" : "STA disconnected";
  if (record.recordId == DIAG_ID_WIFI_STA_STATE) {
    switch (record.value) {
      case 0: return "not configured";
      case 1: return "connected";
      case 2: return "connecting";
      case 3: return "manually disconnected";
      default: return "disconnected";
    }
  }
  if (record.recordId == DIAG_ID_CAN_LOG_STATE) {
    switch (record.value) {
      case 1: return "recording";
      case 2: return "stopping/draining";
      default: return "stopped";
    }
  }
  if (record.recordId == DIAG_ID_BOOT) return "firmware V10 boot/enable marker";
  return "";
}

void recordDiagnostic(uint16_t recordId, uint8_t dataType, uint32_t value,
                      uint32_t context, uint8_t flags) {
  if (!verboseLoggingEnabled || verboseLoggingFrozen) return;
  uint32_t startedUs = micros();

  DiagnosticRecord record = {};
  record.timestampUs = (uint64_t)esp_timer_get_time();
  record.recordId = recordId;
  record.dataType = dataType;
  record.flags = flags;
  record.value = value;
  record.context = context;

  portENTER_CRITICAL(&diagnosticMux);
  if (verboseLoggingEnabled && !verboseLoggingFrozen) {
    diagnosticRecords[diagnosticRecordHead] = record;
    diagnosticRecordHead = (diagnosticRecordHead + 1) % DIAG_RECORD_CAPACITY;
    if (diagnosticRecordCount < DIAG_RECORD_CAPACITY) {
      diagnosticRecordCount++;
    } else {
      diagnosticRecordsOverwritten++;
    }
    diagnosticRecordsTotal++;
  }
  portEXIT_CRITICAL(&diagnosticMux);
  diagnosticLastRecordCostUs = micros() - startedUs;
  if (diagnosticLastRecordCostUs > diagnosticMaxRecordCostUs) {
    diagnosticMaxRecordCostUs = diagnosticLastRecordCostUs;
  }
}

void serviceVerboseDiagnostics() {
  if (!verboseLoggingEnabled || verboseLoggingFrozen) return;
  uint32_t now = millis();
  if ((uint32_t)(now - diagnosticLastSampleMs) < diagnosticSampleIntervalMs) return;
  diagnosticLastSampleMs = now;
  uint32_t sampleStartedUs = micros();

  // Fixed, low-rate samples. High-value transitions and operation timings are
  // recorded at their source; this sample set captures subsystem trends.
  recordDiagnostic(DIAG_ID_HEAP_FREE_BYTES, DIAG_TYPE_U32, ESP.getFreeHeap());
  recordDiagnostic(DIAG_ID_HEAP_LARGEST_BYTES, DIAG_TYPE_U32,
                   heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
  recordDiagnostic(DIAG_ID_LOOP_LAST_GAP_US, DIAG_TYPE_U32, loopLastGapUs);
  recordDiagnostic(DIAG_ID_LOOP_LAST_DURATION_US, DIAG_TYPE_U32, loopLastDurationUs);
  uint32_t canFramesSnapshot, canReceiveErrorsSnapshot, canQueueDepthSnapshot;
  portENTER_CRITICAL(&canMux);
  canFramesSnapshot = canFramesReceived;
  canReceiveErrorsSnapshot = canReceiveErrors;
  canQueueDepthSnapshot = canDriverRxQueueDepth;
  portEXIT_CRITICAL(&canMux);
  recordDiagnostic(DIAG_ID_WIFI_RSSI_DBM, DIAG_TYPE_I32,
                   (uint32_t)(int32_t)(wifiStaConnected ? WiFi.RSSI() : -127),
                   wifiStaConnected ? 1 : 0);
  recordDiagnostic(DIAG_ID_CAN_FRAMES_RECEIVED, DIAG_TYPE_U32, canFramesSnapshot);
  recordDiagnostic(DIAG_ID_CAN_RECEIVE_ERRORS, DIAG_TYPE_U32, canReceiveErrorsSnapshot);
  recordDiagnostic(DIAG_ID_CAN_RX_QUEUE_DEPTH, DIAG_TYPE_U32, canQueueDepthSnapshot);
  recordDiagnostic(DIAG_ID_CAN_LOG_QUEUE_DEPTH, DIAG_TYPE_U32,
                   canLogQueue ? uxQueueMessagesWaiting(canLogQueue) : 0);
  recordDiagnostic(DIAG_ID_CAN_LOG_QUEUE_DROPS, DIAG_TYPE_U32, canLogQueueDrops);
  recordDiagnostic(DIAG_ID_SD_WRITE_FAILURES, DIAG_TYPE_U32, sdWriteFailures);
  recordDiagnostic(DIAG_ID_SD_READ_FAILURES, DIAG_TYPE_U32, sdReadFailures);
  recordDiagnostic(DIAG_ID_ACTIVE_FAULT_COUNT, DIAG_TYPE_U32, getActiveFaultCount());

  if ((uint32_t)(now - diagnosticLastSlowSampleMs) >= DIAG_SLOW_SAMPLE_INTERVAL_MS) {
    diagnosticLastSlowSampleMs = now;
    recordDiagnostic(DIAG_ID_HEAP_MIN_FREE_BYTES, DIAG_TYPE_U32, ESP.getMinFreeHeap());
    recordDiagnostic(DIAG_ID_HEAP_INTERNAL_FREE_BYTES, DIAG_TYPE_U32,
                     heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    recordDiagnostic(DIAG_ID_WIFI_AP_CLIENTS, DIAG_TYPE_U32, WiFi.softAPgetStationNum());
    uint32_t staState = wifiStaConnected ? 1 : (wifiStaConnecting ? 2 :
                         (wifiStaManuallyDisconnected ? 3 :
                         (configuredStaSsid.length() == 0 ? 0 : 4)));
    recordDiagnostic(DIAG_ID_WIFI_STA_STATE, DIAG_TYPE_U32, staState,
                     (uint32_t)(lastStaDisconnectReason + 1));
    recordDiagnostic(DIAG_ID_WIFI_DISCONNECTS, DIAG_TYPE_U32, staDisconnectCount);
    recordDiagnostic(DIAG_ID_SD_MOUNTED, DIAG_TYPE_BOOL, sdReady ? 1 : 0);
    recordDiagnostic(DIAG_ID_SD_MOUNT_FAILURES, DIAG_TYPE_U32, sdMountFailures);
    recordDiagnostic(DIAG_ID_SD_WRITE_OPERATIONS, DIAG_TYPE_U32, sdWriteOperations);
    recordDiagnostic(DIAG_ID_SD_READ_OPERATIONS, DIAG_TYPE_U32, sdReadOperations);
    recordDiagnostic(DIAG_ID_SD_LAST_WRITE_MS, DIAG_TYPE_U32, sdLastWriteDurationMs);
    recordDiagnostic(DIAG_ID_SD_LAST_READ_MS, DIAG_TYPE_U32, sdLastReadDurationMs);
    recordDiagnostic(DIAG_ID_HTTP_REQUESTS, DIAG_TYPE_U32, httpRequests);
    recordDiagnostic(DIAG_ID_HTTP_ERRORS, DIAG_TYPE_U32, httpErrorResponses);
    recordDiagnostic(DIAG_ID_HTTP_CLIENT_ERRORS, DIAG_TYPE_U32, httpClientErrorReports);
    recordDiagnostic(DIAG_ID_LOOP_ITERATIONS, DIAG_TYPE_U32, loopCount);
    recordDiagnostic(DIAG_ID_LOOP_MAX_DURATION_US, DIAG_TYPE_U32, loopMaxDurationUs);
    recordDiagnostic(DIAG_ID_LOOP_MAX_GAP_US, DIAG_TYPE_U32, loopMaxGapUs);
    recordDiagnostic(DIAG_ID_CAN_BUS_OFF_COUNT, DIAG_TYPE_U32, canBusOffCount);
    recordDiagnostic(DIAG_ID_CAN_RECOVERY_COUNT, DIAG_TYPE_U32, canRecoveryCount);
    recordDiagnostic(DIAG_ID_CAN_BUS_ERRORS, DIAG_TYPE_U32, canDriverBusErrorCount);
    recordDiagnostic(DIAG_ID_CAN_DRIVER_MISSED, DIAG_TYPE_U32,
                     canDriverRxMissedCount + canDriverRxOverrunCount);
    recordDiagnostic(DIAG_ID_CAN_LOG_STATE, DIAG_TYPE_BOOL,
                     canLoggingActive ? 1 : (canLoggingStopping ? 2 : 0));
    recordDiagnostic(DIAG_ID_CAN_LOG_FRAMES_WRITTEN, DIAG_TYPE_U32, canLogFramesWritten);
    recordDiagnostic(DIAG_ID_CAN_LOG_WRITE_FAILURES, DIAG_TYPE_U32, canLogWriteFailures);
    recordDiagnostic(DIAG_ID_CAN_LOG_LAST_WRITE_MS, DIAG_TYPE_U32, canLogLastWriteMs);
    recordDiagnostic(DIAG_ID_FAULT_QUEUE_DROPS, DIAG_TYPE_U32, faultQueueDropCount);
  }

  diagnosticLastSampleCostUs = micros() - sampleStartedUs;
  if (diagnosticLastSampleCostUs > diagnosticMaxSampleCostUs) {
    diagnosticMaxSampleCostUs = diagnosticLastSampleCostUs;
  }
}

void setVerboseLogging(bool enabled) {
  if (enabled) {
    verboseLoggingEnabled = true;
    verboseLoggingFrozen = false;
    diagnosticLastSampleMs = 0;
    recordDiagnostic(DIAG_ID_BOOT, DIAG_TYPE_EVENT, 8, 0, 1);
    addEvent("INFO", "Verbose diagnostic recording enabled");
  } else {
    addEvent("INFO", "Verbose diagnostic recording disabled");
    verboseLoggingEnabled = false;
    verboseLoggingFrozen = false;
  }
  if (diagnosticPreferences.begin("diagcfg", false)) {
    diagnosticPreferences.putBool("verbose", enabled);
    diagnosticPreferences.end();
  }
}

void clearVerboseRecords() {
  portENTER_CRITICAL(&diagnosticMux);
  diagnosticRecordHead = 0;
  diagnosticRecordCount = 0;
  diagnosticClearCount++;
  portEXIT_CRITICAL(&diagnosticMux);
  addEvent("INFO", "Verbose diagnostic buffer cleared");
}

void printVerboseStatus() {
  uint16_t count;
  uint32_t total, overwritten, clears;
  portENTER_CRITICAL(&diagnosticMux);
  count = diagnosticRecordCount;
  total = diagnosticRecordsTotal;
  overwritten = diagnosticRecordsOverwritten;
  clears = diagnosticClearCount;
  portEXIT_CRITICAL(&diagnosticMux);
  Serial.println();
  Serial.println(F("VERBOSE DIAGNOSTIC RECORDER"));
  Serial.println(F("---------------------------"));
  Serial.print(F("State:                 "));
  Serial.println(!verboseLoggingEnabled ? F("DISABLED") :
                 (verboseLoggingFrozen ? F("FROZEN") : F("RECORDING")));
  Serial.print(F("Records retained:      ")); Serial.print(count);
  Serial.print('/'); Serial.println(DIAG_RECORD_CAPACITY);
  Serial.print(F("Records total:         ")); Serial.println(total);
  Serial.print(F("Records overwritten:   ")); Serial.println(overwritten);
  Serial.print(F("Buffer clears:         ")); Serial.println(clears);
  Serial.print(F("Buffer memory:         ")); Serial.print(sizeof(diagnosticRecords)); Serial.println(F(" bytes"));
  Serial.print(F("Sample interval:       ")); Serial.print(diagnosticSampleIntervalMs); Serial.println(F(" ms"));
  Serial.print(F("Loop event threshold:  ")); Serial.print(loopDiagnosticThresholdMs); Serial.println(F(" ms"));
  Serial.print(F("Format version:        ")); Serial.println(DIAG_RECORD_FORMAT_VERSION);
  Serial.print(F("Record cost last/max:  ")); Serial.print(diagnosticLastRecordCostUs);
  Serial.print(F(" / ")); Serial.print(diagnosticMaxRecordCostUs); Serial.println(F(" us"));
  Serial.print(F("Sample cost last/max:  ")); Serial.print(diagnosticLastSampleCostUs);
  Serial.print(F(" / ")); Serial.print(diagnosticMaxSampleCostUs); Serial.println(F(" us"));
  Serial.println();
}

String makeVerboseStatusJson() {
  uint16_t count;
  uint32_t total, overwritten, clears;
  portENTER_CRITICAL(&diagnosticMux);
  count = diagnosticRecordCount;
  total = diagnosticRecordsTotal;
  overwritten = diagnosticRecordsOverwritten;
  clears = diagnosticClearCount;
  portEXIT_CRITICAL(&diagnosticMux);
  String json = "{\"enabled\":" + String(verboseLoggingEnabled ? "true" : "false");
  json += ",\"frozen\":" + String(verboseLoggingFrozen ? "true" : "false");
  json += ",\"records\":" + String(count);
  json += ",\"capacity\":" + String(DIAG_RECORD_CAPACITY);
  json += ",\"total\":" + String(total);
  json += ",\"overwritten\":" + String(overwritten);
  json += ",\"clears\":" + String(clears);
  json += ",\"recordSize\":" + String(sizeof(DiagnosticRecord));
  json += ",\"bufferBytes\":" + String(sizeof(diagnosticRecords));
  json += ",\"sampleIntervalMs\":" + String(diagnosticSampleIntervalMs);
  json += ",\"slowSampleIntervalMs\":" + String(DIAG_SLOW_SAMPLE_INTERVAL_MS);
  json += ",\"loopThresholdMs\":" + String(loopDiagnosticThresholdMs);
  json += ",\"formatVersion\":" + String(DIAG_RECORD_FORMAT_VERSION);
  json += ",\"lastRecordCostUs\":" + String(diagnosticLastRecordCostUs);
  json += ",\"maxRecordCostUs\":" + String(diagnosticMaxRecordCostUs);
  json += ",\"lastSampleCostUs\":" + String(diagnosticLastSampleCostUs);
  json += ",\"maxSampleCostUs\":" + String(diagnosticMaxSampleCostUs) + "}";
  return json;
}

// -----------------------------------------------------------------------------
// Diagnostic event history
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------

void addEvent(const char *severity, const char *message) {
  DiagnosticEvent &event = eventHistory[eventHead];

  event.uptimeMs = millis() - bootMillis;
  snprintf(event.severity, sizeof(event.severity), "%s", severity);
  snprintf(event.message, sizeof(event.message), "%s", message);

  eventHead = (eventHead + 1) % EVENT_HISTORY_SIZE;
  if (eventCount < EVENT_HISTORY_SIZE) eventCount++;
  uint32_t severityCode = !strcmp(severity, "ERROR") ? 3 :
                          !strcmp(severity, "WARN") ? 2 : 1;
  recordDiagnostic(DIAG_ID_EVENT_HASH, DIAG_TYPE_EVENT,
                   diagnosticStringHash(message), severityCode);
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
  recordDiagnostic(DIAG_ID_LOOP_GAP_US, DIAG_TYPE_U32, loopLastGapUs, httpLastStatusCode);
  recordDiagnostic(DIAG_ID_LOOP_DURATION_US, DIAG_TYPE_U32, loopLastDurationUs,
                   diagnosticStringHash(httpLastRoute));
}

void printLoopEvents() {
  Serial.println();
  Serial.printf("RECENT LOOP TIMING EVENTS (threshold %lu ms)\n",
                (unsigned long)loopDiagnosticThresholdMs);
  Serial.println(F("------------------------------------------------"));
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
    verboseLoggingEnabled = diagnosticPreferences.getBool("verbose", false);
    loopDiagnosticThresholdMs = constrain(diagnosticPreferences.getUInt("loopMs", 100), DIAG_MIN_LOOP_THRESHOLD_MS, DIAG_MAX_LOOP_THRESHOLD_MS);
    diagnosticSampleIntervalMs = constrain(diagnosticPreferences.getUInt("diagMs", DIAG_DEFAULT_SAMPLE_INTERVAL_MS), DIAG_MIN_SAMPLE_INTERVAL_MS, DIAG_MAX_SAMPLE_INTERVAL_MS);
    diagnosticPreferences.end();
  }

  if (!faultBootIdPersistent) {
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
  recordDiagnostic(DIAG_ID_FAULT_STATE, DIAG_TYPE_EVENT, code, id, 1);
  addEvent("ERROR", description);
}

void clearFault(FaultId id, const char *description) {
  if (id >= FAULT_ID_COUNT || !activeFaults[id].active) return;

  ActiveFault &fault = activeFaults[id];
  enqueueFaultEvent(fault.subsystem, "RECOVERED", fault.code, description);
  recordDiagnostic(DIAG_ID_FAULT_STATE, DIAG_TYPE_EVENT, fault.code, id, 0);
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
  if (ok) refreshSDUsageInformation();
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
  if (ok) refreshSDUsageInformation();
  return ok;
}

void serviceFaultLog() {
  if (!sdReady || sdOperationActive) return;
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
  if (faultLogReady) Serial.println(FAULT_LOG_FILE);
  else Serial.println(F("NOT READY"));
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
// Read-only SD file inspection
// -----------------------------------------------------------------------------

bool isAllowedSDPath(const String &path, bool allowDirectory) {
  if (path.length() == 0 || path.length() > 96) return false;
  if (path.indexOf("..") >= 0 || path.indexOf('\\') >= 0 || path.indexOf("//") >= 0) return false;
  if (path == "/diag") return allowDirectory;
  if (!path.startsWith("/diag/")) return false;
  // Restrict inspection to the diagnostic tree. No hidden traversal or
  // absolute-volume paths are accepted.
  return true;
}

bool isTextDiagnosticFile(const String &path) {
  String lower = path;
  lower.toLowerCase();
  return lower.endsWith(".csv") || lower.endsWith(".txt") ||
         lower.endsWith(".log") || lower.endsWith(".json");
}

void printSDDirectory(const char *path) {
  if (canLoggingActive || canLoggingStopping) { Serial.println(F("[SD] File inspection paused while CAN capture is active.")); return; }
  if (!sdReady || sdOperationActive) {
    Serial.println(F("[SD] Unavailable or busy."));
    return;
  }
  String directoryPath(path);
  if (!isAllowedSDPath(directoryPath, true)) {
    Serial.println(F("[SD] Path rejected. Only /diag and its contents are accessible."));
    return;
  }
  sdOperationActive = true;
  sdFileListRequests++;
  File dir = SD.open(directoryPath.c_str());
  if (!dir || !dir.isDirectory()) {
    if (dir) dir.close();
    sdOperationActive = false;
    Serial.println(F("[SD] Directory open failed."));
    sdFileOperationFailures++;
    return;
  }
  Serial.print(F("\nSD DIRECTORY: "));
  Serial.println(directoryPath);
  File entry = dir.openNextFile();
  uint16_t count = 0;
  while (entry) {
    String name = entry.name();
    Serial.print(entry.isDirectory() ? F("DIR  ") : F("FILE "));
    Serial.print(entry.isDirectory() ? 0 : (unsigned long)entry.size());
    Serial.print(F("  "));
    Serial.println(name);
    entry.close();
    count++;
    entry = dir.openNextFile();
    if ((count % 8) == 0) yield();
  }
  dir.close();
  Serial.print(F("Entries: "));
  Serial.println(count);
  sdOperationActive = false;
}

void printSDFile(const char *path, const char *mode, uint16_t lineCount) {
  if (canLoggingActive || canLoggingStopping) { Serial.println(F("[SD] File inspection paused while CAN capture is active.")); return; }
  if (!sdReady || sdOperationActive) {
    Serial.println(F("[SD] Unavailable or busy."));
    return;
  }
  String filePath(path);
  if (!isAllowedSDPath(filePath, false) || !isTextDiagnosticFile(filePath)) {
    Serial.println(F("[SD] Path rejected. Use a text file inside /diag (.csv, .txt, .log, .json)."));
    return;
  }
  sdOperationActive = true;
  uint32_t started = millis();
  File file = SD.open(filePath.c_str(), FILE_READ);
  if (!file || file.isDirectory()) {
    if (file) file.close();
    sdOperationActive = false;
    sdFileOperationFailures++;
    sdReadFailures++;
    Serial.println(F("[SD] File open failed."));
    return;
  }

  const uint32_t fileSize = file.size();
  Serial.print(F("\n--- "));
  Serial.print(filePath);
  Serial.print(F(" ("));
  Serial.print(fileSize);
  Serial.println(F(" bytes) ---"));

  if (!strcmp(mode, "tail")) {
    if (lineCount < 1) lineCount = 10;
    if (lineCount > 50) lineCount = 50;
    uint32_t starts[50];
    uint8_t found = 0;
    uint32_t pos = fileSize;
    if (pos > 0) {
      file.seek(pos - 1);
      if (file.read() == '\n') pos--;
    }
    while (pos > 0 && found < lineCount) {
      file.seek(pos - 1);
      int c = file.read();
      if (c == '\n') starts[found++] = pos;
      pos--;
    }
    if (found < lineCount && pos == 0 && found < 50) starts[found++] = 0;
    for (int i = (int)found - 1; i >= 0; --i) {
      file.seek(starts[i]);
      while (file.available()) {
        int c = file.read();
        if (c < 0) break;
        Serial.write((uint8_t)c);
        if (c == '\n') break;
      }
      if ((i % 4) == 0) yield();
    }
  } else {
    uint16_t lines = 0;
    while (file.available()) {
      int c = file.read();
      if (c < 0) break;
      Serial.write((uint8_t)c);
      if (c == '\n') {
        lines++;
        if (!strcmp(mode, "head") && lines >= lineCount) break;
      }
      if ((file.position() % 256) == 0) yield();
    }
  }
  file.close();
  sdReadOperations++;
  sdLastReadDurationMs = millis() - started;
  if (sdLastReadDurationMs > sdMaxReadDurationMs) sdMaxReadDurationMs = sdLastReadDurationMs;
  sdTotalReadDurationMs += sdLastReadDurationMs;
  Serial.println(F("\n--- END ---"));
  sdOperationActive = false;
}

void handleFileList() {
  uint32_t started = micros();
  if (canLoggingActive || canLoggingStopping) { sendHttpResponse(503, "application/json", "{\"error\":\"CAN capture active; file inspection paused\"}"); recordHttp("GET", "/api/files", started, 503); return; }
  String path = server.hasArg("path") ? server.arg("path") : "/diag";
  if (!sdReady || !isAllowedSDPath(path, true)) {
    sendHttpResponse(sdReady ? 400 : 503, "application/json", "{\"error\":\"SD unavailable or path rejected\"}");
    sdFileOperationFailures++;
    recordHttp("GET", "/api/files", started, sdReady ? 400 : 503);
    return;
  }
  if (sdOperationActive) {
    sendHttpResponse(503, "application/json", "{\"error\":\"SD busy\"}");
    recordHttp("GET", "/api/files", started, 503);
    return;
  }
  sdOperationActive = true;
  sdFileListRequests++;
  String json = "{\"path\":\"" + jsonEscape(path) + "\",\"entries\":[";
  File dir = SD.open(path.c_str());
  if (!dir || !dir.isDirectory()) {
    if (dir) dir.close();
    sdOperationActive = false;
    sdFileOperationFailures++;
    sendHttpResponse(404, "application/json", "{\"error\":\"Directory not found\"}");
    recordHttp("GET", "/api/files", started, 404);
    return;
  }
  bool first = true;
  File entry = dir.openNextFile();
  while (entry) {
    if (!first) json += ',';
    first = false;
    String name = entry.name();
    json += "{\"name\":\"" + jsonEscape(name) + "\",\"directory\":" + String(entry.isDirectory() ? "true" : "false");
    json += ",\"size\":" + String(entry.isDirectory() ? 0 : entry.size()) + "}";
    entry.close();
    entry = dir.openNextFile();
    if (json.length() > 5000) break;
  }
  dir.close();
  json += "]}";
  sdOperationActive = false;
  sendHttpResponse(200, "application/json", json);
  recordHttp("GET", "/api/files", started, 200);
}

void handleFileView() {
  uint32_t started = micros();
  if (canLoggingActive || canLoggingStopping) { sendHttpResponse(503, "text/plain", "CAN capture active; file inspection paused"); recordHttp("GET", "/api/file", started, 503); return; }
  String path = server.arg("path");
  if (!sdReady || !isAllowedSDPath(path, false) || !isTextDiagnosticFile(path) || sdOperationActive) {
    int code = !sdReady ? 503 : (sdOperationActive ? 503 : 400);
    sendHttpResponse(code, "text/plain", "SD unavailable, busy, or path rejected");
    sdFileOperationFailures++;
    recordHttp("GET", "/api/file", started, code);
    return;
  }
  sdOperationActive = true;
  File file = SD.open(path.c_str(), FILE_READ);
  if (!file || file.isDirectory()) {
    if (file) file.close();
    sdOperationActive = false;
    sdFileOperationFailures++;
    sendHttpResponse(404, "text/plain", "File not found");
    recordHttp("GET", "/api/file", started, 404);
    return;
  }
  sdFileReadRequests++;
  server.sendHeader("Cache-Control", "no-store");
  String filename = path.substring(path.lastIndexOf('/') + 1);
  filename.replace("\"", "_"); filename.replace("\r", "_"); filename.replace("\n", "_");
  server.sendHeader("Content-Disposition", "inline; filename=\"" + filename + "\"");
  size_t expected = file.size();
  size_t sent = server.streamFile(file, "text/plain; charset=utf-8");
  file.close();
  sdOperationActive = false;
  if (sent != expected) sdFileOperationFailures++;
  recordHttp("GET", "/api/file", started, sent == expected ? 200 : 500);
}

void handleFileDownload() {
  uint32_t started = micros();
  if (canLoggingActive || canLoggingStopping) { sendHttpResponse(503, "text/plain", "CAN capture active; file inspection paused"); recordHttp("GET", "/download", started, 503); return; }
  String path = server.arg("path");
  if (!sdReady || !isAllowedSDPath(path, false) || sdOperationActive) {
    int code = !sdReady ? 503 : (sdOperationActive ? 503 : 400);
    sendHttpResponse(code, "text/plain", "SD unavailable, busy, or path rejected");
    sdFileOperationFailures++;
    recordHttp("GET", "/download", started, code);
    return;
  }
  sdOperationActive = true;
  File file = SD.open(path.c_str(), FILE_READ);
  if (!file || file.isDirectory()) {
    if (file) file.close();
    sdOperationActive = false;
    sdFileOperationFailures++;
    sendHttpResponse(404, "text/plain", "File not found");
    recordHttp("GET", "/download", started, 404);
    return;
  }
  String filename = path.substring(path.lastIndexOf('/') + 1);
  filename.replace("\"", "_"); filename.replace("\r", "_"); filename.replace("\n", "_");
  sdFileDownloadRequests++;
  server.sendHeader("Cache-Control", "no-store");
  server.sendHeader("Content-Disposition", "attachment; filename=\"" + filename + "\"");
  size_t expected = file.size();
  size_t sent = server.streamFile(file, "application/octet-stream");
  file.close();
  sdOperationActive = false;
  if (sent != expected) sdFileOperationFailures++;
  recordHttp("GET", "/download", started, sent == expected ? 200 : 500);
}

void flushPendingFaultEvents(uint32_t timeoutMs) {
  uint32_t started = millis();
  while (sdReady && faultQueueCount > 0 && millis() - started < timeoutMs) {
    uint8_t before = faultQueueCount;
    serviceFaultLog();
    if (faultQueueCount == before && !faultLogReady) break;
    delay(1);
  }
}

bool finalizeCaptureForReset(uint32_t timeoutMs) {
  if (!canLogFileOpen) return true;

  Serial.println(F("[SYSTEM] Finalizing active CAN capture before reset..."));
  stopCANLogging(false);
  uint32_t startedMs = millis();
  while (canLogFileOpen && (uint32_t)(millis() - startedMs) < timeoutMs) {
    serviceCANLogger();
    if (canLogFileOpen) vTaskDelay(pdMS_TO_TICKS(1));
  }

  if (canLogFileOpen) {
    Serial.println(F("[SYSTEM] Reset aborted: CAN capture did not finalize before timeout."));
    addEvent("ERROR", "Software reset aborted because CAN capture finalization timed out");
    return false;
  }

  Serial.println(F("[SYSTEM] Active CAN capture finalized."));
  return true;
}

void requestSoftwareReset(const char *source) {
  Serial.print(F("[SYSTEM] Software reset requested through "));
  Serial.println(source);
  if (!finalizeCaptureForReset()) return;
  addEvent("WARN", "Software reset requested");
  flushPendingFaultEvents(1000);
  Serial.flush();
  delay(250);
  ESP.restart();
}

void handleResetRequest() {
  uint32_t started = micros();
  server.sendHeader("Cache-Control", "no-store");
  if (!finalizeCaptureForReset()) {
    server.send(503, "text/plain", "Restart aborted: CAN capture could not be finalized");
    recordHttp("POST", "/api/reset", started, 503);
    return;
  }
  server.send(202, "text/plain", "Software restart requested");
  recordHttp("POST", "/api/reset", started, 202);
  flushPendingFaultEvents(1000);
  delay(250);
  ESP.restart();
}

// -----------------------------------------------------------------------------
// Native ESP32 TWAI controller: fixed 250 kbit/s, listen-only
// -----------------------------------------------------------------------------

const char *getCANStateText() {
  return canStateText.c_str();
}

void initializeCANController() {
  canInitAttempts++;
  canStateText = "INITIALIZING";
  canInitialized = false;
  canRecoveryRequested = false;

  twai_general_config_t general = TWAI_GENERAL_CONFIG_DEFAULT(
      (gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN, TWAI_MODE_LISTEN_ONLY);
  // A 64-message driver queue provides burst tolerance. A dedicated task at
  // priority 3 drains it independently of synchronous WebServer handlers.
  general.rx_queue_len = CAN_RX_DRIVER_QUEUE_LENGTH;
  general.tx_queue_len = 0;
  general.alerts_enabled = TWAI_ALERT_RX_QUEUE_FULL | TWAI_ALERT_BUS_OFF |
                           TWAI_ALERT_RECOVERY_IN_PROGRESS | TWAI_ALERT_BUS_RECOVERED |
                           TWAI_ALERT_BUS_ERROR | TWAI_ALERT_ARB_LOST;
  twai_timing_config_t timing = TWAI_TIMING_CONFIG_250KBITS();
  twai_filter_config_t filter = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  esp_err_t result = twai_driver_install(&general, &timing, &filter);
  if (result != ESP_OK) {
    canInitFailures++;
    canLastError = (uint32_t)result;
    canStateText = "INSTALL FAILED";
    setFault(FAULT_CAN_DRIVER, "CAN", 1, "TWAI driver installation failed");
    Serial.printf("[CAN] TWAI driver install failed: 0x%X\n", (unsigned)result);
    return;
  }
  canDriverInstalled = true;

  result = twai_start();
  if (result != ESP_OK) {
    canInitFailures++;
    canLastError = (uint32_t)result;
    canStateText = "START FAILED";
    setFault(FAULT_CAN_DRIVER, "CAN", 2, "TWAI controller failed to start");
    Serial.printf("[CAN] TWAI start failed: 0x%X\n", (unsigned)result);
    twai_driver_uninstall();
    canDriverInstalled = false;
    return;
  }

  canInitialized = true;
  canLogQueue = xQueueCreate(CAN_LOG_QUEUE_LENGTH, sizeof(CanLogRecord));
  if (!canLogQueue) {
    Serial.println(F("[CAN LOG] ERROR: could not allocate bounded logging queue; CAN reception will continue without SD logging."));
    addEvent("ERROR", "CAN logging queue allocation failed");
  }

  BaseType_t taskResult = xTaskCreatePinnedToCore(
      canReceiveTask, "can_rx", CAN_RX_TASK_STACK_BYTES, nullptr,
      CAN_RX_TASK_PRIORITY, &canRxTaskHandle, CAN_RX_TASK_CORE);
  if (taskResult != pdPASS) {
    canRxTaskStarted = false;
    canStateText = "RX TASK FAILED";
    setFault(FAULT_CAN_DRIVER, "CAN", 8,
             "CAN receive task could not be created; frames are not being acquired");
    Serial.println(F("[CAN] ERROR: receive task creation failed."));
    return;
  }

  canRxTaskStarted = true;
  canStateText = "RUNNING - LISTEN ONLY";
  clearFault(FAULT_CAN_DRIVER, "TWAI controller and receive task initialized successfully");
  addEvent("INFO", "TWAI running at 250 kbit/s in listen-only mode; CAN receive task active");
  Serial.println(F("[CAN] TWAI initialized: 250 kbit/s, LISTEN-ONLY."));
  Serial.print(F("[CAN] RX GPIO")); Serial.print(CAN_RX_PIN);
  Serial.print(F("; TX GPIO")); Serial.println(CAN_TX_PIN);
  Serial.print(F("[CAN] RX task: priority ")); Serial.print(CAN_RX_TASK_PRIORITY);
  Serial.print(F(", core ")); Serial.print(CAN_RX_TASK_CORE);
  Serial.print(F(", driver queue ")); Serial.println(CAN_RX_DRIVER_QUEUE_LENGTH);
}

// Runs independently of loop()/WebServer. The bounded ring is diagnostic
// history only: overwriting an old history entry does not mean a CAN frame was
// lost from acquisition. Driver-reported missed/overrun counters are separate.
void canReceiveTask(void *parameter) {
  (void)parameter;
  portENTER_CRITICAL(&canMux);
  canRxTaskRunning = true;
  portEXIT_CRITICAL(&canMux);

  uint8_t framesSinceYield = 0;
  for (;;) {
    twai_message_t message;
    esp_err_t result = twai_receive(&message, pdMS_TO_TICKS(20));
    if (result == ESP_OK) {
      CanFrameRecord record = {};
      record.timestampUs = (uint64_t)esp_timer_get_time();
      record.identifier = message.identifier;
      record.dataLengthCode = message.data_length_code > 8 ? 8 : message.data_length_code;
      record.extended = message.extd;
      record.remote = message.rtr;
      if (!record.remote) {
        for (uint8_t i = 0; i < record.dataLengthCode; i++) {
          record.data[i] = message.data[i];
        }
      }

      portENTER_CRITICAL(&canMux);
      canFramesReceived++;
      if (record.extended) canExtendedFrames++;
      else canStandardFrames++;
      if (record.remote) canRemoteFrames++;
      if (canFrameHistoryCount == CAN_FRAME_HISTORY_SIZE) {
        canHistoryOverwrites++;
      } else {
        canFrameHistoryCount++;
      }
      canFrameHistory[canFrameHistoryHead] = record;
      canFrameHistoryHead = (canFrameHistoryHead + 1) % CAN_FRAME_HISTORY_SIZE;
      canLastFrameTimestampUs = record.timestampUs;
      bool loggingSnapshot = canLoggingActive;
      if (loggingSnapshot) canLogEnqueueInFlight++;
      portEXIT_CRITICAL(&canMux);

      if (loggingSnapshot && canLogQueue) {
        CanLogRecord diskRecord = {};
        diskRecord.timestampUs = record.timestampUs;
        diskRecord.identifier = record.identifier;
        diskRecord.flags = (record.extended ? 0x01 : 0) | (record.remote ? 0x02 : 0);
        diskRecord.dlc = record.dataLengthCode;
        memcpy(diskRecord.data, record.data, sizeof(diskRecord.data));
        if (xQueueSend(canLogQueue, &diskRecord, 0) == pdTRUE) {
          canLogFramesQueued++;
          UBaseType_t waiting = uxQueueMessagesWaiting(canLogQueue);
          if (waiting > canLogQueueHighWater) canLogQueueHighWater = waiting;
        } else {
          canLogQueueDrops++;
        }
        portENTER_CRITICAL(&canMux);
        if (canLogEnqueueInFlight > 0) canLogEnqueueInFlight--;
        portEXIT_CRITICAL(&canMux);
      }

      // Under a continuously busy bus, periodically let the lower-priority
      // Arduino loop task run for one tick (HTTP, Wi-Fi, diagnostics).
      framesSinceYield++;
      if (framesSinceYield >= 16) {
        framesSinceYield = 0;
        vTaskDelay(1);
      }
    } else if (result != ESP_ERR_TIMEOUT) {
      portENTER_CRITICAL(&canMux);
      canReceiveErrors++;
      canLastError = (uint32_t)result;
      portEXIT_CRITICAL(&canMux);
      vTaskDelay(pdMS_TO_TICKS(1));
    }
  }
}

// -----------------------------------------------------------------------------
// CAN binary logger. Only the Arduino loop task accesses the SD file. The
// high-priority CAN task copies records into a bounded FreeRTOS queue and never
// waits for SD I/O. A full queue causes an explicitly counted logging drop.
// -----------------------------------------------------------------------------

bool startCANLogging() {
  if (!sdReady) { Serial.println(F("[CAN LOG] SD card is not mounted.")); return false; }
  if (canLoggingActive || canLogFileOpen) { Serial.println(F("[CAN LOG] Capture already active.")); return false; }
  if (!canLogQueue) { Serial.println(F("[CAN LOG] Logging queue unavailable.")); return false; }
  if (sdOperationActive) { Serial.println(F("[CAN LOG] SD is busy; try again.")); return false; }

  xQueueReset(canLogQueue); // discard any stale records from a prior stopped/failed session
  char path[40];
  bool found = false;
  for (uint32_t i = 1; i <= 99999; ++i) {
    snprintf(path, sizeof(path), "/diag/CAN_%05lu.BIN", (unsigned long)i);
    if (!SD.exists(path)) { canLogFileSequence = i; found = true; break; }
  }
  if (!found) { Serial.println(F("[CAN LOG] No unused capture filename available.")); return false; }

  sdOperationActive = true;
  canLogFile = SD.open(path, FILE_WRITE);
  if (!canLogFile) {
    sdOperationActive = false;
    canLogWriteFailures++;
    addEvent("ERROR", "Could not create CAN capture file");
    Serial.println(F("[CAN LOG] File creation failed."));
    return false;
  }

  CanLogFileHeader header = {};
  memcpy(header.magic, "SCNL", 4);
  header.formatVersion = CAN_LOG_FORMAT_VERSION;
  header.headerSize = sizeof(CanLogFileHeader);
  header.recordSize = sizeof(CanLogRecord);
  header.bitrate = CAN_BITRATE;
  header.bootId = faultBootId;
  header.startTimestampUs = (uint64_t)esp_timer_get_time();
  size_t written = canLogFile.write((const uint8_t *)&header, sizeof(header));
  canLogFile.flush();
  if (written != sizeof(header)) {
    canLogFile.close();
    sdOperationActive = false;
    canLogWriteFailures++;
    addEvent("ERROR", "CAN capture header write failed");
    Serial.println(F("[CAN LOG] Header write failed."));
    return false;
  }
  snprintf(canLogCurrentPath, sizeof(canLogCurrentPath), "%s", path);
  canLogFileOpen = true;
  canLoggingStopping = false;
  canLogFramesQueued = 0;
  canLogFramesWritten = 0;
  canLogQueueDrops = 0;
  canLogWriteFailures = 0;
  canLogWriteOperations = 0;
  canLogLastWriteMs = 0;
  canLogMaxWriteMs = 0;
  canLogLastBatchRecords = 0;
  canLogQueueHighWater = 0;
  canLogStartTimestampUs = (uint32_t)header.startTimestampUs;
  canLogLastFlushMs = millis();
  portENTER_CRITICAL(&canMux);
  canLoggingActive = true;
  portEXIT_CRITICAL(&canMux);
  canLogFilesCreated++;
  sdOperationActive = false;
  refreshSDUsageInformation();
  addEvent("INFO", "CAN binary capture started");
  Serial.print(F("[CAN LOG] Capture started: ")); Serial.println(canLogCurrentPath);
  return true;
}

void serviceCANLogger() {
  if (!canLogFileOpen || !canLogQueue || sdOperationActive) return;
  CanLogRecord batch[CAN_LOG_BATCH_RECORDS];
  uint8_t count = 0;
  while (count < CAN_LOG_BATCH_RECORDS && xQueueReceive(canLogQueue, &batch[count], 0) == pdTRUE) count++;
  if (count == 0) {
    if (canLoggingStopping && uxQueueMessagesWaiting(canLogQueue) == 0) stopCANLogging(true);
    return;
  }
  sdOperationActive = true;
  uint32_t started = millis();
  size_t bytesExpected = (size_t)count * sizeof(CanLogRecord);
  size_t bytesWritten = canLogFile.write((const uint8_t *)batch, bytesExpected);
  canLogFile.flush();
  canLogLastWriteMs = millis() - started;
  if (canLogLastWriteMs > canLogMaxWriteMs) canLogMaxWriteMs = canLogLastWriteMs;
  canLogWriteOperations++;
  canLogLastBatchRecords = count;
  recordDiagnostic(DIAG_ID_CAN_LOG_WRITE_MS, DIAG_TYPE_U32, canLogLastWriteMs, count);
  recordDiagnostic(DIAG_ID_CAN_LOG_BATCH_RECORDS, DIAG_TYPE_U32, count,
                   canLogQueue ? uxQueueMessagesWaiting(canLogQueue) : 0);
  if (bytesWritten == bytesExpected) {
    canLogFramesWritten += count;
  } else {
    canLogWriteFailures++;
    canLogQueueDrops += count;
    addEvent("ERROR", "CAN capture write failed; capture stopped to prevent silent corruption");
    portENTER_CRITICAL(&canMux);
    canLoggingActive = false;
    portEXIT_CRITICAL(&canMux);
    uint32_t waitStarted = millis();
    for (;;) {
      uint32_t inFlight;
      portENTER_CRITICAL(&canMux); inFlight = canLogEnqueueInFlight; portEXIT_CRITICAL(&canMux);
      if (inFlight == 0 || millis() - waitStarted >= 50) break;
      vTaskDelay(1);
    }
    canLogQueueDrops += uxQueueMessagesWaiting(canLogQueue);
    xQueueReset(canLogQueue);
    canLoggingStopping = true;
  }
  sdOperationActive = false;
  if (canLoggingStopping && uxQueueMessagesWaiting(canLogQueue) == 0) stopCANLogging(true);
}

bool stopCANLogging(bool reportResult) {
  if (!canLogFileOpen) {
    if (reportResult) Serial.println(F("[CAN LOG] No capture file is open."));
    return false;
  }
  portENTER_CRITICAL(&canMux);
  canLoggingActive = false;
  portEXIT_CRITICAL(&canMux);
  // Wait for any receive-task iteration that observed the previous active
  // state to finish its non-blocking queue send. No new enqueue can begin once
  // canLoggingActive has been cleared under the same critical section.
  uint32_t waitStarted = millis();
  for (;;) {
    uint32_t inFlight;
    portENTER_CRITICAL(&canMux);
    inFlight = canLogEnqueueInFlight;
    portEXIT_CRITICAL(&canMux);
    if (inFlight == 0 || millis() - waitStarted >= 50) break;
    vTaskDelay(1);
  }
  if (canLogQueue && uxQueueMessagesWaiting(canLogQueue) > 0) {
    canLoggingStopping = true;
    if (reportResult) Serial.println(F("[CAN LOG] Stop requested; draining queued records."));
    return true;
  }
  if (sdOperationActive) { canLoggingStopping = true; return true; }
  sdOperationActive = true;
  canLogFile.flush();
  canLogFile.close();
  canLogFileOpen = false;
  canLoggingStopping = false;
  sdOperationActive = false;
  refreshSDUsageInformation();
  addEvent("INFO", "CAN binary capture stopped and file closed");
  if (reportResult) {
    Serial.print(F("[CAN LOG] Capture finalized: ")); Serial.println(canLogCurrentPath);
    printCANLoggingStatus();
  }
  return true;
}

void printCANLoggingStatus() {
  Serial.println(); Serial.println(F("CAN BINARY LOGGING")); Serial.println(F("------------------"));
  Serial.print(F("State:                ")); Serial.println(canLoggingActive ? F("RECORDING") : (canLoggingStopping ? F("STOPPING / DRAINING") : F("STOPPED")));
  Serial.print(F("File:                 ")); Serial.println(canLogCurrentPath[0] ? canLogCurrentPath : "None");
  Serial.print(F("Format version:       ")); Serial.println(CAN_LOG_FORMAT_VERSION);
  Serial.print(F("Record size:          ")); Serial.print(sizeof(CanLogRecord)); Serial.println(F(" bytes"));
  Serial.print(F("Frames queued:        ")); Serial.println(canLogFramesQueued);
  Serial.print(F("Frames written:       ")); Serial.println(canLogFramesWritten);
  Serial.print(F("Queue depth:          ")); Serial.println(canLogQueue ? uxQueueMessagesWaiting(canLogQueue) : 0);
  Serial.print(F("Queue high-water:     ")); Serial.println(canLogQueueHighWater);
  Serial.print(F("Queue/write drops:    ")); Serial.println(canLogQueueDrops);
  Serial.print(F("Write failures:       ")); Serial.println(canLogWriteFailures);
  Serial.print(F("Write operations:     ")); Serial.println(canLogWriteOperations);
  Serial.print(F("Last/max write:       ")); Serial.print(canLogLastWriteMs); Serial.print(F(" / ")); Serial.print(canLogMaxWriteMs); Serial.println(F(" ms"));
  Serial.print(F("Last batch:           ")); Serial.println(canLogLastBatchRecords);
}

String makeCANLoggingJson() {
  String json = "{";
  json += "\"active\":" + String(canLoggingActive ? "true" : "false") + ",";
  json += "\"stopping\":" + String(canLoggingStopping ? "true" : "false") + ",";
  json += "\"fileOpen\":" + String(canLogFileOpen ? "true" : "false") + ",";
  json += "\"file\":\"" + jsonEscape(String(canLogCurrentPath)) + "\",";
  json += "\"formatVersion\":" + String(CAN_LOG_FORMAT_VERSION) + ",";
  json += "\"recordSize\":" + String(sizeof(CanLogRecord)) + ",";
  json += "\"framesQueued\":" + String(canLogFramesQueued) + ",";
  json += "\"framesWritten\":" + String(canLogFramesWritten) + ",";
  json += "\"queueDepth\":" + String(canLogQueue ? uxQueueMessagesWaiting(canLogQueue) : 0) + ",";
  json += "\"queueHighWater\":" + String(canLogQueueHighWater) + ",";
  json += "\"queueDrops\":" + String(canLogQueueDrops) + ",";
  json += "\"writeFailures\":" + String(canLogWriteFailures) + ",";
  json += "\"writeOperations\":" + String(canLogWriteOperations) + ",";
  json += "\"lastWriteMs\":" + String(canLogLastWriteMs) + ",";
  json += "\"maxWriteMs\":" + String(canLogMaxWriteMs) + ",";
  json += "\"lastBatchRecords\":" + String(canLogLastBatchRecords) + "}";
  return json;
}

String makeCANFramesJson() {
  CanFrameRecord snapshot[CAN_FRAME_HISTORY_SIZE];
  uint8_t count = 0;

  portENTER_CRITICAL(&canMux);
  count = canFrameHistoryCount;
  uint8_t first = (canFrameHistoryHead + CAN_FRAME_HISTORY_SIZE - count) % CAN_FRAME_HISTORY_SIZE;
  for (uint8_t i = 0; i < count; i++) {
    snapshot[i] = canFrameHistory[(first + i) % CAN_FRAME_HISTORY_SIZE];
  }
  portEXIT_CRITICAL(&canMux);

  String json;
  json.reserve(3000);
  json = "{\"frames\":[";
  for (uint8_t i = 0; i < count; i++) {
    const CanFrameRecord &frame = snapshot[i];
    if (i) json += ",";
    json += "{\"timestampUs\":" + String((unsigned long long)frame.timestampUs);
    json += ",\"id\":" + String(frame.identifier);
    json += ",\"extended\":" + String(frame.extended ? "true" : "false");
    json += ",\"remote\":" + String(frame.remote ? "true" : "false");
    json += ",\"dlc\":" + String(frame.dataLengthCode);
    json += ",\"data\":[";
    for (uint8_t b = 0; b < frame.dataLengthCode && !frame.remote; b++) {
      if (b) json += ",";
      json += String(frame.data[b]);
    }
    json += "]}";
  }
  json += "]}";
  return json;
}

void printCANFrames(uint8_t requestedCount) {
  CanFrameRecord snapshot[CAN_FRAME_HISTORY_SIZE];
  uint8_t count = 0;
  uint8_t limit = constrain(requestedCount, 1, CAN_FRAME_HISTORY_SIZE);

  portENTER_CRITICAL(&canMux);
  uint8_t available = canFrameHistoryCount;
  count = available < limit ? available : limit;
  uint8_t firstAvailable = (canFrameHistoryHead + CAN_FRAME_HISTORY_SIZE - available) % CAN_FRAME_HISTORY_SIZE;
  uint8_t first = (firstAvailable + available - count) % CAN_FRAME_HISTORY_SIZE;
  for (uint8_t i = 0; i < count; i++) {
    snapshot[i] = canFrameHistory[(first + i) % CAN_FRAME_HISTORY_SIZE];
  }
  portEXIT_CRITICAL(&canMux);

  Serial.println();
  Serial.println(F("RECENT CAN FRAMES (timestamps are microseconds since boot)"));
  Serial.println(F("---------------------------------------------------------"));
  if (!count) {
    Serial.println(F("No CAN frames received."));
    return;
  }
  for (uint8_t i = 0; i < count; i++) {
    const CanFrameRecord &frame = snapshot[i];
    Serial.printf("%llu us  ID=0x%0*lX  %s  %s  DLC=%u  DATA=",
                  (unsigned long long)frame.timestampUs,
                  frame.extended ? 8 : 3,
                  (unsigned long)frame.identifier,
                  frame.extended ? "EXT" : "STD",
                  frame.remote ? "RTR" : "DATA",
                  frame.dataLengthCode);
    if (frame.remote) {
      Serial.print(F("<remote request>"));
    } else {
      for (uint8_t b = 0; b < frame.dataLengthCode; b++) {
        if (b) Serial.print(' ');
        if (frame.data[b] < 16) Serial.print('0');
        Serial.print(frame.data[b], HEX);
      }
    }
    Serial.println();
  }
  Serial.println();
}

void updateCANState() {
  if (!canDriverInstalled) return;
  if (millis() - canLastStatePollMs < 100) return;
  canLastStatePollMs = millis();

  twai_status_info_t info;
  esp_err_t result = twai_get_status_info(&info);
  if (result != ESP_OK) {
    canInitialized = false;
    canLastError = (uint32_t)result;
    canStateText = "STATUS ERROR";
    if (!isFaultActive(FAULT_CAN_DRIVER)) {
      setFault(FAULT_CAN_DRIVER, "CAN", 3, "TWAI status query failed");
    }
    return;
  }

  portENTER_CRITICAL(&canMux);
  canDriverRxQueueDepth = info.msgs_to_rx;
  if (canDriverRxQueueDepth > canDriverRxQueueMaxDepth) {
    canDriverRxQueueMaxDepth = canDriverRxQueueDepth;
  }
  canDriverRxMissedCount = info.rx_missed_count;
  canDriverRxOverrunCount = info.rx_overrun_count;
  canDriverRxErrorCounter = info.rx_error_counter;
  canDriverTxErrorCounter = info.tx_error_counter;
  canDriverBusErrorCount = info.bus_error_count;
  canDriverArbitrationLostCount = info.arb_lost_count;
  canDriverTxFailedCount = info.tx_failed_count;
  portEXIT_CRITICAL(&canMux);

  uint32_t alerts = 0;
  if (twai_read_alerts(&alerts, 0) == ESP_OK) {
    if (alerts & TWAI_ALERT_RX_QUEUE_FULL) {
      portENTER_CRITICAL(&canMux);
      canRxQueueFullCount++;
      portEXIT_CRITICAL(&canMux);
    }
  }

  switch (info.state) {
    case TWAI_STATE_RUNNING:
      canInitialized = true;
      if (canRxTaskStarted) {
        canStateText = "RUNNING - LISTEN ONLY";
        if (isFaultActive(FAULT_CAN_DRIVER)) {
          clearFault(FAULT_CAN_DRIVER, "TWAI controller returned to running state");
        }
      } else {
        canStateText = "RX TASK FAILED";
        if (!isFaultActive(FAULT_CAN_DRIVER)) {
          setFault(FAULT_CAN_DRIVER, "CAN", 8, "CAN receive task is not running");
        }
      }
      if (canRecoveryRequested) canRecoveryCount++;
      canRecoveryRequested = false;
      canBusOffLatched = false;
      break;
    case TWAI_STATE_BUS_OFF:
      canInitialized = false;
      canStateText = "BUS OFF";
      if (!canBusOffLatched) {
        canBusOffCount++;
        canBusOffLatched = true;
      }
      if (!canRecoveryRequested) {
        esp_err_t recovery = twai_initiate_recovery();
        canLastError = (uint32_t)recovery;
        canRecoveryRequested = (recovery == ESP_OK);
        if (canRecoveryRequested) {
          canRecoveryAttempts++;
          canStateText = "RECOVERING";
          enqueueFaultEvent("CAN", "RECOVERY_ATTEMPT", 4, "TWAI bus-off recovery requested");
        } else {
          setFault(FAULT_CAN_DRIVER, "CAN", 4, "TWAI bus-off recovery request failed");
        }
      }
      if (!isFaultActive(FAULT_CAN_DRIVER)) {
        setFault(FAULT_CAN_DRIVER, "CAN", 4, "TWAI controller entered bus-off state");
      }
      break;
    case TWAI_STATE_RECOVERING:
      canInitialized = false;
      canStateText = "RECOVERING";
      break;
    case TWAI_STATE_STOPPED:
      canInitialized = false;
      canStateText = canRecoveryRequested ? "RECOVERY RESTARTING" : "STOPPED";
      if (!canRecoveryRequested && !isFaultActive(FAULT_CAN_DRIVER)) {
        setFault(FAULT_CAN_DRIVER, "CAN", 6, "TWAI controller stopped unexpectedly");
      }
      if (canRecoveryRequested) {
        esp_err_t restartResult = twai_start();
        canLastError = (uint32_t)restartResult;
        if (restartResult != ESP_OK) {
          canStateText = "RESTART FAILED";
          setFault(FAULT_CAN_DRIVER, "CAN", 5, "TWAI restart after bus-off recovery failed");
          canRecoveryRequested = false;
        }
      }
      break;
    default:
      canInitialized = false;
      canStateText = "UNKNOWN STATE";
      setFault(FAULT_CAN_DRIVER, "CAN", 7, "TWAI controller reported an unknown state");
      break;
  }
}

// -----------------------------------------------------------------------------
// Nonblocking WS2812 LED diagnostics
// -----------------------------------------------------------------------------

void setLedColor(uint8_t red, uint8_t green, uint8_t blue) {
  // Arduino-ESP32 3.3.x provides this RMT-backed WS2812 helper. Explicit GRB
  // order matches the standard WS2812B data order; arguments remain RGB.
  // A one-pixel RMT frame is transmitted synchronously; the application
  // pattern scheduler itself remains nonblocking and calls this only on edges.
  rgbLedWriteOrdered(RGB_LED_DATA_PIN, LED_COLOR_ORDER_GRB, red, green, blue);
}

void setLedRed(uint8_t brightness) {
  setLedColor(brightness, 0, 0);
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
  if (diagnosticLedActive || diagnosticLedAwaitingConfirmation) return;  // Preserve the diagnostic LED result until operator confirmation.
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

void startWiFiScan() {
  if (wifiScanInProgress) {
    Serial.println(F("[Wi-Fi] A scan is already in progress."));
    return;
  }

  Serial.println(F("[Wi-Fi] Starting asynchronous network scan..."));
  Serial.println(F("[Wi-Fi] Scan is asynchronous; the main loop continues, though Wi-Fi response may briefly slow."));

  int16_t result = WiFi.scanNetworks(true, true);
  if (result == WIFI_SCAN_RUNNING) {
    wifiScanInProgress = true;
    return;
  }

  if (result == WIFI_SCAN_FAILED) {
    Serial.println(F("[Wi-Fi] Scan could not be started."));
    WiFi.scanDelete();
    return;
  }

  printWiFiScanResults(result);
  WiFi.scanDelete();
}

void serviceWiFiScan() {
  if (!wifiScanInProgress) return;

  int16_t result = WiFi.scanComplete();
  if (result == WIFI_SCAN_RUNNING) return;

  wifiScanInProgress = false;
  if (result == WIFI_SCAN_FAILED) {
    Serial.println(F("[Wi-Fi] Scan failed."));
    WiFi.scanDelete();
    return;
  }

  printWiFiScanResults(result);
  WiFi.scanDelete();
}

void printWiFiScanResults(int16_t count) {
  Serial.println();
  Serial.println(F("WI-FI SCAN RESULTS"));
  Serial.println(F("------------------"));

  if (count <= 0) {
    Serial.println(F("No networks found."));
    Serial.println();
    return;
  }

  Serial.print(F("Networks found: "));
  Serial.println(count);
  Serial.println(F("#  RSSI(dBm)  CH  SECURITY  SSID"));
  Serial.println(F("-----------------------------------------------"));

  for (int16_t i = 0; i < count; ++i) {
    Serial.print(i + 1);
    Serial.print(F("  "));
    Serial.print(WiFi.RSSI(i));
    Serial.print(F("       "));
    Serial.print(WiFi.channel(i));
    Serial.print(F("   "));
    Serial.print(WiFi.encryptionType(i) == WIFI_AUTH_OPEN ? F("OPEN") : F("SECURED"));
    Serial.print(F("    "));
    String ssid = WiFi.SSID(i);
    Serial.println(ssid.length() ? ssid : String("<hidden SSID>"));
  }
  Serial.println();
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
  Serial.println(systemInfo.apMac);
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
  Serial.println(systemInfo.staMac);
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
    sdCardTypeText = "Unavailable";
    sdCardCapacityBytes = 0;
    sdTotalBytes = 0;
    sdUsedBytes = 0;
    sdMountFailures++;
    sdLastResult = "Card mount failed";
    setFault(FAULT_SD_MOUNT, "SD", 1, "SD card mount failed");
    addEvent("ERROR", "SD mount failed");
    Serial.println(F("[SD] Mount: FAIL"));
    Serial.println(F("[SD] Check card insertion, format, and pinout."));
  }
}

void remountSDCard() {
  if (canLoggingActive || canLoggingStopping) {
    Serial.println(F("[SD] Remount refused while CAN capture is active or draining."));
    return;
  }
  if (sdOperationActive || sdTestRunning) {
    Serial.println(F("[SD] Cannot remount while another SD operation is active."));
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
  sdCardTypeText = "Unavailable";
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

void refreshSDInformation(StatusJsonTimings *timings) {
  if (!sdReady) return;

  uint32_t operationStartUs = micros();
  uint8_t cardType = SD.cardType();
  if (timings) timings->sdCardTypeUs = micros() - operationStartUs;

  switch (cardType) {
    case CARD_MMC:  sdCardTypeText = "MMC"; break;
    case CARD_SD:   sdCardTypeText = "SDSC"; break;
    case CARD_SDHC: sdCardTypeText = "SDHC/SDXC"; break;
    case CARD_NONE: sdCardTypeText = "No card"; break;
    default:        sdCardTypeText = "Unknown"; break;
  }

  operationStartUs = micros();
  sdCardCapacityBytes = SD.cardSize();
  if (timings) timings->sdCardSizeUs = micros() - operationStartUs;

  operationStartUs = micros();
  sdTotalBytes = SD.totalBytes();
  if (timings) timings->sdTotalBytesUs = micros() - operationStartUs;

  operationStartUs = micros();
  sdUsedBytes = SD.usedBytes();
  if (timings) timings->sdUsedBytesUs = micros() - operationStartUs;
}

// Filesystem capacity is fixed for the mounted volume. Only used space needs
// refreshing after operations that create or extend files.
void refreshSDUsageInformation() {
  if (!sdReady) return;
  sdUsedBytes = SD.usedBytes();
}

// -----------------------------------------------------------------------------
// SD write/append/readback diagnostic
// -----------------------------------------------------------------------------

bool runSDTest() {
  if (canLoggingActive || canLoggingStopping) {
    Serial.println(F("[SD TEST] Refused while CAN capture is active or draining."));
    return false;
  }
  if (sdOperationActive) {
    Serial.println(F("[SD TEST] Cannot run while another SD operation is active."));
    return false;
  }
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
           "SD-CANbus-Logger V10 - TEST %lu - WRITE PASS",
           (unsigned long)sdTestCount);
  snprintf(secondRecord, sizeof(secondRecord),
           "SD-CANbus-Logger V10 - TEST %lu - APPEND PASS",
           (unsigned long)sdTestCount);

  Serial.println();
  Serial.println(F("[SD TEST] Starting write/append/readback verification..."));

  auto failTest = [&](const char *reason) -> bool {
    sdTestPassed = false;
    sdLastResult = reason;
    sdTestDurationMs = millis() - testStartMs;
    sdTestRunning = false;
    refreshSDUsageInformation();
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
  recordDiagnostic(DIAG_ID_SD_WRITE_DURATION_MS, DIAG_TYPE_U32,
                   sdLastWriteDurationMs, sdWriteOperations);

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
  recordDiagnostic(DIAG_ID_SD_WRITE_DURATION_MS, DIAG_TYPE_U32,
                   appendDurationMs, sdWriteOperations);

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
  recordDiagnostic(DIAG_ID_SD_READ_DURATION_MS, DIAG_TYPE_U32,
                   sdLastReadDurationMs, sdReadOperations);

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

  refreshSDUsageInformation();

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
// V10 hardware diagnostic test framework
// -----------------------------------------------------------------------------

void printDiagnosticTestResult(uint8_t index) {
  if (index >= DIAG_TEST_COUNT) return;
  const DiagnosticTestResult &test = diagnosticTests[index];
  Serial.printf("[DIAG] %-13s state=%s result=%s",
                test.id, test.state, test.result);
  if (test.durationMs) Serial.printf(" duration=%lu ms", (unsigned long)test.durationMs);
  Serial.printf(" detail=%s\n", test.detail);
}

void printDiagnosticTests() {
  Serial.println();
  Serial.println(F("HARDWARE DIAGNOSTIC TESTS"));
  Serial.println(F("-------------------------"));
  for (uint8_t i = 0; i < DIAG_TEST_COUNT; ++i) printDiagnosticTestResult(i);
  if (diagnosticTestActive >= 0) {
    Serial.printf("[DIAG] Active test: %s\n", diagnosticTests[diagnosticTestActive].id);
  } else if (diagnosticLedAwaitingConfirmation) {
    Serial.println(F("[DIAG] LED visual check is awaiting operator confirmation."));
  } else {
    Serial.println(F("[DIAG] No test running."));
  }
}

bool startDiagnosticTest(const char *testId) {
  if (!testId || !*testId) {
    Serial.println(F("[DIAG] Specify led, sd, can, or wifi."));
    return false;
  }
  if (diagnosticTestActive >= 0 || diagnosticLedAwaitingConfirmation) {
    Serial.println(F("[DIAG] Another test is active or awaiting confirmation."));
    return false;
  }

  int8_t index = -1;
  for (uint8_t i = 0; i < DIAG_TEST_COUNT; ++i) {
    if (!strcmp(testId, diagnosticTests[i].id)) {
      index = (int8_t)i;
      break;
    }
  }
  if (index < 0) {
    Serial.println(F("[DIAG] Unknown test. Use diag list."));
    return false;
  }

  DiagnosticTestResult &test = diagnosticTests[index];
  snprintf(test.state, sizeof(test.state), "RUNNING");
  snprintf(test.result, sizeof(test.result), "NOT TESTED");
  snprintf(test.detail, sizeof(test.detail), "Test in progress");
  test.lastRunMs = millis();
  test.durationMs = 0;
  diagnosticTestStartedMs = millis();
  diagnosticTestActive = index;
  addEvent("INFO", "Hardware diagnostic test started");

  Serial.printf("[DIAG] Starting %s...\n", test.id);

  if (index == DIAG_TEST_LED) {
    if (!ledInitialized) {
      snprintf(test.state, sizeof(test.state), "COMPLETE");
      snprintf(test.result, sizeof(test.result), "FAIL");
      snprintf(test.detail, sizeof(test.detail), "LED driver is not initialized");
      test.durationMs = millis() - diagnosticTestStartedMs;
      diagnosticTestActive = -1;
      printDiagnosticTestResult(index);
      return true;
    }
    diagnosticLedActive = true;
    diagnosticLedAwaitingConfirmation = false;
    diagnosticLedPhase = 0;
    setLedColor(32, 0, 0);
    diagnosticLedNextMs = millis() + DIAGNOSTIC_LED_PHASE_MS;
    snprintf(test.detail, sizeof(test.detail), "Observe three red, three green, and three blue pulses; then confirm pass or fail");
    return true;
  }

  if (index == DIAG_TEST_SD) {
    // A deliberate resource conflict is not evidence of SD hardware failure.
    if (canLoggingActive || canLoggingStopping || sdOperationActive || sdTestRunning) {
      snprintf(test.state, sizeof(test.state), "COMPLETE");
      snprintf(test.result, sizeof(test.result), "WARN");
      snprintf(test.detail, sizeof(test.detail), "%s",
               (canLoggingActive || canLoggingStopping) ?
               "Not run: CAN capture is active or draining" :
               "Not run: another SD operation is active");
      test.durationMs = millis() - diagnosticTestStartedMs;
      diagnosticTestActive = -1;
      printDiagnosticTestResult(index);
      return true;
    }
    // Reuse the established SD verification routine and its existing safety
    // interlocks. This operation is synchronous, as the V08e sdtest is.
    bool passed = runSDTest();
    snprintf(test.state, sizeof(test.state), "COMPLETE");
    snprintf(test.result, sizeof(test.result), "%s", passed ? "PASS" : "FAIL");
    snprintf(test.detail, sizeof(test.detail), "%s",
             passed ? "Write, append, and readback verified" : sdLastResult.c_str());
    test.durationMs = millis() - diagnosticTestStartedMs;
    diagnosticTestActive = -1;
    addEvent(passed ? "INFO" : "ERROR", "SD hardware diagnostic completed");
    printDiagnosticTestResult(index);
    return true;
  }

  if (index == DIAG_TEST_CAN) {
    if (!canInitialized) {
      snprintf(test.state, sizeof(test.state), "COMPLETE");
      snprintf(test.result, sizeof(test.result), "FAIL");
      snprintf(test.detail, sizeof(test.detail), "TWAI controller is not initialized");
      test.durationMs = millis() - diagnosticTestStartedMs;
      diagnosticTestActive = -1;
      printDiagnosticTestResult(index);
      return true;
    }
    diagnosticCanStartFrames = canFramesReceived;
    diagnosticCanStartMs = millis();
    snprintf(test.detail, sizeof(test.detail), "Observing CAN receive activity for 5 seconds");
    return true;
  }

  if (index == DIAG_TEST_WIFI) {
    bool apOk = wifiApStarted && WiFi.softAPIP() != IPAddress(0, 0, 0, 0);
    bool staExpected = configuredStaSsid.length() > 0 && !wifiStaManuallyDisconnected;
    bool staOk = wifiStaConnected && WiFi.status() == WL_CONNECTED;
    const char *verdict = !apOk ? "FAIL" : (staExpected && !staOk ? "WARN" : "PASS");
    snprintf(test.state, sizeof(test.state), "COMPLETE");
    snprintf(test.result, sizeof(test.result), "%s", verdict);
    if (!apOk) {
      snprintf(test.detail, sizeof(test.detail), "Access point is not operational");
    } else if (staExpected && !staOk) {
      snprintf(test.detail, sizeof(test.detail), "AP operational; configured infrastructure STA is not connected");
    } else if (staOk) {
      snprintf(test.detail, sizeof(test.detail), "AP and infrastructure STA are connected");
    } else {
      snprintf(test.detail, sizeof(test.detail), "AP operational; no infrastructure STA connection is configured/expected");
    }
    test.durationMs = millis() - diagnosticTestStartedMs;
    diagnosticTestActive = -1;
    addEvent(!strcmp(verdict, "FAIL") ? "ERROR" :
             (!strcmp(verdict, "WARN") ? "WARN" : "INFO"),
             "Wi-Fi diagnostic completed");
    printDiagnosticTestResult(index);
    return true;
  }

  diagnosticTestActive = -1;
  return false;
}

void serviceDiagnosticTests() {
  if (diagnosticTestActive == DIAG_TEST_CAN) {
    if ((uint32_t)(millis() - diagnosticCanStartMs) >= 5000) {
      DiagnosticTestResult &test = diagnosticTests[DIAG_TEST_CAN];
      uint32_t frames = canFramesReceived - diagnosticCanStartFrames;
      snprintf(test.state, sizeof(test.state), "COMPLETE");
      snprintf(test.result, sizeof(test.result), "%s", frames ? "PASS" : "WARN");
      if (frames) {
        snprintf(test.detail, sizeof(test.detail),
                 "%lu frames received during 5-second observation",
                 (unsigned long)frames);
      } else {
        snprintf(test.detail, sizeof(test.detail),
                 "No frames observed; silence alone does not prove a CAN hardware fault");
      }
      test.durationMs = millis() - diagnosticTestStartedMs;
      diagnosticTestActive = -1;
      addEvent(frames ? "INFO" : "WARN", "CAN activity diagnostic completed");
      printDiagnosticTestResult(DIAG_TEST_CAN);
    }
  }

  if (diagnosticLedActive && (int32_t)(millis() - diagnosticLedNextMs) >= 0) {
    diagnosticLedPhase++;
    if (diagnosticLedPhase >= 18) {
      setLedColor(0, 0, 0);
      diagnosticLedActive = false;
      diagnosticLedAwaitingConfirmation = true;
      DiagnosticTestResult &test = diagnosticTests[DIAG_TEST_LED];
      snprintf(test.state, sizeof(test.state), "AWAITING_CONFIRMATION");
      snprintf(test.result, sizeof(test.result), "NOT TESTED");
      snprintf(test.detail, sizeof(test.detail), "Three red, three green, and three blue pulses displayed; operator confirmation required");
      test.durationMs = millis() - diagnosticTestStartedMs;
      Serial.println(F("[DIAG] LED RGB sequence complete. Inspect red, green, and blue pulses, then enter: diag confirm pass|fail"));
      return;
    }
    uint8_t colorPhase = diagnosticLedPhase / 6;
    bool colorOn = (diagnosticLedPhase % 2) == 0;
    uint8_t level = colorOn ? 32 : 0;
    if (colorPhase == 0) setLedColor(level, 0, 0);
    else if (colorPhase == 1) setLedColor(0, level, 0);
    else setLedColor(0, 0, level);
    diagnosticLedNextMs = millis() + DIAGNOSTIC_LED_PHASE_MS;
  }
}

String makeDiagnosticsJson() {
  String json;
  json.reserve(900);
  json = "{\"active\":";
  if (diagnosticTestActive >= 0) {
    json += "\""; json += diagnosticTests[diagnosticTestActive].id; json += "\"";
  } else if (diagnosticLedAwaitingConfirmation) {
    json += "\"led_confirmation\"";
  } else {
    json += "null";
  }
  json += ",\"tests\":[";
  for (uint8_t i = 0; i < DIAG_TEST_COUNT; ++i) {
    if (i) json += ",";
    const DiagnosticTestResult &test = diagnosticTests[i];
    json += "{\"id\":\""; json += test.id;
    json += "\",\"name\":\""; json += jsonEscape(String(test.name));
    json += "\",\"state\":\""; json += test.state;
    json += "\",\"result\":\""; json += test.result;
    json += "\",\"detail\":\""; json += jsonEscape(String(test.detail));
    json += "\",\"last_run_ms\":"; json += String(test.lastRunMs);
    json += ",\"duration_ms\":"; json += String(test.durationMs);
    json += "}";
  }
  json += "]}";
  return json;
}

void handleDiagnosticsStatus() {
  uint32_t started = micros();
  sendHttpResponse(200, "application/json", makeDiagnosticsJson());
  recordHttp("GET", "/api/diagnostics", started, 200);
}

void handleDiagnosticsRun() {
  uint32_t started = micros();
  String testId = server.arg("test");
  bool accepted = startDiagnosticTest(testId.c_str());
  sendHttpResponse(accepted ? 202 : 409, "application/json", makeDiagnosticsJson());
  recordHttp("POST", "/api/diagnostics/run", started, accepted ? 202 : 409);
}

void completeLedDiagnostic(bool passed) {
  DiagnosticTestResult &test = diagnosticTests[DIAG_TEST_LED];
  diagnosticLedAwaitingConfirmation = false;
  diagnosticTestActive = -1;  // Release the global diagnostic lock after operator confirmation.
  snprintf(test.state, sizeof(test.state), "COMPLETE");
  snprintf(test.result, sizeof(test.result), "%s", passed ? "PASS" : "FAIL");
  snprintf(test.detail, sizeof(test.detail), "%s",
           passed ? "Operator confirmed visible red, green, and blue LED output" :
                    "Operator reported one or more RGB LED outputs were not correct");
  addEvent(passed ? "INFO" : "ERROR", passed ?
           "LED RGB visual diagnostic confirmed by operator" :
           "LED RGB visual diagnostic failed by operator confirmation");
  printDiagnosticTestResult(DIAG_TEST_LED);
}

void handleDiagnosticsConfirm() {
  uint32_t started = micros();
  String verdict = server.arg("result");
  bool valid = diagnosticLedAwaitingConfirmation &&
               (verdict == "pass" || verdict == "fail");
  if (valid) {
    completeLedDiagnostic(verdict == "pass");
  }
  sendHttpResponse(valid ? 200 : 409, "application/json", makeDiagnosticsJson());
  recordHttp("POST", "/api/diagnostics/confirm", started, valid ? 200 : 409);
}

// -----------------------------------------------------------------------------
// HTTP server and instrumentation
// -----------------------------------------------------------------------------

void configureWebServer() {
  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/status", HTTP_GET, handleStatus);
  server.on("/api/can-frames", HTTP_GET, handleCANFrames);
  server.on("/api/logging", HTTP_GET, handleCANLoggingStatus);
  server.on("/api/logging/start", HTTP_POST, handleCANLoggingStart);
  server.on("/api/logging/stop", HTTP_POST, handleCANLoggingStop);
  server.on("/api/events", HTTP_GET, handleEvents);
  server.on("/api/http-errors", HTTP_GET, handleHttpErrors);
  server.on("/api/loop-events", HTTP_GET, handleLoopEvents);
  server.on("/api/diagnostics", HTTP_GET, handleDiagnosticsStatus);
  server.on("/api/diagnostics/status", HTTP_GET, handleDiagnosticsStatus);
  server.on("/api/diagnostics/run", HTTP_POST, handleDiagnosticsRun);
  server.on("/api/diagnostics/confirm", HTTP_POST, handleDiagnosticsConfirm);
  server.on("/api/verbose", HTTP_GET, handleVerboseStatus);
  server.on("/api/verbose/start", HTTP_POST, handleVerboseStart);
  server.on("/api/verbose/stop", HTTP_POST, handleVerboseStop);
  server.on("/api/verbose/clear", HTTP_POST, handleVerboseClear);
  server.on("/api/verbose/freeze", HTTP_POST, handleVerboseFreeze);
  server.on("/api/verbose/resume", HTTP_POST, handleVerboseResume);
  server.on("/download-verbose", HTTP_GET, handleVerboseDownload);
  server.on("/api/faults", HTTP_GET, handleFaults);
  server.on("/api/files", HTTP_GET, handleFileList);
  server.on("/api/file", HTTP_GET, handleFileView);
  server.on("/download", HTTP_GET, handleFileDownload);
  server.on("/api/reset", HTTP_POST, handleResetRequest);
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
  snprintf(httpLastRoute, sizeof(httpLastRoute), "%s", route);
  httpLastStatusCode = statusCode;
  if (duration > httpMaxDurationUs) {
    httpMaxDurationUs = duration;
    snprintf(httpMaxRoute, sizeof(httpMaxRoute), "%s", route);
    httpMaxStatusCode = statusCode;
  }

  recordDiagnostic(DIAG_ID_HTTP_HANDLER_US, DIAG_TYPE_U32, duration, statusCode);
  recordDiagnostic(DIAG_ID_HTTP_ROUTE_HASH, DIAG_TYPE_EVENT,
                   diagnosticStringHash(route),
                   !strcmp(method, "POST") ? 2 : 1);
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
  StatusJsonTimings timings = {};

  // SD metadata is cached at mount and after completed SD mutations. Do not
  // interrogate the card/filesystem as part of a routine status refresh.
  uint32_t phaseStartUs = micros();
  String json = makeStatusJson(&timings);
  // Includes String construction and all status JSON field generation.
  uint32_t jsonBuildElapsedUs = micros() - phaseStartUs;
  timings.jsonTotalUs = jsonBuildElapsedUs;

  phaseStartUs = micros();
  sendHttpResponse(200, "application/json", json);
  timings.responseSendUs = micros() - phaseStartUs;

  // Record the breakdown after sending the response so recorder writes do not
  // contaminate the measured JSON construction or response-send intervals.
  recordDiagnostic(DIAG_ID_STATUS_SD_REFRESH_US, DIAG_TYPE_U32, timings.sdRefreshUs);
  recordDiagnostic(DIAG_ID_STATUS_SD_CARD_TYPE_US, DIAG_TYPE_U32, timings.sdCardTypeUs);
  recordDiagnostic(DIAG_ID_STATUS_SD_CARD_SIZE_US, DIAG_TYPE_U32, timings.sdCardSizeUs);
  recordDiagnostic(DIAG_ID_STATUS_SD_TOTAL_BYTES_US, DIAG_TYPE_U32, timings.sdTotalBytesUs);
  recordDiagnostic(DIAG_ID_STATUS_SD_USED_BYTES_US, DIAG_TYPE_U32, timings.sdUsedBytesUs);
  recordDiagnostic(DIAG_ID_STATUS_SYSTEM_MEMORY_US, DIAG_TYPE_U32, timings.systemMemoryUs);
  recordDiagnostic(DIAG_ID_STATUS_WIFI_US, DIAG_TYPE_U32, timings.wifiUs);
  recordDiagnostic(DIAG_ID_STATUS_SD_JSON_US, DIAG_TYPE_U32, timings.sdJsonUs);
  recordDiagnostic(DIAG_ID_STATUS_HTTP_LOOP_FAULT_US, DIAG_TYPE_U32, timings.httpLoopFaultUs);
  recordDiagnostic(DIAG_ID_STATUS_CAN_SNAPSHOT_US, DIAG_TYPE_U32, timings.canSnapshotUs);
  recordDiagnostic(DIAG_ID_STATUS_CAN_JSON_US, DIAG_TYPE_U32, timings.canJsonUs);
  recordDiagnostic(DIAG_ID_STATUS_JSON_TOTAL_US, DIAG_TYPE_U32, timings.jsonTotalUs);
  recordDiagnostic(DIAG_ID_STATUS_RESPONSE_SEND_US, DIAG_TYPE_U32, timings.responseSendUs);
  recordDiagnostic(DIAG_ID_STATUS_SYS_HEAP_US, DIAG_TYPE_U32, timings.systemHeapUs);
  recordDiagnostic(DIAG_ID_STATUS_SYS_DEVICE_US, DIAG_TYPE_U32, timings.systemDeviceUs);
  recordDiagnostic(DIAG_ID_STATUS_SYS_RUNTIME_US, DIAG_TYPE_U32, timings.systemRuntimeUs);
  recordDiagnostic(DIAG_ID_STATUS_SYS_HEALTH_US, DIAG_TYPE_U32, timings.systemHealthUs);
  recordDiagnostic(DIAG_ID_STATUS_SYS_HEALTH_DETAILS_US, DIAG_TYPE_U32, timings.systemHealthDetailsUs);
  recordDiagnostic(DIAG_ID_STATUS_SYS_OTHER_US, DIAG_TYPE_U32, timings.systemOtherUs);
  recordDiagnostic(DIAG_ID_STATUS_SYS_CHIP_MODEL_US, DIAG_TYPE_U32, timings.systemChipModelUs);
  recordDiagnostic(DIAG_ID_STATUS_SYS_CHIP_REVISION_US, DIAG_TYPE_U32, timings.systemChipRevisionUs);
  recordDiagnostic(DIAG_ID_STATUS_SYS_CPU_FREQ_US, DIAG_TYPE_U32, timings.systemCpuFreqUs);
  recordDiagnostic(DIAG_ID_STATUS_SYS_FLASH_SIZE_US, DIAG_TYPE_U32, timings.systemFlashSizeUs);
  recordDiagnostic(DIAG_ID_STATUS_SYS_SKETCH_SIZE_US, DIAG_TYPE_U32, timings.systemSketchSizeUs);
  recordDiagnostic(DIAG_ID_STATUS_SYS_FREE_SKETCH_US, DIAG_TYPE_U32, timings.systemFreeSketchUs);

  recordHttp("GET", "/api/status", startUs, 200);
}

void handleCANFrames() {
  uint32_t startUs = micros();
  String json = makeCANFramesJson();
  sendHttpResponse(200, "application/json", json);
  recordHttp("GET", "/api/can-frames", startUs, 200);
}

void handleCANLoggingStatus() {
  uint32_t started = micros();
  sendHttpResponse(200, "application/json", makeCANLoggingJson());
  recordHttp("GET", "/api/logging", started, 200);
}

void handleCANLoggingStart() {
  uint32_t started = micros();
  bool ok = startCANLogging();
  sendHttpResponse(ok ? 200 : 409, "application/json", makeCANLoggingJson());
  recordHttp("POST", "/api/logging/start", started, ok ? 200 : 409);
}

void handleCANLoggingStop() {
  uint32_t started = micros();
  bool ok = stopCANLogging(false);
  sendHttpResponse(ok ? 200 : 409, "application/json", makeCANLoggingJson());
  recordHttp("POST", "/api/logging/stop", started, ok ? 200 : 409);
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

void handleVerboseStatus() {
  uint32_t started = micros();
  sendHttpResponse(200, "application/json", makeVerboseStatusJson());
  recordHttp("GET", "/api/verbose", started, 200);
}

void handleVerboseStart() {
  uint32_t started = micros();
  setVerboseLogging(true);
  sendHttpResponse(200, "application/json", makeVerboseStatusJson());
  recordHttp("POST", "/api/verbose/start", started, 200);
}

void handleVerboseStop() {
  uint32_t started = micros();
  setVerboseLogging(false);
  sendHttpResponse(200, "application/json", makeVerboseStatusJson());
  recordHttp("POST", "/api/verbose/stop", started, 200);
}

void handleVerboseClear() {
  uint32_t started = micros();
  clearVerboseRecords();
  sendHttpResponse(200, "application/json", makeVerboseStatusJson());
  recordHttp("POST", "/api/verbose/clear", started, 200);
}

void handleVerboseFreeze() {
  uint32_t started = micros();
  addEvent("INFO", "Verbose diagnostic buffer frozen");
  verboseLoggingFrozen = true;
  sendHttpResponse(200, "application/json", makeVerboseStatusJson());
  recordHttp("POST", "/api/verbose/freeze", started, 200);
}

void handleVerboseResume() {
  uint32_t started = micros();
  setVerboseLogging(true);
  sendHttpResponse(200, "application/json", makeVerboseStatusJson());
  recordHttp("POST", "/api/verbose/resume", started, 200);
}

void handleVerboseDownload() {
  uint32_t started = micros();
  if (canLoggingActive || canLoggingStopping) {
    sendHttpResponse(409, "application/json",
                     "{\"error\":\"Stop CAN capture before downloading verbose diagnostics.\"}");
    recordHttp("GET", "/download-verbose", started, 409);
    return;
  }
  uint16_t snapshotHead;
  uint16_t snapshotCount;
  portENTER_CRITICAL(&diagnosticMux);
  snapshotHead = diagnosticRecordHead;
  snapshotCount = diagnosticRecordCount;
  portEXIT_CRITICAL(&diagnosticMux);

  server.sendHeader("Content-Disposition", "attachment; filename=DIAG_VERBOSE.csv");
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/csv", "");
  char metadata[128];
  snprintf(metadata, sizeof(metadata),
           "# diagnostic_format_version=%u,record_size=%u,boot_id=%lu,timestamp=esp_timer_get_time_us_since_boot\r\n",
           (unsigned)DIAG_RECORD_FORMAT_VERSION, (unsigned)sizeof(DiagnosticRecord),
           (unsigned long)faultBootId);
  server.sendContent(metadata);
  server.sendContent("timestamp_us,record_id,name,type,value,context,flags,detail\r\n");

  String chunk;
  chunk.reserve(768);
  uint16_t first = (snapshotHead + DIAG_RECORD_CAPACITY - snapshotCount) % DIAG_RECORD_CAPACITY;
  for (uint16_t i = 0; i < snapshotCount; i++) {
    DiagnosticRecord record;
    portENTER_CRITICAL(&diagnosticMux);
    record = diagnosticRecords[(first + i) % DIAG_RECORD_CAPACITY];
    portEXIT_CRITICAL(&diagnosticMux);

    chunk += String((unsigned long long)record.timestampUs);
    chunk += ","; chunk += String(record.recordId);
    chunk += ","; chunk += diagnosticRecordName(record.recordId);
    chunk += ","; chunk += diagnosticRecordTypeName(record.dataType);
    chunk += ",";
    if (record.dataType == DIAG_TYPE_I32) chunk += String((long)(int32_t)record.value);
    else chunk += String((unsigned long)record.value);
    chunk += ","; chunk += String((unsigned long)record.context);
    chunk += ","; chunk += String(record.flags);
    chunk += ",\""; chunk += diagnosticRecordDetail(record); chunk += "\"";
    chunk += "\r\n";
    if (chunk.length() >= 640) {
      server.sendContent(chunk);
      chunk = "";
    }
  }
  if (chunk.length()) server.sendContent(chunk);
  server.sendContent("");
  recordHttp("GET", "/download-verbose", started, 200);
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

void initializeSystemInfoCache() {
  // These values are fixed for this boot. ESP.getSketchSize() is deliberately
  // called here, not from makeStatusJson(), because it is unusually expensive
  // on the installed Arduino-ESP32 core.
  systemInfo.arduinoCoreVersion = getArduinoCoreVersion();
  systemInfo.espIdfVersion = ESP.getSdkVersion();
  systemInfo.chipModel = ESP.getChipModel();
  systemInfo.chipRevision = ESP.getChipRevision();
  systemInfo.cpuMHz = ESP.getCpuFreqMHz();
  systemInfo.flashChipSize = ESP.getFlashChipSize();
  systemInfo.sketchSize = ESP.getSketchSize();
  systemInfo.freeSketchSpace = ESP.getFreeSketchSpace();
  systemInfo.resetReason = getResetReasonText();
  systemInfo.initialized = true;
}

void initializeSystemNetworkIdentityCache() {
  // The AP must be started before its MAC address can be queried.
  systemInfo.apMac = WiFi.softAPmacAddress();
  systemInfo.staMac = WiFi.macAddress();
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
      faultQueueDropCount || faultLogWriteFailures || canRxQueueFullCount ||
      canDriverRxMissedCount || canDriverRxOverrunCount ||
      (canDriverInstalled && !canRxTaskStarted) ||
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
  if (canLogQueueDrops) details += "CAN capture queue/write drops=" + String(canLogQueueDrops) + "; ";
  if (canLogWriteFailures) details += "CAN capture write failures=" + String(canLogWriteFailures) + "; ";
  if (canRxQueueFullCount) details += "TWAI RX queue-full alerts=" + String(canRxQueueFullCount) + "; ";
  if (canDriverRxMissedCount) details += "TWAI driver missed frames=" + String(canDriverRxMissedCount) + "; ";
  if (canDriverRxOverrunCount) details += "TWAI driver RX overruns=" + String(canDriverRxOverrunCount) + "; ";
  if (canDriverInstalled && !canRxTaskStarted) details += "CAN receive task is not running; ";
  if (details.length() == 0) return "No active health warnings.";
  details.remove(details.length() - 2);
  return details;
}

void printSystemHealthDetails() {
  Serial.print(F("Health detail:        "));
  Serial.println(getSystemHealthDetails());
}

String makeStatusJson(StatusJsonTimings *timings) {
  uint32_t functionStartUs = micros();
  uint32_t phaseStartUs = functionStartUs;
  uint32_t systemOperationStartUs = timings ? micros() : 0;
  uint32_t freeHeap = ESP.getFreeHeap();
  uint32_t largestBlock = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
  uint32_t internalFree = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  if (timings) timings->systemHeapUs += micros() - systemOperationStartUs;
  float fragmentation = freeHeap ? 100.0f * (1.0f - ((float)largestBlock / (float)freeHeap)) : 0.0f;

  uint32_t canFramesReceivedSnapshot, canStandardFramesSnapshot, canExtendedFramesSnapshot;
  uint32_t canRemoteFramesSnapshot, canHistoryOverwritesSnapshot, canRxQueueFullSnapshot;
  uint32_t canReceiveErrorsSnapshot, canQueueDepthSnapshot, canQueueMaxDepthSnapshot;
  uint32_t canRxMissedSnapshot, canRxOverrunSnapshot, canRxErrorSnapshot, canTxErrorSnapshot;
  uint32_t canBusErrorsSnapshot, canArbLostSnapshot, canTxFailedSnapshot;
  uint32_t canHistoryCountSnapshot, canLastErrorSnapshot;
  uint64_t canLastFrameTimestampSnapshot;
  bool canRxTaskRunningSnapshot;
  uint32_t canSnapshotStartUs = micros();
  portENTER_CRITICAL(&canMux);
  canFramesReceivedSnapshot = canFramesReceived;
  canStandardFramesSnapshot = canStandardFrames;
  canExtendedFramesSnapshot = canExtendedFrames;
  canRemoteFramesSnapshot = canRemoteFrames;
  canHistoryOverwritesSnapshot = canHistoryOverwrites;
  canRxQueueFullSnapshot = canRxQueueFullCount;
  canReceiveErrorsSnapshot = canReceiveErrors;
  canQueueDepthSnapshot = canDriverRxQueueDepth;
  canQueueMaxDepthSnapshot = canDriverRxQueueMaxDepth;
  canRxMissedSnapshot = canDriverRxMissedCount;
  canRxOverrunSnapshot = canDriverRxOverrunCount;
  canRxErrorSnapshot = canDriverRxErrorCounter;
  canTxErrorSnapshot = canDriverTxErrorCounter;
  canBusErrorsSnapshot = canDriverBusErrorCount;
  canArbLostSnapshot = canDriverArbitrationLostCount;
  canTxFailedSnapshot = canDriverTxFailedCount;
  canLastFrameTimestampSnapshot = canLastFrameTimestampUs;
  canHistoryCountSnapshot = canFrameHistoryCount;
  canLastErrorSnapshot = canLastError;
  canRxTaskRunningSnapshot = canRxTaskRunning;
  portEXIT_CRITICAL(&canMux);
  if (timings) timings->canSnapshotUs = micros() - canSnapshotStartUs;

  String json;
  json.reserve(4400);

  json += "{";
  json += "\"project\":\"" + String(PROJECT_NAME) + "\",";
  json += "\"firmware\":\"" + String(FIRMWARE_VERSION) + "\",";
  json += "\"build\":\"" + String(BUILD_LABEL) + "\",";

  {
    if (timings) systemOperationStartUs = micros();
    json += "\"arduinoCore\":\"" + jsonEscape(systemInfo.arduinoCoreVersion) + "\",";
  }

  {
    if (timings) systemOperationStartUs = micros();
    json += "\"espIdf\":\"" + jsonEscape(systemInfo.espIdfVersion) + "\",";
  }

  json += "\"chip\":\"" + jsonEscape(systemInfo.chipModel) + "\",";
  json += "\"chipRevision\":" + String(systemInfo.chipRevision) + ",";
  json += "\"cpuMHz\":" + String(systemInfo.cpuMHz) + ",";
  json += "\"flashMB\":" + String((double)systemInfo.flashChipSize / 1048576.0, 1) + ",";
  json += "\"sketchKB\":" + String((double)systemInfo.sketchSize / 1024.0, 1) + ",";
  json += "\"freeSketchKB\":" + String((double)systemInfo.freeSketchSpace / 1024.0, 1) + ",";

  json += "\"uptimeMs\":" + String(millis() - bootMillis) + ",";
  {
    if (timings) systemOperationStartUs = micros();
    String uptimeText = formatUptime(millis() - bootMillis);
    if (timings) timings->systemRuntimeUs += micros() - systemOperationStartUs;
    json += "\"uptime\":\"" + uptimeText + "\",";
  }

  {
    if (timings) systemOperationStartUs = micros();
    json += "\"resetReason\":\"" + systemInfo.resetReason + "\",";
  }

  {
    if (timings) systemOperationStartUs = micros();
    String systemHealth = getSystemHealth();
    if (timings) timings->systemHealthUs += micros() - systemOperationStartUs;
    json += "\"health\":\"" + systemHealth + "\",";
  }

  {
    if (timings) systemOperationStartUs = micros();
    String systemHealthDetails = getSystemHealthDetails();
    if (timings) timings->systemHealthDetailsUs += micros() - systemOperationStartUs;
    json += "\"healthDetails\":\"" + jsonEscape(systemHealthDetails) + "\",";
  }

  json += "\"heapFree\":" + String(freeHeap) + ",";
  if (timings) systemOperationStartUs = micros();
  uint32_t minimumFreeHeap = ESP.getMinFreeHeap();
  if (timings) timings->systemHeapUs += micros() - systemOperationStartUs;
  json += "\"heapMinimum\":" + String(minimumFreeHeap) + ",";
  json += "\"heapLargestBlock\":" + String(largestBlock) + ",";
  json += "\"heapInternalFree\":" + String(internalFree) + ",";
  json += "\"heapFragmentationPercent\":" + String(fragmentation, 1) + ",";
  if (timings) {
    timings->systemMemoryUs = (micros() - phaseStartUs) - timings->canSnapshotUs;
    uint32_t measuredSystemOperationsUs = timings->systemHeapUs +
                                          timings->systemDeviceUs +
                                          timings->systemRuntimeUs +
                                          timings->systemHealthUs +
                                          timings->systemHealthDetailsUs;
    timings->systemOtherUs = timings->systemMemoryUs > measuredSystemOperationsUs
                               ? timings->systemMemoryUs - measuredSystemOperationsUs
                               : 0;
  }
  phaseStartUs = micros();

  json += "\"wifiMode\":\"AP+STA\",";
  json += "\"apStarted\":";
  json += (wifiApStarted ? "true" : "false");
  json += ",\"apSsid\":\"" + String(WIFI_AP_SSID) + "\",";
  json += "\"apIp\":\"" + WiFi.softAPIP().toString() + "\",";
  json += "\"apMac\":\"" + systemInfo.apMac + "\",";
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
  json += "\"staMac\":\"" + systemInfo.staMac + "\",";
  json += "\"staAttempts\":" + String(staConnectAttempts) + ",";
  json += "\"staDisconnects\":" + String(staDisconnectCount) + ",";
  json += "\"staLastDisconnectReason\":" + String(lastStaDisconnectReason) + ",";
  json += "\"staLastConnectionDurationMs\":" + String(lastStaConnectionDurationMs) + ",";
  json += "\"mdnsStarted\":";
  json += (mdnsStarted ? "true" : "false");
  json += ",\"mdnsHostname\":\"" + String(MDNS_HOSTNAME) + ".local\",";
  if (timings) timings->wifiUs = micros() - phaseStartUs;
  phaseStartUs = micros();

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
  if (timings) timings->sdJsonUs = micros() - phaseStartUs;
  phaseStartUs = micros();

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
  json += "\"httpMaxRoute\":\"" + String(httpMaxRoute) + "\",";
  json += "\"httpMaxStatusCode\":" + String(httpMaxStatusCode) + ",";
  json += "\"httpErrorHistoryCount\":" + String(httpErrorCount) + ",";

  json += "\"loopCount\":" + String(loopCount) + ",";
  json += "\"loopLastDurationUs\":" + String(loopLastDurationUs) + ",";
  json += "\"loopMaxDurationUs\":" + String(loopMaxDurationUs) + ",";
  json += "\"loopLastGapUs\":" + String(loopLastGapUs) + ",";
  json += "\"loopMaxGapUs\":" + String(loopMaxGapUs) + ",";
  json += "\"loopTimingEventCount\":" + String(loopTimingEventCount) + ",";
  json += "\"loopDiagnosticThresholdMs\":" + String(loopDiagnosticThresholdMs) + ",";
  json += "\"loopEventHistoryCount\":" + String(loopEventCount) + ",";
  json += "\"eventCount\":" + String(eventCount) + ",";
  json += "\"bootCheckpointCount\":" + String(bootCheckpointCount) + ",";
  json += "\"faultActiveCount\":" + String(getActiveFaultCount()) + ",";
  json += "\"faultQueueCount\":" + String(faultQueueCount) + ",";
  json += "\"faultQueueDrops\":" + String(faultQueueDropCount) + ",";
  json += "\"faultLogReady\":" + String(faultLogReady ? "true" : "false") + ",";
  json += "\"faultLogWriteFailures\":" + String(faultLogWriteFailures) + ",";
  json += "\"ledState\":\"" + String(getLedPatternText()) + "\",";
  if (timings) timings->httpLoopFaultUs = micros() - phaseStartUs;
  phaseStartUs = micros();
  json += "\"canInitialized\":" + String(canInitialized ? "true" : "false") + ",";
  json += "\"canState\":\"" + jsonEscape(canStateText) + "\",";
  json += "\"canBitrate\":" + String(CAN_BITRATE) + ",";
  json += "\"canRxPin\":" + String(CAN_RX_PIN) + ",";
  json += "\"canTxPin\":" + String(CAN_TX_PIN) + ",";
  json += "\"canInitAttempts\":" + String(canInitAttempts) + ",";
  json += "\"canInitFailures\":" + String(canInitFailures) + ",";
  json += "\"canBusOffCount\":" + String(canBusOffCount) + ",";
  json += "\"canRecoveryCount\":" + String(canRecoveryCount) + ",";
  json += "\"canRecoveryAttempts\":" + String(canRecoveryAttempts) + ",";
  json += "\"canRxTaskStarted\":" + String(canRxTaskStarted ? "true" : "false") + ",";
  json += "\"canRxTaskRunning\":" + String(canRxTaskRunningSnapshot ? "true" : "false") + ",";
  json += "\"canFramesReceived\":" + String(canFramesReceivedSnapshot) + ",";
  json += "\"canStandardFrames\":" + String(canStandardFramesSnapshot) + ",";
  json += "\"canExtendedFrames\":" + String(canExtendedFramesSnapshot) + ",";
  json += "\"canRemoteFrames\":" + String(canRemoteFramesSnapshot) + ",";
  json += "\"canHistoryCount\":" + String(canHistoryCountSnapshot) + ",";
  json += "\"canHistoryOverwrites\":" + String(canHistoryOverwritesSnapshot) + ",";
  json += "\"canRxQueueFullCount\":" + String(canRxQueueFullSnapshot) + ",";
  json += "\"canReceiveErrors\":" + String(canReceiveErrorsSnapshot) + ",";
  json += "\"canDriverRxQueueDepth\":" + String(canQueueDepthSnapshot) + ",";
  json += "\"canDriverRxQueueMaxDepth\":" + String(canQueueMaxDepthSnapshot) + ",";
  json += "\"canDriverRxMissedCount\":" + String(canRxMissedSnapshot) + ",";
  json += "\"canDriverRxOverrunCount\":" + String(canRxOverrunSnapshot) + ",";
  json += "\"canDriverRxErrorCounter\":" + String(canRxErrorSnapshot) + ",";
  json += "\"canDriverTxErrorCounter\":" + String(canTxErrorSnapshot) + ",";
  json += "\"canDriverBusErrorCount\":" + String(canBusErrorsSnapshot) + ",";
  json += "\"canDriverArbitrationLostCount\":" + String(canArbLostSnapshot) + ",";
  json += "\"canDriverTxFailedCount\":" + String(canTxFailedSnapshot) + ",";
  json += "\"canLastFrameTimestampUs\":" + String((unsigned long long)canLastFrameTimestampSnapshot) + ",";
  json += "\"canLastError\":" + String(canLastErrorSnapshot) + ",";
  json += "\"canLogActive\":" + String(canLoggingActive ? "true" : "false") + ",";
  json += "\"canLogFramesWritten\":" + String(canLogFramesWritten) + ",";
  json += "\"canLogQueueDrops\":" + String(canLogQueueDrops) + ",";
  json += "\"sdFileListRequests\":" + String(sdFileListRequests) + ",";
  json += "\"sdFileReadRequests\":" + String(sdFileReadRequests) + ",";
  json += "\"sdFileDownloadRequests\":" + String(sdFileDownloadRequests) + ",";
  json += "\"sdFileOperationFailures\":" + String(sdFileOperationFailures);
  json += "}";
  if (timings) {
    timings->canJsonUs = micros() - phaseStartUs;
    timings->jsonTotalUs = micros() - functionStartUs;
  }

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
  page.reserve(42000);

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
main{max-width:1180px;margin:auto}
.page-nav{display:flex;flex-wrap:wrap;gap:6px;margin:0 0 16px;padding:8px;background:#e7ebf2;border-radius:10px}
.page-nav button{margin:0;padding:9px 12px;background:#475467}
.page-nav button.active{background:#155eef}
.page-panel{display:none}
.page-panel.active{display:block}
.page-heading{margin:2px 0 12px}
.page-description{color:#667085;font-size:.88rem;margin:-6px 0 14px}
.page-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(290px,1fr));gap:12px;align-items:start}
.page-grid>.card{margin:0}
.page-grid>.card:only-child{max-width:none}
@media(max-width:600px){body{padding:10px}.page-nav{gap:4px}.page-nav button{font-size:.82rem;padding:8px 9px}}
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
<p class="sub">Browser interface - V10</p>
<nav class="page-nav" aria-label="Main navigation">
  <button type="button" data-page="overview" onclick="showPage('overview')">Overview</button>
  <button type="button" data-page="can" onclick="showPage('can')">CAN Monitor</button>
  <button type="button" data-page="logging" onclick="showPage('logging')">CAN Logging</button>
  <button type="button" data-page="diagnostics" onclick="showPage('diagnostics')">Hardware Diagnostics</button>
  <button type="button" data-page="system" onclick="showPage('system')">System</button>
  <button type="button" data-page="files" onclick="showPage('files')">Files &amp; Tools</button>
  <button type="button" data-page="help" onclick="showPage('help')">Help / About</button>
</nav>
<div id="pageHost"></div>

<div class="grid" id="legacyCardGrid">
  <section class="card">
    <h2>Hardware diagnostics</h2>
    <p class="small muted">Run individual checks. The LED test requires visual confirmation; CAN silence is reported as a warning, not a hardware failure.</p>
    <div>
      <button onclick="runDiagnostic('led')">LED</button>
      <button onclick="runDiagnostic('sd')">SD</button>
      <button onclick="runDiagnostic('can')">CAN activity</button>
      <button onclick="runDiagnostic('wifi')">Wi-Fi</button>
      <button class="secondary" onclick="refreshDiagnostics()">Refresh</button>
    </div>
    <div id="diagnosticTests" class="small">Loading diagnostic results...</div>
    <div id="diagnosticConfirm" hidden>
      <p>Inspect the LED output. Did the LED pulse red, then green, then blue (three pulses per color)?</p>
      <button onclick="confirmDiagnostic('pass')">Pass</button>
      <button class="secondary" onclick="confirmDiagnostic('fail')">Fail</button>
    </div>
    <p id="diagnosticMessage" class="small"></p>
  </section>

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
    <div class="row"><span>Last handler time / route</span><span class="value" id="httpTiming">--</span></div>
    <div class="row"><span>Maximum handler time / route</span><span class="value" id="httpMaxTiming">--</span></div>
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
  </section>

  <section class="card">
    <h2>CAN controller</h2>
    <div class="row"><span>Controller</span><span class="value" id="canState">--</span></div>
    <div class="row"><span>Receive task</span><span class="value" id="canRxTask">--</span></div>
    <div class="row"><span>Mode / bitrate</span><span class="value">Listen-only / 250 kbit/s</span></div>
    <div class="row"><span>RX / TX pins</span><span class="value">GPIO26 / GPIO27</span></div>
    <div class="row"><span>Init attempts / failures</span><span class="value" id="canInit">--</span></div>
    <div class="row"><span>Bus-off / recoveries</span><span class="value" id="canRecovery">--</span></div>
    <div class="row"><span>Frames received</span><span class="value" id="canFramesReceived">--</span></div>
    <div class="row"><span>Standard / extended / remote</span><span class="value" id="canFrameTypes">--</span></div>
    <div class="row"><span>History retained / overwritten</span><span class="value" id="canHistory">--</span></div>
    <div class="row"><span>Driver RX queue current / max</span><span class="value" id="canQueueDepth">--</span></div>
    <div class="row"><span>RX queue full alerts</span><span class="value" id="canQueueFull">--</span></div>
    <div class="row"><span>Driver missed / overrun</span><span class="value" id="canDriverLoss">--</span></div>
    <div class="row"><span>RX / TX error counters</span><span class="value" id="canErrorCounters">--</span></div>
    <div class="row"><span>Bus errors / arbitration lost</span><span class="value" id="canBusErrors">--</span></div>
    <div class="row"><span>Receive API errors</span><span class="value" id="canReceiveErrors">--</span></div>
  </section>
</div>

<section class="card">
  <h2>Verbose diagnostic recording</h2>
  <div class="row"><span>Recorder state</span><span class="value" id="verboseState">--</span></div>
  <div class="row"><span>Records retained / capacity</span><span class="value" id="verboseRecords">--</span></div>
  <div class="row"><span>Total / overwritten</span><span class="value" id="verboseTotals">--</span></div>
  <div class="row"><span>RAM allocation</span><span class="value" id="verboseMemory">--</span></div>
  <div class="row"><span>Sample interval</span><span class="value" id="verboseInterval">--</span></div>
  <div class="row"><span>Loop event threshold</span><span class="value" id="verboseThreshold">--</span></div>
  <div class="row"><span>Recorder last / max cost</span><span class="value" id="verboseRecordCost">--</span></div>
  <div class="row"><span>Sample last / max cost</span><span class="value" id="verboseSampleCost">--</span></div>
  <button onclick="startVerbose()">Start recording</button>
  <button class="secondary" onclick="stopVerbose()">Stop</button>
  <button class="secondary" onclick="freezeVerbose()">Freeze buffer</button>
  <button class="secondary" onclick="resumeVerbose()">Resume</button>
  <button class="secondary" onclick="clearVerbose()">Clear buffer</button>
  <button class="secondary" onclick="refreshVerbose()">Refresh status</button>
  <a href="/download-verbose">Download diagnostic CSV</a>
  <p class="small muted">RAM-only circular recording. Enabled state is retained across reboot. Oldest records are overwritten when the buffer fills. Stop CAN capture before downloading diagnostics; CAN frames remain in the separate binary capture.</p>
  <div id="verboseMessage" class="small muted"></div>
</section>

<section class="card">
  <h2>CAN capture</h2>
  <div class="row"><span>Capture state</span><span class="value" id="logState">--</span></div>
  <div class="row"><span>Current file</span><span class="value" id="logFile">--</span></div>
  <div class="row"><span>Frames queued / written</span><span class="value" id="logFrames">--</span></div>
  <div class="row"><span>Queue depth / high-water</span><span class="value" id="logQueue">--</span></div>
  <div class="row"><span>Dropped / write failures</span><span class="value" id="logDrops">--</span></div>
  <div class="row"><span>Last / max write</span><span class="value" id="logTiming">--</span></div>
  <button onclick="startCapture()">Start new capture</button>
  <button class="secondary" onclick="stopCapture()">Stop capture</button>
  <button class="secondary" onclick="refreshLogging()">Refresh capture status</button>
  <p class="small muted">Binary format V1. Files are stored in /diag as CAN_XXXXX.BIN. Download for offline decoding; browser preview is not applicable.</p>
</section>

<section class="card">
  <h2>Recent CAN frames</h2>
  <div id="canFrames" class="small muted">Waiting for CAN frame history...</div>
</section>

<section class="card">
  <h2>SD verification</h2>
  <button onclick="runTest()">Run SD write/read test</button>
  <button class="secondary" onclick="refreshStatus()">Refresh status</button>
  <p id="message"></p>
</section>

<section class="card">
  <h2>SD diagnostic files</h2>
  <div class="row"><span>Directory</span><span class="value">/diag</span></div>
  <button class="secondary" onclick="refreshFiles()">Refresh file list</button>
  <div id="sdFiles" class="small muted">Loading SD files...</div>
  <label for="fileContents">File preview</label>
  <pre id="fileContents">Select a file to view its contents.</pre>
</section>

<section class="card">
  <h2>System control</h2>
  <button class="secondary" onclick="requestReset()">Restart device</button>
  <p class="small muted">An active CAN capture is stopped, drained, and closed before restart. If capture finalization cannot complete within 5 seconds, the restart is aborted. The Wi-Fi access point and dashboard will be temporarily unavailable.</p>
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

<p class="small muted">Dashboard status refresh interval: 5 seconds. Loop timing event threshold defaults to 100 ms and is configurable over Serial. Verbose diagnostic recording uses a bounded RAM circular buffer and can be enabled independently of browser navigation. CAN/TWAI remains at 250 kbit/s in listen-only mode; CAN frames are received in a dedicated task and written to versioned binary captures under /diag using a bounded queue.</p>
</main>

<script>
let captureAccessPaused=false;
let activePageId='overview';
const dashboardPages=[
  {id:'overview',title:'Overview',description:'Device condition and key operating indicators.'},
  {id:'can',title:'CAN Monitor',description:'Live CAN reception and recent frame history.'},
  {id:'logging',title:'CAN Logging',description:'Start, stop, and inspect binary CAN captures.'},
  {id:'diagnostics',title:'Hardware Diagnostics',description:'Run explicit hardware checks and review subsystem results.'},
  {id:'system',title:'System',description:'Network, memory, timing, fault, and firmware diagnostics.'},
  {id:'files',title:'Files & Tools',description:'Inspect diagnostic files and access maintenance functions.'},
  {id:'help',title:'Help / About',description:'Device operation, access, and firmware information.'}
];
const dashboardCardPages={
  'System health':'overview','Device identity':'overview','Memory':'overview','Fault diagnostics':'overview',
  'Recent CAN frames':'can','CAN controller':'can','CAN capture':'logging',
  'Hardware diagnostics':'diagnostics','microSD':'diagnostics','SD operation diagnostics':'diagnostics','SD verification':'diagnostics',
  'Wi-Fi access point':'system','Infrastructure Wi-Fi':'system','HTTP diagnostics':'system','Main-loop diagnostics':'system',
  'Verbose diagnostic recording':'system','System control':'system','Boot memory checkpoints':'system','Recent diagnostic events':'system',
  'SD diagnostic files':'files'
};
function organizeDashboardPages(){
  const host=document.getElementById('pageHost');
  const panels={};
  dashboardPages.forEach(page=>{
    const panel=document.createElement('section');
    panel.className='page-panel';panel.id='page-'+page.id;
    const heading=document.createElement('h2');heading.className='page-heading';heading.textContent=page.title;
    const description=document.createElement('p');description.className='page-description';description.textContent=page.description;
    const grid=document.createElement('div');grid.className='page-grid';
    panel.append(heading,description,grid);host.appendChild(panel);panels[page.id]=grid;
  });
  const cards=Array.from(document.querySelectorAll('#legacyCardGrid > .card, main > .card'));
  cards.forEach(card=>{
    const heading=card.querySelector('h2');
    const title=heading?heading.textContent.trim():'';
    const target=dashboardCardPages[title]||'system';
    panels[target].appendChild(card);
  });
  const legacy=document.getElementById('legacyCardGrid');
  if(legacy)legacy.remove();
  const help=document.createElement('section');help.className='card';
  help.innerHTML='<h2>Help / About</h2><p>SD-CANbus-Logger is a local-first CAN monitoring and logging device. The browser interface is hosted by the device; internet access and a dedicated application are not required.</p><div class="row"><span>Firmware</span><span class="value">V10</span></div><div class="row"><span>Local access point</span><span class="value">192.168.4.1</span></div><div class="row"><span>mDNS</span><span class="value">sd-canbus-logger.local</span></div><p class="small muted">CAN operates at 250 kbit/s in listen-only mode. Captures are stored as versioned binary files under /diag and can be converted offline with the supplied BIN-to-ASC utility. Stop capture before browsing or downloading SD files.</p><p class="small muted">Infrastructure Wi-Fi credentials are configured through the USB serial command interface. The access point remains available as a recovery interface.</p>';
  panels.help.appendChild(help);
}
function showPage(pageId){
  if(!dashboardPages.some(page=>page.id===pageId))pageId='overview';
  activePageId=pageId;
  document.querySelectorAll('.page-panel').forEach(panel=>panel.classList.toggle('active',panel.id==='page-'+pageId));
  document.querySelectorAll('.page-nav button[data-page]').forEach(button=>{
    const selected=button.dataset.page===pageId;
    button.classList.toggle('active',selected);
    if(selected)button.setAttribute('aria-current','page');else button.removeAttribute('aria-current');
  });
  refreshActivePage();
}
function refreshActivePage(){
  switch(activePageId){
    case 'overview': refreshFaults(); break;
    case 'can': refreshCanFrames(); break;
    case 'diagnostics': refreshDiagnostics(); break;
    case 'system': refreshFaults();refreshEvents();refreshHttpErrors();refreshLoopEvents();refreshVerbose();break;
    case 'files': refreshFiles();break;
  }
}
function setText(id,value){document.getElementById(id).textContent=value;}
function showCaptureFilePause(){
  const host=document.getElementById('sdFiles');
  host.textContent='SD file browsing and downloads are paused while CAN capture is active. Stop capture to access files.';
  document.getElementById('fileContents').textContent='File preview is unavailable during CAN capture.';
}
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
async function refreshFiles(){
  const host=document.getElementById('sdFiles');
  if(captureAccessPaused){showCaptureFilePause();return;}
  try{
    const data=await fetchJson('/api/files?path=%2Fdiag');
    host.replaceChildren();
    if(!data.entries.length){host.textContent='No files found.';return;}
    data.entries.forEach(entry=>{
      const row=document.createElement('div');
      row.className='row';
      const name=document.createElement('span');
      name.textContent=entry.name+(entry.directory?'/':'');
      const actions=document.createElement('span');
      actions.className='value';
      if(!entry.directory){
        const isBinary=entry.name.toLowerCase().endsWith('.bin');
        if(!isBinary){
          const view=document.createElement('button');view.className='secondary';view.textContent='View';
          view.onclick=()=>viewFile(entry.name);
          actions.append(view);
        }
        const download=document.createElement('button');download.textContent='Download';
        download.onclick=()=>{if(captureAccessPaused){setText('message','Stop CAN capture before downloading SD files.');return;}window.location.href='/download?path='+encodeURIComponent(entry.name.startsWith('/')?entry.name:'/diag/'+entry.name);};
        actions.append(download);
      }else{actions.textContent='Directory';}
      row.append(name,actions);host.appendChild(row);
    });
  }catch(e){host.textContent='File listing failed: '+e.message;}
}
async function viewFile(name){
  if(captureAccessPaused){setText('fileContents','File preview is unavailable during CAN capture.');return;}
  if(name.toLowerCase().endsWith('.bin')){setText('fileContents','Binary CAN captures cannot be displayed as text. Download the file and convert it to ASC using the supplied converter.');return;}
  const path=name.startsWith('/')?name:'/diag/'+name;
  const preview=document.getElementById('fileContents');
  preview.textContent='Loading '+path+'...';
  try{
    const response=await fetch('/api/file?path='+encodeURIComponent(path),{cache:'no-store'});
    if(!response.ok) throw new Error('HTTP '+response.status);
    preview.textContent=await response.text();
  }catch(e){preview.textContent='File view failed: '+e.message;}
}
async function requestReset(){
  if(!confirm('Restart SD-CANbus-Logger now? If CAN capture is active, it will be stopped and finalized first. The dashboard connection will be interrupted.')) return;
  try{
    const response=await fetch('/api/reset',{method:'POST'});
    if(!response.ok) throw new Error('HTTP '+response.status);
    setText('message','Restart requested. Reconnect after the device boots.');
  }catch(e){setText('message','Restart request failed: '+e.message);}
}
async function refreshLogging(){
  try{const s=await fetchJson('/api/logging');
    const wasPaused=captureAccessPaused;
    captureAccessPaused=!!(s.active||s.stopping);
    if(captureAccessPaused) showCaptureFilePause();
    else if(wasPaused) refreshFiles();
    setText('logState',s.active?'RECORDING':(s.stopping?'STOPPING / DRAINING':'STOPPED'));
    setText('logFile',s.file||'None');
    setText('logFrames',s.framesQueued+' / '+s.framesWritten);
    setText('logQueue',s.queueDepth+' / '+s.queueHighWater);
    setText('logDrops',s.queueDrops+' / '+s.writeFailures);
    setText('logTiming',s.lastWriteMs+' / '+s.maxWriteMs+' ms');
  }catch(e){setText('logState','Unavailable: '+e.message);}
}
async function startCapture(){
  if(!confirm('Start a new CAN capture file on the SD card?'))return;
  try{const r=await fetch('/api/logging/start',{method:'POST'});const s=await r.json();if(!r.ok)throw new Error('HTTP '+r.status);captureAccessPaused=true;showCaptureFilePause();setText('message','Capture started: '+s.file);refreshLogging();}
  catch(e){setText('message','Start capture failed: '+e.message);}
}
async function stopCapture(){
  if(!confirm('Stop CAN capture and finalize the file?'))return;
  try{const r=await fetch('/api/logging/stop',{method:'POST'});const s=await r.json();if(!r.ok)throw new Error('HTTP '+r.status);captureAccessPaused=false;setText('message','Capture stopped and file finalized. Refreshing SD file list.');refreshLogging();refreshFiles();}
  catch(e){setText('message','Stop capture failed: '+e.message);}
}

async function refreshCanFrames(){
  const host=document.getElementById('canFrames');
  try{
    const result=await fetchJson('/api/can-frames');
    if(!result.frames.length){host.textContent='No CAN frames received. Connect a CAN bus to exercise reception.';return;}
    const lines=result.frames.slice().reverse().map(f=>{
      const id='0x'+Number(f.id).toString(16).toUpperCase().padStart(f.extended?8:3,'0');
      const data=f.remote?'RTR':f.data.map(b=>Number(b).toString(16).toUpperCase().padStart(2,'0')).join(' ');
      return f.timestampUs+' us  '+id+'  '+(f.extended?'EXT':'STD')+'  DLC '+f.dlc+'  '+data;
    });
    host.textContent=lines.join('\n');
    host.style.whiteSpace='pre-wrap';
  }catch(e){host.textContent='CAN frame history unavailable: '+e.message;}
}

async function refreshVerbose(){
  try{
    const s=await fetchJson('/api/verbose');
    setText('verboseState',!s.enabled?'DISABLED':(s.frozen?'FROZEN':'RECORDING'));
    setText('verboseRecords',s.records+' / '+s.capacity);
    setText('verboseTotals',s.total+' / '+s.overwritten);
    setText('verboseMemory',s.bufferBytes+' bytes');
    setText('verboseInterval',s.sampleIntervalMs+' ms');
    setText('verboseThreshold',s.loopThresholdMs+' ms');
    setText('verboseRecordCost',s.lastRecordCostUs+' / '+s.maxRecordCostUs+' us');
    setText('verboseSampleCost',s.lastSampleCostUs+' / '+s.maxSampleCostUs+' us');
  }catch(e){setText('verboseState','Unavailable: '+e.message);}
}
async function verboseAction(path,description){
  try{
    const r=await fetch(path,{method:'POST'});
    if(!r.ok)throw new Error('HTTP '+r.status);
    const s=await r.json();
    setText('verboseMessage',description+'; '+s.records+' records retained.');
    await refreshVerbose();
  }catch(e){setText('verboseMessage','Verbose recorder request failed: '+e.message);}
}
function startVerbose(){verboseAction('/api/verbose/start','Recording enabled');}
function stopVerbose(){verboseAction('/api/verbose/stop','Recording disabled');}
function freezeVerbose(){verboseAction('/api/verbose/freeze','Buffer frozen');}
function resumeVerbose(){verboseAction('/api/verbose/resume','Recording resumed');}
function clearVerbose(){if(confirm('Clear the retained verbose diagnostic records?'))verboseAction('/api/verbose/clear','Buffer cleared');}

async function refreshDiagnostics(){
  try{
    const data=await fetchJson('/api/diagnostics');
    const host=document.getElementById('diagnosticTests');
    host.replaceChildren();
    data.tests.forEach(test=>{
      const row=document.createElement('div');
      row.className='row';
      const label=document.createElement('span');
      label.textContent=test.name;
      const value=document.createElement('span');
      value.className='value';
      value.textContent=test.state+' / '+test.result+' — '+test.detail;
      row.append(label,value);
      host.appendChild(row);
    });
    document.getElementById('diagnosticConfirm').hidden=
      !(data.tests.some(t=>t.id==='led'&&t.state==='AWAITING_CONFIRMATION'));
    if(data.active) setText('diagnosticMessage','Running: '+data.active);
  }catch(e){
    setText('diagnosticMessage','Diagnostic status unavailable: '+e.message);
  }
}
async function runDiagnostic(testId){
  setText('diagnosticMessage','Starting '+testId+' diagnostic...');
  try{
    const response=await fetch('/api/diagnostics/run',{
      method:'POST',
      headers:{'Content-Type':'application/x-www-form-urlencoded'},
      body:'test='+encodeURIComponent(testId)
    });
    const data=await response.json();
    if(!response.ok) throw new Error('HTTP '+response.status);
    setText('diagnosticMessage',testId+' diagnostic accepted.');
    await refreshDiagnostics();
  }catch(e){
    setText('diagnosticMessage','Diagnostic request rejected: '+e.message);
    await refreshDiagnostics();
  }
}
async function confirmDiagnostic(result){
  try{
    const response=await fetch('/api/diagnostics/confirm',{
      method:'POST',
      headers:{'Content-Type':'application/x-www-form-urlencoded'},
      body:'result='+encodeURIComponent(result)
    });
    if(!response.ok) throw new Error('HTTP '+response.status);
    setText('diagnosticMessage','LED result recorded: '+result.toUpperCase());
    await refreshDiagnostics();
  }catch(e){setText('diagnosticMessage','LED confirmation failed: '+e.message);}
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
    setText('httpTiming',s.httpLastDurationUs+' us - '+s.httpLastRoute+' / '+s.httpLastStatusCode);
    setText('httpMaxTiming',s.httpMaxDurationUs+' us - '+s.httpMaxRoute+' / '+s.httpMaxStatusCode);
    setText('loopTimingEvents',s.loopTimingEventCount);
    setText('loopCount',s.loopCount);
    setText('loopDuration',s.loopLastDurationUs+' / '+s.loopMaxDurationUs+' us');
    setText('loopGap',s.loopLastGapUs+' / '+s.loopMaxGapUs+' us');
    setText('eventCount',s.eventCount);
    setText('faultActiveCount',s.faultActiveCount);
    setText('ledState',s.ledState);
    setText('canState',s.canState);
    setText('canInit',s.canInitAttempts+' / '+s.canInitFailures);
    setText('canRecovery',s.canBusOffCount+' / '+s.canRecoveryCount);
    setText('canRxTask',s.canRxTaskRunning?'RUNNING':(s.canRxTaskStarted?'STARTING':'NOT RUNNING'));
    setText('canFramesReceived',s.canFramesReceived);
    setText('canFrameTypes',s.canStandardFrames+' / '+s.canExtendedFrames+' / '+s.canRemoteFrames);
    setText('canHistory',s.canHistoryCount+' / '+s.canHistoryOverwrites);
    setText('canQueueDepth',s.canDriverRxQueueDepth+' / '+s.canDriverRxQueueMaxDepth);
    setText('canQueueFull',s.canRxQueueFullCount);
    setText('canDriverLoss',s.canDriverRxMissedCount+' / '+s.canDriverRxOverrunCount);
    setText('canErrorCounters',s.canDriverRxErrorCounter+' / '+s.canDriverTxErrorCounter);
    setText('canBusErrors',s.canDriverBusErrorCount+' / '+s.canDriverArbitrationLostCount);
    setText('canReceiveErrors',s.canReceiveErrors);
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
organizeDashboardPages();
showPage('overview');
refreshStatus();
refreshLogging();
setInterval(refreshStatus,5000);
// Logging state remains polled on every page because it gates SD file access.
setInterval(refreshLogging,5000);
// Other page-specific endpoints are polled only while their page is visible.
setInterval(refreshActivePage,5000);
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
  } else if (!strcmp(command, "diag list") || !strcmp(command, "diag status")) {
    printDiagnosticTests();
  } else if (!strncmp(command, "diag run ", 9)) {
    startDiagnosticTest(command + 9);
  } else if (!strcmp(command, "diag confirm pass")) {
    if (diagnosticLedAwaitingConfirmation) completeLedDiagnostic(true);
    else Serial.println(F("[DIAG] No LED test is awaiting confirmation."));
  } else if (!strcmp(command, "diag confirm fail")) {
    if (diagnosticLedAwaitingConfirmation) completeLedDiagnostic(false);
    else Serial.println(F("[DIAG] No LED test is awaiting confirmation."));
  } else if (!strcmp(command, "verbose status")) {
    printVerboseStatus();
  } else if (!strcmp(command, "verbose on")) {
    setVerboseLogging(true);
    printVerboseStatus();
  } else if (!strcmp(command, "verbose off")) {
    setVerboseLogging(false);
    printVerboseStatus();
  } else if (!strcmp(command, "verbose freeze")) {
    addEvent("INFO", "Verbose diagnostic buffer frozen");
    verboseLoggingFrozen = true;
    printVerboseStatus();
  } else if (!strcmp(command, "verbose resume")) {
    setVerboseLogging(true);
    printVerboseStatus();
  } else if (!strcmp(command, "verbose clear")) {
    clearVerboseRecords();
    printVerboseStatus();
  } else if (!strncmp(command, "verbose interval ", 17)) {
    uint32_t value = (uint32_t)strtoul(command + 17, nullptr, 10);
    if (value < DIAG_MIN_SAMPLE_INTERVAL_MS || value > DIAG_MAX_SAMPLE_INTERVAL_MS) {
      Serial.printf("[VERBOSE] Interval must be %lu-%lu ms.\n",
                    (unsigned long)DIAG_MIN_SAMPLE_INTERVAL_MS,
                    (unsigned long)DIAG_MAX_SAMPLE_INTERVAL_MS);
    } else {
      diagnosticSampleIntervalMs = value;
      diagnosticLastSampleMs = 0;
      if (diagnosticPreferences.begin("diagcfg", false)) {
        diagnosticPreferences.putUInt("diagMs", value);
        diagnosticPreferences.end();
      }
      printVerboseStatus();
    }
  } else if (!strcmp(command, "diag threshold") || !strcmp(command, "diag threshold status")) {
    Serial.printf("[DIAG] Loop timing threshold: %lu ms (range %lu-%lu).\n",
                  (unsigned long)loopDiagnosticThresholdMs,
                  (unsigned long)DIAG_MIN_LOOP_THRESHOLD_MS,
                  (unsigned long)DIAG_MAX_LOOP_THRESHOLD_MS);
  } else if (!strncmp(command, "diag threshold ", 15)) {
    uint32_t value = (uint32_t)strtoul(command + 15, nullptr, 10);
    if (value < DIAG_MIN_LOOP_THRESHOLD_MS || value > DIAG_MAX_LOOP_THRESHOLD_MS) {
      Serial.printf("[DIAG] Threshold must be %lu-%lu ms.\n",
                    (unsigned long)DIAG_MIN_LOOP_THRESHOLD_MS,
                    (unsigned long)DIAG_MAX_LOOP_THRESHOLD_MS);
    } else {
      loopDiagnosticThresholdMs = value;
      if (diagnosticPreferences.begin("diagcfg", false)) {
        diagnosticPreferences.putUInt("loopMs", value);
        diagnosticPreferences.end();
      }
      Serial.printf("[DIAG] Loop timing threshold set to %lu ms.\n", (unsigned long)value);
    }
  } else if (!strcmp(command, "log status")) {
    printCANLoggingStatus();
  } else if (!strcmp(command, "log start")) {
    startCANLogging();
  } else if (!strcmp(command, "log stop")) {
    stopCANLogging(true);
  } else if (!strcmp(command, "can")) {
    printCANFrames(16);
  } else if (!strncmp(command, "can frames", 10) && (command[10] == '\0' || command[10] == ' ')) {
    char *countText = command + 10;
    while (*countText == ' ') countText++;
    uint8_t count = *countText ? (uint8_t)constrain(atoi(countText), 1, CAN_FRAME_HISTORY_SIZE) : 16;
    printCANFrames(count);
  } else if (!strcmp(command, "sdtest")) {
    runSDTest();
  } else if (!strcmp(command, "sd remount")) {
    remountSDCard();
  } else if (!strcmp(command, "reset")) {
    requestSoftwareReset("serial command");
  } else if (!strncmp(command, "ls", 2) && (command[2] == '\0' || command[2] == ' ')) {
    char *path = command + 2;
    while (*path == ' ') path++;
    printSDDirectory(*path ? path : "/diag");
  } else if (!strncmp(command, "cat ", 4)) {
    printSDFile(command + 4, "cat", 0);
  } else if (!strncmp(command, "head ", 5)) {
    char *path = command + 5;
    char *space = strrchr(path, ' ');
    uint16_t lines = 10;
    if (space) { *space = '\0'; lines = (uint16_t)constrain(atoi(space + 1), 1, 200); }
    printSDFile(path, "head", lines);
  } else if (!strncmp(command, "tail ", 5)) {
    char *path = command + 5;
    char *space = strrchr(path, ' ');
    uint16_t lines = 10;
    if (space) { *space = '\0'; lines = (uint16_t)constrain(atoi(space + 1), 1, 50); }
    printSDFile(path, "tail", lines);
  } else if (!strcmp(command, "wifi status")) {
    printWiFiStatus();
  } else if (!strcmp(command, "wifi connect")) {
    if (configuredStaSsid.length() == 0) {
      Serial.println(F("[Wi-Fi] No saved infrastructure credentials. Use wifi connect SSID|PASSWORD first."));
      return;
    }
    Serial.print(F("[Wi-Fi] Reconnecting using saved credentials for SSID: "));
    Serial.println(configuredStaSsid);
    startStationConnection(configuredStaSsid, configuredStaPassword, false);
  } else if (!strcmp(command, "wifi scan")) {
    startWiFiScan();
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
  Serial.println(F("loopevents                   Recent loop timing anomalies (configured threshold)"));
  Serial.println(F("checkpoints                  Boot memory checkpoints"));
  Serial.println(F("diag list/status              List hardware diagnostic tests and results"));
  Serial.println(F("diag run <test>               Run led, sd, can, or wifi test"));
  Serial.println(F("diag confirm pass|fail        Confirm the LED visual test result"));
  Serial.println(F("verbose status               Verbose recorder status"));
  Serial.println(F("verbose on/off               Enable/disable RAM diagnostic recording"));
  Serial.println(F("verbose freeze/resume        Freeze/resume the RAM record buffer"));
  Serial.println(F("verbose clear                Clear retained RAM records"));
  Serial.println(F("verbose interval <ms>        Set sample interval (250-10000 ms)"));
  Serial.println(F("diag threshold [ms]          Show/set loop event threshold (10-1000 ms)"));
  Serial.println(F("can                          Show the 16 most recent CAN frames"));
  Serial.println(F("can frames [count]           Show 1-32 recent CAN frames"));
  Serial.println(F("log start                    Start a new binary CAN capture"));
  Serial.println(F("log stop                     Stop, drain, and close capture"));
  Serial.println(F("log status                   Show CAN logging counters"));
  Serial.println(F("sdtest                       Run SD write/append/readback test"));
  Serial.println(F("sd remount                    Unmount, remount, and verify the SD card"));
  Serial.println(F("reset                        Finalize active capture, then restart"));
  Serial.println(F("ls [path]                    List /diag files (default /diag)"));
  Serial.println(F("cat <path>                   Display complete diagnostic file"));
  Serial.println(F("head <path> [lines]          Display first N lines (default 10)"));
  Serial.println(F("tail <path> [lines]          Display last N lines (default 10)"));
  Serial.println(F("wifi status                  Show AP and infrastructure status"));
  Serial.println(F("wifi connect                  Reconnect using saved credentials"));
  Serial.println(F("wifi connect SSID|PASSWORD   Save credentials and connect"));
  Serial.println(F("wifi scan                     Scan nearby networks (asynchronous)"));
  Serial.println(F("wifi disconnect              Disconnect; retain saved credentials"));
  Serial.println(F("wifi forget                  Erase saved infrastructure credentials"));
  Serial.println();
  Serial.println(F("SD file inspection is read-only and restricted to /diag."));
  Serial.println(F("CAN/TWAI is fixed at 250 kbit/s, listen-only; binary CAN capture is stored under /diag."));
  Serial.println(F("Verbose diagnostics are RAM-only; enable with 'verbose on'."));
  Serial.println(F("AP remains enabled as a recovery interface."));
  Serial.println(F("Wi-Fi password is never printed."));
  Serial.println();
}

void printStatus() {
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
  Serial.print(F("Arduino core:         ")); Serial.println(systemInfo.arduinoCoreVersion);
  Serial.print(F("ESP-IDF:              ")); Serial.println(systemInfo.espIdfVersion);
  Serial.print(F("Chip:                 ")); Serial.println(systemInfo.chipModel);
  Serial.print(F("Chip revision:        ")); Serial.println(systemInfo.chipRevision);
  Serial.print(F("CPU frequency:        ")); Serial.print(systemInfo.cpuMHz); Serial.println(F(" MHz"));
  Serial.print(F("Flash capacity:       ")); Serial.print((double)systemInfo.flashChipSize / 1048576.0, 1); Serial.println(F(" MB"));
  Serial.print(F("Sketch size:          ")); Serial.print((double)systemInfo.sketchSize / 1024.0, 1); Serial.println(F(" KB"));
  Serial.print(F("Free app space:       ")); Serial.print((double)systemInfo.freeSketchSpace / 1024.0, 1); Serial.println(F(" KB"));
  Serial.print(F("Uptime:               ")); Serial.println(formatUptime(millis() - bootMillis));
  Serial.print(F("Reset reason:         ")); Serial.println(systemInfo.resetReason);
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
  Serial.print(F("Last handler:         ")); Serial.print(httpLastDurationUs); Serial.print(F(" us - ")); Serial.print(httpLastRoute); Serial.print(F(" / ")); Serial.println(httpLastStatusCode);
  Serial.print(F("Max handler:          ")); Serial.print(httpMaxDurationUs); Serial.print(F(" us - ")); Serial.print(httpMaxRoute); Serial.print(F(" / ")); Serial.println(httpMaxStatusCode);
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
  if (faultLogReady) Serial.println(FAULT_LOG_FILE);
  else Serial.println(F("NOT READY"));
  Serial.print(F("Fault file writes:    ")); Serial.print(faultLogWriteCount); Serial.print(F(" pass; ")); Serial.println(faultLogWriteFailures);
  Serial.print(F("File list/read/download requests: ")); Serial.print(sdFileListRequests); Serial.print(F(" / ")); Serial.print(sdFileReadRequests); Serial.print(F(" / ")); Serial.println(sdFileDownloadRequests);
  Serial.print(F("File operation failures: ")); Serial.println(sdFileOperationFailures);
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
  Serial.println(F("CAN / TWAI"));
  Serial.println(F("----------"));
  Serial.print(F("State:                ")); Serial.println(canStateText);
  Serial.print(F("Mode / bitrate:       LISTEN-ONLY / ")); Serial.println(CAN_BITRATE);
  Serial.print(F("RX / TX pins:         ")); Serial.print(CAN_RX_PIN); Serial.print(F(" / ")); Serial.println(CAN_TX_PIN);
  Serial.print(F("Init attempts/fails:  ")); Serial.print(canInitAttempts); Serial.print(F(" / ")); Serial.println(canInitFailures);
  bool rxTaskRunningSnapshot;
  uint32_t framesReceivedSnapshot, standardFramesSnapshot, extendedFramesSnapshot;
  uint32_t remoteFramesSnapshot, historyCountSnapshot, historyOverwritesSnapshot;
  uint32_t queueDepthSnapshot, queueMaxDepthSnapshot, queueFullSnapshot;
  uint32_t missedSnapshot, overrunSnapshot, rxErrCounterSnapshot, txErrCounterSnapshot;
  uint32_t busErrorSnapshot, arbLostSnapshot, txFailedSnapshot, receiveErrorsSnapshot, lastErrorSnapshot;
  uint64_t lastFrameTimestampSnapshot;
  portENTER_CRITICAL(&canMux);
  rxTaskRunningSnapshot = canRxTaskRunning;
  framesReceivedSnapshot = canFramesReceived;
  standardFramesSnapshot = canStandardFrames;
  extendedFramesSnapshot = canExtendedFrames;
  remoteFramesSnapshot = canRemoteFrames;
  historyCountSnapshot = canFrameHistoryCount;
  historyOverwritesSnapshot = canHistoryOverwrites;
  queueDepthSnapshot = canDriverRxQueueDepth;
  queueMaxDepthSnapshot = canDriverRxQueueMaxDepth;
  queueFullSnapshot = canRxQueueFullCount;
  missedSnapshot = canDriverRxMissedCount;
  overrunSnapshot = canDriverRxOverrunCount;
  rxErrCounterSnapshot = canDriverRxErrorCounter;
  txErrCounterSnapshot = canDriverTxErrorCounter;
  busErrorSnapshot = canDriverBusErrorCount;
  arbLostSnapshot = canDriverArbitrationLostCount;
  txFailedSnapshot = canDriverTxFailedCount;
  receiveErrorsSnapshot = canReceiveErrors;
  lastFrameTimestampSnapshot = canLastFrameTimestampUs;
  lastErrorSnapshot = canLastError;
  portEXIT_CRITICAL(&canMux);
  Serial.print(F("Receive task:         ")); Serial.println(rxTaskRunningSnapshot ? F("RUNNING") : (canRxTaskStarted ? F("STARTING") : F("NOT RUNNING")));
  Serial.print(F("Bus-off/recoveries:   ")); Serial.print(canBusOffCount); Serial.print(F(" / ")); Serial.println(canRecoveryCount);
  Serial.print(F("Frames received:      ")); Serial.println(framesReceivedSnapshot);
  Serial.print(F("Standard/extended:    ")); Serial.print(standardFramesSnapshot); Serial.print(F(" / ")); Serial.println(extendedFramesSnapshot);
  Serial.print(F("Remote frames:        ")); Serial.println(remoteFramesSnapshot);
  Serial.print(F("History retained:     ")); Serial.print(historyCountSnapshot); Serial.print(F(" / overwritten: ")); Serial.println(historyOverwritesSnapshot);
  Serial.print(F("RX queue current/max: ")); Serial.print(queueDepthSnapshot); Serial.print(F(" / ")); Serial.println(queueMaxDepthSnapshot);
  Serial.print(F("RX queue full alerts: ")); Serial.println(queueFullSnapshot);
  Serial.print(F("Driver missed/overrun: ")); Serial.print(missedSnapshot); Serial.print(F(" / ")); Serial.println(overrunSnapshot);
  Serial.print(F("RX/TX error counters: ")); Serial.print(rxErrCounterSnapshot); Serial.print(F(" / ")); Serial.println(txErrCounterSnapshot);
  Serial.print(F("Bus errors/arb lost:  ")); Serial.print(busErrorSnapshot); Serial.print(F(" / ")); Serial.println(arbLostSnapshot);
  Serial.print(F("TX failed count:      ")); Serial.println(txFailedSnapshot);
  Serial.print(F("Receive API errors:   ")); Serial.println(receiveErrorsSnapshot);
  Serial.print(F("Last frame timestamp: ")); Serial.print((unsigned long long)lastFrameTimestampSnapshot); Serial.println(F(" us since boot"));
  Serial.print(F("Recovery attempts:    ")); Serial.println(canRecoveryAttempts);
  Serial.print(F("Last TWAI error:      0x")); Serial.println(lastErrorSnapshot, HEX);
  printCANLoggingStatus();
  Serial.println();
}
