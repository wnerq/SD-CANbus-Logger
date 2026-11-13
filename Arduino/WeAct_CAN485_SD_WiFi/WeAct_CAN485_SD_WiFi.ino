/*
  WeAct_CAN485_SD_WiFi_Test_V01.ino

  Phase 1 foundation test for:
    Board: WeAct CAN485 DevBoard V1.1
    MCU: ESP32-D0WD-V3
    Framework: Arduino-ESP32 (target core 3.3.11)
    SD: onboard microSD, Arduino SD.h over SPI
    Wi-Fi: standalone SoftAP + HTTP status page

  Purpose:
    Validate simultaneous SD-card and Wi-Fi operation before adding CAN.

  Board pin assignments (WeAct documentation):
    SD CS   = GPIO13
    SD SCK  = GPIO14
    SD MOSI = GPIO15
    SD MISO = GPIO2

  CAN pins are intentionally not initialized in this phase:
    CAN RX = GPIO26
    CAN TX = GPIO27

  Serial commands:
    help
    status
    sdtest

  Browser:
    Connect to the configured AP and open http://192.168.4.1/

  Notes:
    - Remove the microSD card while flashing, per WeAct instructions.
    - Select "ESP32 Dev Module" in Arduino IDE.
    - No CAN frames are received or transmitted by this firmware.
*/

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <FS.h>
#include <SD.h>
#include <SPI.h>

// -----------------------------------------------------------------------------
// Firmware identity and user configuration
// -----------------------------------------------------------------------------

static const char FIRMWARE_VERSION[] = "V01";
static const char BUILD_LABEL[] = __DATE__ " " __TIME__;

static const char WIFI_AP_SSID[] = "WeAct-CAN-Logger";
static const char WIFI_AP_PASSWORD[] = "CANlogger01";  // At least 8 characters.

// -----------------------------------------------------------------------------
// WeAct CAN485 V1.1 pin assignments
// -----------------------------------------------------------------------------

static const int SD_CS_PIN   = 13;
static const int SD_SCK_PIN  = 14;
static const int SD_MOSI_PIN = 15;
static const int SD_MISO_PIN = 2;

// Start conservatively. Increase only after stable testing.
static const uint32_t SD_SPI_FREQUENCY = 4000000;

static const char SD_TEST_DIRECTORY[] = "/diag";
static const char SD_TEST_FILE[] = "/diag/SDTEST.TXT";

// -----------------------------------------------------------------------------
// Runtime objects and state
// -----------------------------------------------------------------------------

WebServer server(80);

bool wifiStarted = false;
bool sdReady = false;
bool sdTestPassed = false;
bool sdTestRunning = false;

uint32_t sdTestCount = 0;
uint32_t sdTestDurationMs = 0;
uint32_t lastSdTestMillis = 0;

String sdLastResult = "Not run";
String sdCardTypeText = "Unknown";
uint64_t sdCardCapacityBytes = 0;
uint64_t sdTotalBytes = 0;
uint64_t sdUsedBytes = 0;

char serialCommandBuffer[40];
size_t serialCommandLength = 0;

// -----------------------------------------------------------------------------
// Function declarations
// -----------------------------------------------------------------------------

void startWiFiAccessPoint();
void initializeSDCard();
bool runSDTest();
void refreshSDInformation();

void configureWebServer();
void handleRoot();
void handleStatus();
void handleSDTest();

void serviceSerial();
void processSerialCommand(const char *command);
void printHelp();
void printStatus();

String makeStatusJson();
String htmlPage();

// -----------------------------------------------------------------------------
// Setup
// -----------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println();
  Serial.println(F("=============================================="));
  Serial.println(F("WeAct CAN485 SD + Wi-Fi FOUNDATION TEST"));
  Serial.println(F("=============================================="));
  Serial.print(F("Firmware: "));
  Serial.println(FIRMWARE_VERSION);
  Serial.print(F("Build:    "));
  Serial.println(BUILD_LABEL);
  Serial.println(F("CAN:      NOT INITIALIZED"));
  Serial.println();

  // Start the AP first. SD tests will then run with Wi-Fi already active.
  startWiFiAccessPoint();

  // Configure the SPI pins exactly as documented for the WeAct board.
  Serial.println(F("[SD] Starting SPI and mounting card..."));
  SPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);

  if (SD.begin(SD_CS_PIN, SPI, SD_SPI_FREQUENCY)) {
    sdReady = true;
    Serial.println(F("[SD] Mount: PASS"));
    refreshSDInformation();

    if (!SD.exists(SD_TEST_DIRECTORY)) {
      if (SD.mkdir(SD_TEST_DIRECTORY)) {
        Serial.println(F("[SD] Created /diag directory."));
      } else {
        Serial.println(F("[SD] WARNING: Could not create /diag directory."));
      }
    }

    // Run one short read/write/append verification at boot.
    runSDTest();
  } else {
    sdReady = false;
    sdLastResult = "Card mount failed";
    Serial.println(F("[SD] Mount: FAIL"));
    Serial.println(F("[SD] Check card insertion, format, and board pinout."));
  }

  configureWebServer();
  server.begin();

  Serial.println();
  Serial.println(F("[HTTP] Server started."));
  Serial.print(F("[HTTP] Browse to http://"));
  Serial.print(WiFi.softAPIP());
  Serial.println(F("/"));
  Serial.println();
  printHelp();
  printStatus();
}

// -----------------------------------------------------------------------------
// Main loop
// -----------------------------------------------------------------------------

void loop() {
  server.handleClient();
  serviceSerial();
  delay(2);
}

// -----------------------------------------------------------------------------
// Wi-Fi access point
// -----------------------------------------------------------------------------

void startWiFiAccessPoint() {
  Serial.println(F("[Wi-Fi] Starting access point..."));

  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);

  wifiStarted = WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASSWORD);

  if (wifiStarted) {
    Serial.println(F("[Wi-Fi] Access point: PASS"));
    Serial.print(F("[Wi-Fi] SSID: "));
    Serial.println(WIFI_AP_SSID);
    Serial.print(F("[Wi-Fi] IP:   "));
    Serial.println(WiFi.softAPIP());
  } else {
    Serial.println(F("[Wi-Fi] Access point: FAIL"));
  }
}

// -----------------------------------------------------------------------------
// SD initialization and information
// -----------------------------------------------------------------------------

void initializeSDCard() {
  // Reserved for a future serial re-initialization command if needed.
}

void refreshSDInformation() {
  if (!sdReady) return;

  uint8_t cardType = SD.cardType();

  switch (cardType) {
    case CARD_MMC:
      sdCardTypeText = "MMC";
      break;
    case CARD_SD:
      sdCardTypeText = "SDSC";
      break;
    case CARD_SDHC:
      sdCardTypeText = "SDHC/SDXC";
      break;
    case CARD_NONE:
      sdCardTypeText = "No card";
      break;
    default:
      sdCardTypeText = "Unknown";
      break;
  }

  sdCardCapacityBytes = SD.cardSize();
  sdTotalBytes = SD.totalBytes();
  sdUsedBytes = SD.usedBytes();
}

// -----------------------------------------------------------------------------
// SD diagnostic
//
// Each test:
//   1. Removes the previous test file.
//   2. Creates and writes an identifiable first record.
//   3. Closes the file.
//   4. Reopens it in append mode and writes a second record.
//   5. Closes and reopens for reading.
//   6. Verifies both records are present.
// -----------------------------------------------------------------------------

bool runSDTest() {
  if (!sdReady) {
    sdTestPassed = false;
    sdLastResult = "SD card is not mounted";
    Serial.println(F("[SD TEST] Cannot run: SD is not mounted."));
    return false;
  }

  if (sdTestRunning) {
    Serial.println(F("[SD TEST] Test already in progress."));
    return false;
  }

  sdTestRunning = true;
  sdTestCount++;

  uint32_t startMillis = millis();

  char firstRecord[96];
  char secondRecord[96];

  snprintf(firstRecord, sizeof(firstRecord),
           "SD LOGGER TEST V01 - TEST %lu - WRITE PASS",
           (unsigned long)sdTestCount);

  snprintf(secondRecord, sizeof(secondRecord),
           "SD LOGGER TEST V01 - TEST %lu - APPEND PASS",
           (unsigned long)sdTestCount);

  Serial.println();
  Serial.println(F("[SD TEST] Starting write/read/append verification..."));

  // Start from a known file state so each test remains small.
  if (SD.exists(SD_TEST_FILE) && !SD.remove(SD_TEST_FILE)) {
    sdLastResult = "Could not remove previous test file";
    sdTestPassed = false;
    sdTestRunning = false;
    Serial.println(F("[SD TEST] FAIL: could not remove previous test file."));
    return false;
  }

  // Step 1: create and write the first record.
  File file = SD.open(SD_TEST_FILE, FILE_WRITE);
  if (!file) {
    sdLastResult = "Open for write failed";
    sdTestPassed = false;
    sdTestRunning = false;
    Serial.println(F("[SD TEST] FAIL: open for write."));
    return false;
  }

  size_t firstBytes = file.println(firstRecord);
  file.flush();
  file.close();

  if (firstBytes == 0) {
    sdLastResult = "Initial write failed";
    sdTestPassed = false;
    sdTestRunning = false;
    Serial.println(F("[SD TEST] FAIL: initial write."));
    return false;
  }

  // Step 2: append a second record.
  file = SD.open(SD_TEST_FILE, FILE_APPEND);
  if (!file) {
    sdLastResult = "Open for append failed";
    sdTestPassed = false;
    sdTestRunning = false;
    Serial.println(F("[SD TEST] FAIL: open for append."));
    return false;
  }

  size_t secondBytes = file.println(secondRecord);
  file.flush();
  file.close();

  if (secondBytes == 0) {
    sdLastResult = "Append failed";
    sdTestPassed = false;
    sdTestRunning = false;
    Serial.println(F("[SD TEST] FAIL: append."));
    return false;
  }

  // Step 3: reopen and verify both records.
  file = SD.open(SD_TEST_FILE, FILE_READ);
  if (!file) {
    sdLastResult = "Open for read failed";
    sdTestPassed = false;
    sdTestRunning = false;
    Serial.println(F("[SD TEST] FAIL: open for read."));
    return false;
  }

  String contents;
  contents.reserve(256);

  while (file.available()) {
    contents += (char)file.read();
    // The diagnostic file should be tiny. This guard prevents runaway
    // allocation if the file is unexpectedly corrupted or replaced.
    if (contents.length() > 512) break;
  }
  file.close();

  bool firstFound = contents.indexOf(firstRecord) >= 0;
  bool secondFound = contents.indexOf(secondRecord) >= 0;

  sdTestDurationMs = millis() - startMillis;
  lastSdTestMillis = millis();

  sdTestPassed = firstFound && secondFound;

  if (sdTestPassed) {
    sdLastResult = "PASS - write, append, readback verified";
    Serial.println(F("[SD TEST] Initial write: PASS"));
    Serial.println(F("[SD TEST] Append:        PASS"));
    Serial.println(F("[SD TEST] Readback:      PASS"));
  } else {
    sdLastResult = "FAIL - readback content mismatch";
    Serial.println(F("[SD TEST] FAIL: readback content mismatch."));
    Serial.println(F("[SD TEST] File contents follow:"));
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

  sdTestRunning = false;
  return sdTestPassed;
}

// -----------------------------------------------------------------------------
// HTTP server routes
// -----------------------------------------------------------------------------

void configureWebServer() {
  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/status", HTTP_GET, handleStatus);
  server.on("/api/test", HTTP_GET, handleSDTest);

  server.onNotFound([]() {
    server.send(404, "text/plain", "Not found");
  });
}

void handleRoot() {
  server.send(200, "text/html", htmlPage());
}

void handleStatus() {
  refreshSDInformation();
  server.send(200, "application/json", makeStatusJson());
}

void handleSDTest() {
  bool result = runSDTest();

  String response = makeStatusJson();
  server.send(result ? 200 : 500, "application/json", response);
}

// -----------------------------------------------------------------------------
// Status JSON and browser page
// -----------------------------------------------------------------------------

String makeStatusJson() {
  String json;
  json.reserve(700);

  json += "{";
  json += "\"firmware\":\"";
  json += FIRMWARE_VERSION;
  json += "\",\"build\":\"";
  json += BUILD_LABEL;
  json += "\",\"wifi\":";
  json += (wifiStarted ? "true" : "false");
  json += ",\"ssid\":\"";
  json += WIFI_AP_SSID;
  json += "\",\"ip\":\"";
  json += WiFi.softAPIP().toString();
  json += "\",\"sdReady\":";
  json += (sdReady ? "true" : "false");
  json += ",\"cardType\":\"";
  json += sdCardTypeText;
  json += "\",\"cardMB\":";
  json += String((double)sdCardCapacityBytes / 1048576.0, 1);
  json += ",\"totalMB\":";
  json += String((double)sdTotalBytes / 1048576.0, 1);
  json += ",\"usedMB\":";
  json += String((double)sdUsedBytes / 1048576.0, 1);
  json += ",\"testPassed\":";
  json += (sdTestPassed ? "true" : "false");
  json += ",\"testRunning\":";
  json += (sdTestRunning ? "true" : "false");
  json += ",\"testCount\":";
  json += String(sdTestCount);
  json += ",\"testDurationMs\":";
  json += String(sdTestDurationMs);
  json += ",\"testResult\":\"";
  json += sdLastResult;
  json += "\"}";
  return json;
}

String htmlPage() {
  String page;
  page.reserve(5000);

  page += R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>WeAct CAN Logger - Foundation Test</title>
<style>
body{font-family:Arial,sans-serif;background:#f2f4f7;color:#202833;margin:0;padding:18px}
main{max-width:720px;margin:auto}
h1{font-size:1.45rem;margin-bottom:4px}
.sub{color:#667085;margin-top:0}
.card{background:white;border:1px solid #d9dee7;border-radius:10px;padding:15px;margin:12px 0}
.row{display:flex;justify-content:space-between;gap:12px;padding:7px 0;border-bottom:1px solid #edf0f4}
.row:last-child{border-bottom:0}
.value{font-weight:600;text-align:right;overflow-wrap:anywhere}
button{background:#155eef;color:white;border:0;border-radius:7px;padding:12px 16px;font-size:1rem;margin:5px 5px 5px 0}
button.secondary{background:#475467}
#message{min-height:1.3em;color:#344054}
.small{font-size:.85rem;color:#667085}
</style>
</head>
<body>
<main>
<h1>WeAct CAN485 - SD + Wi-Fi Test</h1>
<p class="sub">Phase 1 hardware/software foundation</p>

<div class="card">
  <h3>System status</h3>
  <div class="row"><span>Firmware</span><span class="value" id="firmware">--</span></div>
  <div class="row"><span>Build</span><span class="value" id="build">--</span></div>
  <div class="row"><span>Wi-Fi AP</span><span class="value" id="wifi">--</span></div>
  <div class="row"><span>SSID</span><span class="value" id="ssid">--</span></div>
  <div class="row"><span>IP address</span><span class="value" id="ip">--</span></div>
</div>

<div class="card">
  <h3>microSD</h3>
  <div class="row"><span>Mount status</span><span class="value" id="sd">--</span></div>
  <div class="row"><span>Card type</span><span class="value" id="cardType">--</span></div>
  <div class="row"><span>Card capacity</span><span class="value" id="cardMB">--</span></div>
  <div class="row"><span>Filesystem total</span><span class="value" id="totalMB">--</span></div>
  <div class="row"><span>Filesystem used</span><span class="value" id="usedMB">--</span></div>
</div>

<div class="card">
  <h3>SD file test</h3>
  <div class="row"><span>Last result</span><span class="value" id="testResult">--</span></div>
  <div class="row"><span>Test count</span><span class="value" id="testCount">--</span></div>
  <div class="row"><span>Duration</span><span class="value" id="testDuration">--</span></div>
  <button onclick="runTest()">Run SD write/read test</button>
  <button class="secondary" onclick="refreshStatus()">Refresh status</button>
  <p id="message"></p>
</div>

<p class="small">CAN is intentionally inactive in this firmware. This test checks SD and Wi-Fi operation together.</p>
</main>

<script>
async function refreshStatus(){
  const message=document.getElementById('message');
  try{
    const response=await fetch('/api/status',{cache:'no-store'});
    if(!response.ok) throw new Error('HTTP '+response.status);
    const s=await response.json();
    document.getElementById('firmware').textContent=s.firmware;
    document.getElementById('build').textContent=s.build;
    document.getElementById('wifi').textContent=s.wifi?'RUNNING':'FAILED';
    document.getElementById('ssid').textContent=s.ssid;
    document.getElementById('ip').textContent=s.ip;
    document.getElementById('sd').textContent=s.sdReady?'MOUNTED':'NOT MOUNTED';
    document.getElementById('cardType').textContent=s.cardType;
    document.getElementById('cardMB').textContent=s.cardMB+' MB';
    document.getElementById('totalMB').textContent=s.totalMB+' MB';
    document.getElementById('usedMB').textContent=s.usedMB+' MB';
    document.getElementById('testResult').textContent=s.testResult;
    document.getElementById('testCount').textContent=s.testCount;
    document.getElementById('testDuration').textContent=s.testDurationMs+' ms';
    message.textContent='';
  }catch(e){message.textContent='Status refresh failed: '+e.message;}
}
async function runTest(){
  const message=document.getElementById('message');
  message.textContent='Running SD test...';
  try{
    const response=await fetch('/api/test',{cache:'no-store'});
    const s=await response.json();
    message.textContent=s.testResult;
    await refreshStatus();
  }catch(e){message.textContent='SD test request failed: '+e.message;}
}
refreshStatus();
setInterval(refreshStatus,5000);
</script>
</body>
</html>
)rawliteral";

  return page;
}

// -----------------------------------------------------------------------------
// Serial diagnostics
// -----------------------------------------------------------------------------

void serviceSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\r') continue;

    if (c == '\n') {
      serialCommandBuffer[serialCommandLength] = '\0';

      if (serialCommandLength > 0) {
        processSerialCommand(serialCommandBuffer);
      }

      serialCommandLength = 0;
      continue;
    }

    if (serialCommandLength < sizeof(serialCommandBuffer) - 1) {
      serialCommandBuffer[serialCommandLength++] = c;
    } else {
      serialCommandLength = 0;
      Serial.println(F("[SERIAL] Command too long; discarded."));
    }
  }
}

void processSerialCommand(const char *command) {
  if (!strcmp(command, "help")) {
    printHelp();
  } else if (!strcmp(command, "status")) {
    printStatus();
  } else if (!strcmp(command, "sdtest")) {
    runSDTest();
  } else {
    Serial.print(F("Unknown command: "));
    Serial.println(command);
    printHelp();
  }
}

void printHelp() {
  Serial.println();
  Serial.println(F("SERIAL COMMANDS"));
  Serial.println(F("----------------"));
  Serial.println(F("help    Show this command list"));
  Serial.println(F("status  Print current Wi-Fi and SD status"));
  Serial.println(F("sdtest  Run SD write/append/readback test"));
  Serial.println();
}

void printStatus() {
  refreshSDInformation();

  Serial.println();
  Serial.println(F("WEACT CAN485 FOUNDATION TEST STATUS"));
  Serial.println(F("------------------------------------"));
  Serial.print(F("Firmware:          ")); Serial.println(FIRMWARE_VERSION);
  Serial.print(F("Build:             ")); Serial.println(BUILD_LABEL);
  Serial.print(F("Wi-Fi AP:          ")); Serial.println(wifiStarted ? F("RUNNING") : F("FAILED"));
  Serial.print(F("SSID:              ")); Serial.println(WIFI_AP_SSID);
  Serial.print(F("AP IP:             ")); Serial.println(WiFi.softAPIP());
  Serial.print(F("SD mounted:        ")); Serial.println(sdReady ? F("YES") : F("NO"));
  Serial.print(F("SD card type:      ")); Serial.println(sdCardTypeText);
  Serial.print(F("SD card capacity:  ")); Serial.print((double)sdCardCapacityBytes / 1048576.0, 1); Serial.println(F(" MB"));
  Serial.print(F("Filesystem total:  ")); Serial.print((double)sdTotalBytes / 1048576.0, 1); Serial.println(F(" MB"));
  Serial.print(F("Filesystem used:   ")); Serial.print((double)sdUsedBytes / 1048576.0, 1); Serial.println(F(" MB"));
  Serial.print(F("SD test count:     ")); Serial.println(sdTestCount);
  Serial.print(F("SD test result:    ")); Serial.println(sdLastResult);
  Serial.print(F("SD test duration:  ")); Serial.print(sdTestDurationMs); Serial.println(F(" ms"));
  Serial.print(F("Free heap:         ")); Serial.print(ESP.getFreeHeap()); Serial.println(F(" bytes"));
  Serial.print(F("Minimum free heap: ")); Serial.print(ESP.getMinFreeHeap()); Serial.println(F(" bytes"));
  Serial.println(F("CAN:               NOT INITIALIZED"));
  Serial.println();
}

