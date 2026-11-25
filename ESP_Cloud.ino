#include <WiFi.h>

// ========================= WiFi / TCP =========================
static const char* WIFI_SSID = "WiFi_04A6";
static const char* WIFI_PSK  = "12345678";
static const char* SERVER_IP = "192.168.23.2";
static const uint16_t SERVER_PORT = 6868;
static const IPAddress WIFI_STATIC_IP(192, 168, 23, 10);
static const IPAddress WIFI_GATEWAY(192, 168, 23, 1);
static const IPAddress WIFI_SUBNET(255, 255, 255, 0);
static const IPAddress WIFI_DNS1(192, 168, 23, 1);
static const IPAddress WIFI_DNS2(8, 8, 8, 8);

static const uint32_t WIFI_BLINK_MS         = 500;
static const uint8_t  WIFI_BLINK_COUNT      = 5;
static const uint32_t WIFI_BOOT_DEADLINE_MS = 90000;
static const uint32_t TCP_RETRY_MS          = 2000;

WiFiClient client;

// ========================= EPD (fixed GPIO) =========================
// DIN(MOSI)=GPIO14, CLK(SCK)=GPIO13, CS=GPIO15, DC=GPIO27, RST=GPIO26, BUSY=GPIO25
static const int EPD_SCK  = 13;
static const int EPD_MOSI = 14;

static const int EPD_CS   = 15;
static const int EPD_DC   = 27;
static const int EPD_RST  = 26;
static const int EPD_BUSY = 25;

#include <SPI.h>
#include <GxEPD2_BW.h>
#include <SPIFFS.h>
#include <math.h>
#include <esp_sleep.h>
#include <esp_mac.h>

using EpdDriver = GxEPD2_420_GDEY042T81;
GxEPD2_BW<EpdDriver, EpdDriver::HEIGHT> display(EpdDriver(EPD_CS, EPD_DC, EPD_RST, EPD_BUSY));

enum WifiBootState : uint8_t
{
  WIFI_BOOT_TRYING,
  WIFI_BOOT_DONE
};

enum WifiIconMode : uint8_t
{
  WIFI_ICON_BLANK,
  WIFI_ICON_CONNECTED,
  WIFI_ICON_NONE
};

// ========================= Framebuffer from RPi =========================
// RPi sends 1bpp raw buffer for 400x300 => 50 bytes/row * 300 = 15000 bytes
static const uint16_t EPD_W = 400;
static const uint16_t EPD_H = 300;
static const uint32_t FB_SIZE = (EPD_W / 8) * EPD_H;
static uint8_t fb[FB_SIZE];

// ========================= Tuning switches (if image abnormal) =========================
static bool INVERT_PIXELS = false;   // black/white inverted
static bool REVERSE_BITS  = false;   // reverse bit order in each byte
static uint8_t ROTATION   = 0;       // 0..3 for setRotation()

static bool g_wifiEverConnected = false;
static bool g_tcpEverConnected = false;
static bool g_tcpConnectingLogged = false;
static bool g_lastWiFiConnected = false;
static bool g_refreshDoneAndSleeping = false;
static bool g_transferStarted = false;
static bool g_wifiIconVisibleOnDisplay = false;
static bool g_wifiIconBackupValid = false;
static bool g_wifiBootGaveUp = false;
static bool g_wifiBlinkVisible = false;
static bool g_wifiBlinkDone = false;
static bool g_epdInited = false;
static bool g_hasFlashBackup = false;
static bool g_transferIconVisible = false;
static uint8_t g_wifiBlinkCount = 0;
static uint32_t g_wifiBootStartMs = 0;
static uint32_t g_wifiBlinkMs = 0;
static uint32_t g_wifiReadyMs = 0;
static uint32_t g_transferStartMs = 0;
static const uint32_t STARTUP_IDLE_SLEEP_MS = 90000;
static const uint32_t TRANSFER_TIMEOUT_MS = 90000;

static WifiBootState g_wifiBootState = WIFI_BOOT_TRYING;
static constexpr uint16_t FB_ROW_BYTES = EPD_W / 8;
static constexpr uint16_t WIFI_ICON_X = 0;
static constexpr uint16_t WIFI_ICON_Y = 0;
static constexpr uint16_t WIFI_ICON_W = 18;
static constexpr uint16_t WIFI_ICON_H = 18;
static constexpr uint16_t WIFI_ICON_BYTES = ((WIFI_ICON_W + 7) / 8) * WIFI_ICON_H;
static uint8_t wifiIconBackup[WIFI_ICON_BYTES];
static constexpr uint16_t TRANSFER_ICON_X = 24;
static constexpr uint16_t TRANSFER_ICON_Y = 0;
static constexpr uint16_t TRANSFER_ICON_W = 18;
static constexpr uint16_t TRANSFER_ICON_H = 18;
static constexpr uint16_t TRANSFER_ICON_BYTES = ((TRANSFER_ICON_W + 7) / 8) * TRANSFER_ICON_H;
static const char* FB_BACKUP_PATH = "/fb.bin";

static uint8_t reverseByte(uint8_t b)
{
  b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
  b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
  b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
  return b;
}

static void applyTransformsInPlace()
{
  if (!INVERT_PIXELS && !REVERSE_BITS) return;

  for (uint32_t i = 0; i < FB_SIZE; i++)
  {
    uint8_t v = fb[i];
    if (REVERSE_BITS) v = reverseByte(v);
    if (INVERT_PIXELS) v = (uint8_t)~v;
    fb[i] = v;
  }
}

static void clearToWhite()
{
  display.setRotation(0);
  display.setFullWindow();
  display.firstPage();
  do
  {
    display.fillScreen(GxEPD_WHITE);
  }
  while (display.nextPage());
}

static void printBluetoothMac()
{
  uint8_t mac[6];
  if (esp_read_mac(mac, ESP_MAC_BT) == ESP_OK)
  {
    Serial.printf("[BT] MAC=%02X:%02X:%02X:%02X:%02X:%02X\n",
                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  }
  else
  {
    Serial.println("[BT] MAC read failed");
  }
}

static void configureWiFi()
{
  if (!WiFi.config(WIFI_STATIC_IP, WIFI_GATEWAY, WIFI_SUBNET, WIFI_DNS1, WIFI_DNS2))
  {
    Serial.println("[NET] WiFi static config failed");
  }
}

static void startWiFiAttempt()
{
  WiFi.begin(WIFI_SSID, WIFI_PSK);
}

static void backupWifiIconArea()
{
  const uint16_t rowBytes = (WIFI_ICON_W + 7) / 8;
  for (uint16_t row = 0; row < WIFI_ICON_H; ++row)
  {
    memcpy(
      wifiIconBackup + row * rowBytes,
      fb + (WIFI_ICON_Y + row) * FB_ROW_BYTES + (WIFI_ICON_X / 8),
      rowBytes
    );
  }
  g_wifiIconBackupValid = true;
}

static void restoreWifiIconArea()
{
  const uint16_t rowBytes = (WIFI_ICON_W + 7) / 8;
  for (uint16_t row = 0; row < WIFI_ICON_H; ++row)
  {
    memcpy(
      fb + (WIFI_ICON_Y + row) * FB_ROW_BYTES + (WIFI_ICON_X / 8),
      wifiIconBackup + row * rowBytes,
      rowBytes
    );
  }
}

static void setFbBlackPixel(uint16_t x, uint16_t y)
{
  if (x >= EPD_W || y >= EPD_H) return;
  fb[y * FB_ROW_BYTES + (x / 8)] &= (uint8_t)~(0x80 >> (x & 7));
}

static void setBitmapBlackPixel(uint8_t* bmp, uint16_t w, uint16_t h, uint16_t x, uint16_t y)
{
  if (x >= w || y >= h) return;
  const uint16_t rowBytes = (w + 7) / 8;
  bmp[y * rowBytes + (x / 8)] &= (uint8_t)~(0x80 >> (x & 7));
}

static bool wifiIconPixel(uint16_t x, uint16_t y)
{
  if (x >= WIFI_ICON_W || y >= WIFI_ICON_H) return false;

  // Dot at the bottom, 3x3.
  if (x >= 7 && x <= 9 && y >= 14 && y <= 16) return true;

  // Three concentric arcs, clipped to a sector so they read as a WiFi fan:
  // biggest arc on top, smallest just above the dot.
  const float cx = 8.5f;
  const float cy = 16.5f;
  const float dx = float(x) - cx;
  const float dyUp = cy - float(y);
  const float d = sqrtf(dx * dx + (float(y) - cy) * (float(y) - cy));

  if (dyUp > 0.0f)
  {
    // Outer arc: wider sector.
    if (fabsf(dx) <= 1.2f * dyUp && fabsf(d - 15.0f) <= 0.9f) return true;
    // Inner arc: narrower sector so it ends a little shorter.
    if (fabsf(dx) <= 0.9f * dyUp && fabsf(d - 8.25f) <= 0.9f) return true;
  }
  return false;
}

static void overlayWifiIconIfConnected()
{
  if (WiFi.status() != WL_CONNECTED) return;

  backupWifiIconArea();
  for (uint16_t y = 0; y < WIFI_ICON_H; ++y)
  {
    for (uint16_t x = 0; x < WIFI_ICON_W; ++x)
    {
      if (wifiIconPixel(x, y))
      {
        setFbBlackPixel(WIFI_ICON_X + x, WIFI_ICON_Y + y);
      }
    }
  }
}

static bool noNetworkIconPixel(uint16_t x, uint16_t y)
{
  if (x >= WIFI_ICON_W || y >= WIFI_ICON_H) return false;

  const int slashX = (int)y;
  if ((int)x == slashX || (int)x == slashX + 1) return true;
  if ((int)x == slashX - 1 || (int)x == slashX + 2) return false;
  return wifiIconPixel(x, y);
}

static void buildWifiIconBitmap(uint8_t* bmp, WifiIconMode mode)
{
  memset(bmp, 0xFF, WIFI_ICON_BYTES);
  if (mode == WIFI_ICON_BLANK) return;

  for (uint16_t y = 0; y < WIFI_ICON_H; ++y)
  {
    for (uint16_t x = 0; x < WIFI_ICON_W; ++x)
    {
      const bool on = (mode == WIFI_ICON_CONNECTED)
                        ? wifiIconPixel(x, y)
                        : noNetworkIconPixel(x, y);
      if (on)
      {
        setBitmapBlackPixel(bmp, WIFI_ICON_W, WIFI_ICON_H, x, y);
      }
    }
  }
}

static bool transferIconPixel(uint16_t x, uint16_t y)
{
  if (x >= TRANSFER_ICON_W || y >= TRANSFER_ICON_H) return false;

  // Two vertical lines: left one carries an up arrow, right one a down arrow.
  // Left column (up arrow + line).
  if (y == 0 && x >= 4 && x <= 5) return true;
  if (y == 1 && x >= 3 && x <= 6) return true;
  if (y == 2 && x >= 2 && x <= 7) return true;
  if (y == 3 && x >= 1 && x <= 8) return true;
  if (y >= 4 && x >= 4 && x <= 5) return true;

  // Right column (line + down arrow).
  if (y <= 13 && x >= 12 && x <= 13) return true;
  if (y == 14 && x >= 9 && x <= 16) return true;
  if (y == 15 && x >= 10 && x <= 15) return true;
  if (y == 16 && x >= 11 && x <= 14) return true;
  if (y == 17 && x >= 12 && x <= 13) return true;

  return false;
}

static void buildTransferIconBitmap(uint8_t* bmp, bool visible)
{
  memset(bmp, 0xFF, TRANSFER_ICON_BYTES);
  if (!visible) return;

  for (uint16_t y = 0; y < TRANSFER_ICON_H; ++y)
  {
    for (uint16_t x = 0; x < TRANSFER_ICON_W; ++x)
    {
      if (transferIconPixel(x, y))
      {
        setBitmapBlackPixel(bmp, TRANSFER_ICON_W, TRANSFER_ICON_H, x, y);
      }
    }
  }
}

static void loadFbFromFlash()
{
  if (!SPIFFS.begin(true))
  {
    Serial.println("[FLASH] SPIFFS mount failed, no backup");
    g_hasFlashBackup = false;
    return;
  }

  File f = SPIFFS.open(FB_BACKUP_PATH, FILE_READ);
  if (!f)
  {
    Serial.println("[FLASH] no backup file");
    g_hasFlashBackup = false;
    return;
  }

  if ((uint32_t)f.size() != FB_SIZE)
  {
    Serial.printf("[FLASH] backup size mismatch (%u != %u)\n", (unsigned)f.size(), (unsigned)FB_SIZE);
    f.close();
    g_hasFlashBackup = false;
    return;
  }

  size_t got = f.read((uint8_t*)fb, FB_SIZE);
  f.close();
  g_hasFlashBackup = (got == FB_SIZE);
  Serial.printf("[FLASH] read backup %s (%u bytes)\n",
                g_hasFlashBackup ? "ok" : "short", (unsigned)got);
}

static void saveFbToFlash()
{
  File f = SPIFFS.open(FB_BACKUP_PATH, FILE_WRITE);
  if (!f)
  {
    Serial.println("[FLASH] open for write failed");
    return;
  }
  size_t put = f.write((const uint8_t*)fb, FB_SIZE);
  f.close();
  Serial.printf("[FLASH] save backup %s (%u bytes)\n",
                (put == FB_SIZE) ? "ok" : "short", (unsigned)put);
}

static void ensureEpdReadyForIcons()
{
  if (g_epdInited) return;
  // false avoids the library's initial full refresh on the first partial draw.
  display.init(0, false, 2, false);
  g_epdInited = true;

  // Establish a known controller RAM baseline before any partial (differential)
  // refresh, to avoid snow/noise from uninitialized RAM on cold boot.
  display.setRotation(ROTATION);
  if (g_hasFlashBackup)
  {
    // Restore the previous image into both current (0x24) and previous (0x26)
    // buffers WITHOUT a refresh, so the screen keeps its content and later
    // partial icon updates only touch the icon area.
    display.epd2.writeImageForFullRefresh(fb, 0, 0, EPD_W, EPD_H);
  }
  else
  {
    // First ever boot (no backup): one full white refresh to build the baseline.
    display.clearScreen(GxEPD_WHITE);
  }
}

static void drawIconAt(const uint8_t* bmp, uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
  ensureEpdReadyForIcons();
  display.setRotation(ROTATION);
  display.drawImage(bmp, x, y, w, h, false, false, false);
  display.powerOff();
}

static void drawWifiIconBitmap(const uint8_t* bmp)
{
  drawIconAt(bmp, WIFI_ICON_X, WIFI_ICON_Y, WIFI_ICON_W, WIFI_ICON_H);
}

static void showTransferIcon(bool visible)
{
  uint8_t iconBmp[TRANSFER_ICON_BYTES];
  buildTransferIconBitmap(iconBmp, visible);
  drawIconAt(iconBmp, TRANSFER_ICON_X, TRANSFER_ICON_Y, TRANSFER_ICON_W, TRANSFER_ICON_H);
  g_transferIconVisible = visible;
}

static void showWifiStatusIcon(bool connected)
{
  uint8_t iconBmp[WIFI_ICON_BYTES];
  buildWifiIconBitmap(iconBmp, connected ? WIFI_ICON_CONNECTED : WIFI_ICON_NONE);
  drawWifiIconBitmap(iconBmp);
  g_wifiIconVisibleOnDisplay = connected;
}

static void showWifiBlinkFrame(bool visible)
{
  uint8_t iconBmp[WIFI_ICON_BYTES];
  buildWifiIconBitmap(iconBmp, visible ? WIFI_ICON_CONNECTED : WIFI_ICON_BLANK);
  drawWifiIconBitmap(iconBmp);
  g_wifiIconVisibleOnDisplay = false;
}

static bool ensureTcpConnected()
{
  if (client.connected())
  {
    if (!g_tcpEverConnected)
    {
      g_tcpEverConnected = true;
      Serial.println("[NET] TCP connected");
    }
    return true;
  }

  if (WiFi.status() != WL_CONNECTED) return false;

  static uint32_t lastTryMs = 0;
  uint32_t now = millis();
  if ((now - lastTryMs) < TCP_RETRY_MS) return false;

  lastTryMs = now;
  client.stop();

  if (!g_tcpConnectingLogged)
  {
    Serial.printf("[NET] TCP connecting %s:%u\n", SERVER_IP, SERVER_PORT);
    g_tcpConnectingLogged = true;
  }
  if (!client.connect(SERVER_IP, SERVER_PORT))
  {
    return false;
  }

  client.setNoDelay(true);
  Serial.println("[NET] TCP connected");
  g_tcpConnectingLogged = false;
  g_tcpEverConnected = true;
  return true;
}

// ========================= tcp_sver.py protocol helpers =========================
static void sendAck(uint8_t checkByte)
{
  // ACK format expected by RPi: $<1byte>#  (payload exactly equals check byte)
  uint8_t msg[3] = { (uint8_t)'$', checkByte, (uint8_t)'#' };
  client.write(msg, sizeof(msg));
}

static void sendMsg(const String& payload)
{
  // Message format expected by RPi Get_msg(): $payload#
  client.print("$");
  client.print(payload);
  client.print("#");
}

static bool readExact(uint8_t* buf, size_t len, uint32_t timeoutMs)
{
  size_t got = 0;
  uint32_t start = millis();
  while (got < len && (millis() - start) < timeoutMs)
  {
    int avail = client.available();
    if (avail <= 0)
    {
      delay(1);
      continue;
    }

    int r = client.read(buf + got, (int)(len - got));
    if (r > 0) got += (size_t)r;
  }
  return got == len;
}

// cmd frame: ';' <ascii cmd> '/' <check>
static bool readCommand(String& cmd, uint8_t& check, uint32_t timeoutMs)
{
  cmd = "";
  uint8_t b = 0;

  uint32_t start = millis();
  while ((millis() - start) < timeoutMs)
  {
    if (!readExact(&b, 1, timeoutMs)) return false;
    if (b == (uint8_t)'/') break;
    cmd += (char)b;
  }

  return readExact(&check, 1, timeoutMs);
}

// data frame: 0x57 + data[...] + check
// Normal block: data length = 9(header) + 1024(payload), then 1 check
// Tail block:   data length = 11, then 1 check
static bool readDataFrame(uint32_t timeoutMs)
{
  uint8_t first12[12];
  if (!readExact(first12, sizeof(first12), timeoutMs)) return false;

  uint32_t length =
    ((uint32_t)first12[4] << 24) |
    ((uint32_t)first12[5] << 16) |
    ((uint32_t)first12[6] << 8)  |
    ((uint32_t)first12[7]);

  if (length == 1024)
  {
    uint32_t addr =
      ((uint32_t)first12[0] << 24) |
      ((uint32_t)first12[1] << 16) |
      ((uint32_t)first12[2] << 8)  |
      ((uint32_t)first12[3]);

    static uint8_t payload[1024];
    payload[0] = first12[9];
    payload[1] = first12[10];
    payload[2] = first12[11];

    if (!readExact(payload + 3, 1021, timeoutMs)) return false;

    uint8_t checkByte = 0;
    if (!readExact(&checkByte, 1, timeoutMs)) return false;

    sendAck(checkByte);

    uint8_t calc = 0;
    for (int i = 0; i < 9; i++) calc ^= first12[i];
    for (int i = 0; i < 1024; i++) calc ^= payload[i];
    if (calc != checkByte)
    {
      Serial.println("[PROTO] block checksum mismatch");
      return true;
    }

    if (addr < FB_SIZE)
    {
      uint32_t copyLen = 1024;
      if (addr + copyLen > FB_SIZE) copyLen = FB_SIZE - addr;
      memcpy(fb + addr, payload, copyLen);
    }

    return true;
  }

  // Tail block: first12 = data11 + check
  uint8_t data11[11];
  memcpy(data11, first12, 11);
  uint8_t checkByte = first12[11];

  sendAck(checkByte);

  uint8_t calc = 0;
  for (int i = 0; i < 11; i++) calc ^= data11[i];
  if (calc != checkByte)
  {
    Serial.println("[PROTO] tail checksum mismatch");
  }

  return true;
}

static void refreshDisplay()
{
  applyTransformsInPlace();

  ensureEpdReadyForIcons();
  display.setRotation(ROTATION);

  display.clearScreen(GxEPD_WHITE);
  overlayWifiIconIfConnected();
  display.epd2.writeImageForFullRefresh(fb, 0, 0, EPD_W, EPD_H);
  display.epd2.refresh(false);
  if (WiFi.status() == WL_CONNECTED) restoreWifiIconArea();
  g_wifiIconVisibleOnDisplay = (WiFi.status() == WL_CONNECTED);
}

static void enterLowPowerStop()
{
  if (g_refreshDoneAndSleeping) return;
  g_refreshDoneAndSleeping = true;

  Serial.println("[POWER] entering hibernate + deep sleep");
  if (g_wifiIconVisibleOnDisplay)
  {
    if (g_wifiIconBackupValid)
    {
      drawWifiIconBitmap(wifiIconBackup);
    }
    else
    {
      uint8_t blankBmp[WIFI_ICON_BYTES];
      buildWifiIconBitmap(blankBmp, WIFI_ICON_BLANK);
      drawWifiIconBitmap(blankBmp);
    }
    g_wifiIconVisibleOnDisplay = false;
  }
  if (g_transferIconVisible)
  {
    showTransferIcon(false);
  }
  client.stop();
  WiFi.disconnect(true, false);
  WiFi.mode(WIFI_OFF);
  delay(20);

  if (g_epdInited)
  {
    display.hibernate();
    delay(20);
  }

  // fb holds the RPi image (WiFi icon area already restored), save it so the
  // next cold boot can restore the controller RAM baseline without a refresh.
  saveFbToFlash();

  // No wake source is configured on purpose.
  // Device will stay in deep sleep until hardware reset/power cycle.
  esp_deep_sleep_start();
}

static void serviceWiFiBoot()
{
  if (g_wifiBootState == WIFI_BOOT_DONE) return;

  const uint32_t now = millis();

  if (WiFi.status() == WL_CONNECTED)
  {
    g_wifiBootState = WIFI_BOOT_DONE;
    g_wifiEverConnected = true;
    g_wifiReadyMs = now;
    g_lastWiFiConnected = true;
    g_wifiBlinkDone = true;

    showWifiStatusIcon(true);
    Serial.print("[NET] WiFi connected, IP=");
    Serial.println(WiFi.localIP());
    return;
  }

  // Auto-reconnect keeps scanning in the background; only a wall-clock
  // deadline can stop it, so give the AP a fixed window to come up.
  if ((now - g_wifiBootStartMs) >= WIFI_BOOT_DEADLINE_MS)
  {
    g_wifiBootGaveUp = true;
    g_wifiBootState = WIFI_BOOT_DONE;
    Serial.println("[POWER] WiFi unavailable after 90s, sleeping");
    enterLowPowerStop();
    return;
  }

  // Blink a few times, then settle on the static "no network" icon.
  if (!g_wifiBlinkDone && (now - g_wifiBlinkMs) >= WIFI_BLINK_MS)
  {
    g_wifiBlinkMs = now;
    if (g_wifiBlinkVisible)
    {
      g_wifiBlinkVisible = false;
      g_wifiBlinkCount++;
      if (g_wifiBlinkCount >= WIFI_BLINK_COUNT)
      {
        g_wifiBlinkDone = true;
        showWifiStatusIcon(false);
      }
      else
      {
        showWifiBlinkFrame(false);
      }
    }
    else
    {
      g_wifiBlinkVisible = true;
      showWifiBlinkFrame(true);
    }
  }
}

static void markTransferStarted()
{
  if (g_transferStarted) return;
  g_transferStarted = true;
  g_transferStartMs = millis();
  Serial.println("[NET] transfer started, 90s timeout armed");
}

void setup()
{
  Serial.begin(115200);
  delay(200);
  Serial.println("[BOOT] ESP Cloud start");
  printBluetoothMac();

  SPI.begin(EPD_SCK, /*MISO*/ -1, EPD_MOSI, EPD_CS);

  memset(fb, 0xFF, sizeof(fb));
  loadFbFromFlash();
  Serial.println("[EPD] previous screen kept, full init deferred until refresh");

  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  WiFi.setAutoReconnect(true);
  configureWiFi();

  g_wifiBootState = WIFI_BOOT_TRYING;
  g_wifiBootStartMs = millis();
  g_wifiBlinkMs = g_wifiBootStartMs;
  g_wifiBlinkVisible = true;
  g_wifiBlinkCount = 0;
  g_wifiBlinkDone = false;
  Serial.printf("[NET] WiFi connecting to %s (auto-retry, 90s deadline)\n", WIFI_SSID);
  startWiFiAttempt();
  showWifiBlinkFrame(true);
}

void loop()
{
  if (g_refreshDoneAndSleeping)
  {
    delay(1000);
    return;
  }

  if (g_transferStarted && (millis() - g_transferStartMs) >= TRANSFER_TIMEOUT_MS)
  {
    Serial.println("[POWER] transfer timeout after 90s, sleeping");
    enterLowPowerStop();
    return;
  }

  serviceWiFiBoot();

  if (g_refreshDoneAndSleeping)
  {
    delay(1000);
    return;
  }

  if (g_wifiBootState != WIFI_BOOT_DONE)
  {
    delay(20);
    return;
  }

  if (g_wifiBootGaveUp)
  {
    delay(1000);
    return;
  }

  if (!g_transferStarted && g_wifiEverConnected &&
      (millis() - g_wifiReadyMs) >= STARTUP_IDLE_SLEEP_MS)
  {
    Serial.println("[POWER] no wifi transfer within 90s, sleeping");
    enterLowPowerStop();
    return;
  }

  bool wifiConnected = (WiFi.status() == WL_CONNECTED);
  if (wifiConnected != g_lastWiFiConnected)
  {
    showWifiStatusIcon(wifiConnected);
    g_lastWiFiConnected = wifiConnected;
  }

  if (!ensureTcpConnected())
  {
    delay(20);
    return;
  }

  int first = client.read();
  if (first < 0)
  {
    delay(1);
    return;
  }

  if (first == ';')
  {
    markTransferStarted();
    String cmd;
    uint8_t check = 0;
    if (!readCommand(cmd, check, 5000))
    {
      Serial.println("[PROTO] command read timeout");
      return;
    }

    // ACK command frame
    sendAck(check);

    if (cmd == "G")
    {
      sendMsg("ESP32_GDEY042T81");
    }
    else if (cmd == "C")
    {
      // Report "unlocked" (anything other than '1' makes RPi skip password)
      sendMsg("0");
    }
    else if (cmd.startsWith("N"))
    {
      sendMsg("1");
    }
    else if (cmd == "b")
    {
      // Return battery number string (RPi multiplies by 3)
      sendMsg("100");
    }
    else if (cmd == "F")
    {
      memset(fb, 0xFF, sizeof(fb));
    }
    else if (cmd == "D")
    {
      Serial.println("[EPD] refresh");
      refreshDisplay();
      g_transferIconVisible = false;
      Serial.println("[EPD] refresh done");
    }
    else if (cmd == "S")
    {
      Serial.println("[NET] server requested stop");
      client.flush();
      delay(50);
      enterLowPowerStop();
    }
    else
    {
      Serial.print("[PROTO] unknown cmd: ");
      Serial.println(cmd);
    }
  }
  else if ((uint8_t)first == 0x57)
  {
    markTransferStarted();
    if (!g_transferIconVisible) showTransferIcon(true);
    if (!readDataFrame(5000))
    {
      Serial.println("[PROTO] data frame read timeout");
    }
  }
  else
  {
    // Ignore unknown byte
  }
}