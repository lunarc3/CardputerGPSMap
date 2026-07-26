/*
 * 离线地图查看器 — Cardputer ADV (ESP32-S3FN8) 无 PSRAM
 *
 * 屏幕:  240×135 TFT
 * GPS:   UART2 → RX=15, TX=13 (ATGM336H, 115200)
 * SD:    CS=12, MOSI=14, CLK=40, MISO=39
 *
 * 按键:
 *   Tab      切换 GPS 信息 / 地图
 *   ; . , /  地图平移 (长按连续)
 *   z / x    地图 缩小 / 放大 (仅按下瞬间)
 *   `        回到 GPS 位置
 *   Enter    截图保存到 SD
 *
 * SD 卡结构:
 *   /gpsmap/
 *   ├── gpsmap.ini
 *   ├── screenshot/shot_NNNN.bmp
 *   └── {z}/{x}/{y}.jpg
 */

#include <M5Cardputer.h>
#include <SD.h>
#include <SPI.h>
#include <JPEGDEC.h>
#include <TinyGPS++.h>
#include <math.h>
#include <time.h>

// ═══════════════════════════════════════════
//  硬件
// ═══════════════════════════════════════════

#define PIN_GPS_RX  15
#define PIN_GPS_TX  13
#define GPS_BAUD    115200

#define SD_CS_PIN   12
#define SD_MOSI_PIN 14
#define SD_SCK_PIN  40
#define SD_MISO_PIN 39
#define LORA_CS_PIN 5

// ═══════════════════════════════════════════
//  路径
// ═══════════════════════════════════════════

#define PATH_BASE     "/gpsmap"
#define PATH_INI      "/gpsmap/gpsmap.ini"
#define PATH_SHOT_DIR "/gpsmap/screenshot"

// ═══════════════════════════════════════════
//  参数
// ═══════════════════════════════════════════

#define TILE_PX       256
#define ZOOM_MIN      6
#define ZOOM_MAX      18
#define ZOOM_DEFAULT  15
#define MAX_JPEG_BUF  36864
#define PAN_STEP      50
#define DIR_REPEAT_MS 200
#define MOVE_THRESH   0.00001
#define GPS_TIMEOUT   5000
#define REDRAW_MS     300
#define SAVE_INTERVAL 60000
#define HDR_H         12

// 背景色 RGB565 (#f2efe9)
#define BG_COLOR 0xF77C

// ═══════════════════════════════════════════
//  屏幕枚举
// ═══════════════════════════════════════════

enum ScreenID { SCR_GPS_INFO = 0, SCR_MAP, SCR_COUNT };
ScreenID currentScreen = SCR_GPS_INFO;
bool     autoSwitched  = false;

// ═══════════════════════════════════════════
//  星座数据 (NMEA 解析)
// ═══════════════════════════════════════════

#define NUM_CONST 4

struct ConstInfo {
    const char*   name;
    uint16_t      color;
    int           visible;
    int           used;
    unsigned long lastGSV;
    unsigned long lastGSA;
};

ConstInfo constInfo[NUM_CONST] = {
    {"GPS",     TFT_GREEN,  0, 0, 0, 0},
    {"GLONASS", 0xF800,     0, 0, 0, 0},
    {"Galileo", TFT_CYAN,   0, 0, 0, 0},
    {"BeiDou",  TFT_YELLOW, 0, 0, 0, 0}
};

int    gsaFixMode    = 1;
float  gsaPDOP       = 99.9f;
int    ggaFixQuality = 0;
String nmeaLineBuf   = "";

// ═══════════════════════════════════════════
//  全局状态
// ═══════════════════════════════════════════

TinyGPSPlus     gps;
HardwareSerial  GPS_Serial(2);
JPEGDEC         jpeg;

int    curZoom   = ZOOM_DEFAULT;
double curLat    = 0.0;
double curLon    = 0.0;
bool   gpsFix    = false;
bool   hasLastPos = false;
bool   dirty     = true;
unsigned long lastDraw = 0;
unsigned long lastSave = 0;
unsigned long lastFixUpdate = 0;

int  panX = 0, panY = 0;
bool panning = false;

bool dirUp = false, dirDown = false, dirLeft = false, dirRight = false;
unsigned long lastDirStep = 0;

int        scrW, scrH;
M5Canvas  *cv         = nullptr;
M5Canvas  *jpegCanvas = nullptr;

bool          showSavedMsg = false;
unsigned long savedMsgTime = 0;
int           shotNum      = 0;

// 缩放按键上升沿检测
bool lastZState = false;
bool lastXState = false;

struct LocalTime {
    int  hour, minute, second;
    int  year, month, day;
    int  utcOffset;
    bool valid, dateValid;
};

// ═══════════════════════════════════════════
//  PMTK
// ═══════════════════════════════════════════

void sendPMTK(const char* body) {
    uint8_t ck = 0;
    for (const char* p = body; *p; p++) ck ^= *p;
    char cmd[80];
    snprintf(cmd, sizeof(cmd), "$%s*%02X\r\n", body, ck);
    GPS_Serial.print(cmd);
}

void initGPS() {
    GPS_Serial.begin(GPS_BAUD, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
    delay(200);
    sendPMTK("PCAS03,1,1,1,1,1,1,0,0");
    sendPMTK("PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0");
}

// ═══════════════════════════════════════════
//  位置保存 / 读取
// ═══════════════════════════════════════════

void saveLastPosition() {
    SD.mkdir(PATH_BASE);
    File f = SD.open(PATH_INI, FILE_WRITE);
    if (f) {
        f.printf("%.6f,%.6f,%d\n", curLat, curLon, curZoom);
        f.close();
    }
}

bool loadLastPosition() {
    File f = SD.open(PATH_INI, FILE_READ);
    if (!f) return false;
    String line = f.readStringUntil('\n');
    f.close();
    int c1 = line.indexOf(',');
    int c2 = line.indexOf(',', c1 + 1);
    if (c1 < 0 || c2 < 0) return false;
    double lat  = atof(line.substring(0, c1).c_str());
    double lon  = atof(line.substring(c1 + 1, c2).c_str());
    int    zoom = line.substring(c2 + 1).toInt();
    if (lat == 0.0 && lon == 0.0) return false;
    if (lat < -90 || lat > 90 || lon < -180 || lon > 180) return false;
    if (zoom < ZOOM_MIN || zoom > ZOOM_MAX) zoom = ZOOM_DEFAULT;
    curLat  = lat;
    curLon  = lon;
    curZoom = zoom;
    panning = true;
    return true;
}

// ═══════════════════════════════════════════
//  截图计数器
// ═══════════════════════════════════════════

void initScreenshotCounter() {
    if (!SD.exists(PATH_SHOT_DIR)) {
        SD.mkdir(PATH_SHOT_DIR);
        shotNum = 0;
        return;
    }
    File dir = SD.open(PATH_SHOT_DIR);
    if (!dir) { shotNum = 0; return; }
    int maxNum = -1;
    while (true) {
        File entry = dir.openNextFile();
        if (!entry) break;
        String name = entry.name();
        entry.close();
        int lastSlash = name.lastIndexOf('/');
        if (lastSlash >= 0) name = name.substring(lastSlash + 1);
        if (name.startsWith("shot_") && name.endsWith(".bmp")) {
            String numStr = name.substring(5, name.length() - 4);
            int num = numStr.toInt();
            if (num > maxNum) maxNum = num;
        }
    }
    dir.close();
    shotNum = (maxNum >= 0) ? maxNum + 1 : 0;
}

// ═══════════════════════════════════════════
//  坐标转换
// ═══════════════════════════════════════════

static inline double deg2rad(double d) {
    return d * 0.017453292519943295;
}

void toTile(double la, double lo, int z, int &tx, int &ty) {
    double n = (double)(1 << z);
    tx = (int)((lo + 180.0) / 360.0 * n);
    ty = (int)((1.0 - asinh(tan(deg2rad(la))) / M_PI) / 2.0 * n);
}

void toPixel(double la, double lo, int z, double &px, double &py) {
    double n = (double)(1 << z) * TILE_PX;
    px = (lo + 180.0) / 360.0 * n;
    py = (1.0 - asinh(tan(deg2rad(la))) / M_PI) / 2.0 * n;
}

// ═══════════════════════════════════════════
//  NMEA 解析
// ═══════════════════════════════════════════

String getNmeaField(const String& line, int num) {
    int field = 0, start = 0;
    for (int i = 0; i <= (int)line.length(); i++) {
        if (i == (int)line.length() || line[i] == ',' || line[i] == '*') {
            if (field == num) return line.substring(start, i);
            field++;
            start = i + 1;
        }
    }
    return "";
}

void handleGSV(const String& line) {
    int ci = -1;
    if      (line.startsWith("$GPGSV")) ci = 0;
    else if (line.startsWith("$GLGSV")) ci = 1;
    else if (line.startsWith("$GAGSV")) ci = 2;
    else if (line.startsWith("$BDGSV")) ci = 3;
    if (ci < 0) return;
    String f3 = getNmeaField(line, 3);
    if (f3.length() > 0) {
        constInfo[ci].visible = f3.toInt();
        constInfo[ci].lastGSV = millis();
    }
}

void handleGSA(const String& line) {
    String f2 = getNmeaField(line, 2);
    if (f2.length() > 0) gsaFixMode = f2.toInt();
    String fp = getNmeaField(line, 15);
    if (fp.length() > 0) gsaPDOP = fp.toFloat();

    int ci = -1;
    if      (line.startsWith("$GPGSA")) ci = 0;
    else if (line.startsWith("$GLGSA")) ci = 1;
    else if (line.startsWith("$GAGSA")) ci = 2;
    else if (line.startsWith("$BDGSA")) ci = 3;
    if (ci < 0) return;

    int used = 0;
    for (int i = 3; i <= 14; i++) {
        if (getNmeaField(line, i).length() > 0) used++;
    }
    constInfo[ci].used    = used;
    constInfo[ci].lastGSA = millis();
}

void handleGGA(const String& line) {
    String f6 = getNmeaField(line, 6);
    if (f6.length() > 0) ggaFixQuality = f6.toInt();
}

void parseNmeaLine(const String& line) {
    if (line.indexOf("GSV") > 0) handleGSV(line);
    if (line.indexOf("GSA") > 0) handleGSA(line);
    if (line.indexOf("GGA") > 0) handleGGA(line);
}

// ═══════════════════════════════════════════
//  时区 & DST
// ═══════════════════════════════════════════

static int calcDow(int y, int m, int d) {
    static const int t[] = {0,3,2,5,0,3,5,1,4,6,2,4};
    if (m < 3) y--;
    return (y + y/4 - y/100 + y/400 + t[m-1] + d) % 7;
}

static int nthWday(int y, int m, int wday, int n) {
    int d1 = calcDow(y, m, 1);
    return 1 + ((wday - d1 + 7) % 7) + (n - 1) * 7;
}

static int lastWday(int y, int m, int wday) {
    static const int dim[] = {31,28,31,30,31,30,31,31,30,31,30,31};
    int days = dim[m - 1];
    if (m == 2 && (y % 4 == 0 && (y % 100 != 0 || y % 400 == 0))) days = 29;
    int dl = calcDow(y, m, days);
    return days - ((dl - wday + 7) % 7);
}

static int getDST(int y, int mon, int d, int utcH, float lat, float lon) {
    if (lat > 24 && lat < 72 && lon > -140 && lon < -50) {
        if (lat < 23 && lon < -154) return 0;
        if (lat > 31 && lat < 37.5f && lon > -115 && lon < -109) return 0;
        int sD = nthWday(y, 3, 0, 2);
        int eD = nthWday(y, 11, 0, 1);
        int so = (int)roundf(lon / 15.0f);
        int lh = utcH + so, ld = d;
        if (lh >= 24) { lh -= 24; ld++; } else if (lh < 0) { lh += 24; ld--; }
        if (mon > 3 && mon < 11) return 1;
        if (mon < 3 || mon > 11) return 0;
        if (mon == 3)  return (ld > sD || (ld == sD && lh >= 2)) ? 1 : 0;
        if (mon == 11) return (ld < eD || (ld == eD && lh < 1)) ? 1 : 0;
        return 0;
    }
    if (lat > 34 && lat < 72 && lon > -12 && lon < 45) {
        int sD = lastWday(y, 3, 0), eD = lastWday(y, 10, 0);
        if (mon > 3 && mon < 10) return 1;
        if (mon < 3 || mon > 10) return 0;
        if (mon == 3)  return (d > sD || (d == sD && utcH >= 1)) ? 1 : 0;
        if (mon == 10) return (d < eD || (d == eD && utcH < 1)) ? 1 : 0;
        return 0;
    }
    if (lat < -28 && lon > 138 && lon < 155) {
        int sD = nthWday(y, 10, 0, 1), eD = nthWday(y, 4, 0, 1);
        int lh = utcH + 10, ld = d;
        if (lh >= 24) { lh -= 24; ld++; }
        if (mon > 10 || mon < 4) return 1;
        if (mon > 4 && mon < 10) return 0;
        if (mon == 10) return (ld > sD || (ld == sD && lh >= 2)) ? 1 : 0;
        if (mon == 4)  return (ld < eD || (ld == eD && lh < 2)) ? 1 : 0;
        return 0;
    }
    if (lat < -34 && lon > 165) {
        int sD = lastWday(y, 9, 0), eD = nthWday(y, 4, 0, 1);
        int lh = utcH + 12, ld = d;
        if (lh >= 24) { lh -= 24; ld++; }
        if (mon > 9 || mon < 4) return 1;
        if (mon > 4 && mon < 9) return 0;
        if (mon == 9) return (ld > sD || (ld == sD && lh >= 2)) ? 1 : 0;
        if (mon == 4) return (ld < eD || (ld == eD && lh < 2)) ? 1 : 0;
        return 0;
    }
    return 0;
}

const char* dayName(int dow) {
    static const char* n[] = {"Sat","Sun","Mon","Tue","Wed","Thu","Fri"};
    return n[dow % 7];
}

LocalTime getLocalTime() {
    LocalTime lt = {0,0,0,0,0,0,0,false,false};
    if (!gps.time.isValid()) return lt;
    lt.valid  = true;
    lt.hour   = gps.time.hour();
    lt.minute = gps.time.minute();
    lt.second = gps.time.second();

    int stdOff = gps.location.isValid()
                     ? (int)roundf(gps.location.lng() / 15.0f)
                     : 8;
    int dstAdj = 0;
    if (gps.location.isValid() && gps.date.isValid())
        dstAdj = getDST(gps.date.year(), gps.date.month(), gps.date.day(),
                        lt.hour, gps.location.lat(), gps.location.lng());
    lt.utcOffset = stdOff + dstAdj;

    if (gps.date.isValid()) {
        struct tm utc = {};
        utc.tm_year = gps.date.year() - 1900;
        utc.tm_mon  = gps.date.month() - 1;
        utc.tm_mday = gps.date.day();
        utc.tm_hour = lt.hour;
        utc.tm_min  = lt.minute;
        utc.tm_sec  = lt.second;
        setenv("TZ", "UTC", 1);
        tzset();
        time_t ep = mktime(&utc) + (long)lt.utcOffset * 3600L;
        struct tm loc;
        gmtime_r(&ep, &loc);
        lt.hour   = loc.tm_hour;
        lt.minute = loc.tm_min;
        lt.second = loc.tm_sec;
        lt.year   = loc.tm_year + 1900;
        lt.month  = loc.tm_mon + 1;
        lt.day    = loc.tm_mday;
        lt.dateValid = true;
    } else {
        lt.hour += lt.utcOffset;
        if (lt.hour >= 24) lt.hour -= 24;
        else if (lt.hour < 0) lt.hour += 24;
    }
    return lt;
}

int getTotalSatellites() {
    int t = 0;
    for (int i = 0; i < NUM_CONST; i++)
        if (millis() - constInfo[i].lastGSV < 10000)
            t += constInfo[i].visible;
    return t;
}

// ═══════════════════════════════════════════
//  JPEG 解码 (字节序修正)
// ═══════════════════════════════════════════

int jpegDrawCB(JPEGDRAW *p) {
    if (!jpegCanvas) return 0;
    if (p->x >= scrW || p->y >= scrH ||
        p->x + p->iWidth <= 0 || p->y + p->iHeight <= 0)
        return 1;

    uint16_t *px = (uint16_t *)p->pPixels;
    int count = p->iWidth * p->iHeight;
    for (int i = 0; i < count; i++) {
        uint16_t c = px[i];
        px[i] = (c >> 8) | ((c & 0xFF) << 8);
    }

    jpegCanvas->pushImage(p->x, p->y, p->iWidth, p->iHeight, px);
    return 1;
}

// ═══════════════════════════════════════════
//  瓦片加载
// ═══════════════════════════════════════════

bool drawTile(int tx, int ty, int z, int sx, int sy) {
    char path[64];
    snprintf(path, sizeof(path), PATH_BASE "/%d/%d/%d.jpg", z, tx, ty);
    File f = SD.open(path, FILE_READ);
    if (!f) return false;
    size_t sz = f.size();
    if (sz == 0 || sz > MAX_JPEG_BUF) { f.close(); return false; }
    uint8_t *buf = (uint8_t *)malloc(sz);
    if (!buf) { f.close(); return false; }
    f.read(buf, sz);
    f.close();
    jpegCanvas = cv;
    if (jpeg.openRAM(buf, sz, jpegDrawCB)) {
        jpeg.decode(sx, sy, 0);
        jpeg.close();
    }
    free(buf);
    return true;
}

// ═══════════════════════════════════════════
//  截图 (BMP)
// ═══════════════════════════════════════════

bool saveScreenshot() {
    SD.mkdir(PATH_BASE);
    SD.mkdir(PATH_SHOT_DIR);
    char path[48];
    snprintf(path, sizeof(path), PATH_SHOT_DIR "/shot_%04d.bmp", shotNum++);

    File f = SD.open(path, FILE_WRITE);
    if (!f) { shotNum--; return false; }

    int w = scrW, h = scrH;
    int rowBytes = w * 3;
    int pad = (4 - (rowBytes % 4)) % 4;
    int imgSize = (rowBytes + pad) * h;

    uint8_t fh[14] = {};
    fh[0] = 'B'; fh[1] = 'M';
    int fs = 14 + 40 + imgSize;
    fh[2] = fs; fh[3] = fs >> 8; fh[4] = fs >> 16; fh[5] = fs >> 24;
    fh[10] = 54;
    f.write(fh, 14);

    uint8_t ih[40] = {};
    ih[0] = 40;
    ih[4] = w;       ih[5] = w >> 8;
    ih[8] = h;       ih[9] = h >> 8;
    ih[12] = 1;      ih[14] = 24;
    ih[20] = imgSize; ih[21] = imgSize >> 8;
    ih[22] = imgSize >> 16; ih[23] = imgSize >> 24;
    f.write(ih, 40);

    uint8_t *row = (uint8_t *)malloc(rowBytes + pad);
    if (!row) { f.close(); return false; }
    if (pad > 0) memset(row + rowBytes, 0, pad);

    for (int y = h - 1; y >= 0; y--) {
        for (int x = 0; x < w; x++) {
            uint16_t c = (uint16_t)cv->readPixel(x, y);
            row[x * 3 + 0] = (uint8_t)((c & 0x1F) << 3);
            row[x * 3 + 1] = (uint8_t)(((c >> 5) & 0x3F) << 2);
            row[x * 3 + 2] = (uint8_t)(((c >> 11) & 0x1F) << 3);
        }
        f.write(row, rowBytes + pad);
    }
    free(row);
    f.close();
    return true;
}

// ═══════════════════════════════════════════
//  GPS 信息页面
// ═══════════════════════════════════════════

void drawScreenGpsInfo() {
    char buf[48];
    int y = 2;

    // ── 时间 ──
    LocalTime lt = getLocalTime();
    if (lt.valid) {
        cv->setTextSize(2);
        cv->setTextColor(TFT_GREEN, TFT_BLACK);
        snprintf(buf, sizeof(buf), "%02d:%02d:%02d",
                 lt.hour, lt.minute, lt.second);
        int tw = strlen(buf) * 12;
        cv->setCursor((scrW - tw) / 2, y);
        cv->print(buf);
        y += 20;

        cv->setTextSize(1);
        cv->setTextColor(TFT_LIGHTGREY, TFT_BLACK);
        if (lt.dateValid) {
            int dow = calcDow(lt.year, lt.month, lt.day);
            snprintf(buf, sizeof(buf), "UTC%+d  %04d-%02d-%02d %s",
                     lt.utcOffset, lt.year, lt.month, lt.day, dayName(dow));
        } else {
            snprintf(buf, sizeof(buf), "UTC%+d", lt.utcOffset);
        }
        int dw = strlen(buf) * 6;
        cv->setCursor((scrW - dw) / 2, y);
        cv->print(buf);
        y += 12;
    } else {
        cv->setTextSize(2);
        cv->setTextColor(TFT_DARKGREY, TFT_BLACK);
        cv->drawCenterString("--:--:--", scrW / 2, y);
        y += 20;
        cv->setTextSize(1);
        cv->drawCenterString("UTC", scrW / 2, y);
        y += 12;
    }

    // ── 定位信息 ──
    cv->setTextSize(1);
    const char* fq = "---";
    if      (ggaFixQuality == 1) fq = "GPS";
    else if (ggaFixQuality == 2) fq = "DGPS";
    else if (ggaFixQuality == 4) fq = "RTK";
    const char* fm = "---";
    if      (gsaFixMode == 2) fm = "2D";
    else if (gsaFixMode == 3) fm = "3D";

    cv->setTextColor(TFT_WHITE, TFT_BLACK);
    float hVal = gps.hdop.isValid() ? gps.hdop.hdop() : 99.9f;
    snprintf(buf, sizeof(buf), "Fix:%s %s  PD:%.1f HD:%.1f",
             fq, fm, gsaPDOP, hVal);
    cv->setCursor(4, y);
    cv->print(buf);
    y += 12;

    cv->drawLine(4, y, scrW - 4, y, TFT_DARKGREY);
    y += 4;

    // ── 星座柱状 ──
    int maxVis = 1;
    for (int i = 0; i < NUM_CONST; i++)
        if (constInfo[i].visible > maxVis) maxVis = constInfo[i].visible;

    int barX = 94, barMaxW = 100;
    for (int i = 0; i < NUM_CONST; i++) {
        ConstInfo &ci = constInfo[i];
        bool act = (millis() - ci.lastGSV < 10000);

        cv->setTextSize(1);
        cv->setTextColor(act ? ci.color : TFT_DARKGREY, TFT_BLACK);
        cv->setCursor(4, y);
        cv->printf("%-7s", ci.name);

        cv->setCursor(50, y);
        if (act) {
            cv->setTextColor(TFT_WHITE, TFT_BLACK);
            cv->printf("  %2d", ci.visible);
        } else {
            cv->setTextColor(0x4208, TFT_BLACK);
            cv->print("  --");
        }

        cv->drawRect(barX, y, barMaxW + 2, 8, TFT_DARKGREY);
        if (act && ci.visible > 0) {
            int visW = ci.visible * barMaxW / maxVis;
            cv->fillRect(barX + 1, y + 1, visW, 6, ci.color);
        }
        y += 11;
    }

    cv->drawLine(4, y, scrW - 4, y, TFT_DARKGREY);
    y += 4;

    // ── 坐标 & 速度 ──
    if (gpsFix || gps.location.isValid()) {
        cv->setTextSize(1);
        cv->setTextColor(TFT_GREEN, TFT_BLACK);
        cv->setCursor(4, y);
        cv->printf("%.6f  %.6f", curLat, curLon);
        y += 11;

        cv->setTextColor(TFT_WHITE, TFT_BLACK);
        cv->setCursor(4, y);
        if (gps.altitude.isValid())
            cv->printf("Alt:%.0fm", gps.altitude.meters());
        cv->setCursor(95, y);
        if (gps.course.isValid())
            cv->printf("HDG:%.0f", gps.course.deg());
        cv->setCursor(165, y);
        cv->printf("Spd:%.1f", gps.speed.kmph());
    } else {
        cv->setTextSize(1);
        cv->setTextColor(TFT_DARKGREY, TFT_BLACK);
        cv->drawCenterString("Waiting for fix...", scrW / 2, y);
    }
}

// ═══════════════════════════════════════════
//  地图页面
// ═══════════════════════════════════════════

void drawScreenMap() {
    // 无任何位置
    if (!gpsFix && !hasLastPos && !panning) {
        cv->setTextColor(TFT_WHITE, TFT_BLACK);
        cv->setTextSize(1);
        cv->drawCenterString("Waiting for GPS...", scrW / 2, scrH / 2 - 8);
        cv->setTextColor(0x8410, TFT_BLACK);
        cv->drawCenterString("Tab to switch", scrW / 2, scrH - 14);
        return;
    }

    // ── 瓦片渲染 ──
    int tx, ty;
    toTile(curLat, curLon, curZoom, tx, ty);
    double gpx, gpy;
    toPixel(curLat, curLon, curZoom, gpx, gpy);
    double inTileX = gpx - tx * TILE_PX + panX;
    double inTileY = gpy - ty * TILE_PX + panY;
    int baseX = scrW / 2 - (int)inTileX;
    int baseY = scrH / 2 - (int)inTileY;

    for (int dx = -1; dx <= 1; dx++) {
        for (int dy = -1; dy <= 1; dy++) {
            int sx = baseX + dx * TILE_PX;
            int sy = baseY + dy * TILE_PX;
            if (sx + TILE_PX < 0 || sx > scrW) continue;
            if (sy + TILE_PX < 0 || sy > scrH) continue;
            if (!drawTile(tx + dx, ty + dy, curZoom, sx, sy)) {
                // 缺失瓦片: 背景色填充 (不是灰色)
                int rx = (sx > 0) ? sx : 0;
                int ry = (sy > 0) ? sy : 0;
                int rw = min((int)TILE_PX, scrW - rx);
                int rh = min((int)TILE_PX, scrH - ry);
                if (rw > 0 && rh > 0)
                    cv->fillRect(rx, ry, rw, rh, BG_COLOR);
            }
        }
    }

    // ── 顶栏 ──
    cv->fillRect(0, 0, scrW, HDR_H, 0x0000);

    // ── 定位光标 ──
    int mx = scrW / 2 - panX;
    int my = scrH / 2 - panY;
    if (0 <= mx && mx < scrW && 0 <= my && my < scrH) {
        cv->fillCircle(mx, my, 4, TFT_WHITE);
        cv->fillCircle(mx, my, 2, TFT_RED);
    }
    cv->setTextSize(1);

    cv->setTextColor(TFT_WHITE, 0x0000);
    cv->setCursor(3, 2);
    if (gpsFix) {
        char la[12], lo[12];
        dtostrf(curLat, 2, 5, la);
        dtostrf(curLon, 2, 5, lo);
        cv->printf("Z:%d %s %s", curZoom, la, lo);
    } else {
        cv->printf("Z:%d [last]", curZoom);
    }

    LocalTime lt = getLocalTime();
    int totalSat = getTotalSatellites();
    if (lt.valid) {
        char infoBuf[20];
        snprintf(infoBuf, sizeof(infoBuf), "[%02d:%02d][%d]",
                 lt.hour, lt.minute, totalSat);
        cv->setTextColor(TFT_WHITE, 0x0000);
        cv->drawString(infoBuf, scrW - cv->textWidth(infoBuf) - 4, 2);
    } else {
        char satBuf[10];
        snprintf(satBuf, sizeof(satBuf), "[%d]", totalSat);
        cv->setTextColor(TFT_WHITE, 0x0000);
        cv->drawString(satBuf, scrW - cv->textWidth(satBuf) - 4, 2);
    }

    if (!gpsFix) {
        cv->fillRect(0, scrH - 12, scrW, 12, 0x0000);
        cv->setTextColor(TFT_YELLOW, 0x0000);
        cv->drawCenterString("Searching GPS...", scrW / 2, scrH - 11);
    }
}

// ═══════════════════════════════════════════
//  屏幕调度
// ═══════════════════════════════════════════

void updateScreen(bool force = false) {
    static unsigned long lastUpdate = 0;
    unsigned long now = millis();

    if (currentScreen == SCR_GPS_INFO) {
        if (!force && now - lastUpdate < 500) return;
    } else {
        if (!force && !dirty) return;
        if (!force && now - lastUpdate < REDRAW_MS) return;
        dirty = false;
    }
    lastUpdate = now;

    cv->fillScreen(TFT_BLACK);

    switch (currentScreen) {
        case SCR_GPS_INFO: drawScreenGpsInfo(); break;
        case SCR_MAP:      drawScreenMap();     break;
    }

    if (showSavedMsg) {
        if (now - savedMsgTime < 1500) {
            cv->fillRect(70, 55, 100, 25, TFT_BLACK);
            cv->drawRect(70, 55, 100, 25, TFT_GREEN);
            cv->setTextColor(TFT_GREEN, TFT_BLACK);
            cv->setTextSize(1);
            cv->drawCenterString("Saved!", scrW / 2, 63);
        } else {
            showSavedMsg = false;
        }
    }

    cv->pushSprite(0, 0);
}

// ═══════════════════════════════════════════
//  GPS 读取
// ═══════════════════════════════════════════

void readGPS() {
    while (GPS_Serial.available()) {
        char c = GPS_Serial.read();
        gps.encode(c);

        if (c == '\n') {
            nmeaLineBuf.trim();
            if (nmeaLineBuf.startsWith("$"))
                parseNmeaLine(nmeaLineBuf);
            nmeaLineBuf = "";
        } else if (c != '\r') {
            if (nmeaLineBuf.length() < 128) nmeaLineBuf += c;
        }
    }

    if (gps.location.isUpdated() && gps.location.isValid()) {
        double nlat = gps.location.lat();
        double nlon = gps.location.lng();
        if (!gpsFix ||
            fabs(nlat - curLat) > MOVE_THRESH ||
            fabs(nlon - curLon) > MOVE_THRESH)
        {
            curLat  = nlat;
            curLon  = nlon;
            gpsFix  = true;
            lastFixUpdate = millis();
            panning = false;
            dirty   = true;

            if (!autoSwitched) {
                currentScreen = SCR_MAP;
                autoSwitched  = true;
            }
        }
    }

    if (gpsFix && millis() - lastFixUpdate > GPS_TIMEOUT) {
        gpsFix = false;
        dirty  = true;
    }

    if (gpsFix && millis() - lastSave >= SAVE_INTERVAL) {
        lastSave = millis();
        saveLastPosition();
    }
}

// ═══════════════════════════════════════════
//  键盘
// ═══════════════════════════════════════════

void handleInput() {
    M5Cardputer.update();

    // 缩放: 上升沿检测 (仅地图页)
    bool curZ = M5Cardputer.Keyboard.isKeyPressed('z');
    bool curX = M5Cardputer.Keyboard.isKeyPressed('x');
    if (currentScreen == SCR_MAP) {
        if (curZ && !lastZState) {
            if (curZoom > ZOOM_MIN) { curZoom--; dirty = true; }
        }
        if (curX && !lastXState) {
            if (curZoom < ZOOM_MAX) { curZoom++; dirty = true; }
        }
    }
    lastZState = curZ;
    lastXState = curX;

    if (M5Cardputer.Keyboard.isChange()) {
        if (M5Cardputer.Keyboard.isPressed()) {
            Keyboard_Class::KeysState status =
                M5Cardputer.Keyboard.keysState();

            // Tab: 切换
            if (status.tab) {
                currentScreen = (ScreenID)((currentScreen + 1) % SCR_COUNT);
                dirUp = dirDown = dirLeft = dirRight = false;
                dirty = true;
                updateScreen(true);
                delay(200);
                return;
            }

            // Enter: 截图
            if (status.enter) {
                if (saveScreenshot()) {
                    showSavedMsg = true;
                    savedMsgTime = millis();
                    dirty = true;
                }
                delay(200);
                return;
            }

            // 地图页按键
            if (currentScreen == SCR_MAP) {
                dirUp = dirDown = dirLeft = dirRight = false;

                for (auto c : status.word) {
                    if (c == 'z' || c == 'x') continue;
                    switch (c) {
                        case ';': dirUp    = true; break;
                        case '.': dirDown  = true; break;
                        case ',': dirLeft  = true; break;
                        case '/': dirRight = true; break;
                        case '`':
                            panX = panY = 0;
                            if (gpsFix) panning = false;
                            dirty = true;
                            break;
                    }
                }

                if (dirUp || dirDown || dirLeft || dirRight) {
                    int fpx = (dirRight ? 1 : 0) - (dirLeft ? 1 : 0);
                    int fpy = (dirDown  ? 1 : 0) - (dirUp   ? 1 : 0);
                    panX += fpx * PAN_STEP;
                    panY += fpy * PAN_STEP;
                    panning = true;
                    dirty   = true;
                    lastDirStep = millis();
                }
            }
        } else {
            dirUp = dirDown = dirLeft = dirRight = false;
        }
    }
}

// ═══════════════════════════════════════════
//  setup
// ═══════════════════════════════════════════

void setup() {
    auto cfg = M5.config();
    M5Cardputer.begin(cfg, true);
    M5Cardputer.Display.setRotation(1);

    scrW = M5Cardputer.Display.width();
    scrH = M5Cardputer.Display.height();

    initGPS();

    pinMode(LORA_CS_PIN, OUTPUT);
    digitalWrite(LORA_CS_PIN, HIGH);

    SPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);

    bool sdOK = false;
    for (int retry = 0; retry < 5; retry++) {
        if (SD.begin(SD_CS_PIN)) { sdOK = true; break; }
        delay(300);
    }

    if (!sdOK) {
        M5Cardputer.Display.fillScreen(TFT_BLACK);
        M5Cardputer.Display.setTextColor(TFT_RED);
        M5Cardputer.Display.setTextSize(2);
        M5Cardputer.Display.drawCenterString("SD ERROR",
                                              scrW / 2, scrH / 2 - 8);
        while (1) delay(1000);
    }

    cv = new M5Canvas(&M5Cardputer.Display);
    if (!cv->createSprite(scrW, scrH)) {
        M5Cardputer.Display.fillScreen(TFT_BLACK);
        M5Cardputer.Display.setTextColor(TFT_RED);
        M5Cardputer.Display.setTextSize(1);
        M5Cardputer.Display.drawString("Canvas alloc fail", 10, 10);
        while (1) delay(1000);
    }

    initScreenshotCounter();
    hasLastPos = loadLastPosition();

    updateScreen(true);
}

// ═══════════════════════════════════════════
//  loop
// ═══════════════════════════════════════════

void loop() {
    readGPS();
    handleInput();

    if (currentScreen == SCR_MAP &&
        (dirUp || dirDown || dirLeft || dirRight) &&
        millis() - lastDirStep >= DIR_REPEAT_MS)
    {
        int fpx = (dirRight ? 1 : 0) - (dirLeft ? 1 : 0);
        int fpy = (dirDown  ? 1 : 0) - (dirUp   ? 1 : 0);
        panX += fpx * PAN_STEP;
        panY += fpy * PAN_STEP;
        panning = true;
        dirty   = true;
        lastDirStep = millis();
    }

    updateScreen();
    delay(10);
}
