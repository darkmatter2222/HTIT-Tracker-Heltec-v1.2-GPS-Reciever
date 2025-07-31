#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>
#include <Adafruit_ST7735.h>
#include <Adafruit_GFX.h>
#include <SPI.h>
#include <EEPROM.h>
#include <esp_sleep.h>

// PIN DEFINITIONS
#define VGNSS_CTRL   3    // GPIO 3 → Vext (active-low) powers UC6580 + ST7735
#define GPS_RX_PIN  33    // UC6580 TX → ESP32 RX
#define GPS_TX_PIN  34    // UC6580 RX ← ESP32 TX
#define VBAT_PIN     A0   // ADC1_CH0 on GPIO 1 (junction of 100 Ω/390 Ω divider)
#define VBAT_EN       2   // GPIO 2 must be HIGH to connect that divider
#define BL_CTRL_PIN  21   // GPIO 21 enables ST7735 backlight (HIGH = on)
#define USER_BTN_PIN  0   // GPIO 0 is the USER button (active-low)

// UI/UX SPACING STANDARDS - COMPACT WHITE TEXT
#define TEXT_LINE_HEIGHT 10     // Compact line spacing for small white text
#define TEXT_ROW_0 0           // Title row
#define TEXT_ROW_1 10          // First data row  
#define TEXT_ROW_2 20          // Second data row
#define TEXT_ROW_3 30          // Third data row
#define TEXT_ROW_4 40          // Fourth data row
#define TEXT_ROW_5 50          // Fifth data row (max for 5-row display)
#define EEPROM_SIZE 512
#define EEPROM_MAGIC 0xA5B4
#define ADDR_MAGIC 0
#define ADDR_WAYPOINT1_LAT 4
#define ADDR_WAYPOINT1_LON 12
#define ADDR_WAYPOINT2_LAT 20
#define ADDR_WAYPOINT2_LON 28
#define ADDR_WAYPOINT3_LAT 36
#define ADDR_WAYPOINT3_LON 44
#define ADDR_WAYPOINT1_SET 52
#define ADDR_WAYPOINT2_SET 53
#define ADDR_WAYPOINT3_SET 54
#define ADDR_SETTINGS 60
#define ADDR_LAST_LAT 70
#define ADDR_LAST_LON 78
#define ADDR_LAST_VALID 86

// SCREEN DEFINITIONS
enum ScreenType {
    SCREEN_STATUS = 0,
    SCREEN_NAVIGATION,
    SCREEN_MAIN_MENU,
    SCREEN_WAYPOINT_MENU,
    SCREEN_WAYPOINT1_NAV,
    SCREEN_WAYPOINT2_NAV,
    SCREEN_WAYPOINT3_NAV,
    SCREEN_SET_WAYPOINT,
    SCREEN_SYSTEM_INFO,
    SCREEN_GPS_STATUS,         // New: Detailed GPS constellation status
    SCREEN_POWER_MENU,
    SCREEN_WAYPOINT_RESET,     // Ask to reset waypoint or navigate
    SCREEN_COUNT
};

// POWER MODES
enum PowerMode {
    POWER_FULL = 0,     // 1s refresh, full performance
    POWER_ECO,          // 3s refresh, balanced performance  
    POWER_ULTRA         // 10s refresh, minimal power consumption
};

// WAYPOINT STRUCTURE
struct Waypoint {
    double lat;
    double lon;
    bool isSet;
    char name[12];
};

// GPS CONSTELLATION TRACKING
struct ConstellationInfo {
    int count;
    float avgSNR;
    bool hasFix;
    bool isActiveForFix;  // New: Is this constellation contributing to the current fix
};

// GPS MOTION STATE
enum MotionState {
    MOTION_UNKNOWN = 0,
    MOTION_STATIONARY,
    MOTION_SLOW,
    MOTION_MOVING,
    MOTION_FAST
};

class HTITTracker {
private:
    // Display instance
    // ST7735 Display pins for Heltec LoRa v3 
#define TFT_CS    18  // Chip Select
#define TFT_RST   21  // Reset
#define TFT_DC    17  // Data/Command
#define TFT_SCLK  5   // SPI Clock
#define TFT_MOSI  6   // SPI MOSI

Adafruit_ST7735 st7735 = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);
    
    // Enhanced GPS Performance
    ConstellationInfo constellations[5];  // GPS, GLONASS, Beidou, Galileo, QZSS
    MotionState currentMotionState;
    unsigned long lastPositionTime;
    double lastStoredLat, lastStoredLon;
    bool hasStoredPosition;
    unsigned long gpsUpdateInterval;      // Adaptive GPS polling rate
    unsigned long lastGPSPoll;
    
    // Satellite counts per constellation
    int gpsCount;
    int glonassCount;
    int beidouCount;
    int galileoCount;
    int qzssCount;
    int totalInView;
    
    // GNSS fix flag and HDOP → accuracy
    bool haveFix;
    float lastHDOP;
    
    // Home navigation variables
    bool homeEstablished;
    double homeLat, homeLon;           // Home coordinates
    double currentLat, currentLon;     // Current coordinates
    bool hasValidPosition;             // Current position is valid
    
    // Enhanced waypoint system (3 waypoints + home)
    Waypoint waypoints[3];             // Waypoint 1, 2, 3
    int activeWaypoint;                // Currently selected waypoint (0=home, 1-3=waypoints)
    int waypointToSet;                 // Which waypoint we're setting
    int waypointToReset;               // Which waypoint we're considering to reset
    
    // Enhanced UI system
    ScreenType currentScreen;          // Current screen being displayed
    ScreenType lastScreen;             // Previous screen for back navigation
    int menuIndex;                     // Current menu selection
    int menuItemCount;                 // Number of items in current menu
    bool inMenu;                       // Are we in a menu?
    
    // Button handling
    bool buttonPressed;                // Button press flag
    unsigned long lastButtonPress;     // Debounce timing
    unsigned long buttonPressStart;    // For long press detection
    bool longPressHandled;             // Prevent multiple long press events
    
    // Screen management
    unsigned long lastActivity;        // Last user activity
    bool forceScreenRedraw;            // Force all screens to redraw on next update
    
    // Speed calculation
    double lastLat, lastLon;           // Previous position for speed calculation
    unsigned long lastSpeedTime;      // Time of last speed calculation
    float currentSpeed;                // Current speed in km/h
    bool hasValidSpeed;                // Speed calculation valid
    
    // Battery monitoring improvements
    float batteryReadings[5];          // Rolling buffer for battery percentage
    int batteryIndex;                  // Current index in rolling buffer
    bool batteryBufferFull;            // Whether we have 5 readings yet
    float lastBatteryVoltage;          // Previous voltage reading for charging detection
    bool isCharging;                   // Whether battery is currently charging
    unsigned long lastChargingCheck;   // Time of last charging check
    
    // Smart Power Management
    PowerMode currentPowerMode;        // Current power mode (Full/Eco/Ultra)
    unsigned long lastScreenUpdate;   // Time of last screen refresh
    
    // Previous display state for flicker-free updates
    char prevFixBuf[16];
    char prevDistBuf[16]; 
    char prevSatBuf[16];
    char prevBattBuf[16];
    char prevAccBuf[16];
    char prevSpeedBuf[16];
    bool prevDisplayValid;
    
    // Buffer for NMEA line accumulation
    char lineBuf[128];
    int linePos;
    
    // Timing for LCD refresh (once per second)
    unsigned long lastLCDupdate;
    static const unsigned long LCD_INTERVAL = 1000; // ms
    
    // Private helper methods
    void processNMEALine(const char* line);
    int parseGSVinView(const char* gsvLine);
    int parseGGAfixQuality(const char* ggaLine);
    float parseGGAHDOP(const char* ggaLine);
    bool parseGGAPosition(const char* ggaLine, double &lat, double &lon);
    float readBatteryVoltageRaw(int &rawADC);
    int voltageToPercent(float vb);
    void updateLCD(int pct_cal);
    void updateStatusScreen(int pct_cal);
    void updateNavigationScreen(int pct_cal);
    
    // Enhanced GPS Performance
    void updateConstellationInfo();
    void analyzeMotionState();
    void adaptGPSUpdateRate();
    void storeLastPosition();
    void loadLastPosition();
    bool isPositionStationary();
    
    // UI/UX Enhancement methods
    void drawProgressBar(int x, int y, int width, int height, int percentage, uint16_t color);
    void drawBatteryIcon(int x, int y, int percentage);
    void drawGPSIcon(int x, int y, int signalStrength);
    void drawPowerModeIcon(int x, int y, PowerMode mode);
    uint16_t getStatusColor(int value, int goodThreshold, int okThreshold);
    
    // Enhanced UI screen methods
    void updateMainMenuScreen();
    void updateWaypointMenuScreen();
    void updateWaypointNavigationScreen(int pct_cal);
    void updateSetWaypointScreen();
    void updateSystemInfoScreen(int pct_cal);
    void updateGPSStatusScreen(int pct_cal);  // New: GPS constellation status
    void updatePowerMenuScreen();
    void updateWaypointResetScreen();
    
    void checkButton();
    void calculateSpeed();
    int getStableBatteryPercent(float voltage);
    void updateChargingStatus(float voltage);
    
    // Smart Power Management
    void setPowerMode(PowerMode mode);
    bool shouldUpdateScreen();
    unsigned long getPowerModeInterval();
    const char* getPowerModeString();
    
    // Enhanced navigation methods
    float calculateBearingToHome();
    float calculateDistanceToHome();
    float calculateBearingToWaypoint(int waypointIndex);
    float calculateDistanceToWaypoint(int waypointIndex);
    const char* getCardinalDirection(float bearingToHome);
    
    // Waypoint management
    void setWaypoint(int index, double lat, double lon, const char* name);
    void loadWaypointsFromEEPROM();
    void saveWaypointsToEEPROM();

public:
    // Constructor
    HTITTracker();
    
    // Public methods
    void begin();
    void update();
    
    // Getters for status information
    bool getFixStatus() const { return haveFix; }
    int getTotalSatellites() const { return totalInView; }
    float getHDOP() const { return lastHDOP; }
    int getGPSCount() const { return gpsCount; }
    int getGLONASSCount() const { return glonassCount; }
    int getBeidouCount() const { return beidouCount; }
    int getGalileoCount() const { return galileoCount; }
    int getQZSSCount() const { return qzssCount; }
    
    // Home navigation getters
    bool isHomeEstablished() const { return homeEstablished; }
    bool hasCurrentPosition() const { return hasValidPosition; }
};

// Implementation of HTITTracker class methods

inline HTITTracker::HTITTracker() 
    : gpsCount(0), glonassCount(0), beidouCount(0), galileoCount(0), 
      qzssCount(0), totalInView(0), haveFix(false), lastHDOP(99.99f),
      linePos(0), lastLCDupdate(0), homeEstablished(false),
      homeLat(0.0), homeLon(0.0), currentLat(0.0), currentLon(0.0),
      hasValidPosition(false), activeWaypoint(0), waypointToSet(0), waypointToReset(0),
      currentScreen(SCREEN_MAIN_MENU), lastScreen(SCREEN_MAIN_MENU),
      menuIndex(0), menuItemCount(0), inMenu(false), buttonPressed(false),
      lastButtonPress(0), buttonPressStart(0), longPressHandled(false),
      lastActivity(0), forceScreenRedraw(false), lastLat(0.0), lastLon(0.0), 
      lastSpeedTime(0), currentSpeed(0.0f), hasValidSpeed(false), 
      prevDisplayValid(false), batteryIndex(0), batteryBufferFull(false), 
      lastBatteryVoltage(0.0f), isCharging(false), lastChargingCheck(0),
      currentPowerMode(POWER_FULL), lastScreenUpdate(0),
      currentMotionState(MOTION_UNKNOWN), lastPositionTime(0),
      lastStoredLat(0.0), lastStoredLon(0.0), hasStoredPosition(false),
      gpsUpdateInterval(1000), lastGPSPoll(0) {
    
    // Initialize constellation info
    for (int i = 0; i < 5; i++) {
        constellations[i].count = 0;
        constellations[i].avgSNR = 0.0f;
        constellations[i].hasFix = false;
        constellations[i].isActiveForFix = false;
    }
    
    // Initialize waypoints as unset
    for (int i = 0; i < 3; i++) {
        waypoints[i].isSet = false;
        waypoints[i].lat = 0.0;
        waypoints[i].lon = 0.0;
        strcpy(waypoints[i].name, "");
    }
    memset(lineBuf, 0, sizeof(lineBuf));
    memset(prevFixBuf, 0, sizeof(prevFixBuf));
    memset(prevDistBuf, 0, sizeof(prevDistBuf));
    memset(prevSatBuf, 0, sizeof(prevSatBuf));
    memset(prevBattBuf, 0, sizeof(prevBattBuf));
    memset(prevAccBuf, 0, sizeof(prevAccBuf));
    memset(prevSpeedBuf, 0, sizeof(prevSpeedBuf));
    
    // Initialize battery readings array
    for (int i = 0; i < 5; i++) {
        batteryReadings[i] = 0.0f;
    }
}

inline void HTITTracker::begin() {
    // 1) USB-Serial for debugging
    Serial.begin(115200);
    while (!Serial) { delay(10); }
    Serial.println();
    Serial.println("HTIT-Tracker v1.2 Enhanced: GPS Performance + UI Improvements");

    // 2) Configure VBAT_EN (GPIO 2) and keep LOW until measurement
    pinMode(VBAT_EN, OUTPUT);
    digitalWrite(VBAT_EN, LOW);

    // 3) Power on GNSS + TFT via Vext (active-low on v1.2)
    pinMode(VGNSS_CTRL, OUTPUT);
    digitalWrite(VGNSS_CTRL, LOW);   // Enable 3.3 V rail for UC6580 + ST7735
    Serial.println("→ VGNSS_CTRL (GPIO 3) = LOW (GNSS + TFT powered)");
    delay(200);  // allow regulator + GNSS to stabilize

    // 4) Enable TFT backlight
    pinMode(BL_CTRL_PIN, OUTPUT);
    digitalWrite(BL_CTRL_PIN, HIGH);  // Turn backlight ON
    Serial.println("→ BL_CTRL (GPIO 21) = HIGH (Backlight ON)");

    // 5) Configure USER button
    pinMode(USER_BTN_PIN, INPUT_PULLUP);
    Serial.println("→ USER_BTN (GPIO 0) configured with pullup");

    // 6) Set ADC attenuation so VBAT/2 (≈0.857–1.07 V) reads accurately
    analogSetPinAttenuation(VBAT_PIN, ADC_11db);

    // 7) Initialize Serial1 @115200 to read UC6580 NMEA
    Serial1.begin(115200, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    Serial.println("→ Serial1.begin(115200, RX=33, TX=34) for UC6580");

    // 8) Initialize ST7735 display with proper Adafruit library
    st7735.initR(INITR_BLACKTAB);  // Initialize with black tab variant
    st7735.fillScreen(ST7735_BLACK);
    
    // Configure text: WHITE color, small size (1), no wrap
    st7735.setTextColor(ST7735_WHITE);
    st7735.setTextSize(1);
    st7735.setTextWrap(false);
    st7735.setRotation(0);
    
    // 9) Initialize EEPROM and load waypoints
    EEPROM.begin(512);  // Initialize EEPROM with 512 bytes
    Serial.println("→ EEPROM initialized (512 bytes)");
    loadWaypointsFromEEPROM();
    loadLastPosition();  // Load GPS warm start data
    
    // 10) Initialize activity timer for screensaver
    lastActivity = millis();
    Serial.println("→ Enhanced GPS and UI features initialized");
}

inline void HTITTracker::update() {
    // A) Check button for screen switching
    checkButton();
    
    // B) Enhanced GPS Performance Updates
    updateConstellationInfo();
    analyzeMotionState();
    adaptGPSUpdateRate();
    
    // C) Read raw NMEA from Serial1, echo to USB-Serial, accumulate lines
    unsigned long now = millis();
    if (now - lastGPSPoll >= gpsUpdateInterval) {
        lastGPSPoll = now;
        
        while (Serial1.available() > 0) {
            char c = (char)Serial1.read();
            Serial.write(c);  // echo raw NMEA
            if (c == '\r' || c == '\n') {
                if (linePos > 0) {
                    lineBuf[linePos] = '\0';
                    processNMEALine(lineBuf);
                    linePos = 0;
                }
            }
            else if (linePos < (int)sizeof(lineBuf) - 1) {
                lineBuf[linePos++] = c;
            }
        }
    }

    // E) Once per second, update display
    if (now - lastLCDupdate >= LCD_INTERVAL) {
        lastLCDupdate = now;

        // 1) Read raw ADC + true VBAT (volts)
        int rawADC = 0;
        float vb = readBatteryVoltageRaw(rawADC);

        // 2) Compute "calibrated VBAT" using 5.05× instead of 4.90×
        float vb_cal = (rawADC / 4095.0f) * 3.3f * 5.05f;

        // 3) Update charging status and get stable battery percentage
        updateChargingStatus(vb_cal);
        int pct_cal = getStableBatteryPercent(vb_cal);

        // 4) Debug print every 2 s
        static unsigned long lastPrint = 0;
        static bool firstTime = true;
        if (firstTime || (now - lastPrint >= 2000)) {
            firstTime = false;
            lastPrint = now;
            float vAD = (rawADC / 4095.0f) * 3.3f;
            Serial.print("Raw ADC = "); Serial.print(rawADC);
            Serial.print("    V_ADC = "); Serial.print(vAD, 3); Serial.print(" V");
            Serial.print("    VBAT = "); Serial.print(vb, 2); Serial.print(" V");
            Serial.print("    VBAT_cal = "); Serial.print(vb_cal, 2); Serial.print(" V");
            Serial.print("    Batt% = "); Serial.print(pct_cal); 
            Serial.print(" %    Charging: "); Serial.println(isCharging ? "Yes" : "No");
        }

        // 5) Draw display based on current screen
        updateLCD(pct_cal);
    }
}

inline void HTITTracker::processNMEALine(const char* line) {
    if (strncmp(line, "$GPGSV", 6) == 0) {
        gpsCount = parseGSVinView(line);
    }
    else if (strncmp(line, "$GLGSV", 6) == 0) {
        glonassCount = parseGSVinView(line);
    }
    else if (strncmp(line, "$GBGSV", 6) == 0) {
        beidouCount = parseGSVinView(line);
    }
    else if (strncmp(line, "$GAGSV", 6) == 0) {
        galileoCount = parseGSVinView(line);
    }
    else if (strncmp(line, "$GQGSV", 6) == 0) {
        qzssCount = parseGSVinView(line);
    }
    else if (strncmp(line, "$GNGGA", 6) == 0) {
        // 1) Fix quality (7th field)
        int fixQual = parseGGAfixQuality(line);
        haveFix = (fixQual > 0);

        // 2) HDOP (9th field) → update lastHDOP if valid
        float hdop = parseGGAHDOP(line);
        if (hdop > 0.0f && hdop < 100.0f) {
            lastHDOP = hdop;
        }
        
        // 3) Parse position (latitude and longitude)
        double lat, lon;
        if (parseGGAPosition(line, lat, lon)) {
            currentLat = lat;
            currentLon = lon;
            hasValidPosition = true;
            lastPositionTime = millis();
            
            // Store position for GPS warm start (every 30 seconds)
            static unsigned long lastPositionStore = 0;
            if (millis() - lastPositionStore > 30000) {
                storeLastPosition();
                lastPositionStore = millis();
            }
            
            // Calculate speed if we have a previous position
            calculateSpeed();
            
            // Establish home if we have a fix and haven't set home yet
            if (haveFix && !homeEstablished) {
                homeLat = lat;
                homeLon = lon;
                homeEstablished = true;
                Serial.println("→ HOME ESTABLISHED!");
                Serial.print("   Home coordinates: ");
                Serial.print(homeLat, 6); Serial.print(", "); Serial.println(homeLon, 6);
            }
        }
    }
    // Sum satellites in view
    totalInView = gpsCount + glonassCount + beidouCount + galileoCount + qzssCount;
}

inline int HTITTracker::parseGSVinView(const char* gsvLine) {
    // Extract 4th field (<totalInView>) from "$GxGSV"
    const char* p = strchr(gsvLine, ',');
    if (!p) return 0;
    p = strchr(p + 1, ','); if (!p) return 0;
    p = strchr(p + 1, ','); if (!p) return 0;
    return atoi(p + 1);
}

inline int HTITTracker::parseGGAfixQuality(const char* ggaLine) {
    // Extract 7th field (<fixQuality>) from "$GNGGA"
    int commaCount = 0;
    const char* p = ggaLine;
    while (*p && commaCount < 6) {
        if (*p == ',') commaCount++;
        p++;
    }
    if (!*p) return 0;
    return atoi(p);
}

inline float HTITTracker::parseGGAHDOP(const char* ggaLine) {
    // Extract 9th field (<HDOP>) from "$GNGGA"
    int commaCount = 0;
    const char* p = ggaLine;
    while (*p && commaCount < 8) {
        if (*p == ',') commaCount++;
        p++;
    }
    if (!*p || *p == ',') return -1.0f;
    return atof(p);
}

inline bool HTITTracker::parseGGAPosition(const char* ggaLine, double &lat, double &lon) {
    // Parse $GNGGA format: $GNGGA,time,lat,N/S,lon,E/W,fix,sats,hdop,alt,M,geoid,M,dgps_age,dgps_id*checksum
    // Fields: 0=GNGGA, 1=time, 2=lat, 3=N/S, 4=lon, 5=E/W, 6=fix_quality
    
    const char* fields[15];
    int fieldCount = 0;
    const char* start = ggaLine;
    
    // Split the line by commas
    for (const char* p = ggaLine; *p && fieldCount < 15; p++) {
        if (*p == ',' || *p == '*') {
            fields[fieldCount++] = start;
            start = p + 1;
        }
    }
    
    if (fieldCount < 6) return false;
    
    // Parse latitude (field 2, format: ddmm.mmmmm)
    const char* latStr = fields[2];
    const char* latDir = fields[3];
    if (strlen(latStr) < 7 || strlen(latDir) < 1) return false;
    
    double latDeg = (latStr[0] - '0') * 10 + (latStr[1] - '0');
    double latMin = atof(latStr + 2);
    lat = latDeg + latMin / 60.0;
    if (latDir[0] == 'S') lat = -lat;
    
    // Parse longitude (field 4, format: dddmm.mmmmm)
    const char* lonStr = fields[4];
    const char* lonDir = fields[5];
    if (strlen(lonStr) < 8 || strlen(lonDir) < 1) return false;
    
    double lonDeg = (lonStr[0] - '0') * 100 + (lonStr[1] - '0') * 10 + (lonStr[2] - '0');
    double lonMin = atof(lonStr + 3);
    lon = lonDeg + lonMin / 60.0;
    if (lonDir[0] == 'W') lon = -lon;
    
    return true;
}

inline float HTITTracker::readBatteryVoltageRaw(int &rawADC) {
    // 1) Drive VBAT_EN HIGH to connect 100 Ω/390 Ω divider
    digitalWrite(VBAT_EN, HIGH);
    delayMicroseconds(10);

    // 2) Read 12-bit ADC (0–4095)
    rawADC = analogRead(VBAT_PIN);

    // 3) Disconnect divider to save battery
    digitalWrite(VBAT_EN, LOW);

    // 4) Convert raw→V_ADC in volts
    const float ADC_MAX = 4095.0f;
    const float REF_V = 3.3f;
    float vAD = (rawADC / ADC_MAX) * REF_V;

    // 5) Return true VBAT = vAD × 4.90 (because divider = 100/(100+390))
    return vAD * 4.90f;
}

inline int HTITTracker::voltageToPercent(float vb) {
    // Accurate lithium battery discharge curve for 3.7V 3000mAh battery
    // Based on typical 18650 lithium-ion discharge characteristics
    
    if (vb >= 4.20f) {
        return 100;  // Fully charged
    } 
    else if (vb >= 4.10f) {
        return (int)roundf(95.0f + (vb - 4.10f)/(4.20f - 4.10f)*5.0f);  // 95-100%
    }
    else if (vb >= 4.00f) {
        return (int)roundf(85.0f + (vb - 4.00f)/(4.10f - 4.00f)*10.0f); // 85-95%
    }
    else if (vb >= 3.90f) {
        return (int)roundf(70.0f + (vb - 3.90f)/(4.00f - 3.90f)*15.0f); // 70-85%
    }
    else if (vb >= 3.80f) {
        return (int)roundf(50.0f + (vb - 3.80f)/(3.90f - 3.80f)*20.0f); // 50-70%
    }
    else if (vb >= 3.70f) {
        return (int)roundf(30.0f + (vb - 3.70f)/(3.80f - 3.70f)*20.0f); // 30-50%
    }
    else if (vb >= 3.60f) {
        return (int)roundf(15.0f + (vb - 3.60f)/(3.70f - 3.60f)*15.0f); // 15-30%
    }
    else if (vb >= 3.50f) {
        return (int)roundf(5.0f + (vb - 3.50f)/(3.60f - 3.50f)*10.0f);  // 5-15%
    }
    else if (vb >= 3.30f) {
        return (int)roundf((vb - 3.30f)/(3.50f - 3.30f)*5.0f);          // 0-5%
    }
    return 0;  // Battery critically low or disconnected
}

// Home navigation helper methods implementation
inline float HTITTracker::calculateBearingToHome() {
    if (!homeEstablished || !hasValidPosition) {
        return 0.0f;  // Default to North
    }
    
    // Calculate bearing from current position to home
    double lat1 = currentLat * PI / 180.0;
    double lon1 = currentLon * PI / 180.0;
    double lat2 = homeLat * PI / 180.0;
    double lon2 = homeLon * PI / 180.0;
    
    double dLon = lon2 - lon1;
    
    double y = sin(dLon) * cos(lat2);
    double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);
    
    double bearing = atan2(y, x) * 180.0 / PI;
    bearing = fmod(bearing + 360.0, 360.0);  // Normalize to 0-360
    
    return bearing;
}

inline float HTITTracker::calculateDistanceToHome() {
    if (!homeEstablished || !hasValidPosition) {
        return 0.0f;  // No distance if no home or position
    }
    
    // Calculate distance using Haversine formula
    double lat1 = currentLat * PI / 180.0;
    double lon1 = currentLon * PI / 180.0;
    double lat2 = homeLat * PI / 180.0;
    double lon2 = homeLon * PI / 180.0;
    
    double dLat = lat2 - lat1;
    double dLon = lon2 - lon1;
    
    double a = sin(dLat/2) * sin(dLat/2) + 
               cos(lat1) * cos(lat2) * 
               sin(dLon/2) * sin(dLon/2);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    
    // Earth's radius in meters
    const double EARTH_RADIUS = 6371000.0;
    
    return EARTH_RADIUS * c;  // Distance in meters
}

// Button handling for screen switching
inline void HTITTracker::checkButton() {
    static bool lastButtonState = true;  // HIGH when not pressed (pullup)
    bool currentButtonState = digitalRead(USER_BTN_PIN);
    static unsigned long buttonPressStart = 0;
    static bool longPressHandled = false;
    
    // Detect button press start (HIGH to LOW transition)
    if (lastButtonState && !currentButtonState) {
        unsigned long now = millis();
        if (now - lastButtonPress > 200) {  // 200ms debounce
            buttonPressStart = now;
            longPressHandled = false;
            lastActivity = now;  // Update activity for potential future features
        }
    }
    
    // Detect long press (1000ms threshold)
    if (!currentButtonState && !longPressHandled && 
        buttonPressStart > 0 && (millis() - buttonPressStart > 1000)) {
        
        longPressHandled = true;
        lastActivity = millis();
        
        // Long press actions - scroll through menu or return to main menu
        if (currentScreen == SCREEN_MAIN_MENU) {
            menuIndex = (menuIndex + 1) % 5;  // 5 items in main menu now (added GPS Status)
            Serial.println("→ Main menu scroll (long press)");
        } else if (currentScreen == SCREEN_WAYPOINT_MENU) {
            menuIndex = (menuIndex + 1) % 4;  // 4 items in waypoint menu
            Serial.println("→ Waypoint menu scroll (long press)");
        } else if (currentScreen == SCREEN_WAYPOINT_RESET) {
            menuIndex = (menuIndex + 1) % 3;  // 3 options: Navigate, Reset, Cancel
            Serial.println("→ Waypoint reset menu scroll (long press)");
        } else if (currentScreen == SCREEN_POWER_MENU) {
            menuIndex = (menuIndex + 1) % 4;  // 4 options: Sleep, Deep Sleep, Screen Off, Back
            Serial.println("→ Power menu scroll (long press)");
        } else {
            // From any other screen, long press returns to main menu
            currentScreen = SCREEN_MAIN_MENU;
            menuIndex = 0;
            Serial.println("→ Long press: Return to Main Menu");
        }
    }
    
    // Detect button release (LOW to HIGH transition)
    if (!lastButtonState && currentButtonState && !longPressHandled) {
        unsigned long now = millis();
        if (buttonPressStart > 0 && (now - buttonPressStart < 1000)) {
            // Short press - select menu item or navigate
            lastButtonPress = now;
            lastActivity = now;
            
            if (currentScreen == SCREEN_MAIN_MENU) {
                // Handle main menu selection (now has 5 items - added GPS Status)
                if (menuIndex == 0) {  // Status
                    currentScreen = SCREEN_STATUS;
                    Serial.println("→ Entered Status Screen");
                } else if (menuIndex == 1) {  // Waypoints
                    currentScreen = SCREEN_WAYPOINT_MENU;
                    menuIndex = 0;
                    Serial.println("→ Entered Waypoint Menu");
                } else if (menuIndex == 2) {  // System Info
                    currentScreen = SCREEN_SYSTEM_INFO;
                    Serial.println("→ Entered System Info");
                } else if (menuIndex == 3) {  // GPS Status
                    currentScreen = SCREEN_GPS_STATUS;
                    Serial.println("→ Entered GPS Status");
                } else if (menuIndex == 4) {  // Power Menu
                    currentScreen = SCREEN_POWER_MENU;
                    Serial.println("→ Entered Power Menu");
                }
                
            } else if (currentScreen == SCREEN_WAYPOINT_MENU) {
                // Handle waypoint menu selection
                if (menuIndex == 0) {  // WP1
                    if (waypoints[0].isSet) {
                        currentScreen = SCREEN_WAYPOINT_RESET;
                        waypointToReset = 0;
                        menuIndex = 0;  // Reset to first option (Navigate)
                        Serial.println("→ WP1 Reset/Navigate Menu");
                    } else {
                        currentScreen = SCREEN_SET_WAYPOINT;
                        waypointToSet = 0;
                        Serial.println("→ Set WP1");
                    }
                } else if (menuIndex == 1) {  // WP2
                    if (waypoints[1].isSet) {
                        currentScreen = SCREEN_WAYPOINT_RESET;
                        waypointToReset = 1;
                        menuIndex = 0;  // Reset to first option (Navigate)
                        Serial.println("→ WP2 Reset/Navigate Menu");
                    } else {
                        currentScreen = SCREEN_SET_WAYPOINT;
                        waypointToSet = 1;
                        Serial.println("→ Set WP2");
                    }
                } else if (menuIndex == 2) {  // WP3
                    if (waypoints[2].isSet) {
                        currentScreen = SCREEN_WAYPOINT_RESET;
                        waypointToReset = 2;
                        menuIndex = 0;  // Reset to first option (Navigate)
                        Serial.println("→ WP3 Reset/Navigate Menu");
                    } else {
                        currentScreen = SCREEN_SET_WAYPOINT;
                        waypointToSet = 2;
                        Serial.println("→ Set WP3");
                    }
                } else if (menuIndex == 3) {  // Back
                    currentScreen = SCREEN_MAIN_MENU;
                    menuIndex = 2;  // Return to Waypoints item
                    Serial.println("→ Back to Main Menu");
                }
                
            } else if (currentScreen == SCREEN_SET_WAYPOINT) {
                // Save waypoint if GPS is ready
                if (hasValidPosition && haveFix) {
                    char name[12];
                    snprintf(name, sizeof(name), "WP%d", waypointToSet + 1);
                    setWaypoint(waypointToSet, currentLat, currentLon, name);
                    Serial.println("→ Waypoint saved!");
                    currentScreen = SCREEN_WAYPOINT_MENU;
                    menuIndex = waypointToSet;
                } else {
                    Serial.println("→ GPS not ready - cannot save waypoint");
                }
                
            } else if (currentScreen == SCREEN_WAYPOINT_RESET) {
                // Handle waypoint reset/navigate menu
                if (menuIndex == 0) {  // Navigate
                    currentScreen = (ScreenType)(SCREEN_WAYPOINT1_NAV + waypointToReset);
                    activeWaypoint = waypointToReset + 1;
                    Serial.printf("→ Navigate to WP%d\n", waypointToReset + 1);
                } else if (menuIndex == 1) {  // Reset
                    // Clear the waypoint and go to set screen
                    waypoints[waypointToReset].isSet = false;
                    waypoints[waypointToReset].lat = 0.0;
                    waypoints[waypointToReset].lon = 0.0;
                    strcpy(waypoints[waypointToReset].name, "");
                    saveWaypointsToEEPROM();
                    
                    currentScreen = SCREEN_SET_WAYPOINT;
                    waypointToSet = waypointToReset;
                    Serial.printf("→ Reset WP%d - now setting new waypoint\n", waypointToReset + 1);
                } else if (menuIndex == 2) {  // Cancel/Back
                    currentScreen = SCREEN_WAYPOINT_MENU;
                    menuIndex = waypointToReset;  // Return to the waypoint item
                    Serial.println("→ Back to Waypoint Menu");
                }
                
            } else if (currentScreen == SCREEN_POWER_MENU) {
                // Handle power mode selection
                if (menuIndex == 0) {  // Full Power Mode
                    setPowerMode(POWER_FULL);
                    currentScreen = SCREEN_MAIN_MENU;
                    menuIndex = 3;  // Return to Power Menu item in main menu
                    Serial.println("→ Set to Full Power Mode (1s refresh)");
                    
                } else if (menuIndex == 1) {  // Eco Mode
                    setPowerMode(POWER_ECO);
                    currentScreen = SCREEN_MAIN_MENU;
                    menuIndex = 3;  // Return to Power Menu item in main menu
                    Serial.println("→ Set to Eco Mode (3s refresh)");
                    
                } else if (menuIndex == 2) {  // Ultra Saver Mode
                    setPowerMode(POWER_ULTRA);
                    currentScreen = SCREEN_MAIN_MENU;
                    menuIndex = 3;  // Return to Power Menu item in main menu
                    Serial.println("→ Set to Ultra Saver Mode (10s refresh)");
                    
                } else if (menuIndex == 3) {  // Back
                    currentScreen = SCREEN_MAIN_MENU;
                    menuIndex = 3;  // Return to Power Menu item in main menu
                    Serial.println("→ Back to Main Menu");
                }
                
            } else {
                // From any other screen, return to main menu
                currentScreen = SCREEN_MAIN_MENU;
                menuIndex = 0;
                Serial.println("→ Return to Main Menu");
            }
        }
        buttonPressStart = 0;
    }
    
    lastButtonState = currentButtonState;
}

// Speed calculation
inline void HTITTracker::calculateSpeed() {
    if (!hasValidPosition) return;
    
    unsigned long now = millis();
    
    // Initialize if first valid position
    if (lastSpeedTime == 0) {
        lastLat = currentLat;
        lastLon = currentLon;
        lastSpeedTime = now;
        return;
    }
    
    // Calculate speed every 2 seconds
    if (now - lastSpeedTime >= 2000) {
        // Calculate distance using Haversine formula
        double lat1 = lastLat * PI / 180.0;
        double lon1 = lastLon * PI / 180.0;
        double lat2 = currentLat * PI / 180.0;
        double lon2 = currentLon * PI / 180.0;
        
        double dLat = lat2 - lat1;
        double dLon = lon2 - lon1;
        
        double a = sin(dLat/2) * sin(dLat/2) + 
                   cos(lat1) * cos(lat2) * 
                   sin(dLon/2) * sin(dLon/2);
        double c = 2 * atan2(sqrt(a), sqrt(1-a));
        
        // Distance in meters
        const double EARTH_RADIUS = 6371000.0;
        double distance = EARTH_RADIUS * c;
        
        // Time difference in hours
        double timeHours = (now - lastSpeedTime) / 3600000.0;
        
        // Speed in km/h
        if (timeHours > 0) {
            currentSpeed = (distance / 1000.0) / timeHours;
            hasValidSpeed = true;
        }
        
        // Update for next calculation
        lastLat = currentLat;
        lastLon = currentLon;
        lastSpeedTime = now;
    }
}

// Rolling average battery percentage for stability
inline int HTITTracker::getStableBatteryPercent(float voltage) {
    // Get the raw percentage
    int rawPercent = voltageToPercent(voltage);
    
    // Add to rolling buffer
    batteryReadings[batteryIndex] = rawPercent;
    batteryIndex = (batteryIndex + 1) % 5;
    
    if (!batteryBufferFull && batteryIndex == 0) {
        batteryBufferFull = true;
    }
    
    // Calculate average
    if (!batteryBufferFull) {
        // Not enough readings yet, return current reading
        return rawPercent;
    }
    
    float sum = 0;
    for (int i = 0; i < 5; i++) {
        sum += batteryReadings[i];
    }
    
    return (int)roundf(sum / 5.0f);
}

// Charging detection based on voltage trends
inline void HTITTracker::updateChargingStatus(float voltage) {
    unsigned long now = millis();
    
    // Only check every 10 seconds to allow for voltage stabilization
    if (now - lastChargingCheck < 10000) {
        return;
    }
    
    // Need at least one previous reading
    if (lastChargingCheck == 0) {
        lastBatteryVoltage = voltage;
        lastChargingCheck = now;
        isCharging = false;  // Default to not charging initially
        return;
    }
    
    // Calculate voltage change over time
    float voltageChange = voltage - lastBatteryVoltage;
    
    // More restrictive charging detection criteria:
    // 1. Voltage is rising significantly (> 0.05V over 10 seconds)
    // 2. Voltage is above 4.15V (active charging range)
    // 3. Significant positive trend indicating external power
    
    if (voltage > 4.15f && voltageChange > 0.05f) {
        isCharging = true;
    } else if (voltage < 4.10f || voltageChange < -0.02f) {
        // Discharging: voltage falling or below charging threshold
        isCharging = false;
    }
    // If voltage is stable (small change), keep previous charging state
    
    lastBatteryVoltage = voltage;
    lastChargingCheck = now;
}

// ========================== SMART POWER MANAGEMENT ==========================

inline void HTITTracker::setPowerMode(PowerMode mode) {
    currentPowerMode = mode;
    forceScreenRedraw = true;  // Force immediate update when mode changes
    Serial.print("→ Power mode changed to: ");
    Serial.println(getPowerModeString());
}

inline bool HTITTracker::shouldUpdateScreen() {
    unsigned long now = millis();
    unsigned long interval = getPowerModeInterval();
    
    if (now - lastScreenUpdate >= interval) {
        lastScreenUpdate = now;
        return true;
    }
    
    // Always allow updates if forced or first time
    if (forceScreenRedraw || lastScreenUpdate == 0) {
        lastScreenUpdate = now;
        return true;
    }
    
    return false;
}

inline unsigned long HTITTracker::getPowerModeInterval() {
    switch (currentPowerMode) {
        case POWER_FULL:  return 1000;   // 1 second - maximum responsiveness
        case POWER_ECO:   return 3000;   // 3 seconds - balanced performance
        case POWER_ULTRA: return 10000;  // 10 seconds - minimal power consumption
        default:          return 1000;   // Default to full power
    }
}

inline const char* HTITTracker::getPowerModeString() {
    switch (currentPowerMode) {
        case POWER_FULL:  return "Full Power";
        case POWER_ECO:   return "Eco Mode";  
        case POWER_ULTRA: return "Ultra Saver";
        default:          return "Unknown";
    }
}

inline void HTITTracker::updateLCD(int pct_cal) {
    // Smart Power Management: Only update screen based on power mode interval
    if (!shouldUpdateScreen()) {
        return;  // Skip screen update to save power
    }
    
    // Reset screen state when switching screens
    static ScreenType lastDisplayedScreen = SCREEN_MAIN_MENU;
    if (currentScreen != lastDisplayedScreen) {
        prevDisplayValid = false;  // Force redraw when switching screens
        forceScreenRedraw = true;  // Force all static screen variables to reset
        lastDisplayedScreen = currentScreen;
    }
    
    switch (currentScreen) {
        case SCREEN_STATUS:
            updateStatusScreen(pct_cal);
            break;
        case SCREEN_NAVIGATION:
            updateNavigationScreen(pct_cal);
            break;
        case SCREEN_MAIN_MENU:
            updateMainMenuScreen();
            break;
        case SCREEN_WAYPOINT_MENU:
            updateWaypointMenuScreen();
            break;
        case SCREEN_WAYPOINT1_NAV:
        case SCREEN_WAYPOINT2_NAV:
        case SCREEN_WAYPOINT3_NAV:
            updateWaypointNavigationScreen(pct_cal);
            break;
        case SCREEN_SET_WAYPOINT:
            updateSetWaypointScreen();
            break;
        case SCREEN_WAYPOINT_RESET:
            updateWaypointResetScreen();
            break;
        case SCREEN_SYSTEM_INFO:
            updateSystemInfoScreen(pct_cal);
            break;
        case SCREEN_GPS_STATUS:
            updateGPSStatusScreen(pct_cal);
            break;
        case SCREEN_POWER_MENU:
            updatePowerMenuScreen();
            break;
        default:
            updateStatusScreen(pct_cal);
            break;
    }
}

inline void HTITTracker::updateStatusScreen(int pct_cal) {
    // Status Screen: Fix, Satellites, Battery, Accuracy
    
    // 1) Generate new strings
    char fixBuf[16];
    if (haveFix) {
        sprintf(fixBuf, "Fix: Yes     ");
    } else {
        sprintf(fixBuf, "Fix: No      ");
    }
    
    char satBuf[16];
    if (totalInView >= 12) {
        sprintf(satBuf, "Sats:%3d 100%%", totalInView);  // Excellent signal (100%)
    } else if (totalInView >= 8) {
        sprintf(satBuf, "Sats:%3d 75%% ", totalInView);  // Good signal (75%)
    } else if (totalInView >= 4) {
        sprintf(satBuf, "Sats:%3d 50%% ", totalInView);  // Basic signal (50%)
    } else if (totalInView >= 1) {
        sprintf(satBuf, "Sats:%3d 25%% ", totalInView);  // Poor signal (25%)
    } else {
        sprintf(satBuf, "Sats:%3d 0%%  ", totalInView);  // No signal (0%)
    }
    
    char battBuf[16];
    sprintf(battBuf, "Batt:%3d%%    ", pct_cal);  // Consistent with GitHub - no charging indicator
    
    float accuracy = lastHDOP * 5.0f;  // HDOP × 5 m
    char accBuf[16];
    if (haveFix && lastHDOP > 0.0f && lastHDOP < 100.0f) {
        if (accuracy < 10.0) {
            sprintf(accBuf, "Acc:%4.1fm   ", accuracy);  // Show decimal for high accuracy
        } else {
            sprintf(accBuf, "Acc:%3.0fm    ", accuracy);  // Integer for lower accuracy
        }
    } else {
        sprintf(accBuf, "Acc: --.-m   ");
    }
    
    // 2) Only update changed rows to prevent flicker
    bool needsFullRedraw = !prevDisplayValid;
    
    if (needsFullRedraw) {
        st7735.fillScreen(ST7735_BLACK);
    }
    
    if (needsFullRedraw || strcmp(fixBuf, prevFixBuf) != 0) {
        st7735.setCursor(0, TEXT_ROW_0);
        st7735.print(fixBuf);
        strcpy(prevFixBuf, fixBuf);
    }
    
    if (needsFullRedraw || strcmp(satBuf, prevSatBuf) != 0) {
        st7735.setCursor(0, TEXT_ROW_1);
        st7735.print(satBuf);
        strcpy(prevSatBuf, satBuf);
    }
    
    if (needsFullRedraw || strcmp(battBuf, prevBattBuf) != 0) {
        st7735.setCursor(0, TEXT_ROW_2);
        st7735.print(battBuf);
        strcpy(prevBattBuf, battBuf);
    }
    
    if (needsFullRedraw || strcmp(accBuf, prevAccBuf) != 0) {
        st7735.setCursor(0, TEXT_ROW_3);
        st7735.print(accBuf);
        strcpy(prevAccBuf, accBuf);
    }
    
    prevDisplayValid = true;
}

inline void HTITTracker::updateNavigationScreen(int pct_cal) {
    // Navigation Screen: Direction to Home, Distance to Home, Current Speed
    
    // 1) Generate new strings
    char dirBuf[16];
    if (haveFix) {
        float bearingToHome = calculateBearingToHome();
        const char* direction = getCardinalDirection(bearingToHome);
        sprintf(dirBuf, "Dir: %s      ", direction);
    } else {
        sprintf(dirBuf, "Dir: O       ");
    }
    
    char distBuf[16];
    if (homeEstablished && hasValidPosition) {
        float distanceToHome = calculateDistanceToHome();
        if (distanceToHome < 100) {
            sprintf(distBuf, "Home:%4.1fm  ", distanceToHome);  // Show decimal for close distances
        } else if (distanceToHome < 1000) {
            sprintf(distBuf, "Home:%3.0fm   ", distanceToHome);  // Integer meters for medium distances
        } else if (distanceToHome < 10000) {
            sprintf(distBuf, "Home:%3.1fkm  ", distanceToHome / 1000.0);  // One decimal for km
        } else {
            sprintf(distBuf, "Home:%3.0fkm  ", distanceToHome / 1000.0);  // Integer km for long distances
        }
    } else {
        sprintf(distBuf, "Home: --.-m   ");
    }
    
    char speedBuf[20];  // Increased buffer size for higher speeds
    if (hasValidSpeed) {
        if (currentSpeed >= 100.0) {
            sprintf(speedBuf, "Spd:%3.0fkm/h ", currentSpeed);  // No decimal for 100+ km/h
        } else {
            sprintf(speedBuf, "Spd:%4.1fkm/h ", currentSpeed);  // One decimal for under 100
        }
    } else {
        sprintf(speedBuf, "Spd: -.-km/h ");
    }
    
    char battBuf[16];
    sprintf(battBuf, "Batt:%3d%%    ", pct_cal);
    
    // 2) Only update changed rows to prevent flicker
    bool needsFullRedraw = !prevDisplayValid;
    
    if (needsFullRedraw) {
        st7735.fillScreen(ST7735_BLACK);
    }
    
    if (needsFullRedraw || strcmp(dirBuf, prevFixBuf) != 0) {
        st7735.setCursor(0, TEXT_ROW_0);
        st7735.print(dirBuf);
        strcpy(prevFixBuf, dirBuf);
    }
    
    if (needsFullRedraw || strcmp(distBuf, prevDistBuf) != 0) {
        st7735.setCursor(0, TEXT_ROW_1);
        st7735.print(distBuf);
        strcpy(prevDistBuf, distBuf);
    }
    
    if (needsFullRedraw || strcmp(speedBuf, prevSpeedBuf) != 0) {
        st7735.setCursor(0, TEXT_ROW_2);
        st7735.print(speedBuf);
        strcpy(prevSpeedBuf, speedBuf);
    }
    
    if (needsFullRedraw || strcmp(battBuf, prevBattBuf) != 0) {
        st7735.setCursor(0, TEXT_ROW_3);
        st7735.print(battBuf);
        strcpy(prevBattBuf, battBuf);
    }
    
    prevDisplayValid = true;
}

inline const char* HTITTracker::getCardinalDirection(float bearingToHome) {
    if (!haveFix) {
        return "O";  // Show "O" when no GPS fix yet
    }
    
    if (!homeEstablished) {
        return "N";  // Point North when no home established but have fix
    }
    
    // Convert bearing to 8 cardinal directions
    if (bearingToHome >= 337.5 || bearingToHome < 22.5) {
        return "N";   // North
    } else if (bearingToHome >= 22.5 && bearingToHome < 67.5) {
        return "NE";  // Northeast
    } else if (bearingToHome >= 67.5 && bearingToHome < 112.5) {
        return "E";   // East
    } else if (bearingToHome >= 112.5 && bearingToHome < 157.5) {
        return "SE";  // Southeast
    } else if (bearingToHome >= 157.5 && bearingToHome < 202.5) {
        return "S";   // South
    } else if (bearingToHome >= 202.5 && bearingToHome < 247.5) {
        return "SW";  // Southwest
    } else if (bearingToHome >= 247.5 && bearingToHome < 292.5) {
        return "W";   // West
    } else {
        return "NW";  // Northwest
    }
}

// ========================== ENHANCED UI METHODS ==========================

inline void HTITTracker::updateMainMenuScreen() {
    static int lastMenuIndex = -1;
    static bool screenInitialized = false;
    
    // Reset if forced or first time
    if (forceScreenRedraw) {
        screenInitialized = false;
        lastMenuIndex = -1;
    }
    
    // Only redraw if menu selection changed or first time
    if (!screenInitialized || lastMenuIndex != menuIndex) {
        st7735.fillScreen(ST7735_BLACK);
        st7735.setCursor(0, TEXT_ROW_0);
        st7735.print("MENU");
        
        // Menu items with selection indicator (5 items total - compact spacing)
        String item0 = (menuIndex == 0) ? ">Status" : " Status";
        String item1 = (menuIndex == 1) ? ">Waypnt" : " Waypnt";
        String item2 = (menuIndex == 2) ? ">System" : " System";
        String item3 = (menuIndex == 3) ? ">GPS St" : " GPS St";
        String item4 = (menuIndex == 4) ? ">Power" : " Power";
        
        st7735.setCursor(0, TEXT_ROW_1);
        st7735.print(item0);
        st7735.setCursor(0, TEXT_ROW_2);
        st7735.print(item1);
        st7735.setCursor(0, TEXT_ROW_3);
        st7735.print(item2);
        st7735.setCursor(0, TEXT_ROW_4);
        st7735.print(item3);
        st7735.setCursor(0, TEXT_ROW_5);
        st7735.print(item4);
        
        lastMenuIndex = menuIndex;
        screenInitialized = true;
        forceScreenRedraw = false;  // Clear the force flag
    }
}

inline void HTITTracker::updateWaypointMenuScreen() {
    static int lastMenuIndex = -1;
    static bool screenInitialized = false;
    static bool lastWaypointStates[3] = {false, false, false};
    
    // Reset if forced
    if (forceScreenRedraw) {
        screenInitialized = false;
        lastMenuIndex = -1;
        for (int i = 0; i < 3; i++) lastWaypointStates[i] = !waypoints[i].isSet;  // Force state change
    }
    
    // Check if waypoint states changed
    bool waypointStatesChanged = false;
    for (int i = 0; i < 3; i++) {
        if (lastWaypointStates[i] != waypoints[i].isSet) {
            waypointStatesChanged = true;
            lastWaypointStates[i] = waypoints[i].isSet;
        }
    }
    
    // Only redraw if menu selection changed, waypoint states changed, or first time
    if (!screenInitialized || lastMenuIndex != menuIndex || waypointStatesChanged) {
        st7735.fillScreen(ST7735_BLACK);
        st7735.setCursor(0, ); st7735.print();
        
        // Show waypoint status with X for unset waypoints - compact text
        String item0, item1, item2, item3;
        
        if (waypoints[0].isSet) {
            item0 = (menuIndex == 0) ? ">Nav W1" : " Nav W1";
        } else {
            item0 = (menuIndex == 0) ? ">Set W1" : " Set W1";
        }
        
        if (waypoints[1].isSet) {
            item1 = (menuIndex == 1) ? ">Nav W2" : " Nav W2";
        } else {
            item1 = (menuIndex == 1) ? ">Set W2" : " Set W2";
        }
        
        if (waypoints[2].isSet) {
            item2 = (menuIndex == 2) ? ">Nav W3" : " Nav W3";
        } else {
            item2 = (menuIndex == 2) ? ">Set W3" : " Set W3";
        }
        
        item3 = (menuIndex == 3) ? ">Back" : " Back";
        
        st7735.setCursor(0, ); st7735.print();
        st7735.setCursor(0, ); st7735.print();
        st7735.setCursor(0, ); st7735.print();
        st7735.setCursor(0, ); st7735.print();
        
        lastMenuIndex = menuIndex;
        screenInitialized = true;
        forceScreenRedraw = false;  // Clear the force flag
    }
}

inline void HTITTracker::updateWaypointNavigationScreen(int pct_cal) {
    // 1) Check if we have a valid waypoint selected
    if (activeWaypoint == 0) {
        // No waypoint selected - switch to waypoint set screen
        currentScreen = SCREEN_SET_WAYPOINT;
        forceScreenRedraw = true;
        return;
    }
    
    // Get the current waypoint index (activeWaypoint is 1-based, array is 0-based)
    int waypointIndex = activeWaypoint - 1;
    
    // 2) Use the PROVEN working approach from GitHub - simple single writes
    
    // 1) Generate new strings EVERY time (for real-time updates) - using EXACT GitHub format
    char dirBuf[16];
    if (haveFix && waypointIndex >= 0 && waypointIndex < 3 && waypoints[waypointIndex].isSet) {
        float bearingToWaypoint = calculateBearingToWaypoint(waypointIndex);
        
        // Use EXACT GitHub cardinal direction logic (but for waypoints)
        const char* direction;
        if (bearingToWaypoint >= 337.5 || bearingToWaypoint < 22.5) {
            direction = "N";   // North
        } else if (bearingToWaypoint >= 22.5 && bearingToWaypoint < 67.5) {
            direction = "NE";  // Northeast
        } else if (bearingToWaypoint >= 67.5 && bearingToWaypoint < 112.5) {
            direction = "E";   // East
        } else if (bearingToWaypoint >= 112.5 && bearingToWaypoint < 157.5) {
            direction = "SE";  // Southeast
        } else if (bearingToWaypoint >= 157.5 && bearingToWaypoint < 202.5) {
            direction = "S";   // South
        } else if (bearingToWaypoint >= 202.5 && bearingToWaypoint < 247.5) {
            direction = "SW";  // Southwest
        } else if (bearingToWaypoint >= 247.5 && bearingToWaypoint < 292.5) {
            direction = "W";   // West
        } else {
            direction = "NW";  // Northwest
        }
        
        sprintf(dirBuf, "Dir: %s      ", direction);  // EXACT GitHub format
    } else {
        sprintf(dirBuf, "Dir: O       ");  // "O" when no fix - EXACT GitHub format
    }
    
    char distBuf[16];
    if (hasValidPosition && waypointIndex >= 0 && waypointIndex < 3 && waypoints[waypointIndex].isSet) {
        float distanceToWaypoint = calculateDistanceToWaypoint(waypointIndex);
        if (distanceToWaypoint < 100) {
            sprintf(distBuf, "WP%d:%4.1fm  ", activeWaypoint, distanceToWaypoint);  // Show decimal for close distances
        } else if (distanceToWaypoint < 1000) {
            sprintf(distBuf, "WP%d:%3.0fm   ", activeWaypoint, distanceToWaypoint);  // Integer meters for medium distances
        } else if (distanceToWaypoint < 10000) {
            sprintf(distBuf, "WP%d:%3.1fkm  ", activeWaypoint, distanceToWaypoint / 1000.0);  // One decimal for km
        } else {
            sprintf(distBuf, "WP%d:%3.0fkm  ", activeWaypoint, distanceToWaypoint / 1000.0);  // Integer km for long distances
        }
    } else {
        sprintf(distBuf, "WP%d: --.-m   ", activeWaypoint);  // EXACT GitHub spacing
    }
    
    char speedBuf[20];  // Increased buffer size for higher speeds
    if (hasValidSpeed) {
        if (currentSpeed >= 100.0) {
            sprintf(speedBuf, "Spd:%3.0fkm/h ", currentSpeed);  // No decimal for 100+ km/h (e.g., "Spd:120km/h")
        } else {
            sprintf(speedBuf, "Spd:%4.1fkm/h ", currentSpeed);  // One decimal for under 100 (e.g., "Spd:85.3km/h")
        }
    } else {
        sprintf(speedBuf, "Spd: -.-km/h ");  // No valid speed data
    }
    
    char battBuf[16];
    sprintf(battBuf, "Batt:%3d%%    ", pct_cal);  // No charging indicator - EXACT GitHub format
    
    // 3) Use EXACT GitHub display method - single writes with String() conversion
    static bool needsFullRedraw = true;
    
    if (needsFullRedraw) {
        st7735.fillScreen(ST7735_BLACK);
        needsFullRedraw = false;
    }
    
    // Compact approach - single write per line with String() conversion
    // Using small white text with tight spacing for 5-row display
    st7735.st7735_write_str(0, TEXT_ROW_0, String(dirBuf));
    st7735.st7735_write_str(0, TEXT_ROW_1, String(distBuf));  
    st7735.st7735_write_str(0, TEXT_ROW_2, String(speedBuf));
    st7735.st7735_write_str(0, TEXT_ROW_3, String(battBuf));
}

inline void HTITTracker::updateWaypointResetScreen() {
    static int lastMenuIndex = -1;
    static bool screenInitialized = false;
    static int lastWaypointToReset = -1;
    
    if (!screenInitialized || forceScreenRedraw || lastMenuIndex != menuIndex || lastWaypointToReset != waypointToReset) {
        st7735.fillScreen(ST7735_BLACK);
        
        // Show which waypoint we're working with
        char header[16];
        snprintf(header, sizeof(header), "WAYPT %d", waypointToReset);
        st7735.setCursor(0, ); st7735.print();
        
        // Show waypoint name if available - compact
        const char* waypointName = "";
        if (waypointToReset == 1 && strlen(waypoints[0].name) > 0) {
            waypointName = waypoints[0].name;
        } else if (waypointToReset == 2 && strlen(waypoints[1].name) > 0) {
            waypointName = waypoints[1].name;
        } else if (waypointToReset == 3 && strlen(waypoints[2].name) > 0) {
            waypointName = waypoints[2].name;
        }
        
        if (strlen(waypointName) > 0) {
            st7735.setCursor(0, ); st7735.print();
        }
        
        // Compact menu options
        const char* item0 = (menuIndex == 0) ? ">Nav" : " Nav";
        const char* item1 = (menuIndex == 1) ? ">Reset" : " Reset";
        const char* item2 = (menuIndex == 2) ? ">Cancel" : " Cancel";
        
        st7735.setCursor(0, ); st7735.print();
        st7735.setCursor(0, ); st7735.print();
        st7735.setCursor(0, ); st7735.print();
        
        lastMenuIndex = menuIndex;
        lastWaypointToReset = waypointToReset;
        screenInitialized = true;
        forceScreenRedraw = false;
    }
}

inline void HTITTracker::updateSetWaypointScreen() {
    static bool screenInitialized = false;
    static bool lastGPSReady = false;
    static int lastSatCount = -1;
    
    if (forceScreenRedraw) {
        screenInitialized = false;
        lastGPSReady = !hasValidPosition;  // Force change
        lastSatCount = -1;
    }
    
    bool gpsReady = (hasValidPosition && haveFix);
    bool needsRedraw = !screenInitialized || (gpsReady != lastGPSReady) || (totalInView != lastSatCount);
    
    if (needsRedraw) {
        st7735.fillScreen(ST7735_BLACK);
        
        char title[20];
        snprintf(title, sizeof(title), "SET WP%d", waypointToSet + 1);
        st7735.st7735_write_str(0, TEXT_ROW_0, String(title));
        
        if (gpsReady) {
            st7735.setCursor(0, ); st7735.print();
            st7735.setCursor(0, ); st7735.print();
        } else {
            st7735.setCursor(0, ); st7735.print();
            st7735.st7735_write_str(0, TEXT_ROW_2, String("Sats: " + String(totalInView)));
        }
        
        lastGPSReady = gpsReady;
        lastSatCount = totalInView;
        screenInitialized = true;
        forceScreenRedraw = false;
    }
}

inline void HTITTracker::updateSystemInfoScreen(int pct_cal) {
    static bool screenInitialized = false;
    static int lastSatCount = -1;
    static int lastBattPercent = -1;
    static PowerMode lastMode = POWER_FULL;
    
    bool needsRedraw = !screenInitialized || (totalInView != lastSatCount) || 
                       (pct_cal != lastBattPercent) || (currentPowerMode != lastMode);
    
    if (needsRedraw) {
        st7735.fillScreen(ST7735_BLACK);
        st7735.setCursor(0, ); st7735.print();
        
        // Compact satellite display with constellation breakdown
        String satText = "Sats:" + String(totalInView);
        if (totalInView > 0) {
            satText += " G:" + String(gpsCount) + " R:" + String(glonassCount);
        }
        st7735.setCursor(0, ); st7735.print();
        
        // Compact battery display with charging indicator
        String battText = "Batt:" + String(pct_cal) + "%";
        if (isCharging) {
            battText += " CHG";  // Shorter charging indicator
        }
        st7735.setCursor(0, ); st7735.print();
        
        // Show current power mode and motion state - compact
        String modeText = getPowerModeString();
        if (currentMotionState == MOTION_STATIONARY) modeText += " STAT";
        else if (currentMotionState == MOTION_MOVING) modeText += " MOV";
        st7735.setCursor(0, ); st7735.print();
        
        // Show GPS update interval for debugging - compact
        st7735.st7735_write_str(0, TEXT_ROW_4, String("GPS:" + String(gpsUpdateInterval/1000) + "s"));
        
        lastSatCount = totalInView;
        lastBattPercent = pct_cal;
        lastMode = currentPowerMode;
        screenInitialized = true;
    }
}

inline void HTITTracker::updateGPSStatusScreen(int pct_cal) {
    static bool screenInitialized = false;
    static int lastTotalSats = -1;
    static bool lastFixStatus = false;
    
    bool needsRedraw = !screenInitialized || (totalInView != lastTotalSats) || (haveFix != lastFixStatus);
    
    if (needsRedraw) {
        st7735.fillScreen(ST7735_BLACK);
        st7735.setCursor(0, ); st7735.print();
        
        // Show overall fix status and total satellites
        String fixLine = (haveFix ? "Fix:YES " : "Fix:NO ") + String("Tot:") + String(totalInView);
        st7735.setCursor(0, ); st7735.print();
        
        // Show constellation counts in two compact lines
        String line1 = "GPS:" + String(gpsCount) + " GLO:" + String(glonassCount);
        String line2 = "BDS:" + String(beidouCount) + " GAL:" + String(galileoCount);
        
        st7735.setCursor(0, ); st7735.print();
        st7735.setCursor(0, ); st7735.print();
        
        // Show primary constellation (most satellites contributing)
        int primaryConstellation = -1;
        int maxActiveSats = 0;
        const char* constNames[] = {"GPS", "GLO", "BDS", "GAL", "QZS"};
        
        for (int i = 0; i < 5; i++) {
            if (constellations[i].isActiveForFix && constellations[i].count > maxActiveSats) {
                maxActiveSats = constellations[i].count;
                primaryConstellation = i;
            }
        }
        
        String primaryText = "Primary:";
        if (primaryConstellation >= 0) {
            primaryText += String(constNames[primaryConstellation]);
        } else {
            primaryText += "None";
        }
        st7735.setCursor(0, ); st7735.print();
        
        lastTotalSats = totalInView;
        lastFixStatus = haveFix;
        screenInitialized = true;
    }
}

inline void HTITTracker::updatePowerMenuScreen() {
    static int lastMenuIndex = -1;
    static bool screenInitialized = false;
    static PowerMode lastPowerMode = POWER_FULL;
    
    bool needsRedraw = !screenInitialized || forceScreenRedraw || 
                      lastMenuIndex != menuIndex || lastPowerMode != currentPowerMode;
    
    if (needsRedraw) {
        st7735.fillScreen(ST7735_BLACK);
        st7735.setCursor(0, ); st7735.print();
        
        // Compact power menu options with current mode indicator
        String item0 = (menuIndex == 0) ? ">" : " ";
        item0 += "Full";
        if (currentPowerMode == POWER_FULL) item0 += "*";
        
        String item1 = (menuIndex == 1) ? ">" : " ";
        item1 += "Eco";
        if (currentPowerMode == POWER_ECO) item1 += "*";
        
        String item2 = (menuIndex == 2) ? ">" : " ";
        item2 += "Ultra";
        if (currentPowerMode == POWER_ULTRA) item2 += "*";
        
        const char* item3 = (menuIndex == 3) ? ">Back" : " Back";
        
        st7735.setCursor(0, ); st7735.print();
        st7735.setCursor(0, ); st7735.print();
        st7735.setCursor(0, ); st7735.print();
        st7735.setCursor(0, ); st7735.print();
        
        lastMenuIndex = menuIndex;
        lastPowerMode = currentPowerMode;
        screenInitialized = true;
        forceScreenRedraw = false;
    }
}

// ========================== WAYPOINT MANAGEMENT ==========================

inline void HTITTracker::setWaypoint(int index, double lat, double lon, const char* name) {
    if (index >= 0 && index < 3) {
        waypoints[index].lat = lat;
        waypoints[index].lon = lon;
        waypoints[index].isSet = true;
        strncpy(waypoints[index].name, name, 11);
        waypoints[index].name[11] = '\0';
        saveWaypointsToEEPROM();
    }
}

inline void HTITTracker::loadWaypointsFromEEPROM() {
    // Check magic number
    uint32_t magic;
    EEPROM.get(ADDR_MAGIC, magic);
    if (magic != EEPROM_MAGIC) {
        // First time setup - initialize with defaults
        for (int i = 0; i < 3; i++) {
            waypoints[i].isSet = false;
            waypoints[i].lat = 0.0;
            waypoints[i].lon = 0.0;
            strcpy(waypoints[i].name, "");
        }
        saveWaypointsToEEPROM();
        Serial.println("→ EEPROM initialized with defaults");
        return;
    }
    
    // Load waypoints from EEPROM
    for (int i = 0; i < 3; i++) {
        int baseAddr = ADDR_WAYPOINT1_LAT + i * 20; // 20 bytes per waypoint
        EEPROM.get(baseAddr, waypoints[i].lat);
        EEPROM.get(baseAddr + 8, waypoints[i].lon);
        EEPROM.get(ADDR_WAYPOINT1_SET + i, waypoints[i].isSet);
        snprintf(waypoints[i].name, sizeof(waypoints[i].name), "WP%d", i + 1);
    }
    Serial.println("→ Waypoints loaded from EEPROM");
}

inline void HTITTracker::saveWaypointsToEEPROM() {
    // Save magic number
    uint32_t magic = EEPROM_MAGIC;
    EEPROM.put(ADDR_MAGIC, magic);
    
    // Save waypoints
    for (int i = 0; i < 3; i++) {
        int baseAddr = ADDR_WAYPOINT1_LAT + i * 20; // 20 bytes per waypoint
        EEPROM.put(baseAddr, waypoints[i].lat);
        EEPROM.put(baseAddr + 8, waypoints[i].lon);
        EEPROM.put(ADDR_WAYPOINT1_SET + i, waypoints[i].isSet);
    }
    EEPROM.commit();
    Serial.println("→ Waypoints saved to EEPROM");
}

inline float HTITTracker::calculateBearingToWaypoint(int waypointIndex) {
    if (waypointIndex < 0 || waypointIndex >= 3 || !waypoints[waypointIndex].isSet || !hasValidPosition) {
        return 0.0f;  // Default to North
    }
    
    // Calculate bearing from current position to waypoint
    double lat1 = currentLat * PI / 180.0;
    double lon1 = currentLon * PI / 180.0;
    double lat2 = waypoints[waypointIndex].lat * PI / 180.0;
    double lon2 = waypoints[waypointIndex].lon * PI / 180.0;
    
    double dLon = lon2 - lon1;
    
    double y = sin(dLon) * cos(lat2);
    double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);
    
    double bearing = atan2(y, x) * 180.0 / PI;
    bearing = fmod(bearing + 360.0, 360.0);  // Normalize to 0-360
    
    return bearing;
}

inline float HTITTracker::calculateDistanceToWaypoint(int waypointIndex) {
    if (waypointIndex < 0 || waypointIndex >= 3 || !waypoints[waypointIndex].isSet || !hasValidPosition) {
        return 0.0f;  // No distance if waypoint not set or no position
    }
    
    // Calculate distance using Haversine formula
    double lat1 = currentLat * PI / 180.0;
    double lon1 = currentLon * PI / 180.0;
    double lat2 = waypoints[waypointIndex].lat * PI / 180.0;
    double lon2 = waypoints[waypointIndex].lon * PI / 180.0;
    
    double dLat = lat2 - lat1;
    double dLon = lon2 - lon1;
    
    double a = sin(dLat/2) * sin(dLat/2) + 
               cos(lat1) * cos(lat2) * 
               sin(dLon/2) * sin(dLon/2);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    
    // Earth's radius in meters
    const double EARTH_RADIUS = 6371000.0;
    
    return EARTH_RADIUS * c;  // Distance in meters
}

// ========================== ENHANCED GPS PERFORMANCE ==========================

inline void HTITTracker::updateConstellationInfo() {
    // Update constellation statistics
    constellations[0].count = gpsCount;      // GPS (USA)
    constellations[1].count = glonassCount;  // GLONASS (Russia)
    constellations[2].count = beidouCount;   // Beidou (China)
    constellations[3].count = galileoCount;  // Galileo (EU)
    constellations[4].count = qzssCount;     // QZSS (Japan)
    
    // Check which constellations have fixes and are contributing to the solution
    for (int i = 0; i < 5; i++) {
        constellations[i].hasFix = (constellations[i].count > 0 && haveFix);
        // For now, assume all constellations with satellites are contributing if we have a fix
        // In a real implementation, this would come from GGA or RMC sentence analysis
        constellations[i].isActiveForFix = (constellations[i].count >= 4 && haveFix);
    }
}

inline void HTITTracker::analyzeMotionState() {
    if (!hasValidPosition || !hasValidSpeed) {
        currentMotionState = MOTION_UNKNOWN;
        return;
    }
    
    // Analyze motion based on speed
    if (currentSpeed < 0.5f) {
        currentMotionState = MOTION_STATIONARY;
    } else if (currentSpeed < 5.0f) {
        currentMotionState = MOTION_SLOW;
    } else if (currentSpeed < 50.0f) {
        currentMotionState = MOTION_MOVING;
    } else {
        currentMotionState = MOTION_FAST;
    }
}

inline void HTITTracker::adaptGPSUpdateRate() {
    unsigned long newInterval;
    
    // Adapt GPS update rate based on motion and power mode
    switch (currentMotionState) {
        case MOTION_STATIONARY:
            newInterval = 5000;  // 5 seconds when stationary
            break;
        case MOTION_SLOW:
            newInterval = 2000;  // 2 seconds when moving slowly
            break;
        case MOTION_MOVING:
        case MOTION_FAST:
            newInterval = 1000;  // 1 second when moving fast
            break;
        default:
            newInterval = 1000;  // Default 1 second
            break;
    }
    
    // Factor in power mode
    switch (currentPowerMode) {
        case POWER_ECO:
            newInterval *= 2;  // Double the interval in eco mode
            break;
        case POWER_ULTRA:
            newInterval *= 3;  // Triple the interval in ultra mode
            break;
        default:
            break;
    }
    
    gpsUpdateInterval = newInterval;
}

inline void HTITTracker::storeLastPosition() {
    if (hasValidPosition) {
        EEPROM.put(ADDR_LAST_LAT, currentLat);
        EEPROM.put(ADDR_LAST_LON, currentLon);
        EEPROM.put(ADDR_LAST_VALID, true);
        EEPROM.commit();
        lastStoredLat = currentLat;
        lastStoredLon = currentLon;
        hasStoredPosition = true;
        Serial.println("→ Position stored for GPS warm start");
    }
}

inline void HTITTracker::loadLastPosition() {
    bool isValid;
    EEPROM.get(ADDR_LAST_VALID, isValid);
    
    if (isValid) {
        EEPROM.get(ADDR_LAST_LAT, lastStoredLat);
        EEPROM.get(ADDR_LAST_LON, lastStoredLon);
        hasStoredPosition = true;
        Serial.println("→ Last position loaded for GPS warm start");
    } else {
        hasStoredPosition = false;
        Serial.println("→ No previous position found");
    }
}

inline bool HTITTracker::isPositionStationary() {
    return (currentMotionState == MOTION_STATIONARY);
}

// ========================== UI/UX ENHANCEMENTS ==========================

inline void HTITTracker::drawProgressBar(int x, int y, int width, int height, int percentage, uint16_t color) {
    // Create text-based progress bar since HT_st7735 doesn't support graphics
    char progressBar[21] = {0}; // 20 characters + null terminator
    int barLength = 20;
    int fillChars = (barLength * percentage) / 100;
    
    progressBar[0] = '[';
    for (int i = 1; i <= barLength; i++) {
        if (i <= fillChars) {
            progressBar[i] = '=';
        } else {
            progressBar[i] = ' ';
        }
    }
    progressBar[barLength + 1] = ']';
    progressBar[barLength + 2] = '\0';
    
    st7735.st7735_write_str(x, y, progressBar);
}

inline void HTITTracker::drawBatteryIcon(int x, int y, int percentage) {
    // Create text-based battery indicator with percentage
    char battIcon[12];
    if (percentage > 75) {
        strcpy(battIcon, "BATT 100%");
    } else if (percentage > 50) {
        strcpy(battIcon, "BATT 75%");
    } else if (percentage > 25) {
        strcpy(battIcon, "BATT 50%");
    } else {
        strcpy(battIcon, "BATT 25%");
    }
    
    st7735.st7735_write_str(x, y, battIcon);
}

inline void HTITTracker::drawGPSIcon(int x, int y, int signalStrength) {
    // Create text-based GPS signal indicator with percentage
    char gpsIcon[12];
    if (signalStrength >= 12) {
        strcpy(gpsIcon, "GPS 100%");
    } else if (signalStrength >= 8) {
        strcpy(gpsIcon, "GPS 75%");
    } else if (signalStrength >= 4) {
        strcpy(gpsIcon, "GPS 50%");
    } else if (signalStrength >= 1) {
        strcpy(gpsIcon, "GPS 25%");
    } else {
        strcpy(gpsIcon, "GPS 0%");
    }
    
    st7735.st7735_write_str(x, y, gpsIcon);
}

inline void HTITTracker::drawPowerModeIcon(int x, int y, PowerMode mode) {
    // Create text-based power mode indicator
    const char* modeStr;
    switch (mode) {
        case POWER_FULL:  modeStr = "PWR:FULL"; break;
        case POWER_ECO:   modeStr = "PWR:ECO"; break;
        case POWER_ULTRA: modeStr = "PWR:ULTRA"; break;
        default:          modeStr = "PWR:???"; break;
    }
    
    st7735.st7735_write_str(x, y, modeStr);
}

inline uint16_t HTITTracker::getStatusColor(int value, int goodThreshold, int okThreshold) {
    if (value >= goodThreshold) {
        return ST7735_GREEN;
    } else if (value >= okThreshold) {
        return ST7735_YELLOW;
    } else {
        return ST7735_RED;
    }
}

#endif // MAIN_H
