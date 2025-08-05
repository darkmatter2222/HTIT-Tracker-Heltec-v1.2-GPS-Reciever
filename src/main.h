#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>
#include <HT_st7735.h>
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
#define LED_PIN      35   // GPIO 35 for white LED indicator (built-in LED on Heltec, active-high)

// EEPROM ADDRESSES
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
    SCREEN_POWER_MENU,
    SCREEN_WAYPOINT_RESET,     // Ask to reset waypoint or navigate
    SCREEN_COUNT
};

// POWER MODES
enum PowerMode {
    POWER_FULL = 0,
    POWER_ECO,
    POWER_SLEEP
};

// WAYPOINT STRUCTURE
struct Waypoint {
    double lat;
    double lon;
    bool isSet;
    char name[12];
};

class HTITTracker {
private:
    // Display instance
    HT_st7735 st7735;
    
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
    
    // LED blink indicator variables
    bool ledBlinkActive;
    unsigned long ledBlinkStartTime;
    static const unsigned long LED_BLINK_DURATION = 50; // 50ms blink duration
    
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
    
    // Enhanced UI screen methods
    void updateMainMenuScreen();
    void updateWaypointMenuScreen();
    void updateWaypointNavigationScreen(int pct_cal);
    void updateSetWaypointScreen();
    void updateSystemInfoScreen(int pct_cal);
    void updatePowerMenuScreen();
    void updateWaypointResetScreen();
    
    void checkButton();
    void calculateSpeed();
    int getStableBatteryPercent(float voltage);
    void updateChargingStatus(float voltage);
    void triggerLEDBlink();
    void updateLEDState();
    
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
      ledBlinkActive(false), ledBlinkStartTime(0) {
    
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
    Serial.println("HTIT-Tracker v1.2: 5-Row Display with Home Navigation");

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

    // 5.5) Configure LED indicator
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);  // Start with LED off
    Serial.println("→ LED_PIN (GPIO 35) configured as output");

    // 6) Set ADC attenuation so VBAT/2 (≈0.857–1.07 V) reads accurately
    analogSetPinAttenuation(VBAT_PIN, ADC_11db);

    // 7) Initialize Serial1 @115200 to read UC6580 NMEA
    Serial1.begin(115200, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    Serial.println("→ Serial1.begin(115200, RX=33, TX=34) for UC6580");

    // 8) Initialize ST7735 display
    st7735.st7735_init();
    st7735.st7735_fill_screen(ST7735_BLACK);
    
    // 9) Initialize EEPROM and load waypoints
    EEPROM.begin(512);  // Initialize EEPROM with 512 bytes
    Serial.println("→ EEPROM initialized (512 bytes)");
    loadWaypointsFromEEPROM();
}

inline void HTITTracker::update() {
    // A) Check button for screen switching
    checkButton();
    
    // A.5) Update LED state
    updateLEDState();
    
    // B) Read raw NMEA from Serial1, echo to USB-Serial, accumulate lines
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

    // C) Once per second, update display
    unsigned long now = millis();
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
            
            // Trigger LED blink when we have a fix and get position data
            if (haveFix) {
                triggerLEDBlink();
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
        }
    }
    
    // Detect long press (1000ms threshold)
    if (!currentButtonState && !longPressHandled && 
        buttonPressStart > 0 && (millis() - buttonPressStart > 1000)) {
        
        longPressHandled = true;
        lastActivity = millis();
        
        // Long press actions - scroll through menu or return to main menu
        if (currentScreen == SCREEN_MAIN_MENU) {
            menuIndex = (menuIndex + 1) % 4;  // 4 items in main menu now (removed Navigation)
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
                // Handle main menu selection (now has 4 items - removed Navigation)
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
                } else if (menuIndex == 3) {  // Power Menu
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
                // Handle power menu actions
                if (menuIndex == 0) {  // Sleep Mode (light sleep with quick wake)
                    st7735.st7735_fill_screen(ST7735_BLACK);
                    st7735.st7735_write_str(0, 0, "ENTERING SLEEP");
                    st7735.st7735_write_str(0, 16, "Press to wake");
                    delay(1000);
                    
                    // Enter light sleep - wakes on button press
                    esp_sleep_enable_ext0_wakeup(GPIO_NUM_0, 0);  // Wake on button press (LOW)
                    esp_light_sleep_start();
                    
                    // When we wake up, return to main menu
                    currentScreen = SCREEN_MAIN_MENU;
                    menuIndex = 0;
                    Serial.println("→ Woke from sleep, returning to main menu");
                    
                } else if (menuIndex == 1) {  // Deep Sleep (full power down)
                    st7735.st7735_fill_screen(ST7735_BLACK);
                    st7735.st7735_write_str(0, 0, "DEEP SLEEP");
                    st7735.st7735_write_str(0, 16, "Hold button");
                    st7735.st7735_write_str(0, 32, "to wake up");
                    delay(2000);
                    
                    // Enter deep sleep - only wakes on button press
                    esp_sleep_enable_ext0_wakeup(GPIO_NUM_0, 0);  // Wake on button press (LOW)
                    esp_deep_sleep_start();
                    // Device will restart when woken
                    
                } else if (menuIndex == 2) {  // Screen Off (turn off display)
                    st7735.st7735_fill_screen(ST7735_BLACK);
                    currentScreen = SCREEN_STATUS;  // Go to status when reactivated
                    Serial.println("→ Screen off mode activated");
                    
                } else if (menuIndex == 3) {  // Back
                    currentScreen = SCREEN_MAIN_MENU;
                    menuIndex = 4;  // Return to Power Menu item in main menu
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

inline void HTITTracker::updateLCD(int pct_cal) {
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
    sprintf(satBuf, "Sats:%3d     ", totalInView);
    
    char battBuf[16];
    sprintf(battBuf, "Batt:%3d%%    ", pct_cal);  // Consistent with GitHub - no charging indicator
    
    float accuracy = lastHDOP * 5.0f;  // HDOP × 5 m
    char accBuf[16];
    if (haveFix && lastHDOP > 0.0f && lastHDOP < 100.0f) {
        sprintf(accBuf, "Acc:%4.1fm   ", accuracy);
    } else {
        sprintf(accBuf, "Acc: --.-m   ");
    }
    
    // 2) Only update changed rows to prevent flicker
    bool needsFullRedraw = !prevDisplayValid;
    
    if (needsFullRedraw) {
        st7735.st7735_fill_screen(ST7735_BLACK);
    }
    
    if (needsFullRedraw || strcmp(fixBuf, prevFixBuf) != 0) {
        st7735.st7735_write_str(0, 0, String(fixBuf));
        strcpy(prevFixBuf, fixBuf);
    }
    
    if (needsFullRedraw || strcmp(satBuf, prevSatBuf) != 0) {
        st7735.st7735_write_str(0, 16, String(satBuf));
        strcpy(prevSatBuf, satBuf);
    }
    
    if (needsFullRedraw || strcmp(battBuf, prevBattBuf) != 0) {
        st7735.st7735_write_str(0, 32, String(battBuf));
        strcpy(prevBattBuf, battBuf);
    }
    
    if (needsFullRedraw || strcmp(accBuf, prevAccBuf) != 0) {
        st7735.st7735_write_str(0, 48, String(accBuf));
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
        if (distanceToHome < 1000) {
            sprintf(distBuf, "Home:%3.0fm   ", distanceToHome);
        } else {
            sprintf(distBuf, "Home:%3.1fkm  ", distanceToHome / 1000.0);
        }
    } else {
        sprintf(distBuf, "Home: --.-m   ");
    }
    
    char speedBuf[16];
    if (hasValidSpeed && currentSpeed < 99.9) {
        sprintf(speedBuf, "Spd:%4.1fkm/h ", currentSpeed);
    } else {
        sprintf(speedBuf, "Spd: -.-km/h ");
    }
    
    char battBuf[16];
    sprintf(battBuf, "Batt:%3d%%    ", pct_cal);
    
    // 2) Only update changed rows to prevent flicker
    bool needsFullRedraw = !prevDisplayValid;
    
    if (needsFullRedraw) {
        st7735.st7735_fill_screen(ST7735_BLACK);
    }
    
    if (needsFullRedraw || strcmp(dirBuf, prevFixBuf) != 0) {
        st7735.st7735_write_str(0, 0, String(dirBuf));
        strcpy(prevFixBuf, dirBuf);
    }
    
    if (needsFullRedraw || strcmp(distBuf, prevDistBuf) != 0) {
        st7735.st7735_write_str(0, 16, String(distBuf));
        strcpy(prevDistBuf, distBuf);
    }
    
    if (needsFullRedraw || strcmp(speedBuf, prevSpeedBuf) != 0) {
        st7735.st7735_write_str(0, 32, String(speedBuf));
        strcpy(prevSpeedBuf, speedBuf);
    }
    
    if (needsFullRedraw || strcmp(battBuf, prevBattBuf) != 0) {
        st7735.st7735_write_str(0, 48, String(battBuf));
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
        st7735.st7735_fill_screen(ST7735_BLACK);
        st7735.st7735_write_str(0, 0, "MAIN MENU");
        
        // Menu items with selection indicator (4 items total - removed Navigation)
        String item0 = (menuIndex == 0) ? "> Status" : "  Status";
        String item1 = (menuIndex == 1) ? "> Waypoints" : "  Waypoints";
        String item2 = (menuIndex == 2) ? "> System Info" : "  System Info";
        String item3 = (menuIndex == 3) ? "> Power Menu" : "  Power Menu";
        
        st7735.st7735_write_str(0, 16, item0);
        st7735.st7735_write_str(0, 32, item1);
        st7735.st7735_write_str(0, 48, item2);
        st7735.st7735_write_str(0, 64, item3);
        
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
        st7735.st7735_fill_screen(ST7735_BLACK);
        st7735.st7735_write_str(0, 0, "WAYPOINTS");
        
        // Show waypoint status with X for unset waypoints
        String item0, item1, item2, item3;
        
        if (waypoints[0].isSet) {
            item0 = (menuIndex == 0) ? "> Nav WP1" : "  Nav WP1";
        } else {
            item0 = (menuIndex == 0) ? "> Set WP1 X" : "  Set WP1 X";
        }
        
        if (waypoints[1].isSet) {
            item1 = (menuIndex == 1) ? "> Nav WP2" : "  Nav WP2";
        } else {
            item1 = (menuIndex == 1) ? "> Set WP2 X" : "  Set WP2 X";
        }
        
        if (waypoints[2].isSet) {
            item2 = (menuIndex == 2) ? "> Nav WP3" : "  Nav WP3";
        } else {
            item2 = (menuIndex == 2) ? "> Set WP3 X" : "  Set WP3 X";
        }
        
        item3 = (menuIndex == 3) ? "> Back" : "  Back";
        
        st7735.st7735_write_str(0, 16, item0);
        st7735.st7735_write_str(0, 32, item1);
        st7735.st7735_write_str(0, 48, item2);
        st7735.st7735_write_str(0, 64, item3);
        
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
        if (distanceToWaypoint < 1000) {
            sprintf(distBuf, "WP%d:%3.0fm   ", activeWaypoint, distanceToWaypoint);  // EXACT GitHub format
        } else {
            sprintf(distBuf, "WP%d:%3.1fkm  ", activeWaypoint, distanceToWaypoint / 1000.0);
        }
    } else {
        sprintf(distBuf, "WP%d: --.-m   ", activeWaypoint);  // EXACT GitHub spacing
    }
    
    char speedBuf[16];
    if (hasValidSpeed && currentSpeed < 99.9) {
        sprintf(speedBuf, "Spd:%4.1fkm/h ", currentSpeed);  // EXACT GitHub format
    } else {
        sprintf(speedBuf, "Spd: -.-km/h ");  // EXACT GitHub spacing
    }
    
    char battBuf[16];
    sprintf(battBuf, "Batt:%3d%%    ", pct_cal);  // No charging indicator - EXACT GitHub format
    
    // 3) Use EXACT GitHub display method - single writes with String() conversion
    static bool needsFullRedraw = true;
    
    if (needsFullRedraw) {
        st7735.st7735_fill_screen(ST7735_BLACK);
        needsFullRedraw = false;
    }
    
    // EXACT GitHub approach - single write per line with String() conversion
    // Using default font and colors for clean, readable display
    st7735.st7735_write_str(0, 0, String(dirBuf));
    st7735.st7735_write_str(0, 16, String(distBuf));  
    st7735.st7735_write_str(0, 32, String(speedBuf));
    st7735.st7735_write_str(0, 48, String(battBuf));
}

inline void HTITTracker::updateWaypointResetScreen() {
    static int lastMenuIndex = -1;
    static bool screenInitialized = false;
    static int lastWaypointToReset = -1;
    
    if (!screenInitialized || forceScreenRedraw || lastMenuIndex != menuIndex || lastWaypointToReset != waypointToReset) {
        st7735.st7735_fill_screen(ST7735_BLACK);
        
        // Show which waypoint we're working with
        char header[20];
        snprintf(header, sizeof(header), "WAYPOINT %d", waypointToReset);
        st7735.st7735_write_str(0, 0, header);
        
        // Show waypoint name if available
        const char* waypointName = "";
        if (waypointToReset == 1 && strlen(waypoints[0].name) > 0) {
            waypointName = waypoints[0].name;
        } else if (waypointToReset == 2 && strlen(waypoints[1].name) > 0) {
            waypointName = waypoints[1].name;
        } else if (waypointToReset == 3 && strlen(waypoints[2].name) > 0) {
            waypointName = waypoints[2].name;
        }
        
        if (strlen(waypointName) > 0) {
            st7735.st7735_write_str(0, 16, waypointName);
        }
        
        // Menu options
        const char* item0 = (menuIndex == 0) ? "> Navigate" : "  Navigate";
        const char* item1 = (menuIndex == 1) ? "> Reset" : "  Reset";
        const char* item2 = (menuIndex == 2) ? "> Cancel" : "  Cancel";
        
        st7735.st7735_write_str(0, 32, item0);
        st7735.st7735_write_str(0, 48, item1);
        st7735.st7735_write_str(0, 64, item2);
        
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
        st7735.st7735_fill_screen(ST7735_BLACK);
        
        char title[20];
        snprintf(title, sizeof(title), "SET WP%d", waypointToSet + 1);
        st7735.st7735_write_str(0, 0, String(title));
        
        if (gpsReady) {
            st7735.st7735_write_str(0, 16, "GPS Ready!");
            st7735.st7735_write_str(0, 32, "Press to save");
        } else {
            st7735.st7735_write_str(0, 16, "Wait for GPS...");
            st7735.st7735_write_str(0, 32, String("Sats: " + String(totalInView)));
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
    
    bool needsRedraw = !screenInitialized || (totalInView != lastSatCount) || (pct_cal != lastBattPercent);
    
    if (needsRedraw) {
        st7735.st7735_fill_screen(ST7735_BLACK);
        st7735.st7735_write_str(0, 0, "SYSTEM INFO");
        st7735.st7735_write_str(0, 16, "FW: v1.2 Enh");
        st7735.st7735_write_str(0, 32, String("Sats: " + String(totalInView)));
        st7735.st7735_write_str(0, 48, String("Batt: " + String(pct_cal) + "%"));
        
        lastSatCount = totalInView;
        lastBattPercent = pct_cal;
        screenInitialized = true;
    }
}

inline void HTITTracker::updatePowerMenuScreen() {
    static int lastMenuIndex = -1;
    static bool screenInitialized = false;
    
    if (!screenInitialized || forceScreenRedraw || lastMenuIndex != menuIndex) {
        st7735.st7735_fill_screen(ST7735_BLACK);
        st7735.st7735_write_str(0, 0, "POWER MENU");
        
        // Power menu options
        const char* item0 = (menuIndex == 0) ? "> Sleep Mode" : "  Sleep Mode";
        const char* item1 = (menuIndex == 1) ? "> Deep Sleep" : "  Deep Sleep";
        const char* item2 = (menuIndex == 2) ? "> Screen Off" : "  Screen Off";
        const char* item3 = (menuIndex == 3) ? "> Back" : "  Back";
        
        st7735.st7735_write_str(0, 16, item0);
        st7735.st7735_write_str(0, 32, item1);
        st7735.st7735_write_str(0, 48, item2);
        st7735.st7735_write_str(0, 64, item3);
        
        lastMenuIndex = menuIndex;
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

// ========================== LED INDICATOR METHODS ==========================

inline void HTITTracker::triggerLEDBlink() {
    // Only trigger if not already blinking
    if (!ledBlinkActive) {
        ledBlinkActive = true;
        ledBlinkStartTime = millis();
        digitalWrite(LED_PIN, HIGH);  // Turn LED on
    }
}

inline void HTITTracker::updateLEDState() {
    // Check if LED blink is active and time has expired
    if (ledBlinkActive) {
        unsigned long now = millis();
        if (now - ledBlinkStartTime >= LED_BLINK_DURATION) {
            ledBlinkActive = false;
            digitalWrite(LED_PIN, LOW);  // Turn LED off
        }
    }
}

#endif // MAIN_H
