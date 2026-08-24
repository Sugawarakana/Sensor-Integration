#include <ACANFD_STM32.h>
#include <ACANFD_STM32_NUCLEO_G431KB-objects.h>
#include <ACANFD_STM32_CANFDMessage.h>
#include <Wire.h>
#include <MICS-VZ-89TE.h>
#include <ACAN2515.h>

// ======================= Settings =======================
static const byte MCP2515_CS  = PA8;   // D9 (default D10 used by fdcan1_rx)
static const byte MCP2515_INT = PB0;   // D3 (default D2 used by fdcan1_tx)
// SPI1: MOSI=PB5(D11), MISO=PB4(D12), SCK=PB3(D13) by default

static const uint32_t QUARTZ_FREQUENCY           = 16UL * 1000UL * 1000UL; // 16MHz
static const uint32_t CAN_BITRATE_TAMER          = 250UL * 1000UL; // 250kbps
static const uint32_t CAN_BITRATE_T3650_SGXBLD2  = 500UL * 1000UL; // 500kbps

static const int  ANALOG_IN_PIN = A0;             // ADC pin
static const unsigned long LOG_INTERVAL = 1000;   // ms

// CAN ID
static const uint32_t ID_SGX     = 0x256;         // SGX-BLD2, standard
static const uint32_t ID_TELAIRE = 0x18FF0CEB;    // T3650, J1939
static const uint32_t PGN_TAMER  = 0x00FF01;      // Li-ion Tamer, J1939

// ---- Onboard Temp Detection ----
#if !defined(ATEMP) || !defined(AVREF)
#error "Board part number error"
#endif

// STM32G4 factory VDDA = 3.0 V / 12 bit ADC
#define TS_CAL1_ADDR      ((const uint16_t *)0x1FFF75A8UL)  //  30 °C
#define TS_CAL2_ADDR      ((const uint16_t *)0x1FFF75CAUL)  // 130 °C
#define VREFINT_CAL_ADDR  ((const uint16_t *)0x1FFF75AAUL)  // VREFINT @ 3.0 V

static const float   CAL_TEMP1 =   30.0f;
static const float   CAL_TEMP2 =  130.0f;
static const float   CAL_VDDA  = 3000.0f;   // mV
static const uint8_t N_AVG     = 16;
static const float   TJ_WARN   = 60.0f;     // °C
static const float   TJ_ALARM  = 85.0f;     // °C

// ======================= Objects =======================
SPIClass       canSPI(PB5, PB4, PB3);
ACAN2515       can(MCP2515_CS, canSPI, MCP2515_INT);
MICS_VZ_89TE   mics;

// ======================= Status =======================
// One slot per device: stores the latest frame of the current cycle, overwriting previous data.
// The 'valid' flag indicates receipt during the current recording cycle and is cleared after logData() executes;
// thus, if a device fails to transmit during a given second, the corresponding entry is missing,
// preserving the diagnostic value regarding the device's "online status."
template <typename T>
struct Latest {
    T    msg;
    bool valid = false;
    void set(const T &m) { msg = m; valid = true; }
    void clear()         { valid = false; }
};

static Latest<CANFDMessage> g_sgx;      // FDCAN1  0x256
static Latest<CANFDMessage> g_tel;      // FDCAN1  0x18FF0CEB
static Latest<CANFDMessage> g_unkFd;    // FDCAN1  unknown
static Latest<CANMessage>   g_tamer;    // MCP2515 Li-ion Tamer
static Latest<CANMessage>   g_unkMcp;   // MCP2515 unknown

static float g_tjMax   = -273.0f;       // Max_temp
static float g_vddaMin = 99999.0f;      // Min_VDDA 

static unsigned long g_nextLog = 0;

static void pollAllCan();

// ======================= Setup Block =======================
static void setupSerial() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}
    Serial.println(F("A: boot"));
}

static void setupFdcan() {
    ACANFD_STM32_Settings fdSettings(CAN_BITRATE_T3650_SGXBLD2, DataBitRateFactor::x1, 32);
    const uint32_t err = fdcan1.beginFD(fdSettings);
    Serial.print(F("B: FDCAN1 beginFD = "));
    Serial.println(err);
}

static void setupMcp2515() {
    canSPI.begin();
    ACAN2515Settings mcpSettings(QUARTZ_FREQUENCY, CAN_BITRATE_TAMER);
    mcpSettings.mRequestedMode     = ACAN2515Settings::NormalMode;
    mcpSettings.mReceiveBufferSize = 32;

    const uint16_t err = can.begin(mcpSettings, [] { can.isr(); });
    if (err == 0) {
        Serial.println(F("C: MCP2515 OK"));
        Serial.print(F("   exact bit rate? "));
        Serial.println(mcpSettings.exactBitRate() ? F("yes") : F("no"));
        Serial.print(F("   sample point: "));
        Serial.print(mcpSettings.samplePointFromBitStart());
        Serial.println('%');
    } else {
        Serial.print(F("C: MCP2515 config error 0x"));
        Serial.println(err, HEX);
    }
}

static void setupSensors() {
    Wire.begin();
    pinMode(ANALOG_IN_PIN, INPUT_ANALOG);
    analogReadResolution(12);       // Resolution in 12 bits
    Serial.println(F("D: sensors ok"));
}

static void setupDieTemp() {
    // Drop some readings for warmup
    for (uint8_t i = 0; i < 8; i++) {
        (void)analogRead(ATEMP);
        (void)analogRead(AVREF);
        delay(5);
    }
    Serial.println(F("E: die-temp cal values"));
    Serial.print(F("   TS_CAL1  ( 30 C) = ")); Serial.println(*TS_CAL1_ADDR);
    Serial.print(F("   TS_CAL2  (130 C) = ")); Serial.println(*TS_CAL2_ADDR);
    Serial.print(F("   VREFINT_CAL      = ")); Serial.println(*VREFINT_CAL_ADDR);
}

void setup() {
    setupSerial();
    setupFdcan();
    setupMcp2515();
    setupSensors();
    setupDieTemp();
    Serial.println(F("F: setup done"));
    g_nextLog = millis() + LOG_INTERVAL;
}

// ======================= Collection =======================
static void pollFdcan() {
    CANFDMessage m;
    while (fdcan1.receiveFD0(m)) {          // Drain completely; FIFO to prevent backlog
        if      (m.id == ID_SGX)     g_sgx.set(m);
        else if (m.id == ID_TELAIRE) g_tel.set(m);
        else                         g_unkFd.set(m);
    }
}

static void pollMcp2515() {
    CANMessage f;
    while (can.receive(f)) {                // Drain completely; FIFO to prevent backlog
        const uint32_t pgn = (f.id >> 8) & 0x03FFFF;
        if (f.ext && pgn == PGN_TAMER && f.len >= 4) g_tamer.set(f);
        else                                         g_unkMcp.set(f);
    }
}

static void pollAllCan() {
    pollFdcan();
    pollMcp2515();
}

// Multiple samples are averaged to suppress ADC noise. The 200 µs interval between samples
// was originally idle time; here, it is repurposed to poll the CAN bus—averaging 16 samples
// across 2 channels takes about 6.4 ms, so the bus cannot be left unattended during this period.
static uint32_t readAvgPolled(uint32_t pin, uint8_t n) {
    uint32_t sum = 0;
    for (uint8_t i = 0; i < n; i++) {
        sum += analogRead(pin);
        const uint32_t t0 = micros();
        do { pollAllCan(); } while (micros() - t0 < 200);
    }
    return sum / n;
}

// ======================= Output =======================
static void printSeparator() {
    Serial.println(F("============================="));
}

static void logAdc() {
    const int sensorValue = analogRead(ANALOG_IN_PIN);
    Serial.print(F("Raw ADC Value: "));     // Start point for datalogger
    Serial.println(sensorValue);

    const float voltage = sensorValue * (3.3f / 4095.0f);
    Serial.print(F("Voltage: "));
    Serial.print(voltage, 3);
    Serial.println(F(" V"));
    printSeparator();
}

// Board-level health: Chip junction temperature (Tj) + actual VDDA.
// Tj refers to die temperature, not ambient temperature; factory calibration accuracy is approximately ±5°C,
// which is sufficient for determining whether overheating is occurring but cannot be used to log ambient temperature.
static void logDieTemp() {
    const uint32_t rawVref = readAvgPolled(AVREF, N_AVG);
    const uint32_t rawTs   = readAvgPolled(ATEMP, N_AVG);

    Serial.println(F("Board health (STM32G431KB):"));
    if (rawVref == 0) {  
        Serial.println(F("VREFINT read failed"));
        printSeparator();
        return;
    }

    // 1) Calculate the actual VDDA using the internal reference VREFINT: VREFINT voltage is constant; a lower reading implies a higher supply voltage
    const float vdda = CAL_VDDA * (float)(*VREFINT_CAL_ADDR) / (float)rawVref;

    // 2) Normalize the temperature channel readings to the 3.0 V reference used during calibration
    const float tsNorm = (float)rawTs * vdda / CAL_VDDA;

    // 3) Linear interpolation between two factory calibration points
    const float span = (float)(*TS_CAL2_ADDR) - (float)(*TS_CAL1_ADDR);
    const float tj   = (CAL_TEMP2 - CAL_TEMP1) / span
                       * (tsNorm - (float)(*TS_CAL1_ADDR)) + CAL_TEMP1;

    if (tj   > g_tjMax)   g_tjMax   = tj;
    if (vdda < g_vddaMin) g_vddaMin = vdda;

    Serial.print(F("Die Temp:           "));
    Serial.print(tj, 1);
    Serial.println(F(" C"));
    Serial.print(F("Die Temp Max:       "));
    Serial.print(g_tjMax, 1);
    Serial.println(F(" C"));
    Serial.print(F("VDDA:               "));
    Serial.print(vdda, 0);
    Serial.println(F(" mV"));
    Serial.print(F("VDDA Min:           "));
    Serial.print(g_vddaMin, 0);
    Serial.println(F(" mV"));
    Serial.print(F("ADC raw (ts/vref):  "));
    Serial.print(rawTs);
    Serial.print('/');
    Serial.println(rawVref);

    if (tj > TJ_ALARM)     Serial.println(F("Board Status:       OVERHEAT <<< INSPECTION NEEDED!"));
    else if (tj > TJ_WARN) Serial.println(F("Board Status:       HIGH"));
    else                   Serial.println(F("Board Status:       OK"));
    printSeparator();
}

static void logMics() {
    mics.readSensor();
    Serial.print(F("CO2 equ(ppm): "));
    Serial.print(mics.getCO2());
    Serial.println(F(" ppm"));
    Serial.print(F("VOC(isobutylene) equ: "));
    Serial.print(mics.getVOC());
    Serial.println(F(" ppb"));
    printSeparator();
}

static void logSgxFlags(uint8_t flags) {
    if (flags == 0) {
        Serial.println(F("No status error"));
        return;
    }
    static const __FlashStringHelper *const NAMES[8] = {
        F(" Overvoltage:        "), F(" TC issue:           "),
        F(" RH issue:           "), F(" H2 out of range:    "),
        F(" Temperature issue:  "), F(" Undervoltage:       "),
        F(" Sensor replacement: "), F(" Low power bit:      ")
    };
    Serial.println(F("--- Flags ---"));
    for (uint8_t i = 0; i < 8; i++) {
        Serial.print(NAMES[i]);
        Serial.println((flags >> i) & 1);
    }
    Serial.println(F("-------------"));
}

static void logSgx() {
    if (!g_sgx.valid || g_sgx.msg.len < 7) return;
    const uint8_t *d = g_sgx.msg.data;

    Serial.println(F("Message received from SGX-BLD2:"));

    Serial.print(F("Temperature: "));
    Serial.print((int)d[0] - 55);
    Serial.println(F(" C"));

    const uint16_t h2_raw = ((uint16_t)d[1] << 8) | d[2];
    Serial.print(F("Hydrogen percent:   "));
    Serial.print(h2_raw * 0.01f, 2);
    Serial.println(F(" %"));

    logSgxFlags(d[3]);

    Serial.print(F("Voltage:            "));
    Serial.print(d[4] * 0.1f, 1);
    Serial.println(F(" V"));

    Serial.print(F("Humidity:           "));
    Serial.print(d[5] * 0.5f, 1);
    Serial.println(F(" %"));

    Serial.print(F("Roll Counter:       "));
    Serial.println(d[6] & 0x0F);
    Serial.print(F("Level CO:           "));
    Serial.println((d[6] >> 4) & 0x0F);
    printSeparator();
}

static void logTelaire() {
    if (!g_tel.valid || g_tel.msg.len < 8) return;
    const uint8_t *d = g_tel.msg.data;

    Serial.println(F("Message received from Telaire T3650:"));

    const uint16_t press_raw = ((uint16_t)d[1] << 8) | d[0];
    Serial.print(F("Pressure:           "));
    Serial.print(press_raw * 0.0078125f - 250.0f, 2);
    Serial.println(F(" kPa"));

    Serial.print(F("Humidity:  "));
    Serial.print(d[2] * 0.4f, 1);
    Serial.println(F(" %"));

    const uint16_t h2_raw = ((uint16_t)d[4] << 8) | d[3];
    Serial.print(F("H2 Concentration:   "));
    Serial.print(h2_raw * 0.0025f, 4);
    Serial.println(F(" %"));

    const uint16_t temp_raw = ((uint16_t)d[6] << 8) | d[5];
    Serial.print(F("Temperature:        "));
    Serial.print(temp_raw * 0.03125f - 273.0f, 2);
    Serial.println(F(" C"));

    Serial.print(F("Sensor Status:      0x"));
    Serial.println(d[7], HEX);
    printSeparator();
}

static void logTamer() {
    if (!g_tamer.valid) return;
    const CANMessage &f = g_tamer.msg;

    const uint8_t sa    = f.id & 0xFF;
    const uint8_t state = f.data[0];
    const int8_t  degC  = (int8_t)f.data[1];
    const int16_t raw   = (int16_t)((uint16_t)f.data[2] |
                                    ((uint16_t)f.data[3] << 8));
    const float   scalar = raw / 100.0f;

    static const char *const STATE_NAMES[] = {
        "Illegal", "Error", "Warmup", "Normal", "Alarm"
    };
    const char *stateName = (state <= 4) ? STATE_NAMES[state] : "Unknown";

    Serial.println(F("Message received from Li-ion Tamer:"));
    Serial.print(F("Tamer SA=0x"));   Serial.print(sa, HEX);
    Serial.print(F("  State="));      Serial.print(stateName);
    Serial.print(F("  Temp="));       Serial.print(degC);
    Serial.print(F("C  OffGas="));    Serial.print(scalar, 2);
    if (state == 4)          Serial.print(F("  <<< ALARM"));
    else if (scalar >= 1.0f) Serial.print(F("  (>= alarm trigger level)"));
    Serial.println();
    printSeparator();
}

static void logUnknownFrame(const char *bus, uint32_t id,
                            const uint8_t *data, uint8_t len) {
    Serial.print(F("Unknown device on "));
    Serial.print(bus);
    Serial.print(F("! ID=0x"));
    Serial.print(id, HEX);
    Serial.print(F(" Data="));
    for (uint8_t i = 0; i < len; i++) {
        Serial.print(data[i], HEX);
        Serial.print(' ');
    }
    Serial.println();
    printSeparator();
}

static void logData() {
    logAdc();
    logDieTemp();
    logMics();
    logSgx();
    logTelaire();
    logTamer();
    if (g_unkFd.valid)
        logUnknownFrame("FDCAN1", g_unkFd.msg.id, g_unkFd.msg.data, g_unkFd.msg.len);
    if (g_unkMcp.valid)
        logUnknownFrame("MCP2515", g_unkMcp.msg.id, g_unkMcp.msg.data, g_unkMcp.msg.len);

    g_sgx.clear();
    g_tel.clear();
    g_tamer.clear();
    g_unkFd.clear();
    g_unkMcp.clear();
}

// ======================= Main Loop =======================
void loop() {
    pollAllCan();                                  // Always in mode

    if ((long)(millis() - g_nextLog) >= 0) {       // rollover safe
        g_nextLog += LOG_INTERVAL;
        logData();
        pollAllCan();                              // Capture to avoid missing
    }
}
