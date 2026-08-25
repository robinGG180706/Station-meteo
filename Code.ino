// ============================================================================
// STATION MÉTÉO - VERSION ROBUSTE
// Conforme aux standards MISRA-C (adaptés pour Arduino)
// Protection contre: débordements, corruption EEPROM, blocages I2C, brownout
// ============================================================================

// Bibliothèques
#include <Wire.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_BMP085.h>
#include <DFRobot_ENS160.h>
#include <TM1640.h>
#include <FastLED.h>
#include <EEPROM.h>
#include <avr/wdt.h> // Watchdog Timer

// ============================================================================
// CONSTANTES (MISRA: Pas de magic numbers)
// ============================================================================

// --- Hardware pins ---
#define TM1640_DIN_PIN 2
#define TM1640_CLK_PIN 3
#define LED_PIN_A 4
#define LED_PIN_B 5
#define NUM_LEDS 5

// --- Timing constants (milliseconds) ---
#define SENSOR_READ_INTERVAL_MS 3000UL
#define EEPROM_SAVE_INTERVAL_MS 60000UL
#define MINMAX_DISPLAY_INTERVAL_MS 60000UL
#define STATE_DISPLAY_DURATION_MS 5000UL
#define LED_BLINK_INTERVAL_MS 500UL
#define I2C_TIMEOUT_US 3000UL
#define ENS160_WARMUP_MS 3000UL
#define WATCHDOG_TIMEOUT_MS WDTO_8S // 8 secondes

// --- EEPROM constants ---
#define EEPROM_MAGIC_V2 0xAB43 // Changé pour forcer réinit avec nouvelle structure
#define EEPROM_NUM_SLOTS 4     // Wear leveling: 4 slots rotatifs
#define EEPROM_SLOT_SIZE 32    // Taille d'un slot (struct + CRC)

// --- Validation des capteurs ---
#define TEMP_MIN_VALID -40.0F
#define TEMP_MAX_VALID 85.0F
#define HUM_MIN_VALID 0.0F
#define HUM_MAX_VALID 100.0F
#define PRES_MIN_VALID 870.0F
#define PRES_MAX_VALID 1085.0F
#define CO2_MIN_VALID 400
#define CO2_MAX_VALID 5000

// --- Float comparison epsilon ---
#define FLOAT_EPSILON 0.001F

// --- Display constants ---
#define DISPLAY_BRIGHTNESS 4
#define TEMP_OFFSET 30 // Offset pour affichage température (soustraire 3.0°C)
#define TEMP_OFFSET_DECIMAL 30

// --- I2C Recovery ---
#define I2C_MAX_RECOVERY_ATTEMPTS 3

// --- LED Configuration ---
#define LED_BRIGHTNESS 10 // 10/255 ≈ 4% pour limiter le courant

// --- CO2 Gradient LEDs ---
const CRGB CO2_COULEURS[NUM_LEDS] = {
    CRGB(0, 255, 0),   // LED 0: vert
    CRGB(128, 255, 0), // LED 1: vert-jaune
    CRGB(255, 255, 0), // LED 2: jaune
    CRGB(255, 128, 0), // LED 3: orange
    CRGB(255, 0, 0)    // LED 4: rouge
};

const uint16_t CO2_SEUILS[NUM_LEDS] = {400, 600, 800, 1000, 1500};

// --- Pressure indicator zones ---
const float PRES_PALIERS[4] = {960.0F, 980.0F, 1000.0F, 1020.0F};

// ============================================================================
// STRUCTURES DE DONNÉES
// ============================================================================

// Structure EEPROM avec CRC16 et métadonnées
struct MinMaxData
{
    uint16_t magic;
    uint8_t version;
    uint8_t slotIndex; // Pour wear leveling
    float minTemp, maxTemp;
    float minHum, maxHum;
    uint16_t minCo2, maxCo2;
    float minPres, maxPres;
    uint16_t crc16; // CRC-16 pour validation
} __attribute__((packed));

// État de la machine d'affichage
enum DisplayState : uint8_t
{
    STATE_NORMAL = 0,
    STATE_SHOW_MIN = 1,
    STATE_SHOW_MAX = 2
};

// Structure pour données des capteurs
struct SensorData
{
    float temperature;
    float humidity;
    float pressure;
    uint16_t eco2;
    bool valid;
};

// Compteurs de diagnostic (optionnel, pour debugging)
struct DiagCounters
{
    uint16_t i2cTimeouts;
    uint16_t i2cRecoveries;
    uint16_t eepromWrites;
    uint16_t invalidReadings;
    uint16_t watchdogResets;
};

// ============================================================================
// VARIABLES GLOBALES (toutes statiques)
// ============================================================================

// Objets hardware
static Adafruit_AHTX0 aht;
static Adafruit_BMP085 bmp;
static DFRobot_ENS160_I2C ens160(&Wire, 0x53);
static TM1640 display(TM1640_DIN_PIN, TM1640_CLK_PIN, false, DISPLAY_BRIGHTNESS);
static CRGB ledsA[NUM_LEDS];
static CRGB ledsB[NUM_LEDS];

// État système
static MinMaxData mmData;
static SensorData currentSensor;
static DiagCounters diagCounters = {0, 0, 0, 0, 0};
static bool eepromDirty = false;
static uint8_t currentEepromSlot = 0;

// Compensation ENS160
static float lastTempComp = -999.0F;
static float lastHumComp = -999.0F;

// ============================================================================
// PROTOTYPES DE FONCTIONS (MISRA compliance)
// ============================================================================

// Core functions
void systemInit(void);
void systemLoop(void);
void handleFatalError(const char *msg);

// EEPROM management
uint16_t calculateCRC16(const uint8_t *data, uint16_t length);
bool validateMinMaxData(const MinMaxData *data);
void loadMinMaxFromEEPROM(void);
void saveMinMaxToEEPROM(void);
uint16_t getEepromSlotAddress(uint8_t slot);

// Sensor management
bool readSensors(SensorData *data);
bool validateSensorData(const SensorData *data);
void compensateENS160(float temp, float hum);

// Display management
void displayValues(float temp, float hum, uint16_t co2, float pres);
void updateLedsNormal(uint16_t co2, float pres);
void updateLedsOff(void);

// MinMax tracking
void updateMinMax(const SensorData *data);

// I2C management
bool recoverI2C(void);

// Utilities
bool floatEqual(float a, float b, float epsilon);
int32_t constrainInt(int32_t value, int32_t min_val, int32_t max_val);

// ============================================================================
// CRC16-CCITT Implementation
// ============================================================================

uint16_t calculateCRC16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFF;

    for (uint16_t i = 0; i < length; i++)
    {
        crc ^= ((uint16_t)data[i] << 8);

        for (uint8_t bit = 0; bit < 8; bit++)
        {
            if ((crc & 0x8000) != 0)
            {
                crc = (crc << 1) ^ 0x1021;
            }
            else
            {
                crc = crc << 1;
            }
        }
    }

    return crc;
}

// ============================================================================
// EEPROM MANAGEMENT (avec wear leveling et CRC)
// ============================================================================

uint16_t getEepromSlotAddress(uint8_t slot)
{
    return (uint16_t)slot * EEPROM_SLOT_SIZE;
}

bool validateMinMaxData(const MinMaxData *data)
{
    // Vérifier le magic number
    if (data->magic != EEPROM_MAGIC_V2)
    {
        return false;
    }

    // Calculer le CRC (exclure le champ crc16 lui-même)
    uint16_t calculatedCRC = calculateCRC16(
        (const uint8_t *)data,
        sizeof(MinMaxData) - sizeof(uint16_t));

    if (calculatedCRC != data->crc16)
    {
        Serial.println(F("[EEPROM] CRC mismatch"));
        return false;
    }

    // Valider les plages
    if (data->minTemp < TEMP_MIN_VALID || data->maxTemp > TEMP_MAX_VALID ||
        data->minHum < HUM_MIN_VALID || data->maxHum > HUM_MAX_VALID ||
        data->minPres < PRES_MIN_VALID || data->maxPres > PRES_MAX_VALID ||
        data->minCo2 < CO2_MIN_VALID || data->maxCo2 > CO2_MAX_VALID)
    {
        Serial.println(F("[EEPROM] Invalid ranges"));
        return false;
    }

    return true;
}

void loadMinMaxFromEEPROM(void)
{
    bool foundValid = false;
    uint8_t newestSlot = 0;
    uint8_t newestVersion = 0;

    // Chercher le slot le plus récent et valide
    for (uint8_t slot = 0; slot < EEPROM_NUM_SLOTS; slot++)
    {
        MinMaxData tempData;
        uint16_t addr = getEepromSlotAddress(slot);
        EEPROM.get(addr, tempData);

        if (validateMinMaxData(&tempData))
        {
            if (!foundValid || tempData.version > newestVersion)
            {
                newestVersion = tempData.version;
                newestSlot = slot;
                mmData = tempData;
                foundValid = true;
            }
        }
    }

    if (foundValid)
    {
        currentEepromSlot = newestSlot;
        Serial.print(F("[EEPROM] Loaded slot "));
        Serial.print(currentEepromSlot);
        Serial.print(F(" v"));
        Serial.println(mmData.version);
    }
    else
    {
        // Initialiser avec des valeurs par défaut
        mmData.magic = EEPROM_MAGIC_V2;
        mmData.version = 0;
        mmData.slotIndex = 0;
        mmData.minTemp = 99.9F;
        mmData.maxTemp = -99.9F;
        mmData.minHum = 100.0F;
        mmData.maxHum = 0.0F;
        mmData.minCo2 = 65535;
        mmData.maxCo2 = 0;
        mmData.minPres = 9999.0F;
        mmData.maxPres = 0.0F;

        // Calculer et stocker le CRC
        mmData.crc16 = calculateCRC16(
            (const uint8_t *)&mmData,
            sizeof(MinMaxData) - sizeof(uint16_t));

        currentEepromSlot = 0;
        EEPROM.put(getEepromSlotAddress(0), mmData);

        Serial.println(F("[EEPROM] Initialized (first boot)"));
    }
}

void saveMinMaxToEEPROM(void)
{
    // Rotation: passer au slot suivant (wear leveling)
    currentEepromSlot = (currentEepromSlot + 1) % EEPROM_NUM_SLOTS;

    // Incrémenter la version
    mmData.version++;
    mmData.slotIndex = currentEepromSlot;

    // Recalculer le CRC
    mmData.crc16 = calculateCRC16(
        (const uint8_t *)&mmData,
        sizeof(MinMaxData) - sizeof(uint16_t));

    // Écrire dans le nouveau slot
    uint16_t addr = getEepromSlotAddress(currentEepromSlot);
    EEPROM.put(addr, mmData);

    // Vérification: relire et valider
    MinMaxData verify;
    EEPROM.get(addr, verify);

    if (!validateMinMaxData(&verify))
    {
        Serial.println(F("[EEPROM] Write verification failed!"));
        diagCounters.invalidReadings++;
    }
    else
    {
        diagCounters.eepromWrites++;
    }

    eepromDirty = false;
}

// ============================================================================
// SENSOR MANAGEMENT
// ============================================================================

bool validateSensorData(const SensorData *data)
{
    if (!data->valid)
    {
        return false;
    }

    if (data->temperature < TEMP_MIN_VALID || data->temperature > TEMP_MAX_VALID)
    {
        Serial.println(F("[SENSOR] Invalid temperature"));
        return false;
    }

    if (data->humidity < HUM_MIN_VALID || data->humidity > HUM_MAX_VALID)
    {
        Serial.println(F("[SENSOR] Invalid humidity"));
        return false;
    }

    if (data->pressure < PRES_MIN_VALID || data->pressure > PRES_MAX_VALID)
    {
        Serial.println(F("[SENSOR] Invalid pressure"));
        return false;
    }

    if (data->eco2 < CO2_MIN_VALID || data->eco2 > CO2_MAX_VALID)
    {
        Serial.println(F("[SENSOR] Invalid CO2"));
        return false;
    }

    return true;
}

bool floatEqual(float a, float b, float epsilon)
{
    float diff = a - b;
    if (diff < 0.0F)
    {
        diff = -diff;
    }
    return diff < epsilon;
}

void compensateENS160(float temp, float hum)
{
    // Ne mettre à jour que si changement significatif (éviter surcommunication I2C)
    float tempDiff = temp - lastTempComp;
    if (tempDiff < 0.0F)
        tempDiff = -tempDiff;

    float humDiff = hum - lastHumComp;
    if (humDiff < 0.0F)
        humDiff = -humDiff;

    if (tempDiff > 0.5F || humDiff > 1.0F)
    {
        ens160.setTempAndHum(temp, hum);
        lastTempComp = temp;
        lastHumComp = hum;
    }
}

bool readSensors(SensorData *data)
{
    if (data == NULL)
    {
        return false;
    }

    // Reset watchdog avant opérations I2C longues
    wdt_reset();

    // Lecture AHT2x (température et humidité)
    sensors_event_t hev, tev;
    bool ahtOk = aht.getEvent(&hev, &tev);

    if (!ahtOk)
    {
        Serial.println(F("[SENSOR] AHT read failed"));
        data->valid = false;
        return false;
    }

    data->temperature = tev.temperature;
    data->humidity = hev.relative_humidity;

    // Lecture BMP180 (pression)
    int32_t rawPressure = bmp.readPressure();
    if (rawPressure <= 0)
    {
        Serial.println(F("[SENSOR] BMP read failed"));
        data->valid = false;
        return false;
    }
    data->pressure = (float)rawPressure / 100.0F;

    // Compensation et lecture ENS160 (CO2)
    compensateENS160(data->temperature, data->humidity);
    data->eco2 = ens160.getECO2();

    // Vérifier timeout I2C
    if (Wire.getWireTimeoutFlag())
    {
        Serial.println(F("[I2C] Timeout detected"));
        Wire.clearWireTimeoutFlag();
        diagCounters.i2cTimeouts++;

        if (!recoverI2C())
        {
            data->valid = false;
            return false;
        }
    }

    data->valid = true;

    // Validation finale
    if (!validateSensorData(data))
    {
        diagCounters.invalidReadings++;
        data->valid = false;
        return false;
    }

    return true;
}

// ============================================================================
// I2C RECOVERY
// ============================================================================

bool recoverI2C(void)
{
    Serial.println(F("[I2C] Attempting recovery..."));

    for (uint8_t attempt = 0; attempt < I2C_MAX_RECOVERY_ATTEMPTS; attempt++)
    {
        wdt_reset();

        Wire.end();
        delay(100);

        Wire.begin();
        Wire.setWireTimeout(I2C_TIMEOUT_US, false);
        delay(100);

        // Test de communication
        Wire.beginTransmission(0x38); // AHT2x
        uint8_t error = Wire.endTransmission();

        if (error == 0)
        {
            Serial.println(F("[I2C] Recovery successful"));
            diagCounters.i2cRecoveries++;
            return true;
        }

        delay(200);
    }

    Serial.println(F("[I2C] Recovery failed"));
    return false;
}

// ============================================================================
// DISPLAY MANAGEMENT
// ============================================================================

int32_t constrainInt(int32_t value, int32_t min_val, int32_t max_val)
{
    if (value < min_val)
        return min_val;
    if (value > max_val)
        return max_val;
    return value;
}

void displayValues(float temp, float hum, uint16_t co2, float pres)
{
    // Température (format: XX.X avec offset de -3.0°C)
    int32_t t = (int32_t)(temp * 10.0F + 0.5F);
    t = constrainInt(t - TEMP_OFFSET_DECIMAL, 0, 999);

    display.setDisplayDigit((uint8_t)(t / 100), 0, false);
    display.setDisplayDigit((uint8_t)((t / 10) % 10), 1, true);
    display.setDisplayDigit((uint8_t)(t % 10), 2, false);

    // Humidité (format: XX)
    int32_t h = constrainInt((int32_t)(hum + 0.5F), 0, 99);
    display.setDisplayDigit((uint8_t)(h / 10), 3, false);
    display.setDisplayDigit((uint8_t)(h % 10), 4, false);

    // CO2 (format: XXXX)
    uint16_t c = constrainInt((int32_t)co2, 0, 9999);
    display.setDisplayDigit((uint8_t)(c / 1000), 5, false);
    display.setDisplayDigit((uint8_t)((c / 100) % 10), 6, false);
    display.setDisplayDigit((uint8_t)((c / 10) % 10), 7, false);
    display.setDisplayDigit((uint8_t)(c % 10), 8, false);

    // Pression (format: XXXX)
    int32_t p = constrainInt((int32_t)(pres + 0.5F), 0, 9999);
    display.setDisplayDigit((uint8_t)(p / 1000), 9, false);
    display.setDisplayDigit((uint8_t)((p / 100) % 10), 10, false);
    display.setDisplayDigit((uint8_t)((p / 10) % 10), 11, false);
    display.setDisplayDigit((uint8_t)(p % 10), 12, false);
}

void updateLedsNormal(uint16_t co2, float pres)
{
    // Bandeau A: gradient CO2
    for (uint8_t i = 0; i < NUM_LEDS; i++)
    {
        if (co2 >= CO2_SEUILS[i])
        {
            ledsA[i] = CO2_COULEURS[i];
        }
        else
        {
            ledsA[i] = CRGB::Black;
        }
    }

    // Bandeau B: zone de pression (1 LED blanche)
    uint8_t zone = 0;
    for (uint8_t i = 0; i < 4; i++)
    {
        if (pres >= PRES_PALIERS[i])
        {
            zone = i + 1;
        }
    }

    fill_solid(ledsB, NUM_LEDS, CRGB::Black);
    if (zone < NUM_LEDS)
    {
        ledsB[zone] = CRGB::White;
    }

    FastLED.show();
}

void updateLedsOff(void)
{
    fill_solid(ledsA, NUM_LEDS, CRGB::Black);
    fill_solid(ledsB, NUM_LEDS, CRGB::Black);
    FastLED.show();
}

// ============================================================================
// MIN/MAX TRACKING
// ============================================================================

void updateMinMax(const SensorData *data)
{
    if (!data->valid)
    {
        return;
    }

    bool changed = false;

    if (data->temperature < mmData.minTemp)
    {
        mmData.minTemp = data->temperature;
        changed = true;
    }
    if (data->temperature > mmData.maxTemp)
    {
        mmData.maxTemp = data->temperature;
        changed = true;
    }

    if (data->humidity < mmData.minHum)
    {
        mmData.minHum = data->humidity;
        changed = true;
    }
    if (data->humidity > mmData.maxHum)
    {
        mmData.maxHum = data->humidity;
        changed = true;
    }

    if (data->eco2 < mmData.minCo2)
    {
        mmData.minCo2 = data->eco2;
        changed = true;
    }
    if (data->eco2 > mmData.maxCo2)
    {
        mmData.maxCo2 = data->eco2;
        changed = true;
    }

    if (data->pressure < mmData.minPres)
    {
        mmData.minPres = data->pressure;
        changed = true;
    }
    if (data->pressure > mmData.maxPres)
    {
        mmData.maxPres = data->pressure;
        changed = true;
    }

    if (changed)
    {
        eepromDirty = true;
    }
}

// ============================================================================
// ERROR HANDLING
// ============================================================================

void handleFatalError(const char *msg)
{
    // Désactiver les interruptions
    cli();

    // Éteindre les LEDs
    fill_solid(ledsA, NUM_LEDS, CRGB::Red);
    fill_solid(ledsB, NUM_LEDS, CRGB::Red);
    FastLED.show();

    // Message série
    Serial.print(F("[FATAL] "));
    Serial.println(msg);

    // Boucle infinie avec watchdog qui va reset le système
    while (true)
    {
        delay(100);
        // Le watchdog va déclencher un reset après 8 secondes
    }
}

// ============================================================================
// SETUP
// ============================================================================

void setup()
{
    // Vérifier la source du dernier reset
    uint8_t mcusr = MCUSR;
    MCUSR = 0; // Clear reset flags

    // Désactiver le watchdog le temps du setup
    wdt_disable();

    // Init Serial
    Serial.begin(115200);
    while (!Serial && millis() < 2000)
        ; // Attendre max 2s

    Serial.println(F("\n=================="));
    Serial.println(F("STATION METEO v2.0"));
    Serial.println(F("=================="));

    // Analyser la cause du reset
    if (mcusr & (1 << WDRF))
    {
        Serial.println(F("[BOOT] Watchdog reset"));
        diagCounters.watchdogResets++;
    }
    if (mcusr & (1 << BORF))
    {
        Serial.println(F("[BOOT] Brown-out reset"));
    }
    if (mcusr & (1 << EXTRF))
    {
        Serial.println(F("[BOOT] External reset"));
    }
    if (mcusr & (1 << PORF))
    {
        Serial.println(F("[BOOT] Power-on reset"));
    }

    // Init I2C
    Wire.begin();
    Wire.setWireTimeout(I2C_TIMEOUT_US, false);
    Serial.println(F("[I2C] Initialized"));

    // Init FastLED (avec limitation de courant au démarrage)
    FastLED.addLeds<WS2812B, LED_PIN_A, GRB>(ledsA, NUM_LEDS);
    FastLED.addLeds<WS2812B, LED_PIN_B, GRB>(ledsB, NUM_LEDS);
    FastLED.setBrightness(LED_BRIGHTNESS);

    // Allumage progressif pour éviter pic de courant
    for (uint8_t brightness = 0; brightness <= LED_BRIGHTNESS; brightness++)
    {
        FastLED.setBrightness(brightness);
        fill_solid(ledsA, NUM_LEDS, CRGB::White);
        fill_solid(ledsB, NUM_LEDS, CRGB::White);
        FastLED.show();
        delay(20);
    }

    Serial.println(F("[LED] Initialized"));

    // Init sensors
    if (!aht.begin())
    {
        handleFatalError("AHT2x (0x38) initialization failed");
    }
    Serial.println(F("[AHT2x] OK"));

    if (!bmp.begin())
    {
        handleFatalError("BMP180 (0x77) initialization failed");
    }
    Serial.println(F("[BMP180] OK"));

    if (ens160.begin() != 0)
    {
        handleFatalError("ENS160 (0x53) initialization failed");
    }
    ens160.setPWRMode(ENS160_STANDARD_MODE);
    Serial.println(F("[ENS160] OK (warming up 3s...)"));

    // Warmup ENS160 avec watchdog désactivé
    delay(ENS160_WARMUP_MS);

    // Init display
    display.setupDisplay(true, DISPLAY_BRIGHTNESS);
    display.clearDisplay();
    Serial.println(F("[TM1640] OK"));

    // Load EEPROM data
    loadMinMaxFromEEPROM();

    // Initialiser les données capteur
    currentSensor.valid = false;
    currentSensor.temperature = 25.0F;
    currentSensor.humidity = 50.0F;
    currentSensor.pressure = 1013.0F;
    currentSensor.eco2 = 400;

    // Activer le watchdog (8 secondes)
    wdt_enable(WATCHDOG_TIMEOUT_MS);
    Serial.println(F("[WDT] Enabled (8s)"));

    Serial.println(F("[BOOT] Complete\n"));
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop()
{
    // Reset watchdog en début de loop
    wdt_reset();

    // Variables statiques (persistent entre les appels)
    static unsigned long lastRead = 0;
    static unsigned long lastMinMax = 0;
    static unsigned long stateStart = 0;
    static unsigned long blinkTimer = 0;
    static unsigned long lastEepromSave = 0;
    static bool blinkOn = true;
    static DisplayState state = STATE_NORMAL;

    const unsigned long now = millis();

    // ========================================================================
    // LECTURE CAPTEURS (toutes les 3 secondes)
    // ========================================================================

    // Protection contre le débordement de millis() (après 49 jours)
    // L'arithmétique non signée garantit que (now - lastRead) fonctionne
    // correctement même après débordement
    if ((now - lastRead) >= SENSOR_READ_INTERVAL_MS)
    {
        lastRead = now;

        wdt_reset(); // Reset avant opération longue

        if (readSensors(&currentSensor))
        {
            // Lecture réussie et valide
            Serial.println(F("============================="));
            Serial.print(F("Temperature : "));
            Serial.print(currentSensor.temperature, 1);
            Serial.println(F(" C"));

            Serial.print(F("Humidite    : "));
            Serial.print(currentSensor.humidity, 1);
            Serial.println(F(" %"));

            Serial.print(F("Pression    : "));
            Serial.print(currentSensor.pressure, 1);
            Serial.println(F(" hPa"));

            Serial.print(F("eCO2        : "));
            Serial.print(currentSensor.eco2);
            Serial.println(F(" ppm"));

            // Mise à jour min/max
            updateMinMax(&currentSensor);

            // Mise à jour affichage si en mode normal
            if (state == STATE_NORMAL)
            {
                displayValues(
                    currentSensor.temperature,
                    currentSensor.humidity,
                    currentSensor.eco2,
                    currentSensor.pressure);
                updateLedsNormal(currentSensor.eco2, currentSensor.pressure);
            }
        }
        else
        {
            Serial.println(F("[WARN] Sensor read failed, using last valid data"));
            // On continue avec les dernières valeurs valides
        }
    }

    // ========================================================================
    // SAUVEGARDE EEPROM (throttlée à 1x/minute max)
    // ========================================================================

    if (eepromDirty && ((now - lastEepromSave) >= EEPROM_SAVE_INTERVAL_MS))
    {
        wdt_reset();

        Serial.println(F("[EEPROM] Saving min/max..."));
        saveMinMaxToEEPROM();
        lastEepromSave = now;

        Serial.print(F("[DIAG] EEPROM writes: "));
        Serial.println(diagCounters.eepromWrites);
    }

    // ========================================================================
    // AFFICHAGE MIN/MAX (toutes les 60 secondes)
    // ========================================================================

    if (state == STATE_NORMAL && ((now - lastMinMax) >= MINMAX_DISPLAY_INTERVAL_MS))
    {
        // Transition vers affichage MIN
        state = STATE_SHOW_MIN;
        stateStart = now;
        blinkOn = true;
        blinkTimer = now;

        displayValues(
            mmData.minTemp,
            mmData.minHum,
            mmData.minCo2,
            mmData.minPres);

        Serial.println(F("[DISPLAY] Showing MIN"));
    }

    // ========================================================================
    // TRANSITIONS MIN → MAX → NORMAL (5 secondes chacun)
    // ========================================================================

    if (state != STATE_NORMAL && ((now - stateStart) >= STATE_DISPLAY_DURATION_MS))
    {
        // Protection: timeout maximal (état bloqué)
        const unsigned long maxStateTime = STATE_DISPLAY_DURATION_MS * 3;
        if ((now - stateStart) > maxStateTime)
        {
            Serial.println(F("[WARN] State timeout, forcing NORMAL"));
            state = STATE_NORMAL;
            lastMinMax = now;
            displayValues(
                currentSensor.temperature,
                currentSensor.humidity,
                currentSensor.eco2,
                currentSensor.pressure);
            updateLedsNormal(currentSensor.eco2, currentSensor.pressure);
        }
        else if (state == STATE_SHOW_MIN)
        {
            // MIN → MAX
            state = STATE_SHOW_MAX;
            stateStart = now;
            blinkOn = true;

            displayValues(
                mmData.maxTemp,
                mmData.maxHum,
                mmData.maxCo2,
                mmData.maxPres);

            Serial.println(F("[DISPLAY] Showing MAX"));
        }
        else if (state == STATE_SHOW_MAX)
        {
            // MAX → NORMAL
            state = STATE_NORMAL;
            lastMinMax = now;

            displayValues(
                currentSensor.temperature,
                currentSensor.humidity,
                currentSensor.eco2,
                currentSensor.pressure);
            updateLedsNormal(currentSensor.eco2, currentSensor.pressure);

            Serial.println(F("[DISPLAY] Back to NORMAL"));
        }
    }

    // ========================================================================
    // CLIGNOTEMENT LEDs pendant MIN/MAX (500ms on/off)
    // ========================================================================

    if (state != STATE_NORMAL && ((now - blinkTimer) >= LED_BLINK_INTERVAL_MS))
    {
        blinkTimer = now;
        blinkOn = !blinkOn;

        if (blinkOn)
        {
            uint16_t co2Disp = (state == STATE_SHOW_MIN) ? mmData.minCo2 : mmData.maxCo2;
            float presDisp = (state == STATE_SHOW_MIN) ? mmData.minPres : mmData.maxPres;
            updateLedsNormal(co2Disp, presDisp);
        }
        else
        {
            updateLedsOff();
        }
    }

    // ========================================================================
    // DIAGNOSTIC PÉRIODIQUE (optionnel, toutes les 5 minutes)
    // ========================================================================

    static unsigned long lastDiag = 0;
    if ((now - lastDiag) >= 300000UL)
    { // 5 minutes
        lastDiag = now;

        Serial.println(F("\n[DIAGNOSTIC]"));
        Serial.print(F("  I2C timeouts: "));
        Serial.println(diagCounters.i2cTimeouts);
        Serial.print(F("  I2C recoveries: "));
        Serial.println(diagCounters.i2cRecoveries);
        Serial.print(F("  EEPROM writes: "));
        Serial.println(diagCounters.eepromWrites);
        Serial.print(F("  Invalid readings: "));
        Serial.println(diagCounters.invalidReadings);
        Serial.print(F("  WDT resets: "));
        Serial.println(diagCounters.watchdogResets);
        Serial.print(F("  Uptime: "));
        Serial.print(now / 1000UL);
        Serial.println(F(" s\n"));
    }

    // Petite pause pour ne pas saturer le CPU (optionnel)
    delay(10);
}

// ============================================================================
// END OF FILE
// ============================================================================
