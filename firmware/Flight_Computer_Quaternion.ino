#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <math.h>
#include <stdio.h>

#define MPU6050_ADDRESS 0x68

const int SD_CS_PIN = 10;
char DATA_FILENAME[24] = "/flight_001.csv";

//Timing

const unsigned long SETUP_DELAY_MS  = 120000UL;  // 2 minutes
const unsigned long ARMING_DELAY_MS = 120000UL;  // 2 minutes

// Stillness qualification before calibration
const unsigned long STILLNESS_REQUIRED_MS = 5000UL;
const float STILL_ACCEL_TOLERANCE_G = 0.08f;

// Launch detection (total accelerometer magnitude)
const float LAUNCH_ACCEL_THRESHOLD_G = 2.0f;
const int LAUNCH_CONFIRM_SAMPLES = 4;  // about 40 ms at 100 Hz

const unsigned long LOG_INTERVAL_US = 10000UL;   // 100 Hz
const unsigned long SD_FLUSH_MS     = 1000UL;

// Serial telemetry rate
const unsigned long SERIAL_INTERVAL_MS = 100UL;  // 10 Hz

// calibration

const int IMU_CALIBRATION_SAMPLES = 500;
const int VERTICAL_CALIBRATION_SAMPLES = 500;

// physics

const float GRAVITY = 9.80665f;

// mpu6050 scale
// Accelerometer: ±16 g
// Gyroscope:     ±2000 deg/s

const float ACCEL_SCALE = 2048.0f;
const float GYRO_SCALE  = 16.4f;

// mpu6050 variables

int16_t rawAccelX = 0;
int16_t rawAccelY = 0;
int16_t rawAccelZ = 0;

int16_t rawGyroX = 0;
int16_t rawGyroY = 0;
int16_t rawGyroZ = 0;

float accelX_g = 0.0f;
float accelY_g = 0.0f;
float accelZ_g = 0.0f;

float gyroX_dps = 0.0f;
float gyroY_dps = 0.0f;
float gyroZ_dps = 0.0f;

// Calibration offsets
float accelBiasX_g = 0.0f;
float accelBiasY_g = 0.0f;
float accelBiasZ_g = 0.0f;

float gyroBiasX_dps = 0.0f;
float gyroBiasY_dps = 0.0f;
float gyroBiasZ_dps = 0.0f;

// quaternion attitude state (body frame to world frame)

float q0 = 1.0f;
float q1 = 0.0f;
float q2 = 0.0f;
float q3 = 0.0f;

float roll_deg = 0.0f;
float pitch_deg = 0.0f;

// Mahony IMU filter proportional correction gain.
// Accelerometer correction is enabled only when its magnitude is
// close to 1 g; otherwise attitude is propagated using the gyroscope.
const float MAHONY_KP = 1.0f;
const float ACCEL_GRAVITY_TOLERANCE_G = 0.15f;

// acceleration variables

float rawAccelZ_ms2 = 0.0f;
float linearAccelZ_ms2 = 0.0f;

float verticalAccel_ms2 = 0.0f;
float filteredVerticalAccel_ms2 = 0.0f;

float verticalAccelBias_ms2 = 0.0f;

// Low-pass filter.
// This is NOT used to force values to zero.
// It only reduces high-frequency sensor noise.

const float ACCEL_FILTER_ALPHA = 0.35f;

// sd file

File dataFile;

// timers

unsigned long startTimeMillis = 0;

unsigned long lastLogMicros = 0;
unsigned long lastFlushMillis = 0;
unsigned long lastSerialMillis = 0;


unsigned long armingStartMillis = 0;
bool armingComplete = false;

bool launchDetected = false;
unsigned long launchMillis = 0;
unsigned long launchCandidateMillis = 0;
int launchCandidateSamples = 0;

// onboard rgb status led
// Nano ESP32 RGB LED outputs are active-low.

enum FlightStatus
{
    STATUS_STARTUP,
    STATUS_SETUP_DELAY,
    STATUS_CALIBRATION,
    STATUS_LOGGING,
    STATUS_ARMED,
    STATUS_LAUNCH,
    STATUS_ERROR
};

FlightStatus flightStatus = STATUS_STARTUP;

void updateStatusLED();


// countdown variables

int lastSetupSecond = -1;
int lastArmingSecond = -1;

// status led functions

void setLEDColor(bool redOn, bool greenOn, bool blueOn)
{
    // The Nano ESP32 RGB LED is active-low: LOW turns a color on.
    digitalWrite(LED_RED,   redOn   ? LOW : HIGH);
    digitalWrite(LED_GREEN, greenOn ? LOW : HIGH);
    digitalWrite(LED_BLUE,  blueOn  ? LOW : HIGH);
}

void initializeStatusLED()
{
    pinMode(LED_RED, OUTPUT);
    pinMode(LED_GREEN, OUTPUT);
    pinMode(LED_BLUE, OUTPUT);
    setLEDColor(false, false, false);
    flightStatus = STATUS_STARTUP;
}

void setFlightStatus(FlightStatus newStatus)
{
    flightStatus = newStatus;
    updateStatusLED();
}

void updateStatusLED()
{
    unsigned long now = millis();
    bool ledOn = false;

    switch (flightStatus)
    {
        case STATUS_STARTUP:
            // Blue quick blink while hardware initializes.
            ledOn = ((now / 250UL) % 2UL) == 0;
            setLEDColor(false, false, ledOn);
            break;

        case STATUS_SETUP_DELAY:
            // Blue slow blink during the two-minute setup delay.
            ledOn = ((now / 750UL) % 2UL) == 0;
            setLEDColor(false, false, ledOn);
            break;

        case STATUS_CALIBRATION:
            // Yellow = waiting for stillness / calibrating sensors.
            setLEDColor(true, true, false);
            break;

        case STATUS_LOGGING:
            // Green heartbeat = data collection loop is running.
            ledOn = (now % 1000UL) < 120UL;
            setLEDColor(false, ledOn, false);
            break;

        case STATUS_ARMED:
            // Solid green = arming countdown has completed.
            setLEDColor(false, true, false);
            break;

        case STATUS_LAUNCH:
            // Red blink = launch threshold was confirmed.
            ledOn = ((now / 200UL) % 2UL) == 0;
            setLEDColor(ledOn, false, false);
            break;

        case STATUS_ERROR:
            // Rapid red blink = initialization/configuration failure.
            ledOn = ((now / 100UL) % 2UL) == 0;
            setLEDColor(ledOn, false, false);
            break;
    }
}

// event writer

void writeEvent(const char *eventName)
{
    if (!dataFile)
        return;

    float time_s =
        (millis() - startTimeMillis) / 1000.0f;

    dataFile.print("EVENT,");
    dataFile.print(eventName);
    dataFile.print(",");
    dataFile.print(time_s, 3);

    // 13 remaining columns complete the 16-column CSV schema.
    for (int i = 0; i < 13; i++)
    {
        dataFile.print(",0");
    }

    dataFile.println();
}

// i2c write

void writeByte(uint8_t address, uint8_t reg, uint8_t value)
{
    Wire.beginTransmission(address);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

// i2c read

uint8_t readByte(uint8_t address, uint8_t reg)
{
    Wire.beginTransmission(address);
    Wire.write(reg);

    if (Wire.endTransmission(false) != 0)
    {
        return 0;
    }

    uint8_t count =
        Wire.requestFrom((uint8_t)address, (uint8_t)1);

    if (count == 1 && Wire.available())
    {
        return Wire.read();
    }

    return 0;
}

// mpu6050 setup

bool setupMPU()
{
    uint8_t whoAmI =
        readByte(MPU6050_ADDRESS, 0x75);

    Serial.print("MPU6050 WHO_AM_I: 0x");
    Serial.println(whoAmI, HEX);

    if (whoAmI != 0x68)
    {
        return false;
    }

    // Wake MPU6050
    writeByte(MPU6050_ADDRESS, 0x6B, 0x00);

    delay(100);

    // Accelerometer ±16g
    writeByte(MPU6050_ADDRESS, 0x1C, 0x18);

    // Gyroscope ±2000 deg/s
    writeByte(MPU6050_ADDRESS, 0x1B, 0x18);

    // Digital low-pass filter
    writeByte(MPU6050_ADDRESS, 0x1A, 0x03);

    return true;
}

// read mpu6050

bool readMPU()
{
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(0x3B);

    if (Wire.endTransmission(false) != 0)
    {
        return false;
    }

    uint8_t count =
        Wire.requestFrom((uint8_t)MPU6050_ADDRESS, (uint8_t)14);

    if (count < 14)
    {
        return false;
    }

    rawAccelX =
        ((int16_t)Wire.read() << 8) |
        Wire.read();

    rawAccelY =
        ((int16_t)Wire.read() << 8) |
        Wire.read();

    rawAccelZ =
        ((int16_t)Wire.read() << 8) |
        Wire.read();

    // Temperature registers - not used
    Wire.read();
    Wire.read();

    rawGyroX =
        ((int16_t)Wire.read() << 8) |
        Wire.read();

    rawGyroY =
        ((int16_t)Wire.read() << 8) |
        Wire.read();

    rawGyroZ =
        ((int16_t)Wire.read() << 8) |
        Wire.read();

    // Convert to engineering units

    accelX_g =
        ((float)rawAccelX / ACCEL_SCALE)
        - accelBiasX_g;

    accelY_g =
        ((float)rawAccelY / ACCEL_SCALE)
        - accelBiasY_g;

    accelZ_g =
        ((float)rawAccelZ / ACCEL_SCALE)
        - accelBiasZ_g;

    gyroX_dps =
        ((float)rawGyroX / GYRO_SCALE)
        - gyroBiasX_dps;

    gyroY_dps =
        ((float)rawGyroY / GYRO_SCALE)
        - gyroBiasY_dps;

    gyroZ_dps =
        ((float)rawGyroZ / GYRO_SCALE)
        - gyroBiasZ_dps;

    return true;
}

// imu calibration

// IMPORTANT:

// Keep the rocket stationary and upright.

// The Z axis should point along the rocket's vertical axis.

// At rest, Z should measure approximately +1 g.


bool calibrateIMU()
{
    Serial.println();
    Serial.println("=================================");
    Serial.println("        IMU CALIBRATION");
    Serial.println("=================================");
    Serial.println("Keep rocket completely stationary.");

    double sumAx = 0.0;
    double sumAy = 0.0;
    double sumAz = 0.0;

    double sumGx = 0.0;
    double sumGy = 0.0;
    double sumGz = 0.0;

    for (int i = 0; i < IMU_CALIBRATION_SAMPLES; i++)
    {
        if (!readMPU())
            return false;

        float axRaw = (float)rawAccelX / ACCEL_SCALE;
        float ayRaw = (float)rawAccelY / ACCEL_SCALE;
        float azRaw = (float)rawAccelZ / ACCEL_SCALE;

        float gxRaw = (float)rawGyroX / GYRO_SCALE;
        float gyRaw = (float)rawGyroY / GYRO_SCALE;
        float gzRaw = (float)rawGyroZ / GYRO_SCALE;

        float aMagnitude = sqrt(
            axRaw * axRaw +
            ayRaw * ayRaw +
            azRaw * azRaw
        );

        // Abort if the accelerometer magnitude leaves the stillness range.
        if (fabs(aMagnitude - 1.0f) > STILL_ACCEL_TOLERANCE_G)
        {
            Serial.println("Movement detected. IMU calibration aborted.");
            return false;
        }

        sumAx += axRaw;
        sumAy += ayRaw;
        sumAz += azRaw;

        sumGx += gxRaw;
        sumGy += gyRaw;
        sumGz += gzRaw;

        delay(2);
    }

    float avgAx = sumAx / IMU_CALIBRATION_SAMPLES;
    float avgAy = sumAy / IMU_CALIBRATION_SAMPLES;
    float avgAz = sumAz / IMU_CALIBRATION_SAMPLES;

    // Do NOT subtract the gravity components from X/Y/Z.
    // Their direction is needed to determine the initial tilt.
    accelBiasX_g = 0.0f;
    accelBiasY_g = 0.0f;
    accelBiasZ_g = 0.0f;

    gyroBiasX_dps = sumGx / IMU_CALIBRATION_SAMPLES;
    gyroBiasY_dps = sumGy / IMU_CALIBRATION_SAMPLES;
    gyroBiasZ_dps = sumGz / IMU_CALIBRATION_SAMPLES;

    // Initialize the body-to-world quaternion from the measured
    // stationary gravity direction, aligning it with world +Z.
    float accelNorm = sqrt(
        avgAx * avgAx + avgAy * avgAy + avgAz * avgAz
    );

    if (accelNorm < 0.001f)
        return false;

    float ax = avgAx / accelNorm;
    float ay = avgAy / accelNorm;
    float az = avgAz / accelNorm;

    q0 = 1.0f + az;
    q1 = ay;
    q2 = -ax;
    q3 = 0.0f;

    float qNorm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);

    // Handle the exceptional case where the sensor is upside down.
    if (qNorm < 0.001f)
    {
        q0 = 0.0f;
        q1 = 1.0f;
        q2 = 0.0f;
        q3 = 0.0f;
    }
    else
    {
        q0 /= qNorm;
        q1 /= qNorm;
        q2 /= qNorm;
        q3 /= qNorm;
    }

    roll_deg = atan2(
        2.0f * (q0*q1 + q2*q3),
        1.0f - 2.0f * (q1*q1 + q2*q2)
    ) * 180.0f / PI;

    float sinPitch = 2.0f * (q0*q2 - q3*q1);
    sinPitch = constrain(sinPitch, -1.0f, 1.0f);
    pitch_deg = asin(sinPitch) * 180.0f / PI;

    Serial.println("IMU calibration complete.");
    Serial.print("Initial roll: ");
    Serial.println(roll_deg, 3);
    Serial.print("Initial pitch: ");
    Serial.println(pitch_deg, 3);
    Serial.print("Gyro X bias: ");
    Serial.println(gyroBiasX_dps, 6);
    Serial.print("Gyro Y bias: ");
    Serial.println(gyroBiasY_dps, 6);
    Serial.print("Gyro Z bias: ");
    Serial.println(gyroBiasZ_dps, 6);

    return true;
}

// quaternion attitude estimation

// Mahony 6-axis IMU filter:
// gyroscope integration propagates the quaternion, while the
// accelerometer corrects roll/pitch drift when its magnitude is
// consistent with gravity. No magnetometer is installed, so yaw
// is not referenced to magnetic north and can drift over time.

void calculateOrientation(float dt)
{
    if (dt <= 0.0f)
        return;

    float gx = gyroX_dps * PI / 180.0f;
    float gy = gyroY_dps * PI / 180.0f;
    float gz = gyroZ_dps * PI / 180.0f;

    float accelMagnitude = sqrt(
        accelX_g * accelX_g +
        accelY_g * accelY_g +
        accelZ_g * accelZ_g
    );

    // Apply gravity-direction feedback only when the accelerometer
    // is likely measuring gravity rather than substantial flight force.
    if (fabs(accelMagnitude - 1.0f) <= ACCEL_GRAVITY_TOLERANCE_G &&
        accelMagnitude > 0.001f)
    {
        float ax = accelX_g / accelMagnitude;
        float ay = accelY_g / accelMagnitude;
        float az = accelZ_g / accelMagnitude;

        // Estimated world-up direction expressed in body coordinates.
        float estimatedAx = 2.0f * (q1*q3 - q0*q2);
        float estimatedAy = 2.0f * (q0*q1 + q2*q3);
        float estimatedAz = 1.0f - 2.0f * (q1*q1 + q2*q2);

        // Cross product between measured and estimated gravity directions.
        float errorX = ay * estimatedAz - az * estimatedAy;
        float errorY = az * estimatedAx - ax * estimatedAz;
        float errorZ = ax * estimatedAy - ay * estimatedAx;

        gx += MAHONY_KP * errorX;
        gy += MAHONY_KP * errorY;
        gz += MAHONY_KP * errorZ;
    }

    // Integrate angular rate using q_dot = 0.5 * q * omega.
    float halfDt = 0.5f * dt;

    float nextQ0 = q0 + (-q1*gx - q2*gy - q3*gz) * halfDt;
    float nextQ1 = q1 + ( q0*gx + q2*gz - q3*gy) * halfDt;
    float nextQ2 = q2 + ( q0*gy - q1*gz + q3*gx) * halfDt;
    float nextQ3 = q3 + ( q0*gz + q1*gy - q2*gx) * halfDt;

    q0 = nextQ0;
    q1 = nextQ1;
    q2 = nextQ2;
    q3 = nextQ3;

    // Normalize to prevent numerical growth away from unit length.
    float qNorm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);

    if (qNorm > 0.000001f)
    {
        q0 /= qNorm;
        q1 /= qNorm;
        q2 /= qNorm;
        q3 /= qNorm;
    }

    // Convert the normalized quaternion to Euler angles for the CSV.
    roll_deg = atan2(
        2.0f * (q0*q1 + q2*q3),
        1.0f - 2.0f * (q1*q1 + q2*q2)
    ) * 180.0f / PI;

    float sinPitch = 2.0f * (q0*q2 - q3*q1);
    sinPitch = constrain(sinPitch, -1.0f, 1.0f);
    pitch_deg = asin(sinPitch) * 180.0f / PI;
}

// calculate vertical acceleration

// The accelerometer measures specific force.

// At rest, the accelerometer measures approximately +1g
// in the rocket's vertical Z direction.

// We rotate the measured acceleration into the vertical
// direction and subtract gravity.


void calculateAcceleration()
{
    float ax =
        accelX_g *
        GRAVITY;

    float ay =
        accelY_g *
        GRAVITY;

    float az =
        accelZ_g *
        GRAVITY;

    // Raw body Z acceleration

    rawAccelZ_ms2 =
        az;

    // Simple Z-only linear acceleration

    linearAccelZ_ms2 =
        (accelZ_g - 1.0f) *
        GRAVITY;

    // Rotate body acceleration into the world vertical axis
    // using the same body-to-world quaternion as the attitude filter.

    float verticalSpecificForce =
        2.0f * (q1*q3 - q0*q2) * ax +
        2.0f * (q0*q1 + q2*q3) * ay +
        (1.0f - 2.0f * (q1*q1 + q2*q2)) * az;

    // Remove gravity

    verticalAccel_ms2 =
        verticalSpecificForce -
        GRAVITY;

    // Remove measured stationary bias

    float correctedAcceleration =
        verticalAccel_ms2 -
        verticalAccelBias_ms2;

    // Low-pass filter

    filteredVerticalAccel_ms2 =
        ACCEL_FILTER_ALPHA *
        correctedAcceleration
        +
        (1.0f - ACCEL_FILTER_ALPHA) *
        filteredVerticalAccel_ms2;

    // IMPORTANT:
    
    // There is intentionally NO "if < X then set to zero"
    // here. Small real acceleration values are retained.
}

// vertical acceleration calibration

bool calibrateVerticalAcceleration()
{
    Serial.println();
    Serial.println("=================================");
    Serial.println("   VERTICAL ACCEL CALIBRATION");
    Serial.println("=================================");

    double sumAcceleration = 0.0;

    for (int i = 0; i < VERTICAL_CALIBRATION_SAMPLES; i++)
    {
        if (!readMPU())
            return false;

        float aMagnitude = sqrt(
            accelX_g * accelX_g +
            accelY_g * accelY_g +
            accelZ_g * accelZ_g
        );

        if (fabs(aMagnitude - 1.0f) > STILL_ACCEL_TOLERANCE_G)
        {
            Serial.println("Movement detected. Vertical calibration aborted.");
            return false;
        }

        float accelRoll = atan2(accelY_g, accelZ_g);
        float accelPitch = atan2(
            -accelX_g,
            sqrt(accelY_g * accelY_g + accelZ_g * accelZ_g)
        );

        float ax = accelX_g * GRAVITY;
        float ay = accelY_g * GRAVITY;
        float az = accelZ_g * GRAVITY;

        float verticalSpecificForce =
            -ax * sin(accelPitch) +
            ay * sin(accelRoll) * cos(accelPitch) +
            az * cos(accelRoll) * cos(accelPitch);

        sumAcceleration += verticalSpecificForce - GRAVITY;
        delay(2);
    }

    verticalAccelBias_ms2 =
        sumAcceleration / VERTICAL_CALIBRATION_SAMPLES;

    filteredVerticalAccel_ms2 = 0.0f;

    Serial.print("Vertical acceleration bias: ");
    Serial.print(verticalAccelBias_ms2, 6);
    Serial.println(" m/s^2");
    Serial.println("Vertical acceleration calibration complete.");

    return true;
}

// csv header

void writeCSVHeader()
{
    dataFile.println(
        "RecordType,Event,Time_s,"
        "AccelX_g,AccelY_g,AccelZ_g,"
        "RawAccelZ_ms2,LinearAccelZ_ms2,"
        "VerticalAccel_ms2,FilteredVerticalAccel_ms2,"
        "Roll_deg,Pitch_deg,"
        "GyroX_dps,GyroY_dps,GyroZ_dps,FlightTime_s"
    );
}

// write data to sd

void writeData()
{
    if (!dataFile)
        return;

    float time_s = (millis() - startTimeMillis) / 1000.0f;

    dataFile.print("DATA,DATA,");
    dataFile.print(time_s, 3);

    dataFile.print(","); dataFile.print(accelX_g, 6);
    dataFile.print(","); dataFile.print(accelY_g, 6);
    dataFile.print(","); dataFile.print(accelZ_g, 6);
    dataFile.print(","); dataFile.print(rawAccelZ_ms2, 6);
    dataFile.print(","); dataFile.print(linearAccelZ_ms2, 6);
    dataFile.print(","); dataFile.print(verticalAccel_ms2, 6);
    dataFile.print(","); dataFile.print(filteredVerticalAccel_ms2, 6);
    dataFile.print(roll_deg, 6);
    dataFile.print(","); dataFile.print(pitch_deg, 6);
    dataFile.print(","); dataFile.print(gyroX_dps, 6);
    dataFile.print(","); dataFile.print(gyroY_dps, 6);
    dataFile.print(","); dataFile.print(gyroZ_dps, 6);
    dataFile.print(",");

    if (launchDetected)
        dataFile.print((millis() - launchMillis) / 1000.0f, 3);
    else
        dataFile.print(-1.0f);

    dataFile.println();
}

// launch detection

void updateLaunchDetection()
{
    // Do not evaluate launch acceleration until the full arming
    // countdown has completed. Ignore and clear any pre-arm samples.
    if (!armingComplete || launchDetected)
    {
        launchCandidateSamples = 0;
        return;
    }

    float accelerationMagnitude = sqrt(
        accelX_g * accelX_g +
        accelY_g * accelY_g +
        accelZ_g * accelZ_g
    );

    if (accelerationMagnitude >= LAUNCH_ACCEL_THRESHOLD_G)
    {
        if (launchCandidateSamples == 0)
            launchCandidateMillis = millis();

        launchCandidateSamples++;

        if (launchCandidateSamples >= LAUNCH_CONFIRM_SAMPLES)
        {
            launchDetected = true;
            launchMillis = launchCandidateMillis;
            setFlightStatus(STATUS_LAUNCH);

            writeEvent("LAUNCH_DETECTED");

            Serial.println();
            Serial.println("*** LAUNCH DETECTED ***");
        }
    }
    else
    {
        launchCandidateSamples = 0;
    }
}

// serial telemetry

// This is intentionally separate from SD logging.

// SD = ~100 Hz
// Serial = 10 Hz


void printSerialTelemetry()
{
    if (millis() - lastSerialMillis <
        SERIAL_INTERVAL_MS)
    {
        return;
    }

    lastSerialMillis = millis();

    float time_s =
        (millis() - startTimeMillis)
        / 1000.0f;

    Serial.print("T=");
    Serial.print(time_s, 1);

    Serial.print("s | Accel=");
    Serial.print(
        filteredVerticalAccel_ms2,
        3
    );

    Serial.print("m/s2 | Roll=");
    Serial.print(
        roll_deg,
        2
    );

    Serial.print(" | Pitch=");
    Serial.print(
        pitch_deg,
        2
    );

    Serial.println();
}

// stillness qualification

bool currentSampleIsStill()
{
    if (!readMPU())
        return false;

    float ax = (float)rawAccelX / ACCEL_SCALE;
    float ay = (float)rawAccelY / ACCEL_SCALE;
    float az = (float)rawAccelZ / ACCEL_SCALE;

    float aMagnitude = sqrt(ax * ax + ay * ay + az * az);

    return fabs(aMagnitude - 1.0f) <= STILL_ACCEL_TOLERANCE_G;
}

void waitForStillness()
{
    setFlightStatus(STATUS_CALIBRATION);
    Serial.println();
    Serial.println("Waiting for 5 continuous seconds of accelerometer stillness...");

    unsigned long stillStart = 0;
    bool timingStillness = false;

    while (true)
    {
        bool still = currentSampleIsStill();

        if (still)
        {
            if (!timingStillness)
            {
                stillStart = millis();
                timingStillness = true;
                Serial.println("Stillness detected. Keep rocket motionless.");
            }

            if (millis() - stillStart >= STILLNESS_REQUIRED_MS)
            {
                Serial.println("Stillness confirmed.");
                return;
            }
        }
        else
        {
            if (timingStillness)
                Serial.println("Movement detected. Stillness timer reset.");

            timingStillness = false;
        }

        delay(10);
    }
}

// setup countdown

void runSetupCountdown()
{
    setFlightStatus(STATUS_SETUP_DELAY);
    Serial.println();
    Serial.println(
        "================================="
    );
    Serial.println(
        "        SETUP COUNTDOWN"
    );
    Serial.println(
        "================================="
    );

    Serial.println(
        "The computer will wait 120 seconds."
    );

    Serial.println(
        "Use this time to install the avionics,"
    );

    Serial.println(
        "close the bay, and stand the rocket upright."
    );

    Serial.println();

    unsigned long countdownStart =
        millis();

    lastSetupSecond = -1;

    while (
        millis() - countdownStart
        <
        SETUP_DELAY_MS
    )
    {
        unsigned long elapsed =
            millis() -
            countdownStart;

        unsigned long remaining =
            SETUP_DELAY_MS -
            elapsed;

        int remainingSeconds =
            remaining / 1000UL;

        if (
            remainingSeconds !=
            lastSetupSecond
        )
        {
            lastSetupSecond =
                remainingSeconds;

            Serial.print(
                "SETUP TIME REMAINING: "
            );

            Serial.print(
                remainingSeconds / 60
            );

            Serial.print(" min ");

            Serial.print(
                remainingSeconds % 60
            );

            Serial.println(" sec");
        }

        updateStatusLED();
        delay(50);
    }

    Serial.println();
    Serial.println(
        "SETUP COUNTDOWN COMPLETE."
    );
    Serial.println();
}

// arming countdown

void runArmingCountdown()
{
    Serial.println();
    Serial.println(
        "================================="
    );
    Serial.println(
        "        ARMING COUNTDOWN"
    );
    Serial.println(
        "================================="
    );

    Serial.println(
        "Sensor data is being recorded."
    );

    Serial.println(
        "MPU6050 inertial data is being recorded."
    );

    Serial.println();

    unsigned long armingStart =
        millis();

    lastArmingSecond = -1;

    while (
        millis() - armingStart
        <
        ARMING_DELAY_MS
    )
    {
        unsigned long elapsed =
            millis() -
            armingStart;

        unsigned long remaining =
            ARMING_DELAY_MS -
            elapsed;

        int remainingSeconds =
            remaining / 1000UL;

        if (
            remainingSeconds !=
            lastArmingSecond
        )
        {
            lastArmingSecond =
                remainingSeconds;

            Serial.print(
                "ARMING TIME REMAINING: "
            );

            Serial.print(
                remainingSeconds / 60
            );

            Serial.print(" min ");

            Serial.print(
                remainingSeconds % 60
            );

            Serial.println(" sec");
        }

        delay(50);
    }

    Serial.println();
    Serial.println(
        "================================="
    );

    Serial.println(
        "       FLIGHT COMPUTER ARMED"
    );

    Serial.println(
        "================================="
    );

    Serial.println();
}

// select unused csv file

void selectDataFilename()
{
    for (int fileNumber = 1; fileNumber <= 999; fileNumber++)
    {
        snprintf(
            DATA_FILENAME,
            sizeof(DATA_FILENAME),
            "/flight_%03d.csv",
            fileNumber
        );

        if (!SD.exists(DATA_FILENAME))
            return;
    }

    Serial.println("FATAL ERROR: No unused flight filename available.");
    setFlightStatus(STATUS_ERROR);
    while (true)
    {
        updateStatusLED();
        delay(10);
    }
}

// setup

void setup()
{
    initializeStatusLED();
    updateStatusLED();

    Serial.begin(115200);

    delay(1000);

    Wire.begin();

    Wire.setClock(400000);

    startTimeMillis =
        millis();

    Serial.println();
    Serial.println(
        "================================="
    );
    Serial.println(
        "       ROCKET FLIGHT LOGGER"
    );
    Serial.println(
        "        MPU6050 + microSD"
    );
    Serial.println(
        "================================="
    );
    Serial.println();

    // MPU6050

    if (!setupMPU())
    {
        Serial.println(
            "FATAL ERROR: MPU6050 not detected."
        );

        setFlightStatus(STATUS_ERROR);
        while (true)
        {
            updateStatusLED();
            delay(10);
        }
    }

    Serial.println(
        "MPU6050 initialized."
    );

    // SD CARD

    SPI.begin();

    if (!SD.begin(SD_CS_PIN, SPI))
    {
        Serial.println(
            "FATAL ERROR: SD card initialization failed."
        );

        setFlightStatus(STATUS_ERROR);
        while (true)
        {
            updateStatusLED();
            delay(10);
        }
    }

    Serial.println(
        "SD card initialized."
    );

    // Select a new filename so previous flight data is never erased.
    selectDataFilename();

    Serial.print("Logging to: ");
    Serial.println(DATA_FILENAME);

    dataFile =
        SD.open(
            DATA_FILENAME,
            FILE_WRITE
        );

    if (!dataFile)
    {
        Serial.println(
            "FATAL ERROR: Cannot create flight_data.csv."
        );

        setFlightStatus(STATUS_ERROR);
        while (true)
        {
            updateStatusLED();
            delay(10);
        }
    }

    writeCSVHeader();

    dataFile.flush();

    writeEvent("POWER_ON");

    // 120 SECOND SETUP COUNTDOWN

    writeEvent("SETUP_DELAY_BEGIN");

    runSetupCountdown();

    writeEvent("SETUP_DELAY_COMPLETE");

    // WAIT FOR STILLNESS AND CALIBRATE

    bool calibrationComplete = false;

    while (!calibrationComplete)
    {
        waitForStillness();

        // Calibrate the IMU after the setup delay, once the rocket
        // has been placed in its intended upright launch position.
        writeEvent("IMU_CALIBRATION_BEGIN");

        bool imuCalibrated = calibrateIMU();

        if (!imuCalibrated)
        {
            writeEvent("CALIBRATION_ABORTED_MOVEMENT");
            continue;
        }

        writeEvent("IMU_CALIBRATION_COMPLETE");

        writeEvent("VERTICAL_ACCEL_CALIBRATION_BEGIN");

        bool verticalCalibrated = calibrateVerticalAcceleration();

        if (!verticalCalibrated)
        {
            writeEvent("CALIBRATION_ABORTED_MOVEMENT");
            continue;
        }

        // Confirm that the rocket stayed still throughout all
        // calibration steps. If not, repeat the complete sequence.
        if (!currentSampleIsStill())
        {
            writeEvent("CALIBRATION_ABORTED_MOVEMENT");
            Serial.println("Movement detected after calibration. Restarting.");
            continue;
        }

        calibrationComplete = true;
    }

    writeEvent("ALL_CALIBRATION_COMPLETE");

    // RESET FILTERED VERTICAL ACCELERATION

    filteredVerticalAccel_ms2 = 0.0f;

    // START DATA COLLECTION

    writeEvent(
        "DATA_COLLECTION_BEGIN"
    );

    setFlightStatus(STATUS_LOGGING);

    Serial.println();
    Serial.println(
        "================================="
    );
    Serial.println(
        "       DATA COLLECTION STARTED"
    );
    Serial.println(
        "================================="
    );

    Serial.println(
        "SD logging: approximately 100 Hz"
    );

    Serial.println(
        "Serial telemetry: 10 Hz"
    );
    Serial.println(
        "Altitude measurement: unavailable (no barometer)"
    );

    Serial.println(
        "Barometer/Kalman fusion: REMOVED"
    );
    Serial.println(
        "Velocity: calculated externally"
    );

    Serial.println();

    // BEGIN LOGGING AND NON-BLOCKING ARMING PERIOD

    armingStartMillis = millis();
    armingComplete = false;

    Serial.println("120-second arming period started.");
    Serial.println("Sensor logging is active.");
    Serial.println("Launch detection is DISABLED until arming completes.");

    dataFile.flush();

    lastLogMicros = micros();
    lastFlushMillis = millis();
    lastSerialMillis = millis();

}

// loop

void loop()
{
    updateStatusLED();

    unsigned long nowMicros =
        micros();

    // 100 Hz SENSOR / SD LOOP

    if (
        (unsigned long)
        (nowMicros - lastLogMicros)
        >=
        LOG_INTERVAL_US
    )
    {
        unsigned long elapsedMicros =
            nowMicros -
            lastLogMicros;

        lastLogMicros =
            nowMicros;

        float dt =
            elapsedMicros /
            1000000.0f;

        // Protect against abnormal timing

        if (
            dt <= 0.0f ||
            dt > 0.1f
        )
        {
            dt =
                LOG_INTERVAL_US /
                1000000.0f;
        }
    
        // Read sensors

        bool imuOK = readMPU();

        if (!imuOK)
        {
            Serial.println("WARNING: MPU6050 read failed.");
        }
        else
        {
            // Launch detection
            updateLaunchDetection();

            // Orientation and vertical acceleration
            calculateOrientation(dt);
            calculateAcceleration();

            // SD logging

            writeData();
        }
    }

    // NON-BLOCKING ARMING TIMER

    if (!armingComplete &&
        millis() - armingStartMillis >= ARMING_DELAY_MS)
    {
        armingComplete = true;
        if (!launchDetected)
            setFlightStatus(STATUS_ARMED);
        writeEvent("ARMING_DELAY_COMPLETE");
        dataFile.flush();

        Serial.println();
        Serial.println("=================================");
        Serial.println("       FLIGHT COMPUTER ARMED");
        Serial.println("=================================");
    }

    // SERIAL TELEMETRY

    printSerialTelemetry();

    // SD FLUSH

    if (
        millis() -
        lastFlushMillis
        >=
        SD_FLUSH_MS
    )
    {
        lastFlushMillis =
            millis();

        dataFile.flush();
    }
}