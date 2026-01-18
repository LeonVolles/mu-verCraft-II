// Magnetometer calibration sketch (one-time upload)
// ==============================================
//
// Goal
// ----
// This program measures min/max raw magnetometer readings while you rotate the craft and
// prints the 3 hard-iron offsets (X/Y/Z) you should paste into hovercraft variables.
//
//
// How to use (recommended)
// ------------------------
// 1) In platformio.ini, temporarily add the build flag:
//      -D MAGNETOMETER_CALIBRATION
//    Example:
//      build_flags =
//        -D SDA_PIN=5
//        -D SCL_PIN=6
//        -D MAGNETOMETER_CALIBRATION
//
// 2) Upload as usual.
// 3) Open Serial Monitor at 115200 baud.
// 4) Follow the on-screen rotation instructions.
// 5) Copy the printed lines like:
//      const int16_t global_MagOffsetX = ...;
//      const int16_t global_MagOffsetY = ...;
//      const int16_t global_MagOffsetZ = ...;
//    into src/hovercraft_variables.cpp (replace the existing values).
//
// Safety / quality tips
// ---------------------
// - Keep motors DISARMED / props removed (recommended).
// - Keep strong magnets, steel tools, and large batteries/ESC current loops away.
// - Do the calibration at the final mounting location of the sensor.
// - Move slowly and smoothly; fast motion can reduce sample quality.
//
// What this calibration does
// --------------------------
// Hard-iron calibration assumes the ideal magnetic field forms a sphere centered at (0,0,0).
// Nearby permanent magnets / DC currents shift the sphere center. We estimate that center by:
//   offset = (min + max) / 2  for each axis.
// This is exactly what your current IMU code expects (offsets in raw sensor units).

#include <Arduino.h>
#include <Wire.h>

// Uses the project's existing local magnetometer library (QMC/HMC/VCM5883L compatible)
#include <DFRobot_QMC5883.h>

static constexpr uint32_t SERIAL_BAUD = 115200;

static void waitForSerial()
{
  uint32_t start = millis();
  while (!Serial && (millis() - start) < 4000)
  {
    delay(10);
  }
}

static void drainSerial()
{
  while (Serial.available() > 0)
  {
    (void)Serial.read();
  }
}

static void waitForUserLine(const char *prompt)
{
  Serial.println();
  Serial.println(prompt);
  Serial.println("Type anything + ENTER to continue...");
  drainSerial();

  String line;
  while (true)
  {
    while (Serial.available() == 0)
    {
      delay(10);
    }
    line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() > 0)
      break;
  }
}

static void i2cScan()
{
  Serial.println();
  Serial.println("I2C scan (7-bit addresses):");
  uint8_t found = 0;
  for (uint8_t addr = 1; addr < 127; addr++)
  {
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();
    if (err == 0)
    {
      Serial.printf("  - Found device at 0x%02X\n", addr);
      found++;
    }
  }
  if (found == 0)
  {
    Serial.println("  (No I2C devices found)");
  }
}

struct AxisMinMax
{
  int16_t minX = INT16_MAX;
  int16_t minY = INT16_MAX;
  int16_t minZ = INT16_MAX;
  int16_t maxX = INT16_MIN;
  int16_t maxY = INT16_MIN;
  int16_t maxZ = INT16_MIN;

  void update(int16_t x, int16_t y, int16_t z)
  {
    if (x < minX)
      minX = x;
    if (y < minY)
      minY = y;
    if (z < minZ)
      minZ = z;
    if (x > maxX)
      maxX = x;
    if (y > maxY)
      maxY = y;
    if (z > maxZ)
      maxZ = z;
  }
};

static void printLiveStats(const AxisMinMax &mm, uint32_t samples)
{
  Serial.printf("Samples=%lu | X[%d..%d] Y[%d..%d] Z[%d..%d]\n",
                (unsigned long)samples,
                (int)mm.minX, (int)mm.maxX,
                (int)mm.minY, (int)mm.maxY,
                (int)mm.minZ, (int)mm.maxZ);
}

static bool collectFor(DFRobot_QMC5883 &mag, AxisMinMax &mm, uint32_t durationMs, uint32_t samplePeriodMs)
{
  const uint32_t start = millis();
  uint32_t lastPrint = start;
  uint32_t samples = 0;

  while ((millis() - start) < durationMs)
  {
    sVector_t v = mag.readRaw();
    mm.update(v.XAxis, v.YAxis, v.ZAxis);
    samples++;

    uint32_t now = millis();
    if ((now - lastPrint) >= 1000)
    {
      printLiveStats(mm, samples);
      lastPrint = now;
    }

    delay(samplePeriodMs);
  }

  printLiveStats(mm, samples);
  return samples > 10;
}

static int16_t calcOffset(int16_t minV, int16_t maxV)
{
  // Use int32 to avoid overflow (min/max are int16).
  const int32_t sum = (int32_t)minV + (int32_t)maxV;
  // Round to nearest integer.
  if (sum >= 0)
    return (int16_t)((sum + 1) / 2);
  return (int16_t)((sum - 1) / 2);
}

void setup()
{
  Serial.begin(SERIAL_BAUD);
  waitForSerial();

  Serial.println();
  Serial.println("==============================");
  Serial.println("Magnetometer calibration tool");
  Serial.println("==============================");

  Serial.println();
  Serial.println("Board: seeed_xiao_esp32s3 (Arduino)");
#if defined(SDA_PIN) && defined(SCL_PIN)
  Serial.printf("I2C pins from build flags: SDA_PIN=%d, SCL_PIN=%d\n", (int)SDA_PIN, (int)SCL_PIN);
#else
  Serial.println("I2C pins: using Wire.begin() defaults (no SDA_PIN/SCL_PIN macros defined)");
#endif

  Serial.println();
  Serial.println("Before starting:");
  Serial.println("- Motors DISARMED / props removed (recommended)");
  Serial.println("- Keep metal tools/magnets away");
  Serial.println("- Calibrate where the sensor is finally mounted");

  waitForUserLine("When ready, power the craft and connect USB/Serial.");

  // I2C init
#if defined(SDA_PIN) && defined(SCL_PIN)
  Wire.begin((int)SDA_PIN, (int)SCL_PIN);
#else
  Wire.begin();
#endif
  Wire.setClock(400000);
  delay(50);
  i2cScan();

  // Magnetometer init
  DFRobot_QMC5883 mag(&Wire);
  bool ok = false;
  for (int attempt = 1; attempt <= 10; attempt++)
  {
    if (mag.begin())
    {
      ok = true;
      break;
    }
    Serial.printf("Mag init failed (attempt %d/10). Retrying...\n", attempt);
    delay(100);
  }
  if (!ok)
  {
    Serial.println();
    Serial.println("ERROR: Magnetometer not detected.");
    Serial.println("Check:");
    Serial.println("- SDA/SCL wiring (GPIO5/GPIO6 per platformio.ini)");
    Serial.println("- Sensor power (3V3/GND)");
    Serial.println("- The I2C scan above should show at least one device (commonly 0x0D for QMC5883)");
    Serial.println();
    Serial.println("Halting.");
    while (true)
      delay(1000);
  }

  mag.setDeclinationAngle(0.0f);
  Serial.println();
  Serial.println("Magnetometer OK.");

  waitForUserLine("We will now record min/max values. Keep the craft in your hands.");

  AxisMinMax mm;
  const uint32_t stepMs = 20000;      // 20s per step
  const uint32_t periodMs = 20;       // ~50 Hz
  const uint32_t settleDelayMs = 500; // short pause between steps

  Serial.println();
  Serial.println("STEP 1/4 (Yaw): Keep craft LEVEL. Rotate 2-3 full turns around vertical axis (Z).\n");
  collectFor(mag, mm, stepMs, periodMs);
  delay(settleDelayMs);

  Serial.println();
  Serial.println("STEP 2/4 (Roll): Rotate around the LONG axis (X). Make full rotations slowly.\n");
  collectFor(mag, mm, stepMs, periodMs);
  delay(settleDelayMs);

  Serial.println();
  Serial.println("STEP 3/4 (Pitch): Rotate around the SIDE axis (Y). Make full rotations slowly.\n");
  collectFor(mag, mm, stepMs, periodMs);
  delay(settleDelayMs);

  Serial.println();
  Serial.println("STEP 4/4 (All axes): Do a slow 'tumbling' / figure-8 motion so every orientation is seen.\n");
  collectFor(mag, mm, stepMs, periodMs);

  const int16_t offX = calcOffset(mm.minX, mm.maxX);
  const int16_t offY = calcOffset(mm.minY, mm.maxY);
  const int16_t offZ = calcOffset(mm.minZ, mm.maxZ);

  Serial.println();
  Serial.println("====================");
  Serial.println("CALIBRATION RESULTS");
  Serial.println("====================");
  Serial.printf("Raw min/max: X[%d..%d] Y[%d..%d] Z[%d..%d]\n",
                (int)mm.minX, (int)mm.maxX,
                (int)mm.minY, (int)mm.maxY,
                (int)mm.minZ, (int)mm.maxZ);
  Serial.printf("Offsets (hard-iron, raw units): offX=%d, offY=%d, offZ=%d\n", (int)offX, (int)offY, (int)offZ);

  Serial.println();
  Serial.println("Paste these into src/hovercraft_variables.cpp (replace the existing values):");
  Serial.printf("const int16_t global_MagOffsetX = %d;\n", (int)offX);
  Serial.printf("const int16_t global_MagOffsetY = %d;\n", (int)offY);
  Serial.printf("const int16_t global_MagOffsetZ = %d;\n", (int)offZ);

  Serial.println();
  Serial.println("Optional quick check:");
  Serial.println("- After pasting offsets + reflashing normal firmware, headings should drift less.");
  Serial.println("- If min/max ranges are tiny, you did not rotate enough around all axes.");
  Serial.println();
  Serial.println("This tool will now continuously stream corrected magnetometer values.");
  Serial.println("(You can stop here; offsets are printed above.)");

  waitForUserLine("I have noted the offsets. Press ENTER to start streaming corrected values.");
  while (true)
  {
    sVector_t v = mag.readRaw();
    v.XAxis = (int16_t)(v.XAxis - offX);
    v.YAxis = (int16_t)(v.YAxis - offY);
    v.ZAxis = (int16_t)(v.ZAxis - offZ);
    Serial.printf("corrX=%d,corrY=%d,corrZ=%d\n", (int)v.XAxis, (int)v.YAxis, (int)v.ZAxis);
    delay(50);
  }
}

void loop()
{
  // Not used; setup() never returns.
}
