/* LSM6DSV320X_AI_Core - Stream a MEMS Studio model and read AI outputs. */
#include <LSM6DSV320X.h>

LSM6DSV320X imu;

bool applyModelLine(const String& line) {
  if (line.length() < 1) return true;
  char* end = nullptr;
  const char* start = line.c_str();
  uint8_t operation = static_cast<uint8_t>(strtoul(start, &end, 10));
  if (end == start || operation > 4) return false;
  start = end;
  uint8_t address = static_cast<uint8_t>(strtoul(start, &end, 16));
  if (end == start) return false;
  start = end;
  uint8_t data = static_cast<uint8_t>(strtoul(start, &end, 16));
  if (end == start) return false;
  LSM6DSV320X::UcfLineExtended command = {
      static_cast<LSM6DSV320X::UcfOperation>(operation), address, data};
  return imu.loadUcfExtended(&command, 1);
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  if (!imu.begin()) {
    Serial.println("LSM6DSV320X startup failed.");
    while (true) {}
  }
  Serial.println("Send: op address data; op=0..4. Send X when complete.");
  while (true) {
    if (!Serial.available()) continue;
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line == "X") break;
    if (!applyModelLine(line)) Serial.println("Model line failed.");
  }
  if (!imu.setFsmEnabled(0xFF) ||
      !imu.setMlcEnabled() ||
      !imu.enableAdaptiveSelfConfiguration()) {
    Serial.println("AI-core enable failed.");
    while (true) {}
  }
}

void loop() {
  uint8_t fsm = 0;
  uint8_t mlc = 0;
  if (!imu.readFsmOutput(0, fsm) || !imu.readMlcOutput(0, mlc)) return;
  Serial.print("FSM1="); Serial.print(fsm);
  Serial.print(" MLC1="); Serial.print(mlc);
  Serial.print(" ASC="); Serial.println(imu.adaptiveSelfConfigurationActive());
  delay(10);
}
