#include <Radar_MR24HPC1.h>  // Radar sensor

// Timing
uint64_t prev_millis = 0;  // unsigned long
#define RADAR_INTERVAL 1000
bool ask_radar = false;

// Init RADAR
Radar_MR24HPC1 radar = Radar_MR24HPC1(&Serial1);

int heartbeat = 0;

void setup() {
  Serial.begin(115200);   // Serial print
  Serial1.begin(115200);  // Radar

  while (!Serial1) {
    Serial.println("Radar disconnected");
    delay(100);
  }
  Serial.println("Radar ready");

  //radar.set_mode(SIMPLE);
  radar.set_mode(ADVANCED);

}

void loop() {
  uint64_t current_millis = millis();

  if ((current_millis - prev_millis) >= RADAR_INTERVAL) {
    ask_radar = true;
    prev_millis = current_millis;
  }

  radar.run(VERBAL);
  //radar.run(NONVERBAL);

  if (ask_radar) {
    ask_radar = false;

    heartbeat = radar.get_heartbeat()
    Serial.print("Heartbeat: ");
    Serial.println(heartbeat);

  }

}
