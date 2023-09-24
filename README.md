# MR24HPC1 radar library

Arduino Library for [Seeed MR24HPC1 radar](https://www.seeedstudio.com/24GHz-mmWave-Sensor-Human-Static-Presence-Module-Lite-p-5524.html)

## Initialice radar

```c++
Radar_MR24HPC1 radar = Radar_MR24HPC1(&Serial1);
```

## set_mode()

Set radar to simple mode:

```c++
radar.set_mode(SIMPLE);
```

Set radar to advandced mode:

```c++
radar.set_mode(ADVANCED);
```

## get_mode()

Returns rada mode:

- 0 == SIMPLE
- 1 == ADVANCED

```c++
if (radar.get_mode() == ADVANCED) {
  Serial.println("Advandced mode");
} else {
  Serial.println("Simple mode");
}
```

## run()

In the main loop should execute _radar.run()_ command.

##### NONVERBAL

In the background. Nothing will printed on the serial monitor.

```c++
void loop() {
  radar.run(NONVERBAL);
}
```

##### VERBAL

In SIMPLE mode:

```c++
void setup() {
  radar.set_mode(SIMPLE);
}
void loop() {
  radar.run(VERBAL);
}
```

![](img/run_simple_verbal.png)

In ADVANCED mode:

```c++
void setup() {
  radar.set_mode(ADVANCED);
}
void loop() {
  radar.run(VERBAL);
}
```

![](img/run_advanced_verbal.png)

## get_heartbeat()

Returns heartbeat counter value. Changes once a minute.

```c++
Serial.print(radar.get_heartbeat());
```

## get_activity()

Works only in SIMPLE mode.
Returns activity value from 0 to 250.

```c++
Serial.println(radar.get_activity());
```

## get_direction()

Works only in SIMPLE mode.
Returns 1 (APPROACHING) when human body moves closer.
Returns 2 (RECEDING) when human body moves away from radar.
Otherwise returns 0.

```c++
if (radar.get_direction() == APPROACHING) {
  Serial.println("Moves closer");
} else if (radar.get_direction() == RECEDING) {
  Serial.println("Moves away");
}
```
