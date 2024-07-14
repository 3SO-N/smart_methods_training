
# Week 3 Electronic Engineering Tasks

## Design and Program an Electronic Circuit with ESP and a Sensor

### Overview

In this task, we will design and program an electronic circuit using an ESP microcontroller and a PIR (Passive Infrared) sensor. The circuit will detect motion using the PIR sensor and light up an LED when motion is detected. The setup and programming will be done using the Wokwi simulation platform.

### Components Used

1. ESP Microcontroller
2. PIR Sensor
3. LED
4. Resistors
5. Breadboard
6. Jumper Wires

### Circuit Design

1. Connect the VCC pin of the PIR sensor to the 3.3V pin of the ESP.
2. Connect the GND pin of the PIR sensor to the GND pin of the ESP.
3. Connect the OUT pin of the PIR sensor to digital pin 15 of the ESP.
4. Connect the anode (long leg) of the LED to digital pin 13 of the ESP through a 220-ohm resistor.
5. Connect the cathode (short leg) of the LED to the GND pin of the ESP.

### Code Explanation

The following code is used to control the circuit:

```cpp
int led = 13;
int pirdata = 15;
int pirstate = LOW;
int value = 0;

void setup() {
  pinMode(led, OUTPUT);
  pinMode(pirdata, INPUT);
  Serial.begin(9600);
}

void loop() {
  value = digitalRead(pirdata);
  if (value == HIGH) {
    digitalWrite(led, HIGH);
    if (pirstate == LOW) {
      Serial.println("Motion detected");
      pirstate = HIGH;
    }
  } else {
    digitalWrite(led, LOW);
    if (pirstate == HIGH) {
      Serial.println("Motion stopped");
      pirstate = LOW;
    }
  }
}
```

### Explanation of Code

1. **Variables Declaration**:
   - `led`: the pin connected to the LED.
   - `pirdata`: the pin connected to the PIR sensor output.
   - `pirstate`: to track the state of motion detection.
   - `value`: to store the sensor reading.

2. **Setup Function**:
   - Set the `led` pin as OUTPUT and `pirdata` pin as INPUT.
   - Initialize serial communication for debugging.

3. **Loop Function**:
   - Read the value from the PIR sensor.
   - If motion is detected (`value == HIGH`), turn on the LED and print "Motion detected" if it was previously off.
   - If no motion is detected (`value == LOW`), turn off the LED and print "Motion stopped" if it was previously on.

### Simulation

You can simulate this circuit using the Wokwi platform. Here is the link to the simulation: [Wokwi Simulation](https://wokwi.com/)

