# CS102 Arduino Project

Arduino rover project for CS102.
/*
  EnviroRover — CS102 Arduino Project
  ------------------------------------
  A two-wheel-drive rover that:
    1. reads temperature + humidity from a DHT sensor,
    2. shows the readings on an LED board (WS2812 / NeoPixel 8x8 matrix),
    3. drives around on its two rear gear motors (L293D / L298N driver),
    4. avoids obstacles: an ultrasonic sensor on a servo scans left/right
       whenever something is in the way, and the rover turns toward open space.

  Parts
    - Arduino Uno
    - 2 x hobby gear motors (rear wheels; front is a caster / free wheel)
    - L293D motor driver module (an L298N works with the same code & wiring)
    - HC-SR04 ultrasonic sensor mounted on an SG90-style servo (faces forward)
    - DHT22 (or DHT11) temperature/humidity sensor
    - 8x8 NeoPixel (WS2812) LED board

  Behaviour
    - LED colour = temperature (blue = cool ... green = comfy ... red = hot)
    - LED fill   = humidity     (rows light from the bottom, 0-100 %)
    - Driving    = forward until an obstacle is closer than OBSTACLE_CM, then
                   stop, sweep the servo left/right, and turn toward the side
                   with more room (or spin around if both sides are blocked).
    - The rover STOPS if it gets too hot or the DHT sensor fails.
    - Serial monitor (9600 baud) logs readings, distances and rover state.

  Wiring (Arduino Uno)
    DHT22/DHT11  DATA -> D2   (10k pull-up to 5V if your module has none)
    NeoPixel     DIN  -> D6   (330 ohm in series; 1000 uF cap across 5V/GND)
    Servo        SIG  -> D9   (power the servo from the 5V rail / battery, NOT D9)
    HC-SR04      TRIG -> D12, ECHO -> D13
    Driver       ENA(EN1,2) -> D5 (PWM)   IN1 -> D7   IN2 -> D8     left motor
                 ENB(EN3,4) -> D3 (PWM)   IN3 -> D10  IN4 -> D11    right motor
    Motors on the driver's OUT1/OUT2 (left) and OUT3/OUT4 (right).
    Power the driver from a separate battery pack and join its GND to Arduino GND.
    (Bare L293D chip: the enable pins are 1,2EN and 3,4EN; module labels vary.)

  Pin note: on an Uno the Servo library disables analogWrite() on D9 and D10,
  which is why the motor enables use D5 and D3 instead.

  Libraries (Arduino IDE > Library Manager):
    - "DHT sensor library" by Adafruit  (+ "Adafruit Unified Sensor")
    - "Adafruit NeoPixel"
    - Servo (built in)

Edit the CONFIG section to match ur build.
*/
