# Tokyo Clamp

## Wiring

### XY-160D motor driving board

**Driver input voltage:** DC 6.5V - 27V
**Number of Channels:** 2
**Rated output current:**  7A (Per Channel)
**PWM frequency range** 0-10 kHz (PWM signal at ENA input is used to regulate speed)

| Pin Function                    | Label          | Connection         | Cable Color | Arduino Pin | Name in Code      |
| ------------------------------- | -------------- | ------------------ | ----------- | ----------- | ----------------- |
| Drive Power Input +ve           | 9-24V          | Battery Positive   |             |             |                   |
| Drive Power Input -ve           | PGND           | Battery Negative   |             |             |                   |
| Power Output - Channel 1 +ve    | OUT1           | M1 +ve             | Red         |             |                   |
| Power Output - Channel 1 -ve    | OUT2           | M1 -ve             | Wht         |             |                   |
| Power Output - Channel 2 +ve    | OUT3           |                    |             |             |                   |
| Power Output - Channel 2 -ve    | OUT4           |                    |             |             |                   |
| Digital Power Input +ve         | +5V            | Arduino 5V         | Blu / Grn   | 5V          |                   |
| Digital Power Input -ve         | GND            | Arduino Ground     | Blk / Brn   | GND         |                   |
| Digital Input - Channel 1 - 1   | IN1            | Arduino GPIO       | Gry         | 8           | m1_driver_in1_pin |
| Digital Input - Channel 1 - 2   | IN2            | Arduino GPIO       | Wht         | 7           | m1_driver_in2_pin |
| Digital Input - Channel 2 - 1   | IN3            | Arduino GPIO       | Org         |             |                   |
| Digital Input - Channel 2 - 2   | IN4            | Arduino GPIO       | Red         |             |                   |
| Digital Input - Channel 1 Speed | ENA (near IN1) | Arduino GPIO (PWM) | Pur         | 9           | m1_driver_ena_pin |
| Digital Input - Channel 2 Speed | ENA (near IN3) | Arduino GPIO (PWM) | Yel         |             |                   |

### GW4058-555 Worm Gearbox 1:54

The integrated hall sensor on the 555 DC motors have 11 steps per rev per channel. Effectively **44 steps per rev**. (100kHz max)

| Pin Function    | Label | Connection  | Cable Color | Arduino Pin | Name in Code    |
| --------------- | ----- | ----------- | ----------- | ----------- | --------------- |
| Encoder         | C1    | Arduino     | Yel         | 2           | m1_encoder1_pin |
| Encoder         | C2    | Arduino     | Grn         | 3           | m1_encoder2_pin |
| Encoder Power + | VCC   | Arduino     | Blu         | 5V          |                 |
| Encoder Power - | GND   | Arduino     | Blk         | Gnd         |                 |
| Motor Power     | M1    | Driver OUT1 | Red         |             |                 |
| Motor Power     | M2    | Driver OUT2 | Wht         |             |                 |

### Homing switch

Utilizing Arduino's Internal pull up resistor (47k). 

Normal closed behavior used. (Broken wire can be detected)

100nF capacitor is added as noise filter between NC and GND, close to the Arduino.

| Pin Function | Label | Connection | Cable Color | Arduino Pin | Name in Code    |
| ------------ | ----- | ---------- | ----------- | ----------- | --------------- |
| COM          | 1     | Arduino    | Blk         | GND         | m1_encoder1_pin |
| NC           | 2     | Arduino    | Red Stripe  | A6          | m1_encoder2_pin |
| NO           | 3     |            |             |             |                 |

### Battery Sense and Regulation

| Pin Function | Label | Connection | Cable Color | Arduino Pin | Name in Code        |
| ------------ | ----- | ---------- | ----------- | ----------- | ------------------- |
| COM          |       | Arduino    | Blk         | GND         |                     |
| Input        |       | Battery +  | Red Stripe  |             |                     |
| Output       |       | Arduino    | Red Stripe  | A7          | battery_monitor_pin |

Battery sense is a simple Voltage Divider

R1 = 47kOhm

R2 = 20kOhm

Theoretical Voltage at 16.8V = 5.01V (1024/1024)

Theoretical Voltage at 14.4V = 4.30V (880/1024)

Theoretical Resolution from 0 to 100% = 144 steps 

According to some questionable source:

4.20v = 100%
4.03v = 76%
3.86v = 52%
3.83v = 42%
3.79v = 30%
3.70v = 11%
3.6?v = 0%