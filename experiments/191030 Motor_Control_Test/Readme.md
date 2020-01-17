# DC Motor Control Tests

This series of test is part of the software development to perform feedback control for **position and velocity** control for a DC motor with a **Hall effect** **Quadrature Decoder**. 

This also includes development of:

1. **Trapezoidal motion profile controller**  that can better handle acceleration and deceleration. 
2. **Monitoring of jamming condition** by monitoring deviation from motion profile and stop motor power supply to avoid overheating.

Various characterization experiments was performed on different motors of interest.

## Electronics setup

### **Controller**

Arduino Nano with new bootloader (Compiled firmware assumes it is Uno).

### Driver

The driver used is this module :

双路直流电机驱动板模块工业级马达正反转PWM调速L298逻辑7A160W

https://detail.tmall.com/item.htm?id=545415074433&spm=a1z09.2.0.0.32392e8dqyPg5S&_u=jl5jjmhde86 

PCB has markings: **XY-160D**

Posted Spec:

| Spec                    | Value              |
| ----------------------- | ------------------ |
| Power Supply Voltage    | 6.5 - 27V          |
| Current Per Channel     | 7A Max (50A surge) |
| Control Logic Voltage   | 3.0 - 6.5V         |
| PWN Frequency (ENA Pin) | 0 - 10kHz          |
| PCB Size                | 55x55x16mm         |
| Mounting hole size      | 3mm                |

Input Truth Table:

In1 In2 controls Out1 Out2 (Used for testing)

In3 In4 controls Out3 Out4

| ENA  | IN1  | IN2  | OUT1 OUT2 Effect             |
| ---- | ---- | ---- | ---------------------------- |
| x    | 0    | 0    | Short (Breaking)             |
| x    | 1    | 1    | Isolate                      |
| PWM  | 1    | 0    | Rotate (TODO: OUT1 > OUT2 ?) |
| PWM  | 0    | 1    | Rotate (TODO: OUT1 > OUT2 ?) |

### Motor

Various motors are used in the tests. This is to better understand the capabilities of each motor and to ensure the controller software can be used for all of them.

| Motor      | Gearbox         | Encoder step/rev | 12V Rated <br />Torque (Nm) | 12V Stall <br />Torque (Nm) |
| ---------- | --------------- | ---------------- | --------------------------- | --------------------------- |
| 42GP-775   | 1:49 Planetary  | 60               | 3.0                         |                             |
| 36GP-555   | 1:51 Planetary  | 44               | 1.7                         | 3.5                         |
| 36GP-555   | 1:100 Planetary | 44               | 3.0                         | > 5.0                       |
| 36GP-555   | 1:139 Planetary | 44               | 4.0                         | > 5.0                       |
| GW4058-555 | 1:54 Worm       | 44               | 3.0                         | 6.0                         |
| GW4058-555 | 1:72 Worm       | 44               | 4.0                         | 7.0                         |




#### **42GP-775  with 1:49 Gearbox**

https://item.taobao.com/item.htm?spm=a1z09.2.0.0.32392e8dqyPg5S&id=538085471288&_u=jl5jjmh1acb 

![775Spec](../190910_Screw_And_Motor_Test/diagrams/775Spec.jpg)

![775Wiring](../190910_Screw_And_Motor_Test/diagrams/775Wiring.jpg)

![775Gearing](../190910_Screw_And_Motor_Test/diagrams/775Gearing.jpg)

![775Encoder](../190910_Screw_And_Motor_Test/diagrams/775Encoder.jpg)

#### **36GP-555  with 1:51 / 1:100 / 1:139 Gearbox**

 https://item.taobao.com/item.htm?spm=a1z09.2.0.0.32392e8dqyPg5S&id=544216961663&_u=jl5jjmhf0dd 

![555](../190910_Screw_And_Motor_Test/diagrams/555.jpg)

![555Spec](../190910_Screw_And_Motor_Test/diagrams/555Spec.jpg)

![555Wiring](../190910_Screw_And_Motor_Test/diagrams/555Wiring.jpg)

![555Gearing](../190910_Screw_And_Motor_Test/diagrams/555Gearing.jpg)

![555Encoder](../190910_Screw_And_Motor_Test/diagrams/555Encoder.jpg)

#### GW4058-555 Worm Gearbox 1:54 / 1:72

The worm gearbox with the 555 motor was investigated for its space-saving 90-degree power transmission.

![Work_Gearbox_GW4085](../190910_Screw_And_Motor_Test/diagrams/Work_Gearbox_GW4085.jpg)

![Work_Gearbox_GW4085_Dimensions](../190910_Screw_And_Motor_Test/diagrams/Work_Gearbox_GW4085_Dimensions.png)

![Work_Gearbox_GW4085_Spec](../190910_Screw_And_Motor_Test/diagrams/Work_Gearbox_GW4085_Spec.png)

### Connections

Generally the following pins are used between Arduino and Driver / Encoder

```C++
#include "DCMotor.h"
const uint8_t m1_driver_ena_pin = 9;      // the pin the motor driver ENA1 is attached to (PWM Pin)
const uint8_t m1_driver_in1_pin = 8;      // the pin the motor driver IN1 is attached to
const uint8_t m1_driver_in2_pin = 7;      // the pin the motor driver IN2 is attached to
DCMotor Motor1(m1_driver_ena_pin, m1_driver_in1_pin, m1_driver_in2_pin);

#include "Encoder.h"
Encoder myEnc(2, 3);
```

### Encoder Details

The encoder on motor 775 has 15 pulse per rev per channel. The two channels combined have **60 pulse per rev**. The gear box is 1:49. Therefore the combined effect is 15 x 4 x 49 pulse = **2940 pulse per rev**.

The encoder on motor 555 has 11 pulse per rev per channel. The two channels combined have **44 pulse per rev**. The gear box is 1:51. Therefore the combined effect is 11 x 4 x 51 pulse = **2244 pulse per rev**.

-----

## Motor01_Encoder

Test to verify the Encoder library and the encoder on the motors are functional.

### Testing Setup

**Motor Tested:** 555 1:51P , 775 1:49P

**Load:** No load

**Power Supply:** Bench Power Supply 13V

**Measurement:** Taken from Arduino's reading of the encoder

### Procedure

1. 10 turns are observed (eyeballed) , and then power is cut. 
2. Number of steps reported by the Arduino is recorded.

| Motor     | Steps per 10 revolutions | Measured steps |
| --------- | ------------------------ | -------------- |
| 555 1:51P | 22440                    | 22428          |
| 775 1:49P | 29400                    | 29597          |

### Conclusion

Encoder is working alright, at least in one motor scenario. 

Encoder library and Arduino interrupt based counter is working okay. At least when pin (2,3) are used. Both of these pins are interrupt pins. 

### Next Steps

It is unsure in two motor scenario, if the encoder library is still operating fine.  

1. Arduino Nano only has 2 interrupt pins, in 2 motor scenario, each motor will only use one interrupt and one normal IO pin for encoder use.
2. There may be interrupt collision. 

## Motor02_EncoderSpeedTest

This test is copied from Encoder library default example to check if Arduino is catching up with the encoder interrupt speed.

### Testing Setup

**Motor:** 555 1:51P
**Load:** No load
**Electronics:** Arduino micro
**Power Supply:** Bench Power Supply 13V
**Measurement:** Voltage measured with multimeter on Pin 12

### Procedure

13V is applied directly from power supply to simulate the motor at fastest speed.

A voltage is measured from pin output 12 from Arduino.  If voltage drop to zero when encoder is fast, then the Arduino is not catching up.

// Test Result for 555 motor Encoder using 2 inturupt pins myEnc(2,3)
// 0V : 2.22,V
// 12V : 2.13V

// Test Result for 555 motor Encoder using 1 inturupt pins myEnc(2,4)
// 0V : 0.798V
// 12V : 0.781V



### Conclusion

Best case double interrupt `myEnc(2,3)` and single interrupt `myEnc(2,4)` have no problems reading the encoder at the motor's maximum speed.

### Next Steps

It is still unsure in two motor scenario, if the encoder library is still operating fine.  

1. There may be interrupt collision. 

## Motor_03_Voltage_Speed_Relationship

This test checks if the voltage speed relationship is linear.

Various voltage is applied from PSU to motor. These are DC voltage, not PWM controlled voltage.

### Testing setup

**Motor:** 775 1:49P
**Load:** No load
**Electronics:** Arduino micro + XY160D Driver
**Power Supply:** Bench Power Supply - Various DC Voltages
**Measurement:** Taken from Arduino's reading of the encoder

![result](results/Motor_03_VoltageSpeedRelationship/result.jpg)

### Conclusion

DC motor voltage speed relationship is as linear as it gets.

## Motor04_PwmFreqEffect

This test finds the best PWM frequency such that the **PWM to Speed** relationship is as linear as possible, and has a large dynamic range for fine control.

Five PWM values are possible from Arduino settings standpoint, but only the 4 tested values are compatible with the motor driver's 10kHz limit.

### Testing setup

**Motor:** 775 1:49P
**Load:** No load
**Electronics:** Arduino micro + XY160D Driver
**Power Supply:** 12V 2A Wall Adapter
**Measurement:** Taken from Arduino's reading of the encoder

### Result

![result](results/Motor_04_PwmFreqEffect/result.jpg)

![result2](results/Motor_04_PwmFreqEffect/result2.jpg)

### Conclusion

**3921.16Hz** seems to be the best option for dynamic range, although it clearly have a large deadband in the low PWM range. 

Use **3921.16Hz** in future PWM control.

### Next Steps

Perhaps a inner control loop or some offset is necessary to fix this deadband. Especially the later PID controller do not take into account this deadband and will traverse the zero crossing from positive to negative while loosing control.

At the moment, the PWM is set with a call to `DCMotor.setSpeed(pwm)`. Which directly sets the PWM.

The implementation of this function could take a dead band parameter and offset the output pwm by that amount.  The new implementation should distinguish `DCMotor.setPwm(int pwm)` and `DCMotor.setSpeedPercentage(float speedPercent)`

In our clamp application, the load is definitely not fixed. So mapping this response perfectly is not going to be meaningful. Therefore a lookup mapping table, or a regression fit is not attempted here.

## Motor04c_PWM_Different_Motor_Speed

The intention is to find out the different speed profile of different motors. 

To observe the difference in the deadband width.

To observe the maximum rpm for each motor under different voltages.

### Testing setup

**Motor:** 

- 555 + 1:51P (Gearbox might have been overloaded in other tests)
- 555 + 1:100P
- 555 + 1:139 P
- 555 + 1:54 W
- 555 +1:72 W
- 36GP-775 with 1:49 gearbox

**Load:** No load
**Electronics:** Arduino micro + XY160D Driver
**Power Supply:** Bench Power Supply - Various DC Voltages

- 16.8V (Simulated 4 Cell - Fully Charged)
- 14.8V (Simulated 4 Cell - Average)
- 12.6V (Simulated 3 Cell - Fully Charged)
- 11.1V (Simulated 3 Cell - Average)
- LiPo Battery (4 Cell)

**Measurement:** Taken from Arduino's reading of the encoder

### Variables

Different PWM Values and direction

- +255 to -255 (PWM Frequency = 3921.16Hz) (Interval = 10)

### Test Setup

The code used is similar to **Motor04_PwmFreqEffect**. No deadband removal will be used.

Large benchtop power supply is used. Current Limit is 8A (Non current limiting )

### Result

**Comparing different voltage input characteristic:**

The difference in dead-band width is rather small in different voltage. (Less then 10 PWM ticks)



![01_Motor_Voltage_051](Motor04c_PWM_Different_Motor_Speed/data/01_Motor_Voltage_051.jpg)

![01_Motor_Voltage_100](Motor04c_PWM_Different_Motor_Speed/data/01_Motor_Voltage_100.jpg)

![01_Motor_Voltage_139](Motor04c_PWM_Different_Motor_Speed/data/01_Motor_Voltage_139.jpg)

**Comparing motor with different gearbox:**

The no load speed difference between 1:100 (slower) and 1:51 (faster) is not even close to 2.0

This is probably due to load by the loss in gearbox. Thus the 1:51 does not move close to two time the speed.

![02_Motor_Gearbox_12V6](Motor04c_PWM_Different_Motor_Speed/data/02_Motor_Gearbox_12V6.jpg)

![02_Motor_Gearbox_16V8](Motor04c_PWM_Different_Motor_Speed/data/02_Motor_Gearbox_16V8.jpg)

### Implication / Next step

The following chart gives the no load linear speed when using the 1204 screw. Bracket is the advertised value on data sheet.



| Supply voltage [V] | Gear Ratio (Planetary / Worm) | Encoder Speed [step/s] | No load Speed [rev/s] (Spec Sheet) | Linear Speed 1204 Screw [mm/s] |
| ------------------ | ----------------------------- | ---------------------- | ---------------------------------- | ------------------------------ |
| 12.6               | 1:51 P                        | 4465                   | 1.9897 (1.916)                     | 7.959                          |
| **16.8**           | **1:51P**                     | **5941**               | **2.6475**                         | **10.590**                     |
| 12.6               | 1:100P                        | 6686                   | 1.5195 (1.0)                       | 6.0782                         |
| 16.8               | 1:100P                        | 9168                   | 2.0836                             | 8.3345                         |
| 12.6               | 1:54 W                        | 6089                   | 2.5627                             | 10.2508                        |
| 16.8               | 1:54 W                        | 8196                   | 3.4495                             | 13.7979                        |
| 12.6               | 1:72 W                        | 5992                   | 1.8914                             | 7.5656                         |
| 16.8               | 1:72 W                        | 7900                   | 2.4936                             | 9.9747                         |

If we use 50% of the capacity of the output from the 1:51 motor,  under 16.8V power supply. The linear speed will be approximately 5mm/s and thus a 120mm stroke will take 24s. 

This hints towards the synchronization test to try speed in the range of 3 to 5 mm/s

## Motor04b_PwmWithoutDeadband

This test confirms if the PWM deadband removal can remove the large deadband when the PWM values crosses from positive to negative.

### Testing Setup

**Motor:** 775 1:49P
**Load:** No load
**Electronics:** Arduino micro + XY160D Driver
**Power Supply:** 12V 2A Wall adapter
**Measurement:** Taken from Arduino's reading of the encoder

## Procedure

PWM 3921.16 Hz is used for the test.

The implementation of the deadband removal avoids very low PWM being applied to motor which are certainly not doing anything.

```c++
void DCMotor::setSpeedPercent(double speedPercent) {
    if (speedPercent == 0) stop();

    else if (speedPercent > 0) {
        digitalWrite(_pin_in1, LOW);
        digitalWrite(_pin_in2, HIGH);
    } else if (speedPercent < 0) {
        digitalWrite(_pin_in1, HIGH);
        digitalWrite(_pin_in2, LOW);
    }
    //Make speed absolute value, map the value to avoid deadband
    double hamonizedSpeed = abs(speedPercent);
    hamonizedSpeed = min(hamonizedSpeed, 1.0);
    int pwm = (255 - _PWMDeadband) * hamonizedSpeed + _PWMDeadband;
    analogWrite(_pin_ena, pwm);
    _running = true;
}
```

Note: The test is performed from negative value to positive value, some hysteresis can be observed in the system.

### Result

![result](results/Motor_04b_PwmWithoutDeadband/result.jpg)

**Deadband 30** appears to be the most continuous through the zero crossing. 

The discontinuous part observed shows some hysteresis. The motor need higher "PWM Value" when it is trying to accelerate from zero speed.

### Conclusion

Use Deadband 30. 

### Next Steps

It is not sure if this deadband implementation will actually helps the following PID control or not. 





## Motor05_PID_Velocity

This is not a test but a base script to verify <PID_v1.h> library.  

The PID is set to a low kp gain to simplify things, the system therefore takes a while to settle. The goal here is not to tune the PID but to verify that the PID can maintain different speed at a reasonable precision.

### Testing Setup

**Motor:** 775 1:49P
**Load:** No load
**Electronics:** Arduino micro + XY160D Driver
**Power Supply:** 12V 2A Wall adapter
**Measurement:** Taken from Arduino's reading of the encoder

The PID controller was programmed to maintain different speed. 
Settling Time before speed measurement: 2s
Number of samples: 40
Sampling Interval 0.1s

### Result

![result_error](results/Motor_05_PIDVelocity/result_error.jpg)

![result_controlval](results/Motor_05_PIDVelocity/result_controlval.jpg)

### Conclusion

The speed controlling accuracy of the PID in this system is within 3% , even without tuning the PID. This is good enough for many things.

However, our goal is position control and motion profile control, this set of result is not conclusive to how well it will follow motion profile.

The use of deadband 30 does not seem to affect the steady error.  This means the PID can take care of the non-linear behavior of the PWM vs Speed problem. However, deadband 30 might be more responsive to velocity changes, which, this result cannot reflect.

## Motor07_PID_MotionProfile

A rectangular motion profile is used for the PID to follow as a positional feedback control.

A speed of 2000step/s (approximately half of the maximum speed) is used as cruising speed. Acceleration and deceleration is instantaneous. Positional error of the system is plotted.

Settling time and error and ocillation are observed at t=0ms and t = 2000ms where acceleration and deceleration occur.


A manual tuning is used to find a good error response.

### Testing setup

**Motor:** 42GP-775 with 1:49 gearbox
**Load:** No load
**Electronics:** Arduino micro + XY160D Driver
**Power Supply:** 12V 2A Wall adapter
**Measurement:** Taken from Arduino's reading of the encoder

### Tuning process

**Result 01**

The following graph, **kp is changed**, larger Kp seems more responsive. Steady offset is observed.

![result_01](results/Motor_07_PID_MotionProfile/result_01.png)

**Result 02**

The following step, **ki is tuned**, increasing the ki seems to result in less steady time error.

![result_02](results/Motor_07_PID_MotionProfile/result_02.png)

**Result 03**

Next step **Kd is added**. increase in kd seems to have dampen the position error nicely 

kd = 0.0004 is almost critically dampened, however, a vibration/chattering is heard when run.



![result_03](results/Motor_07_PID_MotionProfile/result_03.png)

The following setSpeedPercent() output plot shows that ki=0.0002 and 0.0004 have quite a bit of jitter.

![result_03_output](results/Motor_07_PID_MotionProfile/result_03_output.png)

**Result 04**

Back to **tuning Kp**. Further increase in Kp results in much less settling time and much less initial error.

However, it slightly worsens the steady state error and also causes output jitter

![result_04](results/Motor_07_PID_MotionProfile/result_04.png)

![result_04_output](results/Motor_07_PID_MotionProfile/result_04_output.png)

**Result 05**

0.04kp is selected and see if further increase in ki will reduce stead state error

Here, K=0.2 and 0.4 can quickly go back to a mean zero error. But again quite large oscillation and jitter.

![result_05](results/Motor_07_PID_MotionProfile/result_05.png)

![result_05_output](results/Motor_07_PID_MotionProfile/result_05_output.png)

**Result 06**

Here, the effect of both ki (0.1 and 0.2) and kd (0.0001 and 0.0002) is investigated.

The purple line (ki = 0.2 and kd = 0.0002) seems to be most promising. It has the least amount of micro oscillation and settle within 5step of error in about 300ms.

![result_06](results/Motor_07_PID_MotionProfile/result_06.png)

![result_06_output](results/Motor_07_PID_MotionProfile/result_06_output.png)

**Result 07**

Finally we look at how these particularly tuned PID values for 2000 steps/s operate in other speeds (1000 - 4000step/s )

We can see that higher target speed have a higher stable state error. There is a constant 20 steps error at 4000step/s. The output of the PID also saturates in the first 300ms, showing a full acceleration.



![result_07](results/Motor_07_PID_MotionProfile/result_07.png)

![result_07_output](results/Motor_07_PID_MotionProfile/result_07_output.png)

### Conclusion

In general this is a usable controller for following a motion profile. 

The tune is kP = 0.04, kI = 0.20, kD = 0.0002

(Note that this tuning is only applicable for the 775 motor in no load case.)

Despite the acceleration and deceleration period is slightly out of control, it generally deviate less than a 100 steps (12 degrees where one rev has 2940 steps ). At cruising speed, it deviate less than 20 steps (2.5 degrees)

### Next Steps

It is possible that "proportional on measurement" instead of "proportional on error" can have better response. 

It is possible that a trapezoidal velocity profile will allow a more controlled acceleration and deceleration. 

It is possible that a feed forward control with a "velocity feedforward" will help compensate different steady-state error when different speed is used.



## Motor08_PID_TrapezoidalMotionProfile

Date: 2019-10-21



This is the development and validation of a **trapezoidal motion profile generator**. This results in three classes within `MotionProfile.cpp` file.

- `MotionProfile`  - Base class that implemented time keeping methods
- `LinearMotionProfile`  - Linear speed profile with constant velocity
- `TrapezoidalMotionProfile` - Trapezoidal speed profile with acceleration and deceleration phase

The **PID controller** from previous experiment was use with values:

- kP = 0.04, kI = 0.20, kD = 0.0002

### Testing setup

**Motor:** 42GP-555 with 1:51 gearbox (2244 step/rev)
**Load:** No load
**Electronics:** Arduino micro + XY160D Driver 
**Power Supply:** Bench Power Supply 12.6V (8A Max)
**Measurement:** Taken from Arduino's reading of the encoder

**Previous Test have established the maximum speed**:

- 5941 step/s @ 16.8V

- 4465step/s @ 12.6V

### Variables

**Running Speed** 2805 step/s  for 1:51 planetary gearbox (1.25 rpm or 5mm/s @ 1204 screw)

**Running Speed** 2970 step/s  for 1:54 worm gearbox (1.25 rpm or 5mm/s @ 1204 screw)

**Different acceleration** (equal to deceleration) is tested with the same PID values

**Step Error** during the entire time is logged and returned.

### Results

#### 1:51 planetary gearbox

![acceleration_m555_g51p_s2805_error](Motor08_PID_TrapezoidalMotionProfile/data/acceleration_m555_g51p_s2805_error.jpg)

![acceleration_m555_g51p_s2805_pos](Motor08_PID_TrapezoidalMotionProfile/data/acceleration_m555_g51p_s2805_pos.jpg)

#### 1:54 worm gearbox

![acceleration_m555_g54W_s2970_error](Motor08_PID_TrapezoidalMotionProfile/data/acceleration_m555_g54W_s2970_error.jpg)



## Motor09_StoppingCondition

This experiment investigate the pulling force generated by the clamping jaw when the PID controller is active. And **how the force build up until the stopping condition**. This is in preparation for the clamp controller to follow a motion profile while providing good pulling force against resistance.

This is the first position controlled test that involved a force measurement.

When the motors are overloaded, jammed or stalled by resistance, the motion controller needs to cut power to **avoid burning out the motor** and also to **report back to the higher control layer** that it is jammed.

Two stopping conditions are possible as a stopping (jammed) condition for the motion controller. Basic assumption can be:

- Speed is approaching zero (Pull as hard as possible)

- Position error is larger than a threshold (Probably easier to sync)

However, it is practically easier to use position error from the predefined motion profile because it is easier to keep multiple clamps in sync, and in case of jamming, stop all of them.

This experiment investigate the maximum pull force generated at the terminal stopping condition.

Assuming a fixed **error threshold of 100 steps**. (Approximately 0.1 to 0.2 mm error with 4mm pitch screw)

### Independent Variable

Stopping condition - speed threshold

### Dependent Variable

Terminal / Maximum Pull Force

### Testing setup

Motor used is **555 with 1:54 and 1:72 worm drive gearbox**. (Encoder has 2376 and 3168 steps/rev respectively). 

Electronics is similar to previous test. A **trapezoidal motion profile** is used in the controller, with an aggressive acceleration = **5000 step per s^2** and a variable running speed between **500 to 5000 steps/s**.

Testing jig is similar to the linear guide jaw test. 

Pull Force measured by a **750kg loadcell** attached to **Phidgets Bridge 4 Input**. 

Two power source was used. (1) The Large Power Supply at 16.6V 10A (Attempt to simulate LiPo battery power)  (2) LiPo 4 Cell Battery

### Procedure

The motor first move towards the opposite direction to a starting position. A small power settings is set and moved until it is stalled. Encoder value is reset. The trapezoidal motion controller kicks in and attempt to reach a far enough position.

The controller monitors the speed (discrete diff of encoder position), the controller stops the motor when positional error is above threshold. The maximum pull force in the entire pull stroke is recorded.

### Result

Plotting maximum pull force against speed.

![StoppingCondition](Motor09_StoppingCondition/data_collection/StoppingCondition.jpg)

In general the 1:54 Gearbox produce more pull force than the 1:72 Gearbox. This is probably due to lower efficiency of the higher gearing ratio gearbox)

The result from 16.6V 10A power supply is not similar to the LiPo battery. I suspect that the actual LiPo batter has much higher current output. (Spec theoretically 45-90C discharge means [45 to 90] * 1.3A = 58 to 117A , which is a lot more).

The result shows a wide peak between 1000 to 2000 steps/s in both gearbox when powered by LiPo battery.

An extra test is performed to test an artificial power clamp. The output of the PID controller is limited to [-0.8 to 0.8] instead of the normal [-1.0 to 1.0]. The result is an overall decrease in all pull speed.

### Caution

The current method of motor power control is not a true current limiting control to avoid motor overheat.  

In the scenario where the motor is continuously operating in a high current state to overcome friction, the motor might still overheat. 



## Motor09b_StopCondition_PosError

### Independent Variable

Moving speed.

### Dependent Variable

Terminal / Maximum Pull Force

## Motor10_TwoMotorSyncControl

This test is to develop a control for two motors that move synchronously. 

Two **36GP-555 12V** motors are used but their gearbox are different. This is because we do not have 2 of the same kind. However, this should not be a problem. The goal here is to have their **step speed synchronous**.

| Motor # | GearBox | Encoder Step / Rev | Step / mm |
| ------- | ------- | ------------------ | --------- |
| m1      | 1 : 51  | 2244               | 561       |
| m2      | 1 : 100 | 4400               | 1100      |

Note: In Motor04c experiment, we know that the no load speed for the two motors are:

| Motor                   | Speed [step/s] @12.6V | Speed [step/s] @16.8V |
| ----------------------- | --------------------- | --------------------- |
| Motor 1 (Gearbox 1:51)  | 4465                  | 5941                  |
| Motor 2 (Gearbox 1:100) | 6686                  | 9168                  |

### Electrical Connection

Because previous experiments have only one motor. A temporary hack is wired up using the same electronics wiring board.

The driver can handle two motors but there are only two interrupts available for the two motors encoder. Therefore each encoder has one channel that gets one of the interrupt pin.

| Motor Connection  | Arduino Pin Num | Wire Colour |
| ----------------- | --------------- | ----------- |
| m1_driver_ena_pin | 9               |             |
| m1_driver_in1_pin | 8               |             |
| m1_driver_in2_pin | 7               |             |
| m1_encoder_c1_pin | 3               | Yellow      |
| m1_encoder_c2_pin | 2 A2            | Green       |
| m2_driver_ena_pin | 10              |             |
| m2_driver_in1_pin | 12              |             |
| m2_driver_in2_pin | 11              |             |
| m2_encoder_c1_pin | A2 2            | Yellow      |
| m2_encoder_c2_pin | A1              | Green       |

### Controller

One trapezoidal motion profile generator is used and two PID Controllers are used for each motor.

### Result

To be written, but the error for the two motors are really low in initial test.