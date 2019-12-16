# Joint Pressing Machine

This machine is designed to press tight fitting joint samples together in a controlled and measured manner.

## Design goal

- create a machine that can press up to 120 x x 120mm Lap Joint samples together. 
  - Various planar jointing angles can be accommodated
  - Tilted angle will require additional fixing
- Alternatively shortened mortise tenon samples can also be tested.
- 

## Components

### Motor with gearbox

**GW4058-555** 12V with 1:72 worm drive gearbox

**No load:** 105rpm 0.5A/12V

**Rated load:** 4Nm 54rpm 5A/12V

**Stall:** 7Nm 13A/12V

### Ball Screw

SFU1204 - 350 

End Machining according to **FK10 and FF10**

Rigid Coupler Included: **SRJ-25C-8mm-8mm**

Thread Length: **350mm** 

### Screw Support

Fixed End Support: **FK10** 

Supported End Support: **FF10**

### Ball Nut

### Limit Switch

[OMRON Limit Switch Z15GQ-B](https://item.taobao.com/item.htm?spm=a1z0d.6639537.1997196601.4.69d87484FcYLtG&id=582041707223)

### Structural Frame

**Main Frame :** Steel U Channel 60x55x5 

**Pressing Beam:** Steel U Channel 40x25x3.5

## Mechanical Calculation

### Ball screw 1204 Drive Torque

Calculator used: [Torque-Forward Drive](https://www.roton.com/screw-university/formula-calculators/torque-forward-drive/)

0.16 inch lead (~ 4.06mm)
1653 lbs (~750kg Force = Sensor Range)
90% Efficiency

Result: (46.8 in-lbs) **5.28Nm**

This is within the 7Nm range of each motor. (Which we have two of them)

### Ball Screw Bearings support

Both screws are supported on both sides to reduce chance of buckling.

They also act as linear guides for each other. (Precision linear guide is not necessary here.)

The FK10 is really not designed for this much force. FK10 **Load Limit is 1.9kN**.  **Basic Dynamic Load Rating Ca = 6.4kN**

But for a quick test, this is fine.

## Machining Tools Required

Drill press with steel drill bits.

Three large diameter drills required: **22mm, 28mm, 34mm**

Welder to weld structural steel

