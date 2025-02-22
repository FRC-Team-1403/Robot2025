# C.java

- <span style="color:orange; font-size:20px">Integer</span>
- <span style="color:green; font-size:20px">String</span>
- <span style="color:lightblue; font-size:20px">Double</span>

## Motors
~~- SetMotor(Speed, id)~~
~~- SetMotor(Speed, "Name")~~
~~- SetMotorName(id, "Name")~~
- SetBoundsMotor(Angle1, Angle2, id)
- SetBoundsMotor(Angle1, Angle2, "Name")

## Encoders
- ResetEncoder(id)
- ResetEncoder("Name")
- SetEncoderName(id, "Name")
- ReadEncoder(id)
- ReadEncoder("Name")
- set0Rotation(id)
- set0Rotation("Name")

~~## Input/Output~~
~~- isTriggered(id)~~
~~- isTriggered("Name")~~
~~- SetIOName(id, "Name")~~
~~- isTriggered(id, flipped)~~
~~- isTriggered("Name", flipped)~~

~~## Ranges~~
~~- GetDistance(id)~~
~~- GetDistance("Name")~~
~~- SetRangeName(id, "Name")~~

~~## Utilities~~
- CanBusMetrics() (OPTIONAL)
~~- MotorType(type)~~
~~- RangeType(type)~~`
~~- InputType(type)~~

~~## Math~~
~~- Pi()~~~
~~- E()~~
~~- toRadians(double)~~
~~- toDegrees(double)~~
