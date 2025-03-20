# Agni-Flight-Computer-V2
Hi, this is Agni Flight Computer Version 2. This flight computer is designed only for Ground Control Station (GCS) purposes and is not used in the rocket because it runs on an Arduino Nano, which has limited processing power. However, if you want to use this flight computer for your rocket project, feel free to use it 🚀


https://github.com/user-attachments/assets/e2c30fc4-770d-4295-aec8-807851ece405


This Arduino-based project interfaces with the MPU6050 IMU sensor to perform motion tracking, orientation estimation, shock detection, and displacement measurement. It integrates Madgwick’s sensor fusion algorithm, which relies on quaternions to accurately determine roll (φ), pitch (θ), and yaw (ψ) angles from raw accelerometer and gyroscope data.

Quaternion Representation
A quaternion is a four-component vector:

𝑞
=
(
𝑞
0
,
𝑞
1
,
𝑞
2
,
𝑞
3
)
q=(q 
0
​
 ,q 
1
​
 ,q 
2
​
 ,q 
3
​
 )
where 
𝑞
0
q 
0
​
  is the scalar component, and 
(
𝑞
1
,
𝑞
2
,
𝑞
3
)
(q 
1
​
 ,q 
2
​
 ,q 
3
​
 ) form the vector part.

The orientation is derived using:

roll
=
tan
⁡
−
1
(
2
(
𝑞
0
𝑞
1
+
𝑞
2
𝑞
3
)
1
−
2
(
𝑞
1
2
+
𝑞
2
2
)
)
roll=tan 
−1
 ( 
1−2(q 
1
2
​
 +q 
2
2
​
 )
2(q 
0
​
 q 
1
​
 +q 
2
​
 q 
3
​
 )
​
 )
pitch
=
sin
⁡
−
1
(
2
(
𝑞
0
𝑞
2
−
𝑞
3
𝑞
1
)
)
pitch=sin 
−1
 (2(q 
0
​
 q 
2
​
 −q 
3
​
 q 
1
​
 ))
yaw
=
tan
⁡
−
1
(
2
(
𝑞
0
𝑞
3
+
𝑞
1
𝑞
2
)
1
−
2
(
𝑞
2
2
+
𝑞
3
2
)
)
yaw=tan 
−1
 ( 
1−2(q 
2
2
​
 +q 
3
2
​
 )
2(q 
0
​
 q 
3
​
 +q 
1
​
 q 
2
​
 )
​
 )
Madgwick Filter Quaternion Update
The quaternion derivative is given by:

𝑑
𝑞
𝑑
𝑡
=
1
2
𝑞
⊗
𝜔
−
𝛽
∇
𝑓
dt
dq
​
 = 
2
1
​
 q⊗ω−β∇f
where 
𝜔
ω is the gyroscope angular velocity vector, and 
𝛽
β is the filter gain.

Displacement Estimation
After removing gravity (
𝑔
=
9.81
𝑚
/
𝑠
2
g=9.81m/s 
2
 ), displacement is estimated using double integration:

𝑉
𝑓
=
𝑉
𝑖
+
𝑎
⋅
𝑑
𝑡
V 
f
​
 =V 
i
​
 +a⋅dt
𝑆
=
𝑆
𝑖
+
𝑉
⋅
𝑑
𝑡
S=S 
i
​
 +V⋅dt
where 
𝑉
V is velocity, 
𝑎
a is linear acceleration, and 
𝑆
S is displacement.

Shock and Vibration Detection
Shock is detected if:

∥
𝑎
⃗
∥
>
15
 m/s
2
∥ 
a
 ∥>15 m/s 
2
 
Vibration is analyzed using a rolling standard deviation.

This project logs real-time CSV data, supports EEPROM calibration, and integrates WS2812B LEDs and a buzzer for visual and audio feedback. 🚀


![Image](https://github.com/user-attachments/assets/806f6656-7732-4c62-aef4-8b691b7bf2f5)


![Image](https://github.com/user-attachments/assets/e2ef9f01-2d2e-4c77-a882-bb957035c3e4)
