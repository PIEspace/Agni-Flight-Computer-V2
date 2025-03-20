Agni-Flight-Computer-V2

Hi, this is Agni Flight Computer Version 2. This flight computer is designed only for Ground Control Station (GCS) purposes and is not used in the rocket because it runs on an Arduino Nano, which has limited processing power. However, if you want to use this flight computer for your rocket project, feel free to use it 🚀



Features

This Arduino-based project interfaces with the MPU6050 IMU sensor to perform:

Motion tracking

Orientation estimation

Shock detection

Displacement measurement

It integrates Madgwick’s sensor fusion algorithm, which relies on quaternions to accurately determine roll (φ), pitch (θ), and yaw (ψ) angles from raw accelerometer and gyroscope data.

Quaternion Representation

A quaternion is a four-component vector:



where  is the scalar component, and  form the vector part.

The orientation is derived using:







Madgwick Filter Quaternion Update

The quaternion derivative is given by:



where  is the gyroscope angular velocity vector, and  is the filter gain.

Displacement Estimation

After removing gravity , displacement is estimated using double integration:




where  is velocity,  is linear acceleration, and  is displacement.

Shock and Vibration Detection

Shock is detected if:



Vibration is analyzed using a rolling standard deviation.

Additional Features

Logs real-time CSV data

Supports EEPROM calibration

Integrates WS2812B LEDs and a buzzer for visual and audio feedback

🚀
