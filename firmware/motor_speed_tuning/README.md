# Motor Speed Tuning Firmware (ESP32)

Standalone firmware I used to tune wheel-speed control on one robot (four Mecanum-wheel DC motors with encoders, BT7960 drivers). Commands come over Serial, so each motor and the whole robot body can be tested without ROS 2. The tuned values were then used in the integrated robot firmware.

## How it works

Every 100 ms, for each motor:

1. **Target ramp:** the commanded RPM is approached in steps of at most 4 RPM, so the wheels don't jerk.
2. **Speed measurement:** quadrature encoder ticks (counted in interrupts) are converted to RPM and smoothed with a low-pass filter (alpha = 0.4).
3. **Feedforward:** a per-motor linear model (offset + slope × target RPM, with separate low- and high-speed fits) gives most of the PWM. A small extra boost helps the weaker motors at low speed, separately for forward and reverse.
4. **PI correction:** gain-scheduled Kp and Ki (five RPM bands per motor) trim the remaining error. Kd is 0, because the derivative would amplify encoder noise. The integral is clamped to prevent windup.
5. **PWM slew limit:** PWM may rise by at most 24 and fall by at most 10 per cycle; the sign selects forward or reverse on the driver.

## Mecanum kinematics

`BODY vx vy wz` converts a body velocity (m/s, m/s, rad/s) into four wheel RPMs with the Mecanum inverse kinematics (wheel radius 0.05 m, lx = ly = 0.10 m). Per-wheel forward/reverse scale factors compensate for differences between motors. The forward kinematics converts the measured wheel RPMs back to the actual body velocity, so commanded and actual motion can be compared in the log.

## Serial commands (115200 baud, newline)

| Command | Effect |
|---|---|
| `BODY 0.10 0.00 0.00` | Drive +X at 0.1 m/s |
| `BODY 0.00 0.10 0.00` | Strafe +Y |
| `BODY 0.00 0.00 0.50` | Rotate in place |
| `5 20` / `5 -20` | One motor (ID 5-8) to ±20 RPM |
| `ALL 20` | All motors to 20 RPM |
| `STOP` | Stop everything |
| `LOG ON` / `LOG OFF` / `LOGM 5` / `STATUS` | Logging |
