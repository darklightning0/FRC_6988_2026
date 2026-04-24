# FRC Team 6988 - 2026 Software Architecture & Reference Guide

This document is the definitive reference for the 2026 competition robot's software architecture. It breaks down every subsystem, the mathematical logic behind the automation, every programmable constant, and a complete mapping of all driver and operator controls.

---

## 1. Drivetrain & Vision (3D MegaTag2 Odometry)
The robot utilizes a CTRE Phoenix 6 Swerve Drivetrain paired with a Pigeon 2 IMU and dual Limelight 4s.

### Architecture & Logic
* **Divergent Vision:** Two Limelights are mounted facing 18 degrees outward (Left Yaw: `18`, Right Yaw: `-18`). This overlapping panoramic view prevents defenders from blinding both cameras simultaneously.
* **MegaTag2 Odometry:** The Limelights do not use 2D crosshairs (`TX`/`TY`). Instead, they read AprilTags to calculate the robot's absolute `(X, Y)` position in meters on the field. This location is fed into the `SwerveDrivePoseEstimator` in `Robot.java`.
* **The Gyro Rule:** For MegaTag2 to remain perfectly stable, the code explicitly feeds the Pigeon 2's Yaw to the Limelights exactly one line before asking for the 3D pose (`LimelightHelpers.SetRobotOrientation`).

### Configurable Variables & Toggles
* **`enableAutoRotation` (Boolean):** Located in `RobotContainer.java`. 
  * Default: `true`
  * Action: When true, holding the Driver `Y` button will mathematically calculate the angle between the robot and the target, forcing the swerve drive to automatically spin and lock onto the target using a P-loop (`rotRate = headingErrorRadians * 4.0`).
  * Toggle: The driver can toggle this boolean on/off mid-match by pressing the **Driver BACK Button**. If turned off, the `Y` button will only spool the flywheels, and the driver must rotate manually using the right joystick.
* **Alliance Coordinates:** The code checks `DriverStation.getAlliance()` to automatically swap target coordinates.
  * Blue Alliance Target: `X = 4.62, Y = 4.0`
  * Red Alliance Target: `X = 11.9, Y = 4.0`

---

## 2. Shooter Subsystem (Flywheels & Hopper)
The shooter separates the flywheel spooling from the hopper feeding to completely eliminate jamming. 

### Architecture & Logic
* **Master/Slave Flywheels:** The main shaft uses two Falcon 500s. Due to a broken encoder, the second motor is configured as a `Follower` with `MotorAlignmentValue.Opposed`.
* **Step-Up Ratio:** The flywheels are geared 18:46. The code handles this via `SensorToMechanismRatio = 0.3...`, meaning the code targets the speed of the *wheels*, not the motor.
* **Safety Lock (`isAtSpeed()`):** A boolean method that checks if the flywheels are within `2.0` RPS of the target speed. The Hopper Kraken (Motor 34) will strictly remain at `0` RPS until this returns true.

### Complete Control Mapping & Speeds
**Auto Mode Controls (Driver Joystick):**
* **Hold `Y` Button:** Initiates the auto-sequence. 
  1. Calculates True Distance (meters) to the target.
  2. Looks up the required RPS in the `tyToShooterSpeed` interpolation map.
  3. Commands `ShooterMode.Rev` to spool flywheels.
  4. Auto-rotates the drivetrain (if `enableAutoRotation` is true).
  5. Once aimed (< 3 degrees error) AND `isAtSpeed()` is true, automatically fires the Hopper at 40 RPS (`ShooterMode.Shoot`).

**Manual Mode Controls (Operator Joystick):**
The operator sets a target speed using the preset buttons or joystick, and then manually fires the hopper.
* **`Y` Button:** Sets target to **60 RPS**.
* **`B` Button:** Sets target to **50 RPS**.
* **`A` Button:** Sets target to **40 RPS**.
* **`X` Button:** Sets target to **30 RPS**.
* **`Left Y-Axis`:** Variable manual revving. Maps the stick distance (`0.1` to `1.0`) directly to a speed between **0 and 60 RPS**.
* **`Left Stick Click`:** Executes `ShooterMode.Shoot`. Blasts the Hopper Kraken at 40 RPS to feed the ball.
* **`Right Y-Axis > 0.2`:** Executes `ShooterMode.Reverse`. Spins both the flywheels and the hopper backwards at slow speeds (-10 RPS and -20 RPS) to unjam a stuck note.

---

## 3. Intake Subsystem
The intake is a deployable arm powered by a Kraken X60 (geared 9.14:1) utilizing CTRE Motion Magic, with a Falcon 500 running the rollers.

### Architecture & Logic
* **Motion Magic & Gravity:** The intake uses `MotionMagicVoltage` paired with a `GravityTypeValue.Arm_Cosine` feedforward (`kG`). This ensures the motor actively holds the arm's weight in mid-air when manual controls are released, rather than falling to the bumper.
* **Strict Software Limits:** The arm is clamped using WPILib's `MathUtil.clamp` between `0.0` (Stowed) and `0.25` (Deployed).
* **Current Limits:** Both the Deployer Kraken and the Hopper Kraken are software-limited to **40 Amps** (`StatorCurrentLimit = 40`) to prevent popping PDP breakers during aggressive defense or mechanical jams.

### Complete Control Mapping
**Operator Joystick:**
* **`Right Stick Click`:** Toggles the automatic Deploy/Stow. Snaps the arm to exactly `0.25` or `0.0` using the Motion Magic PID.
* **`Right Y-Axis`:** Manual Jog. Pushing Up/Down increments the `targetPosition` by exactly `0.005` rotations per loop. This provides extremely fine, safe control without snapping the arm.
* **`D-Pad UP` (POV 0):** Commands `IntakeMode.Intake` (Spins rollers inward).
* **`D-Pad DOWN` (POV 180):** Commands `IntakeMode.Reverse` (Spits game piece out).
* **`Start` Button:** The Re-Zero function. If the belt skips a tooth during a collision, the operator pulls the arm physically against the hard stop and presses Start to reset the encoder back to `0.0`.

---

## 4. The Programmer's Pre-Match Tuning Guide
Before hitting the carpet at competition, the following exact values must be verified and tuned.

### A. The Interpolation Map (`Remote.java`)
Because the code uses 3D map geometry instead of Limelight crosshairs, the shooting map operates on **Meters to RPS**.
* Find the `tyToShooterSpeed` map in the `Remote()` constructor.
* Place the robot on the field at known distances (e.g., 1.5m, 3.0m, 5.0m). Read `Vision/True Distance (M)` on SmartDashboard.
* Tune the RPS until the shot hits perfectly, and update the code:
  `tyToShooterSpeed.put(1.5, 35.0); // 1.5 meters = 35 RPS`

### B. Limelight Offset Configuration (Web GUI)
The Java code assumes the Limelights know they are angled. If the physical Limelights are reset or swapped, you MUST update their internal web dashboards (`limelight-left.local:5801`):
* Tab: Standard 3D / MegaTag
* Left Camera **Yaw**: `18`
* Right Camera **Yaw**: `-18`
* *Verification:* Spin the physical robot in place. The AdvantageScope 3D robot must spin cleanly without "orbiting." If it orbits, slightly adjust the Yaw numbers.

### C. Intake PID & Limits (`Intake.java`)
* **`DEPLOY_POSITION`:** Currently set to `0.25` (1/4 of a full arm rotation based on the 9.14 ratio). If the arm does not fully reach the bumper, manually jog it down and read `Intake/Arm Actual Position` on SmartDashboard to find the true limit, then update this constant.
* **`kG` (Gravity Feedforward):** Set `kP` to 0 in Tuner X. Pull the arm horizontally and increase `kG` until it floats perfectly against gravity, then update `deployerConfig.Slot0.kG` in the code.
