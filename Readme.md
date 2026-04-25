# FRC Team 6988 - 2026 Software Architecture & Reference Guide

This document is the definitive reference for the 2026 competition robot's software architecture. It breaks down every subsystem, the mathematical logic behind the automation, every programmable constant, and a complete mapping of all driver and operator controls.

---

## 📋 Table of Contents
1. [Drivetrain & Vision](#1-drivetrain--vision-3d-megatag2-odometry)
2. [Shooter Subsystem](#2-shooter-subsystem-unified-4-motor-control)
3. [Intake Subsystem](#3-intake-subsystem)
4. [Control Mappings](#4-complete-joystick-control-mappings)
5. [Tuning Guide](#5-the-programmers-pre-match-tuning-guide)
6. [Patch Notes](#6-patch-notes--recent-updates)

---

## 1. Drivetrain & Vision (3D MegaTag2 Odometry)
The robot utilizes a CTRE Phoenix 6 Swerve Drivetrain paired with a Pigeon 2 IMU, dual Limelight 4s, and real-time field visualization.

### Architecture & Logic
* **Divergent Vision:** Two Limelights are mounted facing 18 degrees outward (Left Yaw: `18`, Right Yaw: `-18`). This overlapping panoramic view prevents defenders from blinding both cameras simultaneously.
* **MegaTag2 Odometry:** The Limelights do not use 2D crosshairs (`TX`/`TY`). Instead, they read AprilTags to calculate the robot's absolute `(X, Y)` position in meters on the field. This location is fed into the `SwerveDrivePoseEstimator` in `Robot.java`.
* **The Gyro Rule:** For MegaTag2 to remain perfectly stable, the code explicitly feeds the Pigeon 2's Yaw to the Limelights exactly one line before asking for the 3D pose (`LimelightHelpers.SetRobotOrientation`).
* **Real-Time Field Visualization:** The `Robot.robotPeriodic()` fuses the Pigeon 2 gyro angle with swerve module positions to calculate robot pose in real-time. This is published to SmartDashboard and visible in AdvantageScope as a moving robot icon on the field.

### Configurable Variables & Toggles
* **`enableAutoRotation` (Boolean):** Located in `RobotContainer.java`. 
  * Default: `true`
  * Action: When true, holding the Driver `Y` button will mathematically calculate the angle between the robot and the target, forcing the swerve drive to automatically spin and lock onto the target using a P-loop (`rotRate = headingErrorRadians * 4.0`).
  * Toggle: The driver can toggle this boolean on/off mid-match by pressing the **Driver BACK Button**. If turned off, the `Y` button will only spool the flywheels, and the driver must rotate manually using the right joystick.
* **Alliance Coordinates:** The code checks `DriverStation.getAlliance()` to automatically swap target coordinates.
  * Blue Alliance Target: `X = 4.62, Y = 4.0`
  * Red Alliance Target: `X = 11.9, Y = 4.0`

### Motor IDs & Configuration
| Component | Motor ID | Type | Role |
|-----------|----------|------|------|
| Front Left Drive | 1 | TalonFX | Swerve Drive Motor |
| Front Left Steer | 2 | TalonFX | Swerve Steer Motor |
| Front Right Drive | 7 | TalonFX | Swerve Drive Motor |
| Front Right Steer | 8 | TalonFX | Swerve Steer Motor |
| Back Left Drive | 3 | TalonFX | Swerve Drive Motor |
| Back Left Steer | 4 | TalonFX | Swerve Steer Motor |
| Back Right Drive | 5 | TalonFX | Swerve Drive Motor |
| Back Right Steer | 6 | TalonFX | Swerve Steer Motor |
| Pigeon 2 IMU | 21 | Can Bus | Gyro Sensor |

---

## 2. Shooter Subsystem (Unified 4-Motor Control)
The shooter is a unified system with **main flywheels**, **intake hopper**, and **feed belt** working together. The subsystem separates flywheel spooling from hopper feeding to completely eliminate jamming and provide intelligent reverse control.

### Architecture & Logic
* **Master/Slave Flywheels (IDs: 16, 999):** The main shaft uses two Falcon 500s. Due to a broken encoder on the slave, the second motor is configured as a `Follower` with `MotorAlignmentValue.Opposed` (opposite rotation but same control).
* **Step-Up Ratio:** The flywheels are geared 18:46. The code handles this via `SensorToMechanismRatio = 0.3913043478`, meaning the code targets the speed of the *wheels*, not the motor shaft.
* **Shooter Intake Kraken (ID: 34):** Pulls game pieces from the intake arm and feeds them toward the hopper belt.
* **Hopper Belt Falcon (ID: 35):** Carries the game piece into the flywheel shoot zone, then stops when shooting to prevent jamming.
* **Velocity Control:** All four motors use `VelocityVoltage` commands for precise RPS (rotations per second) control with PID + feedforward.
* **Safety Lock (`isAtSpeed()`):** A boolean method that checks if the flywheels are within `2.0` RPS of the target speed. The shooter intake and hopper only feed when this returns true, preventing half-speed launches.

### Motor Configuration Table
| Motor | ID | Type | Purpose | Current Limit | Config |
|-------|----|----|---------|----------------|--------|
| Main Shaft L | 16 | Falcon 500 | Flywheel Leader | 60A | `kP=0.38, kV=0.14` |
| Main Shaft R | 999 | Falcon 500 | Flywheel Follower | 60A | Opposed follower |
| Shooter Intake | 34 | Kraken X60 | Feed intake → hopper | 40A | Coast mode |
| Hopper Belt | 35 | Falcon 500 | Feed hopper → flywheels | 40A | Coast mode |

### Shooter States & Motor Behavior

| State | Flywheels | Intake (34) | Hopper (35) | Use Case |
|-------|-----------|------------|------------|----------|
| **Idle** | 0 RPS | 0 RPS | 0 RPS | Robot at rest |
| **Rev** | Target RPS | 0 RPS | 0 RPS | Spinning up, not feeding |
| **Shoot** | Target RPS | 40 RPS | 40 RPS | Actively launching note |
| **Reverse** | Negative RPS | -30 RPS | -30 RPS | Unjamming stuck note |

### Complete Control Mapping & Speeds

**Manual Mode Controls (Operator Joystick):**

The operator controls shooter speed and feeding independently using left joystick + buttons.

#### Speed Setting Methods (Priority Order):
1. **`Y` Button** → **60 RPS** (maximum forward speed)
2. **`B` Button** → **50 RPS** (mid-high speed)
3. **`A` Button** → **40 RPS** (standard shooting speed)
4. **`X` Button** → **30 RPS** (low speed, close range)
5. **`Left Y-Axis (±0.1 to ±1.0)`** → **-60 to +60 RPS** (full variable range)
   * Push UP = positive speed (launch forward)
   * Pull DOWN = negative speed (reverse)
   * Deadzone: ±0.1 (ignores small movements)

#### Firing Actions:
* **`Left Stick Button (Click Left Stick)`** → **Shoot Mode**
  * If current speed **≥ 0 (positive):** Executes `ShooterMode.Shoot`
    * Flywheels spin at target speed
    * Intake Kraken spins at 40 RPS (feeds note forward)
    * Hopper Belt spins at 40 RPS (advances note into flywheels)
    * **Result:** Note launches at set speed
  * If current speed **< 0 (negative):** Executes `ShooterMode.Reverse`
    * All four motors spin backward
    * Flywheels: Full negative speed (matches joystick)
    * Intake Kraken: -30 RPS (pulls note backward)
    * Hopper Belt: -30 RPS (retracts note out of flywheels)
    * **Result:** Aggressive unjamming without speed loss

#### Modes Without Firing:
* **Just holding `Y`/`B`/`A`/`X` (no click)** → `ShooterMode.Rev`
  * Flywheels spin at preset speed
  * Intake and Hopper remain at 0 RPS (no feeding)
  * **Result:** Safe spooling before manual fire

* **`Left Y-Axis` moved (no button, no click)** → `ShooterMode.Rev`
  * Flywheels spin at joystick-controlled variable speed
  * Intake and Hopper remain at 0 RPS
  * **Result:** Smooth variable revving for fine-tuning shots

---

## 3. Intake Subsystem
The intake is a deployable arm powered by a Kraken X60 (geared 9.14:1) utilizing CTRE Motion Magic, with intelligent auto-toggle and manual jogging capabilities.

### Architecture & Logic
* **Motion Magic & Gravity:** The intake uses `MotionMagicVoltage` paired with a `GravityTypeValue.Arm_Cosine` feedforward (`kG`). This ensures the motor actively holds the arm's weight when manual controls are released, rather than falling to the bumper.
* **Absolute Encoder:** CANcoder provides 0-1.0 rotations tracking. Position `0.0` = fully stowed (against frame), Position `0.25` = fully deployed (at bumper).
* **Strict Software Limits:** The arm is clamped using WPILib's `MathUtil.clamp` between `0.0` (Stowed) and `0.25` (Deployed). The motor cannot exceed these bounds.
* **Current Limits:** The Deployer Kraken is software-limited to **40 Amps** (`StatorCurrentLimit = 40`) to prevent popping PDP breakers during aggressive defense or mechanical jams.
* **Smart Auto-Toggle:** When the operator clicks the right stick, the code reads the current encoder position and decides:
  * If closer to stowed (< 0.125) → Deploy to 0.25
  * If closer to deployed (≥ 0.125) → Stow to 0.0

### Motor Configuration
| Motor | ID | Type | Purpose | Ratio | Config |
|-------|----|----|---------|-------|--------|
| Intake Deployer | 13 | Kraken X60 | Arm up/down | 9.14:1 | Motion Magic + kG |
| Intake Rollers | 14 | Falcon 500 | Spin wheels | 1:1 | % Output |

### Complete Control Mapping

**Operator Joystick Controls:**

| Button/Axis | Action | Result |
|-------------|--------|--------|
| **`Right Stick Click`** | Smart toggle | Automatically Deploy (0.25) or Stow (0.0) based on current position |
| **`Right Y-Axis` UP (< -0.2)** | Manual jog UP | Increments position +0.005 per loop (fine control) |
| **`Right Y-Axis` DOWN (> +0.2)** | Manual jog DOWN | Decrements position -0.005 per loop (fine control) |
| **`D-Pad UP` (POV 0)** | Intake mode | Spin rollers inward (pulling game piece in) |
| **`D-Pad DOWN` (POV 180)** | Reverse mode | Spin rollers outward (spitting game piece out) |
| **`Start` Button** | Re-zero encoder | Resets encoder to 0.0 if belt skips tooth |

### Intake Roller Modes

| Mode | Falcon Speed | Use Case |
|------|--------------|----------|
| **Intake** | Inward | Pulling game piece from field into deployer arm |
| **Reverse** | Outward | Spitting stuck piece out or clearing jam |
| **Idle** | 0% | Holding position, no spinning |

---

## 4. Complete Joystick Control Mappings

### Driver Joystick (Xbox Controller)

| Button/Axis | Action | Details |
|-------------|--------|---------|
| **`Left Stick`** | Drive Forward/Back | Y-axis controls velocity in field direction |
| **`Left Stick`** | Drive Left/Right | X-axis controls lateral velocity |
| **`Right Stick X-Axis`** | Rotate Robot | Controls angular velocity (spin left/right) |
| **`Right Bumper`** | Toggle Slow Mode | Halves drive speed for precision (20% vs 40% max) |
| **`Y` Button** | Auto Aim + Shoot | Hold to auto-rotate to target and fire when ready |
| **`A` Button** | Brake | Locks wheels in X pattern (anti-push) |
| **`B` Button** | Point Wheels | Points all wheels in joystick direction (for pushing) |
| **`POV UP`** | Drive Forward | Full-speed forward (for testing) |
| **`POV DOWN`** | Drive Backward | Full-speed backward (for testing) |
| **`Back` Button** | Toggle Auto-Rotation | Enable/disable automatic rotation during Y-button hold |

### Operator Joystick (Xbox Controller)

#### Left Side (Shooter Control)
| Button/Axis | Action | Details |
|-------------|--------|---------|
| **`Y` Button** | Set 60 RPS | Maximum forward speed preset |
| **`B` Button** | Set 50 RPS | Mid-high speed preset |
| **`A` Button** | Set 40 RPS | Standard shooting speed preset |
| **`X` Button** | Set 30 RPS | Low speed preset (close range) |
| **`Left Y-Axis`** | Variable Speed | -60 to +60 RPS (up=forward, down=reverse) |
| **`Left Stick Click`** | Fire/Shoot | Executes Shoot (if speed ≥ 0) or Reverse (if speed < 0) |

#### Right Side (Intake Arm Control)
| Button/Axis | Action | Details |
|-------------|--------|---------|
| **`Right Stick Click`** | Smart Deploy/Stow | Auto-toggles arm based on position |
| **`Right Y-Axis UP`** | Manual Jog UP | Arm raises slowly (+0.005 per loop) |
| **`Right Y-Axis DOWN`** | Manual Jog DOWN | Arm lowers slowly (-0.005 per loop) |
| **`D-Pad UP` (POV 0)** | Intake Wheels | Spin rollers to pull piece in |
| **`D-Pad DOWN` (POV 180)** | Intake Reverse | Spin rollers to spit piece out |

#### Emergency/System
| Button/Axis | Action | Details |
|-------------|--------|---------|
| **`Start` Button`** | Re-zero Intake | Resets encoder if mechanism goes out of sync |
| **`Back` Button`** | (Reserved) | Available for future use |

---

## 5. The Programmer's Pre-Match Tuning Guide

Before hitting the carpet at competition, the following exact values must be verified and tuned.

### A. Shooter Velocity Tuning (Most Critical)

The shooter uses a PID + feedforward model to maintain precise RPS. The constants are in `Shooter.java` constructor:

```java
leaderConfig.Slot0.kP = 0.38;    // Proportional gain (error correction)
leaderConfig.Slot0.kV = 0.14;    // Velocity feedforward (steady-state speed)
```

**Tuning Procedure:**
1. Open `Shooter.java` and set `manual_shooter_speed = 50.0` (fixed 50 RPS target)
2. Run the robot in test mode and watch SmartDashboard `Shooter/Percent` for 10 seconds
3. Observe the behavior:
   - **Overshoots then settles** → Increase `kV` (feedforward too low)
   - **Oscillates wildly** → Decrease `kP` (proportional gain too high)
   - **Never reaches target** → Increase `kP` or `kV`
   - **Perfectly smooth** → Done! Don't change
4. Test at multiple speeds (30, 50, 60 RPS) to verify consistency

### B. Shooting Distance Map (`Remote.java`)

Because the code uses 3D map geometry instead of Limelight crosshairs, the shooting map operates on **Meters to RPS**.

Located in `Remote()` constructor:
```java
tyToShooterSpeed.put(1.5, 30.0); // Subwoofer at 1.5m = 30 RPS
tyToShooterSpeed.put(3.0, 45.0); // Mid-range at 3.0m = 45 RPS
tyToShooterSpeed.put(5.0, 60.0); // Far shot at 5.0m = 60 RPS
```

**Tuning Procedure:**
1. Place robot on field at known distance (use field markings or measuring tape)
2. Read `Telemetry/Limelight Distance (M)` on SmartDashboard
3. Hold operator `Y` button, wait 3 seconds for spin-up
4. Click left stick to fire
5. Observe where the note lands relative to the target
6. Adjust RPS and retest:
   - **Shot too short** → Increase RPS by 5
   - **Shot too long** → Decrease RPS by 5
   - **Shot curving left/right** → Adjust rotation, not speed
7. Update the code with verified distances and speeds

### C. Intake Motion Magic & Gravity Tuning (`Intake.java`)

Located in `Intake()` constructor:

```java
deployerConfig.MotionMagic.MotionMagicCruiseVelocity = 0.5;
deployerConfig.MotionMagic.MotionMagicAcceleration = 0.1;
deployerConfig.Slot0.kP = 5.0;
deployerConfig.Slot0.kG = 0.2;  // ← Gravity feedforward (critical)
```

**kP Tuning (Position Accuracy):**
1. Open SmartDashboard, navigate to `Intake/Arm Target Position`
2. Manually click right stick to toggle deploy/stow
3. Watch the arm snap to position. Does it:
   - **Arrive late** → Increase `kP` (faster response)
   - **Bounce oscillate** → Decrease `kP` (overdamped)
   - **Hit smoothly** → Done!

**kG Tuning (Gravity Compensation - Most Important):**
1. Set both `kP` and `kV` to 0 in `deployerConfig`
2. Command the arm to mid-position (0.125 rotations)
3. Let it stabilize and disconnect the operator joystick
4. Arm should **float in place** without falling
5. If it falls:
   - Increase `kG` by +0.05, retest
   - Repeat until arm holds position indefinitely
6. Once stable, set `kP = 5.0` and `kV = 0.1` for responsiveness

### D. Current Limit Verification

Current limits prevent motor burnout and PDP breaker trips. Verify these are set correctly:

| Subsystem | Motor | Limit | Location |
|-----------|-------|-------|----------|
| Shooter Flywheels | IDs 16, 999 | 60A | `Shooter.java` constructor, `leaderConfig` |
| Shooter Intake | ID 34 | 40A | `Shooter.java` constructor, `feedConfig` |
| Hopper Belt | ID 35 | 40A | `Shooter.java` constructor, `feedConfig` |
| Intake Deployer | ID 13 | 40A | `Intake.java` constructor, `deployerConfig` |

If any breaker trips during operation, increase the corresponding limit by 5A and retest.

### E. Limelight Offset Configuration (Web GUI)

The Java code assumes the Limelights know they are angled. If the physical Limelights are reset or swapped, you MUST update their internal web dashboards:

**For Left Limelight (`limelight-left.local:5801`):**
1. Click **3D** tab
2. Set **Yaw Offset**: `18` degrees
3. Click **Save**

**For Right Limelight (`limelight-right.local:5801`):**
1. Click **3D** tab
2. Set **Yaw Offset**: `-18` degrees
3. Click **Save**

**Verification:**
1. Place the robot on field and spin it in place 360°
2. Watch the AdvantageScope 3D robot visualization
3. If the robot icon **spins cleanly in place** → Correct!
4. If the robot icon **orbits the center** → Adjust Yaw by ±5° and retest

### F. AdvantageScope Field Visualization

The real-time field visualization is powered by the `SwerveDrivePoseEstimator` in `Robot.java`.

**To view your robot moving on the field:**
1. Open **AdvantageScope**
2. Click **File** → **Connect to Robot** → Enter team number `6988`
3. Create a new **2D Field** tab
4. Drag `SmartDashboard/Field` into the main view
5. You should see a robot icon that moves and rotates as the physical robot does

**If the robot doesn't appear or moves wrong:**
- Verify drivetrain is working (`Swerve/GyroDegrees` updates on SmartDashboard)
- Check that Pigeon 2 is connected (should see angle change when you spin the robot)
- Verify swerve module positions are reporting (should see values in 0-1 range)
- Reset the pose by pressing the `Start` button on driver joystick at match start

---

## 6. Patch Notes & Recent Updates

### v2.0 - February 2026 (Current Build)

#### [FIX] Manual Velocity Drop / Jitter
* **Bug:** When the operator held a preset button (e.g., 60 RPS) and clicked the Left Stick to fire, the speed would momentarily drop or change due to state-transition latency.
* **Root Cause:** The `currentTargetSpeed` variable was being recalculated during the state transition from `Rev` to `Shoot`, causing speed fluctuations.
* **Fix:** Streamlined the `Shooter.java` mainloop. The `currentTargetSpeed` is now locked in at the very start of the loop (`currentTargetSpeed = (this.custom_speed > 0) ? this.custom_speed : manualSpeed;`), ensuring the speed remains perfectly constant through state transitions.
* **Impact:** Shots are now accurate and consistent. No more unexpected speed variations.

#### [UPGRADE] 4-Motor Shooter Integration
* **Added:** The `shooterIntakeKraken` (ID: 34) and `hopperBeltFalcon` (ID: 35) are now fully integrated into the `Shooter.java` state machine.
* **Behavior:** 
  - During `ShooterMode.Rev`: Intake and hopper remain at 0 RPS (safe revving without feeding)
  - During `ShooterMode.Shoot`: Both spin at exactly 40 RPS (synchronized feeding)
  - During `ShooterMode.Reverse`: Both spin at -30 RPS (synchronized unjamming)
* **Benefit:** All four shooter motors now act as a unified system. No more misaligned feeding or partial jams.

#### [UPGRADE] Unified Reverse Protocol & Full-Range Joystick Control
* **Previous Issue:** The shooter reverse logic used the Right Y-Axis, which conflicted with Intake Arm manual jogging.
* **New Feature:** The shooter now uses **full -60 to +60 RPS range** on the Left Y-Axis:
  * Push stick UP = positive speed (launch forward)
  * Pull stick DOWN = negative speed (reverse/unjam)
* **Smart Reverse Mode:** When the operator pulls the left stick down (negative speed) and clicks the left stick button:
  - `ShooterMode.Reverse` is triggered
  - The main flywheels spin at the **exact negative speed** from the joystick (not a fixed -10 RPS)
  - Intake Kraken and Hopper Belt spin at -30 RPS (optimal unjamming speed)
  - **Result:** Aggressive, variable-speed unjamming without losing speed control
* **Code:** In `Remote.java`:
  ```java
  if (operatorJoystickDef.getLeftStickButton()) {
      if (manual_shooter_speed < 0) {
          shooter_mode = ShooterMode.Reverse;  // Negative speed = reverse everything
      } else {
          shooter_mode = ShooterMode.Shoot;    // Positive speed = normal fire
      }
  }
  ```
* **Impact:** More intuitive shooter control. Pull-down-to-reverse makes physical sense on the stick.

#### [UPGRADE] Context-Aware "Smart Toggle" for Intake Deploy/Stow
* **Previous Issue:** The Right Stick Click used a blind boolean toggle (`shouldDeploy = !shouldDeploy`). If the arm was manually jogged, the toggle would get out of sync and push the arm the wrong way.
* **New Feature:** Replaced the boolean with **encoder position logic**:
  ```java
  double deployThreshold = robotContainer.m_intake.getDeployPosition() * 0.5;
  if(robotContainer.m_intake.intakeDeployer.getPosition().getValueAsDouble() < deployThreshold) {
      intake_deploy_mode = IntakeDeployMode.Deploy;
  } else {
      intake_deploy_mode = IntakeDeployMode.Stow;
  }
  ```
* **Behavior:** When you click right stick, the code reads the **actual arm position**:
  - If arm is closer to stowed (< 0.125) → Deploy to 0.25
  - If arm is closer to deployed (≥ 0.125) → Stow to 0.0
* **Benefit:** The arm **always moves the correct direction**. No more accidentally flipping the toggle.

#### [SAFETY] Stator Current Limits
* **Added:** Hardcoded 40A `StatorCurrentLimit`s to the Intake Kraken (ID: 34) and Hopper Falcon (ID: 35) in `Shooter.java`:
  ```java
  feedConfig.CurrentLimits.StatorCurrentLimit = 40;
  feedConfig.CurrentLimits.StatorCurrentLimitEnable = true;
  ```
* **Reason:** If a game piece gets violently wedged during aggressive defense or jamming scenarios, the motor can draw unlimited current and pop the main breaker.
* **Benefit:** Protects PDP and enables safer, more aggressive play without risk of electrical shutdown.

#### [FEATURE] Real-Time Field Visualization in AdvantageScope
* **Added:** `SwerveDrivePoseEstimator` in `Robot.java` that fuses Pigeon 2 gyro data with swerve module positions
* **Implementation:**
  - `robotPeriodic()` updates pose every loop
  - Gyro angle + module positions → calculated robot pose
  - Pose published to SmartDashboard as `Field` object
* **Benefit:** See your robot moving and rotating on the field in AdvantageScope in real-time. Perfect for autonomous debugging and visualizing path following.

---

## 🔧 Contact & Support

For questions about this codebase or subsystem tuning, contact the FRC 6988 programming team.

**Team:** FRC 6988  
**Year:** 2026  
**Last Updated:** February 2026

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
