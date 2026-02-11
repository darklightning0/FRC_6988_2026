# AdvantageScope Setup - FRC 6988 2026

## ✅ Completed Implementation

All three parts of the AdvantageScope visualization have been implemented in your robot code.

### Part 1: Basic Data (Mechanisms & Gyro) ✓

**Added to `Intake.java`:**
```java
public double getMotorOutputPercent() {
    return pulleyCIM.getMotorOutputPercent();
}
```

**Added to `Shooter.java`:**
```java
public double getMotorOutputPercent() {
    return leftMain.getMotorOutputPercent();
}
```

**Added to `Robot.teleopPeriodic()`:**
- `SmartDashboard.putString("Intake/Mode", intakeMode.toString());`
- `SmartDashboard.putString("Shooter/Mode", shooterMode.toString());`
- `SmartDashboard.putNumber("Intake/Percent", m_robotContainer.m_intake.getMotorOutputPercent());`
- `SmartDashboard.putNumber("Shooter/Percent", m_robotContainer.m_shooter.getMotorOutputPercent());`
- `SmartDashboard.putNumber("Swerve/GyroDegrees", m_robotContainer.pigeon2.getYaw());`

### Part 2: Advanced Real-Time Field Visualization ✓

**Added imports to `Robot.java`:**
```java
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
```

**Added fields to `Robot.java`:**
```java
private final Field2d m_field = new Field2d();
private SwerveDrivePoseEstimator m_poseEstimator;
```

**Updated `Robot.robotInit()`:**
```java
// Initialize Pose Estimator for AdvantageScope
m_poseEstimator = new SwerveDrivePoseEstimator(
    m_robotContainer.drivetrain.getKinematics(),
    Rotation2d.fromDegrees(m_robotContainer.pigeon2.getYaw()),
    m_robotContainer.drivetrain.getState().ModulePositions,
    new Pose2d() // Start at 0,0
);

// Put the Field object to SmartDashboard so AdvantageScope can find it
SmartDashboard.putData("Field", m_field);
```

**Updated `Robot.robotPeriodic()` to fuse Pigeon 2 data with Swerve module positions:**
```java
// 1. Get Current Gyro Angle
Rotation2d gyroAngle = Rotation2d.fromDegrees(m_robotContainer.pigeon2.getYaw());

// 2. Get Module Positions from the Drivetrain
var modulePositions = m_robotContainer.drivetrain.getState().ModulePositions;

// 3. Update the Pose Estimator
m_poseEstimator.update(gyroAngle, modulePositions);

// 4. Update the Field Object with the calculated pose
m_field.setRobotPose(m_poseEstimator.getEstimatedPosition());
```

## 🚀 Using AdvantageScope

### Step 1: Connect to Robot
1. Open **AdvantageScope**
2. Click **File** → **Connect to Robot**
3. Enter your team number: **6988**
4. Click **Connect**

### Step 2: View Basic Mechanism Data
1. Create a new **Line Graph** tab
2. In the sidebar under **SmartDashboard**, find:
   - `Intake/Mode` - Current intake mode (Intake/Reverse/Idle)
   - `Intake/Percent` - Intake motor output percentage
   - `Shooter/Mode` - Current shooter mode (Shoot/Reverse/Idle)
   - `Shooter/Percent` - Shooter motor output percentage
   - `Swerve/GyroDegrees` - Current gyro heading in degrees
3. Drag these into the graph to visualize them over time

### Step 3: View Real-Time Field Visualization (The Cool Part!)
1. Create a new **2D Field** tab
2. In the sidebar under **SmartDashboard**, find:
   - `Field` - The robot's real-time position and rotation
3. Drag the **Field** object into the main view
4. Now you'll see:
   - A **robot icon** that moves and rotates exactly as your real robot does
   - Real-time position tracking
   - Rotation/heading updates from your Pigeon 2 gyro
   - Perfect for debugging autonomous routines!

## 🔍 What Data is Being Used

The field visualization is powered by:
- **Gyro Angle**: From your Pigeon 2 IMU (`pigeon2.getYaw()`)
- **Module Positions**: From your swerve drivetrain (`drivetrain.getState().ModulePositions`)
- **Kinematics**: From your swerve drivetrain configuration

Together, these form a **Pose Estimator** that calculates your robot's position on the field in real-time.

## 🐛 Troubleshooting

### Robot doesn't appear on field?
- Make sure your robot is powered on and connected
- Check that the drivetrain is working (run your default swerve command)
- Verify SmartDashboard is receiving data by checking other telemetry first

### Field position is wrong?
- Your gyro angle might need to be zeroed
- Call `m_robotContainer.pigeon2.setYaw(0)` at the start of match
- This is already done in `autonomousInit()`

### Data not updating?
- Make sure `Robot.robotPeriodic()` is being called
- Check the event log in AdvantageScope for connection issues

## 📊 SmartDashboard Key Format

All data is published to SmartDashboard with these keys:
```
SmartDashboard/Intake/Mode       (String)
SmartDashboard/Intake/Percent    (Number)
SmartDashboard/Shooter/Mode      (String)
SmartDashboard/Shooter/Percent   (Number)
SmartDashboard/Swerve/GyroDegrees (Number)
SmartDashboard/Field             (Field2d Object)
```

## ✨ Next Steps

- Use AdvantageScope to debug your autonomous routines
- Verify your swerve drive kinematics are correct
- Test that your gyro is zeroing properly at match start
- Visualize intake/shooter operations alongside field movement

---

**Implementation Date**: February 11, 2026
**Team**: FRC 6988
**Status**: ✅ Ready for field testing
