// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Remote;
import frc.robot.subsystems.Remote.ClimbMode;
import frc.robot.subsystems.Remote.HoodMode;
import frc.robot.subsystems.Remote.IntakeArmMode;
import frc.robot.subsystems.Remote.IntakeWheelMode;
import frc.robot.subsystems.Remote.ShooterMode;
import frc.robot.subsystems.Remote.IntakeMode;

import edu.wpi.first.cameraserver.CameraServer;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix6.SignalLogger;

// AdvantageScope visualization imports
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;


public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  private final boolean kUseLimelight = false;

  // AdvantageScope Field Visualization
  private final Field2d m_field = new Field2d();
  private SwerveDrivePoseEstimator m_poseEstimator;

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
	public void robotInit() {
		CameraServer.startAutomaticCapture();
    LimelightHelpers.setPipelineIndex("limelight",1);
    m_robotContainer.pigeon2.setYaw(0.0);

    // Initialize Pose Estimator for AdvantageScope
    m_poseEstimator = new SwerveDrivePoseEstimator(
        m_robotContainer.drivetrain.getKinematics(),
        Rotation2d.fromDegrees(m_robotContainer.pigeon2.getYaw()),
        m_robotContainer.drivetrain.getState().ModulePositions,
        new Pose2d() // Start at 0,0
    );

    SignalLogger.start();
    SignalLogger.setPath("/home/lvuser/logs/");

    // Put the Field object to SmartDashboard so AdvantageScope can find it
    SmartDashboard.putData("input_field", m_field);
	}

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    // 1. Get Current Gyro Angle
    Rotation2d gyroAngle = Rotation2d.fromDegrees(m_robotContainer.pigeon2.getYaw());

    // 2. Get Module Positions from the Drivetrain
    var modulePositions = m_robotContainer.drivetrain.getState().ModulePositions;

    // 3. Update the Pose Estimator
    m_poseEstimator.update(gyroAngle, modulePositions);

    // 4. Update the Field Object with the calculated pose
    m_field.setRobotPose(m_poseEstimator.getEstimatedPosition());

    // Debug telemetry
    double [] ypr = new double[3];
		m_robotContainer.pigeon2.getYawPitchRoll(ypr);
		SmartDashboard.putNumber("input_pigeonYaw", ypr[0]);
    System.out.println(LimelightHelpers.getTX("limelight"));
    System.out.println(LimelightHelpers.getTY("limelight"));
    /*
     * This example of adding Limelight is very simple and may not be sufficient for on-field use.
     * Users typically need to provide a standard deviation that scales with the distance to target
     * and changes with number of tags available.
    *
     * This example is sufficient to show that vision integration is possible, though exact implementation
     * of how to use vision should be tuned per-robot and to the team's specification.
     */
    if (kUseLimelight) {
      var driveState = m_robotContainer.drivetrain.getState();
      double headingDeg = driveState.Pose.getRotation().getDegrees();
      double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);

      LimelightHelpers.SetRobotOrientation("limelight", headingDeg, 0, 0, 0, 0, 0);
      var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
      if (llMeasurement != null && llMeasurement.tagCount > 0 && Math.abs(omegaRps) < 2.0) {
        m_robotContainer.drivetrain.addVisionMeasurement(llMeasurement.pose, llMeasurement.timestampSeconds);
      }
    }
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  //double autoInit = 0;

  //double getTime(){

    //return ((double)System.currentTimeMillis()) / 1000;

  //+}

  

  @Override
  public void autonomousInit() {
    m_robotContainer.pigeon2.setYaw(0);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();


    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

    //autoInit = getTime();

  }

  @Override
  public void autonomousPeriodic() {

    //double time = getTime() - autoInit;
    //ShooterMode shooterMode = ShooterMode.Idle;

    

    



    

    //IntakeArmMode intakeArmMode=IntakeArmMode.Idle;
    //m_robotContainer.m_intakeArm.mainloop(intakeArmMode);

    ShooterMode autoShooterMode=m_robotContainer.getAutoShooterMode();
    
    m_robotContainer.m_shooter.mainloop(autoShooterMode);
    
  }

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
		m_robotContainer.m_remote.resetTargets();

  }

  @Override
  public void teleopPeriodic() {
    m_robotContainer.m_remote.mainloop();
		double innerElevatorTarget = m_robotContainer.m_remote.getInnerElevatorTarget();
		double outerElevatorTarget = m_robotContainer.m_remote.getOuterElevatorTarget();
		IntakeWheelMode intakeWheelMode = m_robotContainer.m_remote.getIntakeWheelMode();
		IntakeArmMode intakeArmMode = m_robotContainer.m_remote.getIntakeArmMode();
		ShooterMode shooterMode = m_robotContainer.m_remote.getShooterMode();
    IntakeMode intakeMode = m_robotContainer.m_remote.getIntakeMode();
    ClimbMode climbMode = m_robotContainer.m_remote.getClimbMode();

		// 1. Log Mechanism Modes (Strings)
    SmartDashboard.putString("input_intake_mode", intakeMode.toString());
    SmartDashboard.putString("input_shooter_mode", shooterMode.toString());
    SmartDashboard.putString("input_climb_mode", climbMode.toString());


    // 2. Log Percent Outputs (Numbers) for AdvantageScope
    SmartDashboard.putNumber("input_intake_percent", m_robotContainer.m_intake.getMotorOutputPercent());
    SmartDashboard.putNumber("input_shooter_percent", m_robotContainer.m_shooter.getMotorOutputPercent());

    // 3. Log Gyro (Pigeon 2)
    SmartDashboard.putNumber("input_gyro_degrees", m_robotContainer.pigeon2.getYaw());



		// Check ultrasonic sensor
		// m_robotContainer.m_ultrasonicSensor.measureDistance();
		// double ultrasonicDistance =
		// m_robotContainer.m_ultrasonicSensor.getDistanceCm();
		// boolean objectSeen = ultrasonicDistance <
		// Constants.SubsystemConstants.Other.ULTRASONIC_DETECTION_THRESHOLD_CM;

		// Inner Elevator PID Tuning
		// if (driverJoystickDef.getYButton()) {
		// m_robotContainer.m_innerElevator.config();
		// }

    double currentTY = LimelightHelpers.getTY("limelight");

    if(m_robotContainer.m_remote.hood_mode == HoodMode.AutoAim){
      if(LimelightHelpers.getTV("limelight")){
        double requiredTicks = m_robotContainer.m_remote.tyToHoodTicks.get(currentTY);
        m_robotContainer.m_hood.setTargetTicks(requiredTicks);

        double requireSpeed = m_robotContainer.m_remote.tyToShooterSpeed.get(currentTY);
        m_robotContainer.m_shooter.setCustomSpeed(requireSpeed);

        boolean isHoodReady = m_robotContainer.m_hood.atTarget();
        boolean isRobotAimed = Math.abs(LimelightHelpers.getTX("limelight")) < 2.0;

        if (isHoodReady && isRobotAimed){
          m_robotContainer.m_shooter.mainloop(Remote.ShooterMode.Shoot);
        } else {
          m_robotContainer.m_shooter.mainloop(Remote.ShooterMode.Rev);
        }
      }
     else { 
    if(m_robotContainer.m_remote.hood_mode == HoodMode.ManualUp){
      m_robotContainer.m_hood.adjustTicks(300);
    } else if(m_robotContainer.m_remote.hood_mode == HoodMode.ManualDown){
      m_robotContainer.m_hood.adjustTicks(-300);
    } else if(m_robotContainer.m_remote.hood_mode == HoodMode.ReturnToZero){
      m_robotContainer.m_hood.setTargetTicks(0);
    }
    // Shooter
		m_robotContainer.m_shooter.mainloop(m_robotContainer.m_remote.getShooterMode());
  }
    m_robotContainer.m_hood.mainloop();
    }

    //Intake
    m_robotContainer.m_intake.mainloop(intakeMode);
    m_robotContainer.m_climb.mainloop(climbMode);

		// Intake wheel
		//m_robotContainer.m_intakeWheels.mainloop(intakeWheelMode);

		// Intake arm
		//m_robotContainer.m_intakeArm.mainloop(intakeArmMode);
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
