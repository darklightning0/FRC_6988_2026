// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Remote;
import frc.robot.subsystems.Remote.ClimbMode;
import frc.robot.subsystems.Remote.HoodMode;

import frc.robot.subsystems.Remote.ShooterMode;
import frc.robot.subsystems.Remote.IntakeMode;
import frc.robot.subsystems.Remote.IntakeDeployMode;
import edu.wpi.first.cameraserver.CameraServer;

import static frc.robot.Constants.ControllerConstants.driverJoystick;
import static frc.robot.Constants.ControllerConstants.operatorJoystick;

import com.ctre.phoenix6.SignalLogger;

// AdvantageScope visualization imports

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;


public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  // AdvantageScope Field Visualization
  private final Field2d m_field = new Field2d();


  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
	public void robotInit() {
		CameraServer.startAutomaticCapture();

    m_robotContainer.pigeon2.setYaw(0.0);
    m_robotContainer.m_intake.resetStartEncoder();

    // Initialize Pose Estimator for AdvantageScope
    

    SignalLogger.start();
    SignalLogger.setPath("/home/lvuser/logs/");

    // Put the Field object to SmartDashboard so AdvantageScope can find it
    SmartDashboard.putData("input_field", m_field);
	}

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();


  
    double currentYaw = m_robotContainer.pigeon2.getYaw().getValueAsDouble();
    LimelightHelpers.SetRobotOrientation("limelight-left", currentYaw, 0, 0, 0, 0, 0);
    LimelightHelpers.SetRobotOrientation("limelight-right", currentYaw, 0, 0, 0, 0, 0);

    // 1. Read Left Camera
    LimelightHelpers.PoseEstimate mt2Left = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-left");

    if (mt2Left != null && mt2Left.tagCount > 0) {
        // Tell the swerve drivetrain where the camera thinks we are
        m_robotContainer.drivetrain.addVisionMeasurement(mt2Left.pose, mt2Left.timestampSeconds);
    }

    // 2. Read Right Camera
    LimelightHelpers.PoseEstimate mt2Right = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-right");
    if (mt2Right != null && mt2Right.tagCount > 0) {
        m_robotContainer.drivetrain.addVisionMeasurement(mt2Right.pose, mt2Right.timestampSeconds);
    }

    // 3. Make AdvantageScope display the newly corrected 3D position
    m_field.setRobotPose(m_robotContainer.drivetrain.getState().Pose);
    // ========================================================

    SmartDashboard.putNumber("input_pigeonYaw", m_robotContainer.pigeon2.getYaw().getValueAsDouble());
    SmartDashboard.putNumber("input_pigeonPitch", m_robotContainer.pigeon2.getPitch().getValueAsDouble());

    /*
     * This example of adding Limelight is very simple and may not be sufficient for on-field use.
     * Users typically need to provide a standard deviation that scales with the distance to target
     * and changes with number of tags available.
    *
     * This example is sufficient to show that vision integration is possible, though exact implementation
     * of how to use vision should be tuned per-robot and to the team's specification.
     */

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
    } //onemli degil aq bu efe kapukulak

    //autoInit = getTime();

  }

  @Override
  public void autonomousPeriodic() {

    //double time = getTime() - autoInit;
    //ShooterMode shooterMode = ShooterMode.Idle;

    

    



    

    //IntakeArmMode intakeArmMode=IntakeArmMode.Idle;
    //m_robotContainer.m_intakeArm.mainloop(intakeArmMode);

    ShooterMode autoShooterMode=m_robotContainer.getAutoShooterMode();

    IntakeMode autoIntakeMode=m_robotContainer.getAutoIntakeMode();
    IntakeDeployMode autoIntakeDeployMode=m_robotContainer.getAutoDeployMode();
    
    m_robotContainer.m_shooter.mainloop(autoShooterMode, 0.0);
    m_robotContainer.m_intake.mainloop(autoIntakeMode,autoIntakeDeployMode);
    
    
  }

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

  }

  @Override
  public void teleopPeriodic() {
    m_robotContainer.m_remote.mainloop(m_robotContainer);
		ShooterMode shooterMode = m_robotContainer.m_remote.getShooterMode();
    IntakeMode intakeMode = m_robotContainer.m_remote.getIntakeMode();
    IntakeDeployMode intakeDeployMode = m_robotContainer.m_remote.getIntakeDeployMode();
    ClimbMode climbMode = m_robotContainer.m_remote.getClimbMode();
    HoodMode hoodMode = m_robotContainer.m_remote.getHoodMode();

		// 1. Log Mechanism Modes (Strings)
    SmartDashboard.putString("input_intake_mode", intakeMode.toString());
    SmartDashboard.putString("input_shooter_mode", shooterMode.toString());
    SmartDashboard.putString("input_climb_mode", climbMode.toString());
    SmartDashboard.putString("input_hood_mode", hoodMode.toString());


    // 2. Log Percent Outputs (Numbers) for AdvantageScope

    SmartDashboard.putNumber("input_shooter_percent", m_robotContainer.m_shooter.getMotorOutputPercent());
 

    // 3. Log Gyro (Pigeon 2)
    SmartDashboard.putNumber("input_gyro_degrees", m_robotContainer.pigeon2.getYaw().getValueAsDouble()); 



    SmartDashboard.putNumber("Vision Target TX", LimelightHelpers.getTX("limelight-left"));

  

    // ==========================================
    // 2. SHOOTER CONTROL (Auto-Interpolation vs Manual)
    // ==========================================
    // Check if the operator is holding the Y button to auto-align and shoot
    // Check if the operator is holding the Y button to auto-align and shoot
    if (operatorJoystick.rightBumper().getAsBoolean()) {
        
        // 1. Get our 3D location
        Pose2d currentPose = m_robotContainer.drivetrain.getState().Pose;
        
        // 2. MATCH ROBOT CONTAINER: Determine target based on Alliance
        Translation2d targetLocation;
        var alliance = edu.wpi.first.wpilibj.DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == edu.wpi.first.wpilibj.DriverStation.Alliance.Red) {
            targetLocation = new Translation2d(11.9, 4.0); // Red Target
        } else {
            targetLocation = new Translation2d(4.62, 4.0); // Blue Target
        }


        double distanceMeters = targetLocation.getDistance(currentPose.getTranslation());
        SmartDashboard.putNumber("Vision/True Distance In Meters", distanceMeters);

        //interpolation map dist vs velc
        double requiredSpeed = m_robotContainer.m_remote.tyToShooterSpeed.get(distanceMeters);
        m_robotContainer.m_shooter.setCustomSpeed(requiredSpeed);


        Translation2d difference = targetLocation.minus(currentPose.getTranslation());
        Rotation2d targetAngle = difference.getAngle();
        double aimErrorDegrees = Math.toDegrees(Math.abs(targetAngle.minus(currentPose.getRotation()).getRadians()));
        
        // if angle is tolerable allow shoot
        boolean aimed = aimErrorDegrees < 4.0 ;

        if (m_robotContainer.m_shooter.isAtSpeed() && aimed) {
            m_robotContainer.m_shooter.mainloop(Remote.ShooterMode.Shoot, 0); // FIRLAT AMK
        } else {
            m_robotContainer.m_shooter.mainloop(Remote.ShooterMode.Rev, 0);   // DUR LAN YA AIMLI DEGIL YA DA HIZ PROBLEM
        }

    } else {
        //if nothing is pressed use normal yani
        m_robotContainer.m_shooter.setCustomSpeed(0.0); 
        m_robotContainer.m_shooter.mainloop(shooterMode, m_robotContainer.m_remote.getManualShooterSpeed());
    }



    //Intake reset encoder if necessary mango
    m_robotContainer.m_intake.mainloop(intakeMode, intakeDeployMode);
    if (m_robotContainer.m_remote.getHomeButton()) {
        m_robotContainer.m_intake.resetDeployerEncoder();
    }

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

