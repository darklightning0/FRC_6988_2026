// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.subsystems.Remote.IntakeArmMode;
import frc.robot.subsystems.Remote.IntakeWheelMode;
import frc.robot.subsystems.Remote.ShooterMode;
import frc.robot.subsystems.Remote.IntakeMode;

import edu.wpi.first.cameraserver.CameraServer;
import com.ctre.phoenix.sensors.PigeonIMU;


public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  private final boolean kUseLimelight = false;

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
	public void robotInit() {
		CameraServer.startAutomaticCapture();
    LimelightHelpers.setPipelineIndex("limelight",0);
  
	}

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    double [] ypr = new double[3];
		m_robotContainer.pigeon2.getYawPitchRoll(ypr);
		
		SmartDashboard.putNumber("pigeonYaw", ypr[0]);
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
    m_robotContainer.m_innerElevator.config();
		m_robotContainer.m_outerElevator.config();
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

		SmartDashboard.putString("input_shooterMode", shooterMode.toString());
		SmartDashboard.putString("input_intakeWheelMode", intakeWheelMode.toString());
		SmartDashboard.putString("input_intakeArmMode", intakeArmMode.toString());
		SmartDashboard.putNumber("input_innerElevatorTarget", innerElevatorTarget);
		SmartDashboard.putNumber("input_outerElevatorTarget", outerElevatorTarget);
		SmartDashboard.putBoolean("input_elevatorManual", m_robotContainer.m_remote.getElevatorManual());
    SmartDashboard.putString("input_intakeMode", intakeMode.toString());

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

		// Outer Elevator
		m_robotContainer.m_outerElevator.setEnabled(true);
		m_robotContainer.m_outerElevator.mainloop(outerElevatorTarget, m_robotContainer.m_remote.getHomeButtonPressed());



		// Inner Elevator
		m_robotContainer.m_innerElevator.setEnabled(false);
		// m_robotContainer.m_innerElevator.setTargetPos(0.40);
		m_robotContainer.m_innerElevator.mainloop(innerElevatorTarget, m_robotContainer.m_remote.getHomeButton());

		// Shooter
		m_robotContainer.m_shooter.mainloop(shooterMode);
    //Intake
    m_robotContainer.m_intake.mainloop(intakeMode);

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
