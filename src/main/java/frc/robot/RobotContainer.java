// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

//import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
//import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import static frc.robot.Constants.ControllerConstants.driverJoystick;
import static frc.robot.Constants.ControllerConstants.driverJoystickID;
import static frc.robot.Constants.ControllerConstants.operatorJoystick;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Remote;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Remote.IntakeMode;
import frc.robot.subsystems.Remote.ShooterMode;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.35).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    PigeonIMU pigeon2 = new PigeonIMU(21);
    

    public final Remote m_remote = new Remote();
    public final Shooter m_shooter = new Shooter();
    public final Intake m_intake = new Intake();
    public final Hood m_hood = new Hood();
    public final Climb m_climb = new Climb();

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    private ShooterMode autoShooterMode= ShooterMode.Idle;
    private IntakeMode autoIntakeMode = IntakeMode.Idle;

    public RobotContainer() {
        
       /*  NamedCommands.registerCommand("ShooterAuto", Commands.run(() -> {
             double rotRate = 0;
                if(LimelightHelpers.getTV("limelight")){
                    rotRate = -LimelightHelpers.getTX("limelight") * 0.03;
                }

                drive.withVelocityX( 0)
                .withVelocityY(0)
                .withRotationalRate(rotRate * MaxAngularRate);
        }));
        */

        NamedCommands.registerCommand("ShooterReverse", Commands.runOnce(() -> {
            autoShooterMode = ShooterMode.Shoot;
        }));
         NamedCommands.registerCommand("ShooterShoot", Commands.runOnce(() -> {
            autoShooterMode = ShooterMode.Reverse;
        }));

             NamedCommands.registerCommand("IntakeStart", Commands.runOnce(() -> {
            autoIntakeMode= IntakeMode.Intake;
        }));
        
             NamedCommands.registerCommand("IntakeStop", Commands.runOnce(() -> {
            autoIntakeMode= IntakeMode.Idle ;
        }));
          NamedCommands.registerCommand("ClimbAuto", Commands.runOnce(() -> {
        
        }));
        

        autoChooser = AutoBuilder.buildAutoChooser("efekapi");
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();
    }

    public ShooterMode getAutoShooterMode(){
        return autoShooterMode;
    }
     public IntakeMode getAutoIntakeMode(){
        return autoIntakeMode;
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driverJoystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driverJoystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driverJoystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        driverJoystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driverJoystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driverJoystick.getLeftY(), -driverJoystick.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driverJoystick.back().and(driverJoystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverJoystick.back().and(driverJoystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverJoystick.start().and(driverJoystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverJoystick.start().and(driverJoystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        driverJoystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);

        
        driverJoystick.y().whileTrue(
            Commands.runOnce(()->LimelightHelpers.setPipelineIndex("limelight", 1))
            .andThen(
                drivetrain.applyRequest(() -> {
                double rotRate = 0;
                if(LimelightHelpers.getTV("limelight")){
                    rotRate = -LimelightHelpers.getTX("limelight") * 0.03;
                }

                return drive.withVelocityX(-driverJoystick.getLeftY() * MaxSpeed)
                .withVelocityY(-driverJoystick.getLeftX() * MaxSpeed)
                 .withRotationalRate(rotRate * MaxAngularRate);
            })
            )
        ).onFalse(Commands.runOnce(() -> LimelightHelpers.setPipelineIndex("limelight", 0))
        );
    }

    public Command getAutonomousCommand() {
        /*final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
            // Reset our field centric heading to match the robot
            // facing away from our alliance station wall (0 deg).
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
            // Then slowly drive forward (away from us) for 5 seconds.
            drivetrain.applyRequest(() ->
                drive.withVelocityX(0.5)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            )
            .withTimeout(5.0),
            // Finally idle for the rest of auton
            drivetrain.applyRequest(() -> idle)
        );*/
        return autoChooser.getSelected();
    }
}
