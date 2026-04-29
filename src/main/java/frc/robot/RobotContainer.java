// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Remote;
import frc.robot.subsystems.Remote.IntakeDeployMode;
import frc.robot.subsystems.Remote.IntakeMode;
import frc.robot.subsystems.Remote.ShooterMode;
import frc.robot.subsystems.Shooter;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.FieldCentricFacingAngle driveFacingAngle = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.05)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driverJoystick = new CommandXboxController(0);

    
    public final Remote m_remote = new Remote();
    public final Shooter m_shooter = new Shooter();
    public final Intake m_intake = new Intake();

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

      private ShooterMode autoShooterMode= ShooterMode.Idle;
    private IntakeMode autoIntakeMode = IntakeMode.Idle;
    private IntakeDeployMode autoIntakeDeployMode = IntakeDeployMode.Stow;
     private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        
        NamedCommands.registerCommand("IntakeDeploy", Commands.runOnce(() -> {
            autoIntakeDeployMode = IntakeDeployMode.Deploy; 
        }));
        NamedCommands.registerCommand("IntakeStow", Commands.runOnce(() -> {
            autoIntakeDeployMode = IntakeDeployMode.Stow;
        }));

 
        NamedCommands.registerCommand("IntakeStart", Commands.runOnce(() -> {
            autoIntakeMode = IntakeMode.Intake;
        }));
        NamedCommands.registerCommand("IntakeStop", Commands.runOnce(() -> {
            autoIntakeMode = IntakeMode.Idle;
        }));
        NamedCommands.registerCommand("IntakeReverse", Commands.runOnce(() -> {
            autoIntakeMode = IntakeMode.Reverse;
        }));

        //dual command
        NamedCommands.registerCommand("IntakeFull", Commands.runOnce(() -> {
            autoIntakeDeployMode = IntakeDeployMode.Deploy; 
            autoIntakeMode = IntakeMode.Intake;
        }));
        NamedCommands.registerCommand("IntakeSafe", Commands.runOnce(() -> {
            autoIntakeDeployMode = IntakeDeployMode.Stow; 
            autoIntakeMode = IntakeMode.Idle;
        }));

    
        NamedCommands.registerCommand("ShooterRevVision", Commands.runOnce(() -> {
            // Tells the shooter to rev based on the Limelight distance!
            m_shooter.setCustomSpeed(0); // 0 triggers the vision math in Robot.java
            autoShooterMode = ShooterMode.Rev;
        }));
        NamedCommands.registerCommand("ShooterShoot", Commands.runOnce(() -> {
            autoShooterMode = ShooterMode.Shoot;
        }));
        NamedCommands.registerCommand("ShooterStop", Commands.runOnce(() -> {
            autoShooterMode = ShooterMode.Idle;
        }));
        NamedCommands.registerCommand("ShooterReverse", Commands.runOnce(() -> {
            autoShooterMode = ShooterMode.Reverse;
        }));


        NamedCommands.registerCommand("ShooterRev1", Commands.runOnce(() -> {
            m_shooter.setCustomSpeed(35.0); 
            autoShooterMode = ShooterMode.Rev;
        }));
        NamedCommands.registerCommand("ShooterRev2", Commands.runOnce(() -> {
            m_shooter.setCustomSpeed(50.0); 
            autoShooterMode = ShooterMode.Rev;
        }));

     
        NamedCommands.registerCommand("StopAll", Commands.runOnce(() -> {
            m_shooter.setCustomSpeed(0);
            autoShooterMode = ShooterMode.Idle;
            autoIntakeMode = IntakeMode.Idle;
            autoIntakeDeployMode = IntakeDeployMode.Stow;
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
       public IntakeDeployMode getAutoDeployMode(){
        return autoIntakeDeployMode;
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
            drivetrain.applyRequest(() -> {
                Pose2d currentPose = drivetrain.getState().Pose;

                Translation2d speakerLocation;
                var alliance = DriverStation.getAlliance();
                
                if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
                    speakerLocation = new Translation2d(11.9, 4.0); //red alliance kule
                } else {
                    speakerLocation = new Translation2d(4.62, 4.0);   //blue alliance kule
                }

                // calc difference in x and y values
                Translation2d difference = speakerLocation.minus(currentPose.getTranslation());

                Rotation2d targetAngle = difference.getAngle();
                
                return driveFacingAngle.withVelocityX(-driverJoystick.getLeftY() * MaxSpeed)
                        .withVelocityY(-driverJoystick.getLeftX() * MaxSpeed)
                        .withTargetDirection(targetAngle);
            })
        );
        

        
    }

    public Command getAutonomousCommand() {
        // Simple drive forward auton
        final var idle = new SwerveRequest.Idle();
        
        return Commands.sequence(
            /* 
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
            */

            // Finally idle for the rest of auton
            drivetrain.applyRequest(() -> idle)
        );
    }
}
