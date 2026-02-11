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

//import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
//import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import static frc.robot.Constants.ControllerConstants.driverJoystick;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.InnerElevator;
import frc.robot.subsystems.OuterElevator;
import frc.robot.subsystems.Remote;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;
//import frc.robot.subsystems.AlgeaIntake.IntakeArm;
//import frc.robot.subsystems.AlgeaIntake.IntakeWheels;
import frc.robot.subsystems.Remote.ShooterMode;
import frc.robot.limelight;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
        .withDeadband(MaxSpeed * 0.1)
        .withRotationalDeadband(MaxAngularRate * 0.1)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);


    PigeonIMU pigeon2 = new PigeonIMU(21);
    

    public final Remote m_remote = new Remote();
    public final OuterElevator m_outerElevator = new OuterElevator();
    public final InnerElevator m_innerElevator = new InnerElevator();
    //public final IntakeWheels m_intakeWheels = new IntakeWheels();
    //public final IntakeArm m_intakeArm = new IntakeArm();
    public final Shooter m_shooter = new Shooter();
    public final Intake m_intake = new Intake();
    public final limelight m_limelight = new limelight();

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    private ShooterMode autoShooterMode= ShooterMode.Idle;

    public RobotContainer() {
        NamedCommands.registerCommand("ShooterForward", Commands.runOnce(() -> {
            autoShooterMode = ShooterMode.Shoot;
        }));
        NamedCommands.registerCommand("ShooterReverse", Commands.runOnce(() -> {
            autoShooterMode = ShooterMode.Idle;
        }));

        autoChooser = AutoBuilder.buildAutoChooser("Taxi_Red");
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();
    }

    public ShooterMode getAutoShooterMode(){
        return autoShooterMode;
    }

    private void configureBindings() {
    // Define the RobotCentric request we will use
    SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(() -> {
            // 1. Get Joystick Inputs
            double vx = m_remote.getDriveX() * MaxSpeed;
            double vy = m_remote.getDriveY() * MaxSpeed;
            double omega = m_remote.getDriveRotate() * MaxAngularRate;

            // 2. Get Real-World Angle from Pigeon 1 (Negated because Pigeon 1 is CCW+)
            Rotation2d gyroAngle = Rotation2d.fromDegrees(pigeon2.getYaw()); 

            // 3. Perform Field-Centric Calculation manually
            // We rotate the desired field vector by the NEGATIVE gyro angle to get robot-relative vector
            double robotVx = vx * gyroAngle.getCos() + vy * gyroAngle.getSin();
            double robotVy = -vx * gyroAngle.getSin() + vy * gyroAngle.getCos();

            // 4. Send to Drivetrain as "Robot Centric" (since we already did the math)
            return robotCentric
                .withVelocityX(robotVx) 
                .withVelocityY(robotVy) 
                .withRotationalRate(omega);
        })
    );

        driverJoystick.leftBumper().whileTrue((drivetrain.applyRequest(() -> {
            if(!LimelightHelpers.getTV("limelight")){
                return drive.withVelocityX(0)
                            .withVelocityY(0)
                            .withRotationalRate(0);

            }

            double rotRateDIDDY = -(LimelightHelpers.getTX("limelight")) * 0.02;

            return drive.withVelocityX(0)
                        .withVelocityY(0)
                        .withRotationalRate(rotRateDIDDY);


            
        })));

        driverJoystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driverJoystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(m_remote.getDriveX(), m_remote.getDriveY()))
        ));

        driverJoystick.pov(0).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        driverJoystick.pov(180).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        //joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        //joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        //joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        //joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press


        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
