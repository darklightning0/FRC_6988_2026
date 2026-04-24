// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

	public static class ControllerConstants {
		public static final int kDriverControllerPort = 1;
		public static final int kOperatorControllerPort = 0;

		public static final int driverJoystickID = 1;
		public static final int operatorJoystickID = 0;

		public static final CommandXboxController driverJoystick = new CommandXboxController(driverJoystickID);
		public static final XboxController driverJoystickDef = new XboxController(driverJoystickID);
		public static final CommandXboxController operatorJoystick = new CommandXboxController(operatorJoystickID);
		public static final XboxController operatorJoystickDef = new XboxController(operatorJoystickID);

	}
	// ALGEA INTAKE

	// must be between 0 and 1
	public static class ReefLayers {
		public static final double L1 = 0.2513;
		public static final double L2 = 0.4426;
		public static final double L3 = 0.6612;
		public static final double L4 = 0.9500;
	}

	// SHOOTER

	public static class SubsystemConstants {
		public static class TalonIDs {
			public static class FX {
				public static final int Intake_Arm = 24; // Intake arm motor
				public static final int leader_climb = 23;
				public static final int follower_climb = 22;  
				public static final int shooter_belt = 16; // Shooter left


			}

			public static class SRX {
				public static final int Intake_Pulley = 14; //Intake Front Wheels
				public static final int shooter_M_left = 18; // Big elevator motor
				public static final int shooter_M_right = 17; // Small elevator motor
				public static final int shooter_belt = 16; // Shooter left
				public static final int shooter_redline = 15; // Shooter left
				public static final int hoodMotor = 20;
			}

			public static class Spark {
				public static final int Intake_Left = 23;
				public static final int Intake_Right = 22;
			}
		}

		public static class RemoteOperatorButtons {
			public static final int intakeUp = 1;
			public static final int intakeDown = 2;
			// public static final int intakeAlgeaIn = 7;
			// public static final int intakeAlgeaShoot = 8;
			public static final int home = 7;
		}

		public static class AMTEncoder {
			public static final int Elevator_Inner_A = 6;
			public static final int Elevator_Inner_B = 7;
			public static final int Elevator_Outer_A = 8;
			public static final int Elevator_Outer_B = 9;
			public static final int hoodEncoderId = 0;
			public static final int hoodEncoderId2 = 1;
		}
		
		public static class Switch {
			public static final int Elevator_Outer_Bottom = 5;
		}

		public static class Output {
			// Intake outputs
			public static final double intakeIntake = 0.4;
			public static final double intakeReverse = -0.4;
			public static final double intakeIdle = 0.0;

			// Outer output amplifier
			public static final double outerMinFactor = 1.0; // should stay 1
			public static final double outerMaxFactor = 1.8;
			
			// Outer output for down and up
			public static final double outerDown = -0.40;
			public static final double outerUp = 0.70;
			
			// Inner output for down (innerDownMax when at bottom, innerDownMin at top)
			public static final double innerDownMin = 0.025;
			public static final double innerDownMax = 0.035;
			// Inner output for hold
			public static final double innerKeep = 0.05;
			// Inner output for up (innerUpMax when at bottom, innerUpMin when at top)
			public static final double innerUpMin = 0.75;
			public static final double innerUpMax = 0.85;

			// Shooter output
			public static final double shooterShoot = 0.4;
			public static final double shooterReverse = -0.4;

			// Climb Output
			public static final double climbClimb = 1.0;
			public static final double climbReverse = -0.1;

			public static final double hoodUp = 0.15;
			public static final double hoodDown = -0.15;
		}

		public static class Voltage {
			// Intake arm voltage
			public static final double intakeArmUp = 3.0;
			public static final double intakeArmDown = -2.0;
			public static final double intakeArmIdle = 0.30;
		}

	}

}