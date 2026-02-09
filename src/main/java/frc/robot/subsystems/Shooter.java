package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.Constants;
import frc.robot.Constants.SubsystemConstants;
import frc.robot.subsystems.Remote.ShooterMode;

public class Shooter {

	public final TalonSRX leftRedline = new TalonSRX(Constants.SubsystemConstants.TalonIDs.SRX.Shooter_Left);
	// public final TalonSRX rightRedline = new
	// TalonSRX(Constants.SubsystemConstants.TalonIDs.SRX.Shooter_Right);

	public Shooter() {
		leftRedline.setInverted(true);// left
		// rightRedline.setInverted(false);

		leftRedline.setNeutralMode(NeutralMode.Brake);
		// rightRedline.setNeutralMode(NeutralMode.Brake);
	}
	/*
	 * public void mainloop(boolean objectSeen) {
	 * if (objectSeen) {
	 * if (mode != ShooterMode.Shoot) {
	 * mode = ShooterMode.See;
	 * }
	 * } else {
	 * if (mode == ShooterMode.See) {
	 * mode = ShooterMode.Hold;
	 * }
	 * }
	 * 
	 * switch (mode) {
	 * case Idle:
	 * vel = 0;
	 * break;
	 * case See:
	 * vel = 1.5;
	 * case Hold:
	 * vel = -0.05;
	 * case Shoot:
	 * vel = 5;
	 * }
	 * 
	 * // Run motors to intake the game piece
	 * //leftRedline.set(ControlMode.PercentOutput, percent);
	 * //rightRedline.set(ControlMode.PercentOutput, percent);
	 * }
	 */

	double modeToPercent(ShooterMode mode) {
		switch (mode) {
			case Shoot:
				return SubsystemConstants.Output.shooterShoot;
			case Reverse:
				return SubsystemConstants.Output.shooterReverse;
			case Idle:
			default:
				return 0.;
		}
	}

	public void mainloop(ShooterMode shooterMode) {
		double percent = modeToPercent(shooterMode);
		// double percent = operatorJoystickDef.getRightTriggerAxis() * 0.2;
		leftRedline.set(ControlMode.PercentOutput, percent);
		// rightRedline.set(ControlMode.PercentOutput, percent);
	}
}