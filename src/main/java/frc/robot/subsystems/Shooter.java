package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.Constants;
import frc.robot.Constants.SubsystemConstants;
import frc.robot.subsystems.Remote.ShooterMode;


public class Shooter {
	double percent =0;

	public final TalonSRX hopperRedline = new TalonSRX(Constants.SubsystemConstants.TalonIDs.SRX.shooter_redline);
	public final TalonSRX leftMain = new TalonSRX(Constants.SubsystemConstants.TalonIDs.SRX.shooter_M_left);
	public final TalonSRX rightMain = new TalonSRX(Constants.SubsystemConstants.TalonIDs.SRX.shooter_M_right);
	public final TalonSRX belt_CIM = new TalonSRX(Constants.SubsystemConstants.TalonIDs.SRX.shooter_belt);

	public Shooter() {
		leftMain.setInverted(true);// left
	}
	
	double modeToPercent(ShooterMode mode) {
		switch (mode) {
			case Shoot:
				return SubsystemConstants.Output.shooterShoot;
			case Reverse:
				return SubsystemConstants.Output.shooterReverse;
			case Idle:
			default:
				return 0.0;
		}
	}

	//Get the current motor output percent for telemetry
	public double getMotorOutputPercent() {
		return percent;
	}

	public void mainloop(ShooterMode shooterMode) {
		if(shooterMode == ShooterMode.Shoot){
			percent = modeToPercent(shooterMode) * Remote.getRightTriggerAxis();
		} else if(shooterMode == ShooterMode.Reverse){
			percent = modeToPercent(shooterMode) * Remote.getLeftTriggerAxis();
		} else {
			percent = modeToPercent(shooterMode);
		}
		hopperRedline.set(ControlMode.PercentOutput, percent);
		leftMain.set(ControlMode.PercentOutput, percent);
		rightMain.set(ControlMode.PercentOutput, percent);
		belt_CIM.set(ControlMode.PercentOutput, percent);
	}

}