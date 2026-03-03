package frc.robot.subsystems;

import static frc.robot.Constants.ControllerConstants.operatorJoystick;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.SubsystemConstants;
import frc.robot.subsystems.Remote.ShooterMode;


public class Shooter {
	double percent;
	double h_percent;

	public final TalonSRX hopperRedline = new TalonSRX(Constants.SubsystemConstants.TalonIDs.SRX.shooter_redline);
	public final TalonSRX leftMain = new TalonSRX(Constants.SubsystemConstants.TalonIDs.SRX.shooter_M_left);
	public final TalonSRX rightMain = new TalonSRX(Constants.SubsystemConstants.TalonIDs.SRX.shooter_M_right);
	public final TalonSRX belt_CIM = new TalonSRX(Constants.SubsystemConstants.TalonIDs.SRX.shooter_belt);

	public Shooter() {
		
		belt_CIM.setInverted(true);
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
		if(shooterMode == ShooterMode.Shoot){/////
			percent = modeToPercent(shooterMode)*operatorJoystick.getLeftY();
			hopperRedline.set(ControlMode.PercentOutput, 0.35);
			belt_CIM.set(ControlMode.PercentOutput, 0.8);
		} else if(shooterMode == ShooterMode.Reverse){
			percent = modeToPercent(shooterMode)* -operatorJoystick.getLeftY();
			hopperRedline.set(ControlMode.PercentOutput, -0.35);
			belt_CIM.set(ControlMode.PercentOutput, -0.8);
		} else {
			percent = modeToPercent(shooterMode);
			hopperRedline.set(ControlMode.PercentOutput, 0);
			belt_CIM.set(ControlMode.PercentOutput, 0);
		}
		
		SmartDashboard.putNumber("hopper output",belt_CIM.getMotorOutputPercent());
		SmartDashboard.putNumber("AA", 555555);
		leftMain.set(ControlMode.PercentOutput, percent);
		rightMain.set(ControlMode.PercentOutput, percent);
		
	}

}