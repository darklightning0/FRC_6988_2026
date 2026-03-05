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
	double percent = 0.0;
	double h_percent = 0.0;
	double custom_speed = 0.0;

	public final TalonSRX hopperRedline = new TalonSRX(Constants.SubsystemConstants.TalonIDs.SRX.shooter_redline);
	public final TalonSRX leftMain = new TalonSRX(Constants.SubsystemConstants.TalonIDs.SRX.shooter_M_left);
	public final TalonSRX rightMain = new TalonSRX(Constants.SubsystemConstants.TalonIDs.SRX.shooter_M_right);
	public final TalonSRX belt_CIM = new TalonSRX(Constants.SubsystemConstants.TalonIDs.SRX.shooter_belt);

	public Shooter() {
		
		belt_CIM.setInverted(false);
		hopperRedline.setInverted(true);
	}
/* 
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
*/
	public void setCustomSpeed(double speed){
		this.custom_speed = speed;
	}

	//Get the current motor output percent for telemetry
	public double getMotorOutputPercent() {
		return percent;
	}

	public void mainloop(ShooterMode shooterMode) {

		double flywheelSpeed = 0.0;
		
		if(this.custom_speed > 0){
			flywheelSpeed = this.custom_speed;
		} else {
			flywheelSpeed = Constants.SubsystemConstants.Output.shooterShoot * Math.abs(operatorJoystick.getLeftY());
		}

		switch(shooterMode){
			case Rev:
				percent = flywheelSpeed;
				hopperRedline.set(ControlMode.PercentOutput, 0.0);
				belt_CIM.set(ControlMode.PercentOutput, 0.0);
				break;

			case Shoot:
				this.custom_speed = 1;
				percent = flywheelSpeed;
				hopperRedline.set(ControlMode.PercentOutput, 0.35);
				belt_CIM.set(ControlMode.PercentOutput, 0.8);
				break;

			case Reverse:
				this.custom_speed = 0;
				percent = Constants.SubsystemConstants.Output.shooterReverse;
				hopperRedline.set(ControlMode.PercentOutput, -0.35);
				belt_CIM.set(ControlMode.PercentOutput, -0.8);
				break;

			case Idle:
			default:
				percent = 0.0;
				hopperRedline.set(ControlMode.PercentOutput, 0.0);
				belt_CIM.set(ControlMode.PercentOutput, 0.0);
				this.custom_speed = 0.0;
				break;
		}
/* 
		if(shooterMode == ShooterMode.Shoot){
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
	*/	
		SmartDashboard.putNumber("Hopper Output",belt_CIM.getMotorOutputPercent());
		SmartDashboard.putNumber("Shooter Target", percent);
		leftMain.set(ControlMode.PercentOutput, percent);
		rightMain.set(ControlMode.PercentOutput, percent);
		
	}

}