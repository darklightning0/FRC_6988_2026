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
	public double manualRevSpeed;

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
		
        // 1. Check preset buttons FIRST (highest priority for manual control)
        double manualRevSpeed;
        if (operatorJoystick.y().getAsBoolean()) {
            manualRevSpeed = 0.8;  // Y Button = 100% Speed
        } else if (operatorJoystick.b().getAsBoolean()) {
            manualRevSpeed = 0.7; // B Button = 85% Speed
        } else if (operatorJoystick.a().getAsBoolean()) {
            manualRevSpeed = 0.65;  // A Button = 60% Speed
        } else if (operatorJoystick.x().getAsBoolean()) {
            manualRevSpeed = 0.6;  // X Button = 40% Speed
        } else {
            // 2. Fall back to joystick axis if no preset is held
            manualRevSpeed = Math.abs(operatorJoystick.getLeftY());
        }

		// 3. Limelight auto-aim speed overrides manual only when active
		if(this.custom_speed > 0){
			flywheelSpeed = this.custom_speed; // Use Auto-Aim Limelight speed
		} else {
			flywheelSpeed = manualRevSpeed;    // Use Manual Joystick/Preset speed
		}

		switch(shooterMode){
			case Rev:
				percent = flywheelSpeed;
				hopperRedline.set(ControlMode.PercentOutput, 0.0);
				belt_CIM.set(ControlMode.PercentOutput, 0.0);
				break;

			case Shoot:
                // No custom_speed override here! It stays at joystick/limelight speed.
				percent = flywheelSpeed; 
				hopperRedline.set(ControlMode.PercentOutput, 0.35);
				belt_CIM.set(ControlMode.PercentOutput, 0.8);
				break;

			case Reverse:
                // Uses the joystick variable to spin everything backwards
				percent = -manualRevSpeed; 
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
		
		SmartDashboard.putNumber("Hopper Output",belt_CIM.getMotorOutputPercent());
		SmartDashboard.putNumber("Shooter Target", percent);
		leftMain.set(ControlMode.PercentOutput, percent);
		rightMain.set(ControlMode.PercentOutput, percent);
	}

}