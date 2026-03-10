package frc.robot.subsystems;

import static frc.robot.Constants.ControllerConstants.operatorJoystick;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.LEDReader;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.SubsystemConstants;
import frc.robot.subsystems.Remote.ShooterMode;

import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;


public class Shooter {
	double percent = 0.0;
	VelocityVoltage velocityRequest = new VelocityVoltage(0);

	double h_percent = 0.0;
	double custom_speed = 0.0;
	public double manualRevSpeed;

	public final TalonSRX hopperRedline = new TalonSRX(Constants.SubsystemConstants.TalonIDs.SRX.shooter_redline);
	public final TalonSRX leftMain = new TalonSRX(Constants.SubsystemConstants.TalonIDs.SRX.shooter_M_left);
	public final TalonSRX rightMain = new TalonSRX(Constants.SubsystemConstants.TalonIDs.SRX.shooter_M_right);
	//public final TalonSRX belt_CIM = new TalonSRX(Constants.SubsystemConstants.TalonIDs.SRX.shooter_belt);
	public final TalonFX shooter_Talon = new TalonFX(16);
 
	public Shooter() {
		
		hopperRedline.setInverted(true);
	
		TalonFXConfiguration leaderConfig = new TalonFXConfiguration();
        leaderConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
		leaderConfig.Slot0.kP= 0.38;
		leaderConfig.Slot0.kI = 0;
		leaderConfig.Slot0.kD = 0;
		leaderConfig.Slot0.kS = 0.14;
		leaderConfig.Slot0.kV = 0.14;
        leaderConfig.CurrentLimits.StatorCurrentLimit = 60;
        leaderConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        leaderConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        shooter_Talon.getConfigurator().apply(leaderConfig);


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
		
    
        if (operatorJoystick.y().getAsBoolean()) {
            manualRevSpeed = 24; // CLIMB ONU
        } else if (operatorJoystick.b().getAsBoolean()) {
            manualRevSpeed = 23;
        } else if (operatorJoystick.a().getAsBoolean()) {
            manualRevSpeed = 22;
        } else if (operatorJoystick.x().getAsBoolean()) {
            manualRevSpeed = 10;  // X Button = 40% Speed
        } else {
            // 2. Fall back to joystick axis if no preset is held
            manualRevSpeed = -35*operatorJoystick.getLeftY();
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
				rightMain.set(ControlMode.PercentOutput,0.0);
				
				break;

			case Shoot:
                // No custom_speed override here! It stays at joystick/limelight speed.
				percent = flywheelSpeed; 
				hopperRedline.set(ControlMode.PercentOutput, 0.35);
				rightMain.set(ControlMode.PercentOutput,0.5);
				
				break;

			case Reverse:
                // Uses the joystick variable to spin everything backwards
				percent = -manualRevSpeed; 
				hopperRedline.set(ControlMode.PercentOutput, -0.35);
				rightMain.set(ControlMode.PercentOutput,-0.5);
				break;

			case Idle:
			default:
				percent = 0.0;
				hopperRedline.set(ControlMode.PercentOutput, 0.0);
				rightMain.set(ControlMode.PercentOutput,0.0);
				this.custom_speed = 0.0;
				break;
		}
		
		SmartDashboard.putNumber("Shooter Target", percent);
	
		shooter_Talon.setControl(velocityRequest.withVelocity(percent));
		 //percent olaylarını deistirmek lazım.
	}
}