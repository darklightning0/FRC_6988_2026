package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.Constants;
import frc.robot.Constants.SubsystemConstants;
import frc.robot.subsystems.Remote.IntakeMode;

public class Intake {

    public final TalonSRX pulleyCIM = new TalonSRX(Constants.SubsystemConstants.TalonIDs.SRX.Intake_Pulley);
    public final TalonSRX gearCIM = new TalonSRX(Constants.SubsystemConstants.TalonIDs.SRX.Intake_Gear);

    public Intake() {
		pulleyCIM.setInverted(true);
	}

    double modeToPercent(IntakeMode mode) {
		switch (mode) {
			case Intake:
				return SubsystemConstants.Output.intakeIntake;
			case Reverse:
				return SubsystemConstants.Output.intakeReverse;
			case Idle:
			default:
				return SubsystemConstants.Output.intakeIdle;
        }
    }
	
/* 
	//Get the current motor output percent for telemetry
	public double getMotorOutputPercent() {
		return percent;
	}
    */
	public void mainloop(IntakeMode intakeMode) {
		double percent = modeToPercent(intakeMode);
		// double percent = operatorJoystickDef.getRightTriggerAxis() * 0.2;
		pulleyCIM.set(ControlMode.PercentOutput, percent);
		gearCIM.set(ControlMode.PercentOutput, percent);
	}

}

