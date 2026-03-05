package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants;
import frc.robot.Constants.SubsystemConstants;
import frc.robot.subsystems.Remote.ClimbMode;
import frc.robot.subsystems.Remote.HoodMode;

public class Hood {

    double percent = 0.0; 

    private final TalonSRX hoodPG = new TalonSRX(Constants.SubsystemConstants.TalonIDs.SRX.hoodMotor);

    double modeToPercent(HoodMode mode) {
		switch (mode) {
			case MoveUp:
				return SubsystemConstants.Output.hoodUp;
			case MoveDown:
				return SubsystemConstants.Output.hoodDown;
			case Idle:  
			default:
				return 0.0;
        }
    }

    public double getMotorOutputPercent() {
		return percent;
	}

	public void mainloop(HoodMode hoodMode) {

		percent = modeToPercent(hoodMode);
        hoodPG.set(ControlMode.PercentOutput, percent);		
	}
}