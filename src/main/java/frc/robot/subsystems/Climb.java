package frc.robot.subsystems;

import static frc.robot.Constants.ControllerConstants.operatorJoystick;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.SubsystemConstants;
import frc.robot.subsystems.Remote.ShooterMode;


public class Climb {
	double percent;

	public final TalonFX leader_climb = new TalonFX(Constants.SubsystemConstants.TalonIDs.FX.leader_climb);
	public final TalonFX follower_climb = new TalonFX(Constants.SubsystemConstants.TalonIDs.FX.follower_climb);

    private final DutyCycleOut m_powerRequest = new DutyCycleOut(0.0);

    public Climb(){

        TalonFXConfiguration leaderConfig = new TalonFXConfiguration();
        leaderConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        leaderConfig.CurrentLimits.StatorCurrentLimit = 60;
        leaderConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        leaderConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        leader_climb.getConfigurator().apply(leaderConfig);

        TalonFXConfiguration followerConfig = new TalonFXConfiguration();
        followerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        follower_climb.getConfigurator().apply(followerConfig);

        follower_climb .setControl(new Follower(Constants.SubsystemConstants.TalonIDs.FX.leader_climb, MotorAlignmentValue.Opposed));

    }


	

	//Get the current motor output percent for telemetry
	public double getMotorOutputPercent() {
		return percent;
	}

	public void mainloop(ShooterMode shooterMode) {
		
		
	}

}