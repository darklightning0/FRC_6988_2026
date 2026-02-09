// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.AlgeaIntake;

import frc.robot.Constants;
import frc.robot.Constants.SubsystemConstants;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.subsystems.Remote.IntakeArmMode;

public class IntakeArm {
	public TalonFX armMotor = new TalonFX(Constants.SubsystemConstants.TalonIDs.FX.Intake_Arm);

	public IntakeArm() {
		// armMotor.setInverted(false);
	}
	
	void config() {
		TalonFXConfiguration config = new TalonFXConfiguration().withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));
		armMotor.getConfigurator().apply(config);
		armMotor.setNeutralMode(NeutralModeValue.Brake);
	}

	double modeToVoltage(IntakeArmMode mode) {
		switch (mode) {
			case Up:
				return SubsystemConstants.Voltage.intakeArmUp;
			case Down:
				return SubsystemConstants.Voltage.intakeArmDown;
			case Idle:
			default:
				return SubsystemConstants.Voltage.intakeArmIdle;
				
		}
	}

	public void mainloop(IntakeArmMode intakeArmMode) {
		double voltage = modeToVoltage(intakeArmMode);
		armMotor.setVoltage(voltage);
		// armMotor.set(TalonSRXControlMode.PercentOutput, percent);
	}
}