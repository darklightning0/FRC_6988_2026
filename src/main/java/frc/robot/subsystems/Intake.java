package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.SubsystemConstants;
import frc.robot.subsystems.Remote.IntakeMode;
import frc.robot.subsystems.Remote.IntakeDeployMode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

public class Intake {
    // Standard duty cycle for the roller Falcon
    DutyCycleOut rollerRequest = new DutyCycleOut(0);
    // Motion Magic request for the deployer Kraken
    MotionMagicVoltage armRequest = new MotionMagicVoltage(0);
    
    private static final double DEPLOYER_SENSOR_TO_MECHANISM_RATIO = 9.14; 
    private static final double DEPLOYER_ROTOR_TO_SENSOR_RATIO = 1.0;


    private static final double deployPosition = 0.0;
    private static final double stowPosition = 0.25; 


    private double targetPosition = stowPosition;

    public final TalonFX intakeDeployer = new TalonFX(998); // Kraken X60
    public final TalonFX intakeMotor = new TalonFX(67);     // Falcon 500 Rollers

    public Intake() {
        
		TalonFXConfiguration deployerConfig = new TalonFXConfiguration();
		deployerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		deployerConfig.Slot0.kP = 0.5;
		deployerConfig.Slot0.kI = 0;
		deployerConfig.Slot0.kD = 0;
		deployerConfig.Slot0.kS = 0.1;
		deployerConfig.Slot0.kV = 0.1;
		deployerConfig.Slot0.kG = 0;
        
        deployerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; 
		deployerConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
		deployerConfig.Feedback.SensorToMechanismRatio = DEPLOYER_SENSOR_TO_MECHANISM_RATIO;
		deployerConfig.Feedback.RotorToSensorRatio = DEPLOYER_ROTOR_TO_SENSOR_RATIO;
		deployerConfig.MotionMagic.MotionMagicCruiseVelocity = 10; //need testing
		deployerConfig.MotionMagic.MotionMagicAcceleration = 20; //need testing

		deployerConfig.CurrentLimits.StatorCurrentLimit = 40;
        deployerConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        intakeDeployer.getConfigurator().apply(deployerConfig);
    }

    double modeToPercent(IntakeMode mode) {
        switch (mode) {
            case Intake: return SubsystemConstants.Output.intakeIntake;
            case Reverse: return SubsystemConstants.Output.intakeReverse;
            case Idle: default: return SubsystemConstants.Output.intakeIdle;
        }
    }

    public void resetDeployerEncoder() {
        intakeDeployer.setPosition(0);
        targetPosition = 0;
    }

        public void resetStartEncoder() {
        intakeDeployer.setPosition(stowPosition);
        targetPosition = stowPosition;
    }

    public double getStowPosition() {
        return stowPosition;
    }

    public void mainloop(IntakeMode intakeMode, IntakeDeployMode intakeDeployMode) {
    

        if (intakeDeployMode == IntakeDeployMode.Deploy) {
            targetPosition = deployPosition; // deployed
        } else if (intakeDeployMode == IntakeDeployMode.Stow) {
            targetPosition = stowPosition; // stowed
        } else if (intakeDeployMode == IntakeDeployMode.ManualUP) {
            targetPosition -= 0.005; // manual
        } else if (intakeDeployMode == IntakeDeployMode.ManualDOWN) {
            targetPosition += 0.005; // manual
        }
   

        targetPosition = MathUtil.clamp(targetPosition, deployPosition, stowPosition);

   
        intakeDeployer.setControl(armRequest.withPosition(targetPosition));


        intakeMotor.setControl(rollerRequest.withOutput(modeToPercent(intakeMode)));
        
    
        SmartDashboard.putNumber("Intake/Arm Target Position", targetPosition);

        SmartDashboard.putNumber("Intake/Arm Actual Position", intakeDeployer.getPosition().getValueAsDouble());
    }
}