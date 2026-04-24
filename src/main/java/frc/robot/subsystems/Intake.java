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
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

public class Intake {
    // Standard duty cycle for the roller Falcon
    DutyCycleOut rollerRequest = new DutyCycleOut(0);
    // Motion Magic request for the deployer Kraken
    MotionMagicVoltage armRequest = new MotionMagicVoltage(0);
    
    private static final double DEPLOYER_SENSOR_TO_MECHANISM_RATIO = 9.14; 
    private static final double DEPLOYER_ROTOR_TO_SENSOR_RATIO = 1.0;

    // THESE MUST BE TUNED ON THE ROBOT
    private static final double STOW_POSITION = 0.0;
    private static final double DEPLOY_POSITION = 0.25; 

    // The variable that tracks where we currently want the arm to be
    private double targetPosition = STOW_POSITION;

    public final TalonFX intakeDeployer = new TalonFX(998); // Kraken X60
    public final TalonFX intakeMotor = new TalonFX(67);     // Falcon 500 Rollers

    public Intake() {
        TalonFXConfiguration deployerConfig = new TalonFXConfiguration();
        deployerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        // PID & Feedforward (Needs Tuning)
        deployerConfig.Slot0.kP = 0.5;
        deployerConfig.Slot0.kS = 0.1;
        deployerConfig.Slot0.kV = 0.1;
        // kG is required to hold the arm against gravity
        deployerConfig.Slot0.kG = 0.2; 
        deployerConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        
        deployerConfig.Feedback.SensorToMechanismRatio = DEPLOYER_SENSOR_TO_MECHANISM_RATIO;
        
        // Motion Magic (Speed and Smoothness)
        deployerConfig.MotionMagic.MotionMagicCruiseVelocity = 10; 
        deployerConfig.MotionMagic.MotionMagicAcceleration = 20; 

        // ==========================================
        // SOFTWARE LIMITS (Strict 0.0 to 5.0)
        // ==========================================
        deployerConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        deployerConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = DEPLOY_POSITION;
        deployerConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        deployerConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = STOW_POSITION;

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

    public void mainloop(IntakeMode intakeMode, IntakeDeployMode intakeDeployMode) {
    
        // 1. Determine where we want the arm to go
        if (intakeDeployMode == IntakeDeployMode.Deploy) {
            targetPosition = DEPLOY_POSITION; // NEED TESTING: Deployed rotations
        } else if (intakeDeployMode == IntakeDeployMode.Stow) {
            targetPosition = STOW_POSITION; // Stowed rotations
        } else if (intakeDeployMode == IntakeDeployMode.ManualUP) {
            targetPosition -= 0.005; // Jogs arm UP. Adjust 0.1 to change manual speed
        } else if (intakeDeployMode == IntakeDeployMode.ManualDOWN) {
            targetPosition += 0.005; // Jogs arm DOWN. Adjust 0.1 to change manual speed
        }
        // Notice: If Idle, targetPosition doesn't change! It just remembers where it was.

        // 2. Safely clamp the position so manual mode can NEVER break the robot limits
        targetPosition = MathUtil.clamp(targetPosition, STOW_POSITION, DEPLOY_POSITION);

        // 3. ALWAYS apply Motion Magic. This keeps the Gravity feedforward active!
        intakeDeployer.setControl(armRequest.withPosition(targetPosition));

        // 4. Run the rollers normally
        intakeMotor.setControl(rollerRequest.withOutput(modeToPercent(intakeMode)));
        
        // Optional: Put values on the dashboard so you can verify it's working
        SmartDashboard.putNumber("Intake/Arm Target Position", targetPosition);
        SmartDashboard.putNumber("Intake/Arm Actual Position", intakeDeployer.getPosition().getValueAsDouble());
    }
}