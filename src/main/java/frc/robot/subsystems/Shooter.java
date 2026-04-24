package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Remote.ShooterMode;

public class Shooter {
    VelocityVoltage mainVelocityReq = new VelocityVoltage(0);
    VelocityVoltage hopperVelocityReq = new VelocityVoltage(0);

    public final TalonFX hopperKraken = new TalonFX(34); 
    public final TalonFX shooter_Talon = new TalonFX(16);   // Falcon Leader
    public final TalonFX shooter_Talon_Slave = new TalonFX(999); // Falcon Follower

    private double custom_speed = 0.0;
    private double currentTargetSpeed = 0.0;

    public Shooter() {
        shooter_Talon_Slave.setControl(new Follower(16, MotorAlignmentValue.Opposed));
        
        TalonFXConfiguration leaderConfig = new TalonFXConfiguration();
        leaderConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        leaderConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        leaderConfig.Feedback.SensorToMechanismRatio = 0.3913043478; // 46:18 step-up
        
        leaderConfig.Slot0.kP = 0.38;
        leaderConfig.Slot0.kV = 0.14;
        leaderConfig.CurrentLimits.StatorCurrentLimit = 60;
        leaderConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        shooter_Talon.getConfigurator().apply(leaderConfig);

        TalonFXConfiguration hopperConfig = new TalonFXConfiguration();
        hopperConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
		hopperConfig.CurrentLimits.StatorCurrentLimit = 40;
        hopperConfig.CurrentLimits.StatorCurrentLimitEnable = true; 
        hopperKraken.getConfigurator().apply(hopperConfig);
    }

    public void setCustomSpeed(double speed) { this.custom_speed = speed; }

	public double getMotorOutputPercent() {
        // Returns the actual RPS velocity of the main shooter shaft
        return shooter_Talon.getVelocity().getValueAsDouble(); 
    }

    public boolean isAtSpeed() {
        double currentSpeed = shooter_Talon.getVelocity().getValueAsDouble();
        return Math.abs(currentSpeed - currentTargetSpeed) < 2.0 && currentTargetSpeed > 5.0; 
    }

    public void mainloop(ShooterMode shooterMode, double manualSpeed) {
        currentTargetSpeed = (this.custom_speed > 0) ? this.custom_speed : manualSpeed;

        switch(shooterMode) {
            case Rev:
                shooter_Talon.setControl(mainVelocityReq.withVelocity(currentTargetSpeed));
                hopperKraken.setControl(hopperVelocityReq.withVelocity(0)); // Hold ball
                break;
            case Shoot:
                shooter_Talon.setControl(mainVelocityReq.withVelocity(currentTargetSpeed));
                hopperKraken.setControl(hopperVelocityReq.withVelocity(40)); // Feed ball
                break;
            case Reverse:
                shooter_Talon.setControl(mainVelocityReq.withVelocity(-10)); 
                hopperKraken.setControl(hopperVelocityReq.withVelocity(-20)); // Unjam
                break;
            case Idle:
            default:
                shooter_Talon.setControl(mainVelocityReq.withVelocity(0));
                hopperKraken.setControl(hopperVelocityReq.withVelocity(0));
                this.custom_speed = 0.0;
                break;
        }
    }
}