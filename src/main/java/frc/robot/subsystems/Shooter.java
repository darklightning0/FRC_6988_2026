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
    VelocityVoltage feedVelocityReq = new VelocityVoltage(0);

    // 4 Motor Setup 
    public final TalonFX shooterIntakeKraken = new TalonFX(34); // Shooter Intake
    public final TalonFX hopperBeltFalcon = new TalonFX(35);    // Hopper Belt 
    public final TalonFX shooter_Talon = new TalonFX(16);       // Falcon Leader
    public final TalonFX shooter_Talon_Slave = new TalonFX(999); // Falcon Follower

    private double custom_speed = 0.0;
    private double currentTargetSpeed = 0.0;

    public Shooter() {
        shooter_Talon_Slave.setControl(new Follower(16, MotorAlignmentValue.Opposed));
        
        TalonFXConfiguration leaderConfig = new TalonFXConfiguration();
        leaderConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        leaderConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        leaderConfig.Feedback.SensorToMechanismRatio = 0.3913043478; 
        
        leaderConfig.Slot0.kP = 0.38;
        leaderConfig.Slot0.kV = 0.14;
        leaderConfig.CurrentLimits.StatorCurrentLimit = 60;
        leaderConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        shooter_Talon.getConfigurator().apply(leaderConfig);

        TalonFXConfiguration feedConfig = new TalonFXConfiguration();
        feedConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
		feedConfig.CurrentLimits.StatorCurrentLimit = 40;
        feedConfig.CurrentLimits.StatorCurrentLimitEnable = true; 
        shooterIntakeKraken.getConfigurator().apply(feedConfig);
        hopperBeltFalcon.getConfigurator().apply(feedConfig); 
    }

    public void setCustomSpeed(double speed) { this.custom_speed = speed; }

	public double getMotorOutputPercent() {
        return shooter_Talon.getVelocity().getValueAsDouble(); 
    }
    //kullanilmiyo aq bu da kalsin yine de efe diddykulak
    public boolean isAtSpeed() {
        double currentSpeed = shooter_Talon.getVelocity().getValueAsDouble();
        return Math.abs(currentSpeed - currentTargetSpeed) < 2.0 && currentTargetSpeed > 5.0; 
    }

    public void mainloop(ShooterMode shooterMode, double manualSpeed) {
        // custom speed overridei var mi yok mu, manuel otoyu ayarliyo
        currentTargetSpeed = (this.custom_speed > 0) ? this.custom_speed : manualSpeed;

        switch(shooterMode) {
            case Rev:
                shooter_Talon.setControl(mainVelocityReq.withVelocity(currentTargetSpeed));
                shooterIntakeKraken.setControl(feedVelocityReq.withVelocity(0)); 
                hopperBeltFalcon.setControl(feedVelocityReq.withVelocity(0));
                break;
            case Shoot:
               
                shooter_Talon.setControl(mainVelocityReq.withVelocity(currentTargetSpeed));
                shooterIntakeKraken.setControl(feedVelocityReq.withVelocity(40)); 
                hopperBeltFalcon.setControl(feedVelocityReq.withVelocity(40));
                break;
            case Reverse:
            
                double reverseSpeed = -20;
                shooter_Talon.setControl(mainVelocityReq.withVelocity(reverseSpeed));
                shooterIntakeKraken.setControl(feedVelocityReq.withVelocity(-30));
                hopperBeltFalcon.setControl(feedVelocityReq.withVelocity(-30));
                break;
            case Idle:
            default:
                shooter_Talon.setControl(mainVelocityReq.withVelocity(0));
                shooterIntakeKraken.setControl(feedVelocityReq.withVelocity(0));
                hopperBeltFalcon.setControl(feedVelocityReq.withVelocity(0)); 
                this.custom_speed = 0.0;
                break;
        }
    }
}