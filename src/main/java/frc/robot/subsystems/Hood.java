package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants;

public class Hood {

    private final TalonSRX hoodAdjuster =
        new TalonSRX(Constants.SubsystemConstants.TalonIDs.SRX.hoodMotor);

    private final Encoder hoodEncoder =
        new Encoder(
            0,1
        );


    private static final double MIN_TICKS = 0;
    private static final double MAX_TICKS = 25000; 


    private final PIDController hoodPID =
        new PIDController(0.0004, 0.0, 0.0);

    private double targetTicks = 0.0;

    public Hood() {

        hoodAdjuster.setNeutralMode(NeutralMode.Brake);
 

        hoodPID.setTolerance(200); 
    }


    public double getTicks() {
        return hoodEncoder.get();
    }
/* 
    public void adjustTicks(double deltaFactor) {
        targetTicks =+ MAX_TICKS*deltaFactor;

        targetTicks = Math.max(MIN_TICKS,
                      Math.min(MAX_TICKS, targetTicks));
    }
                      */


    public void setTargetTicks(double ticks) {
        targetTicks = Math.max(MIN_TICKS,
                      Math.min(MAX_TICKS, ticks));
    }

    public boolean atTarget() {
        return hoodPID.atSetpoint();
    }

    public void resetEncoder() {
        hoodEncoder.reset();
        targetTicks = 0;
    }

    public void mainloop() {

        double currentTicks = getTicks();

        double output =
            hoodPID.calculate(currentTicks, targetTicks);

        // Clamp motor power
        output = Math.max(-0.2, Math.min(0., output));

        hoodAdjuster.set(ControlMode.PercentOutput, output);

        SmartDashboard.putNumber("Hood Ticks", currentTicks);
        SmartDashboard.putNumber("Hood Target Ticks", targetTicks);
        SmartDashboard.putNumber("Hood Output", output);
    }
}