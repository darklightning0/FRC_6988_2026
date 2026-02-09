package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.SubsystemConstants;
import frc.robot.util.Util;

public class InnerElevator {
    private final TalonSRX motor = new TalonSRX(SubsystemConstants.TalonIDs.SRX.Elevator_Inner);

    private final PIDController pid = new PIDController(0, 0, 0);

    // private final double maxPos = 0.80; // 0.80 meters

    private final Encoder encoder = new Encoder(SubsystemConstants.AMTEncoder.Elevator_Inner_A, SubsystemConstants.AMTEncoder.Elevator_Inner_B);

    // private static SimplePID innerPid = new SimplePID("elevatorInner", 12);
    // private static SimplePID outerPid = new SimplePID("elevatorOuter", 12);

    boolean motorEnabled = false;

    double getPrefDouble(String key, double backup) {
        return Preferences.getDouble("elevPid_" + key, backup);
    }

    void setPrefDouble(String key, double value) {
        Preferences.setDouble("elevPid_" + key, value);
    }

    public InnerElevator() {
        double kP = getPrefDouble("outerP", 0.01);
        double kI = getPrefDouble("outerI", 0.00);
        double kD = getPrefDouble("outerD", 0.00);
        setPrefDouble("outerP", kP);
        setPrefDouble("outerI", kI);
        setPrefDouble("outerD", kD);
        config();
    }

    public void setEnabled(boolean value) {
        motorEnabled = value;
    }

    // progress: 0 ~ 1
    public double calculateFactor(double progress) {
        return Util.lerp(Util.clamp(progress, 0., 1.), 1., 1.8);
    }

    public void config() {
        motor.setNeutralMode(NeutralMode.Coast);

        double dpr = 23 * 1.5 / 50; // distance per revolution
        double ppr = 1024.; // pulses per revolution
        encoder.setDistancePerPulse(dpr / ppr);
        encoder.reset();
        encoder.setReverseDirection(true);

        double kP = getPrefDouble("outerP", 0.01);
        double kI = getPrefDouble("outerI", 0.00);
        double kD = getPrefDouble("outerD", 0.00);

        pid.setP(kP);
        pid.setI(kI);
        pid.setD(kD);
    }

    double calculateOutput(double currentPos, double targetPos) {
        if (motorEnabled) {
            return pid.calculate(currentPos, targetPos);

            /*
             * double factor = calculateFactor(currentPos / maxPos);
             * // outerMotor.set(TalonSRXControlMode.PercentOutput, output);
             * 
             * if (Math.abs(targetPos - currentPos) > 0.01) {
             * if (currentPos < targetPos) {
             * // go up
             * return 0.60 * factor;
             * } else {
             * // go down
             * return 0.35 * factor;
             * }
             * } else {
             * return 0;
             * }
             */
        } else {
            return 0;
        }
    }

    double progressToPos(double progress) {
        if (progress > 0.4) {
            return 1.0;
        } else {
            return Util.map(0.0, 0.4, progress, 0.0, 1.0);
        }
    }

    public void mainloop(double targetPos, boolean shouldHome) {
        if (shouldHome) {
            motor.set(ControlMode.PercentOutput, 0);
            encoder.reset();
            return;
        }

        double currentPos = encoder.getDistance();

        // double output = calculateOutput(currentPos, targetPos);
        // output = Util.clamp(output, 0.05, 0.85);

        double output = 0;
        // boolean atBottom = currentPos < 0.05;
        // boolean atTop = currentPos > 0.95;

        // asagi gitmek icin gereken output
        double downOutput = Util.lerp(currentPos, SubsystemConstants.Output.innerDownMax, SubsystemConstants.Output.innerDownMin);
        // yerinde durmak icin gereken output
        double keepOutput = 0.35;
        // yukari gitmek icin gereken output
        double upOutput = Util.lerp(currentPos, SubsystemConstants.Output.innerUpMax, SubsystemConstants.Output.innerUpMin);

        if (Math.abs(currentPos - targetPos) > 0.08) {
            boolean mustGoUp = currentPos < targetPos;
            output = mustGoUp ? upOutput : downOutput;
        } else {
            output = keepOutput;
        }

        // hard limit output
        output = Util.clamp(output, SubsystemConstants.Output.innerDownMin, SubsystemConstants.Output.innerUpMax);
        motor.set(ControlMode.PercentOutput, output);

        SmartDashboard.putNumber("elevatorInnerCurrent", currentPos);
        SmartDashboard.putNumber("elevatorInnerTarget", targetPos);
        SmartDashboard.putNumber("elevatorInnerOutput", output);
    }
}
