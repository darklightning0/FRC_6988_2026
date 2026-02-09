package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.SubsystemConstants;
import frc.robot.util.Util;


public class OuterElevator {
    private final TalonSRX motor = new TalonSRX(Constants.SubsystemConstants.TalonIDs.SRX.Elevator_Outer);

    private final double maxPos = 0.80; // 0.80 meters

    // private final Encoder innerEncoder = new Encoder(null, null);
    private final Encoder encoder = new Encoder(SubsystemConstants.AMTEncoder.Elevator_Outer_A, SubsystemConstants.AMTEncoder.Elevator_Outer_B);

    private final DigitalInput bottomLimitSwitch = new DigitalInput(SubsystemConstants.Switch.Elevator_Outer_Bottom);

    // private static SimplePID innerPid = new SimplePID("elevatorInner", 12);
    // private static SimplePID outerPid = new SimplePID("elevatorOuter", 12);
    boolean bottomLimitSet = false;

    boolean motorEnabled = false;

    public OuterElevator() {}

    public void setEnabled(boolean value) {
        motorEnabled = value;
    }

    // progress: 0 ~ 1
    public double calculateFactor(double progress) {
        return Util.lerp(Util.clamp(progress, 0., 1.), Constants.SubsystemConstants.Output.outerMinFactor, Constants.SubsystemConstants.Output.outerMaxFactor);
    }

    public void config() {
        encoder.setDistancePerPulse(0.005 / 6.284 * 2. * Math.PI / 2048. / 0.550);
        encoder.reset();

        bottomLimitSet = false;

        motor.setNeutralMode(NeutralMode.Coast);
        motor.setInverted(true);
        // motor.config_kP(0, getPrefDouble("outerP", 0.01));
        // motor.config_kI(0, getPrefDouble("outerI", 0.00));
        // motor.config_kD(0, getPrefDouble("outerD", 0.00));
        // motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, Constants.SubsystemConstants.AMTEncoder.Elevator_Outer, 10);
        // motor.setSensorPhase(true);
    }

    double calculateOutput(double currentPos, double targetPos, boolean bottomSwitchReached) {
        if (motorEnabled) {
            double factor = calculateFactor(currentPos / maxPos);
            // outerMotor.set(TalonSRXControlMode.PercentOutput, output);

            if (Math.abs(targetPos - currentPos) > 0.025) {
                if (currentPos < targetPos) {
                    // go up
                    return SubsystemConstants.Output.outerUp * factor;
                } else {
                    // go down
                    if (bottomSwitchReached) return 0;
                    return SubsystemConstants.Output.outerDown * factor;
                }
            } else {
                if (targetPos < 0.01) {
                    // go a little down
                    if (bottomSwitchReached) return 0;
                    return SubsystemConstants.Output.outerDown * factor;
                }
                return 0;
            }
        } else {
            return 0;
        }
    }
    
    double progressToPos(double progress) {
        if (progress < 0.4) {
            return 0;
        } else {
            return Util.map(0.4, 1.0, progress, 0.0, 1.0);
        }
    }

    // return true if outer is using current
    public boolean mainloop(double targetPos, boolean shouldHome) {
        boolean bottomSwitchReached = !bottomLimitSwitch.get();

        if (shouldHome) bottomLimitSet = false;

        if (!bottomLimitSet) {
            // homing
            if (bottomSwitchReached) {
                encoder.reset();
                bottomLimitSet = true;
                motor.set(TalonSRXControlMode.PercentOutput, 0.);
                return true;
            }
            motor.set(TalonSRXControlMode.PercentOutput, -0.30);
            return true;
        }

        double currentPos = encoder.getDistance();

        double output = calculateOutput(currentPos, targetPos, bottomSwitchReached);
        motor.set(TalonSRXControlMode.PercentOutput, output);

        SmartDashboard.putNumber("elevatorOuterOutput", output);
        SmartDashboard.putBoolean("elevatorBottomSwitch", bottomSwitchReached);
        
        // double innerVoltage = innerPid.getSignal(innerCurrentPos);
        // innerMotor.set(ControlMode.PercentOutput, 0, DemandType.Neutral, innerVoltage);
        
        // outerMotor.set(ControlMode.Position, outerTargetPos);
        SmartDashboard.putNumber("elevatorOuterTarget", targetPos);
        SmartDashboard.putNumber("elevatorOuterCurrent", currentPos);

        return (Math.abs(output) > 0.01);
    }
}
