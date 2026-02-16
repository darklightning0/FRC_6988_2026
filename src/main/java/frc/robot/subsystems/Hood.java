package frc.robot.subsystems;

import java.util.function.DoubleUnaryOperator;

import com.ctre.phoenix.Util;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.SubsystemConstants;
import frc.robot.Constants.SubsystemConstants.AMTEncoder;
import frc.robot.subsystems.Remote.IntakeMode;

public class Hood {
    double percent;
    
    public final TalonSRX hoodAdjuster = new TalonSRX(Constants.SubsystemConstants.TalonIDs.SRX.hoodMotor);
    public final Encoder hoodEncoder = new Encoder(Constants.SubsystemConstants.AMTEncoder.hoodEncoderId,Constants.SubsystemConstants.AMTEncoder.hoodEncoderId2);
    
    double currentHoodAngle;
    double targetHoodAngle;
    public double getAngleHood(){
        

        currentHoodAngle = hoodEncoder.getDistance();
        return currentHoodAngle;

    }

    public void setTargetAngle(double angle) {
    targetHoodAngle = angle;
    }



    public Hood() {
        hoodAdjuster.setNeutralMode(NeutralMode.Brake);
    
    }

   
    

    
    public void mainloop() {
        SmartDashboard.putNumber("Hood Encoder Ticks",hoodEncoder.get());
    }

}

  