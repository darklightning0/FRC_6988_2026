// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.AlgeaIntake;

import frc.robot.Constants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Remote.IntakeWheelMode;

public class IntakeWheels {
  public final SparkMax leftNeo = new SparkMax(Constants.SubsystemConstants.TalonIDs.Spark.Intake_Left, MotorType.kBrushless);
  public final SparkMax rightNeo = new SparkMax(Constants.SubsystemConstants.TalonIDs.Spark.Intake_Right, MotorType.kBrushless);
  
  
  private final double neoSpeed = 0.35;


  public IntakeWheels() {
    leftNeo.configure(new SparkMaxConfig().inverted(true), SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
    rightNeo.configure(new SparkMaxConfig().inverted(false), SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);

    // leftNeo.setInverted(true);
    // rightNeo.setInverted(true);
  }

   double modeToPercent(IntakeWheelMode mode){
    switch (mode) {
      case In:
        return 1.;
      case Shoot:
        return -1.;
      case Hold:
        return 0.25;
      case Idle:
      default:
       return 0.;
    }
  }

  public void mainloop(IntakeWheelMode intakeWheelMode){
    double percent = modeToPercent(intakeWheelMode) * neoSpeed;
    leftNeo.set(percent);
    rightNeo.set(percent);
    SmartDashboard.putNumber("intakeWheelOutput", percent);
  }
}
  