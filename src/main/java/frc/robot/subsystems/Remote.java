package frc.robot.subsystems;

import static frc.robot.Constants.ControllerConstants.driverJoystick;
import static frc.robot.Constants.ControllerConstants.driverJoystickDef;
import static frc.robot.Constants.ControllerConstants.operatorJoystick;
import static frc.robot.Constants.ControllerConstants.operatorJoystickDef;

import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.Constants.SubsystemConstants.RemoteOperatorButtons;
import frc.robot.LimelightHelpers.RawFiducial;

// import frc.robot.Constants.SubsystemConstants.RemoteButtons;
// import frc.robot.Constants.SubsystemConstants.RemoteOperatorButtons;
// import frc.robot.util.UltrasonicSensor;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

enum DriveMode {

}

public class Remote {

    public static enum IntakeMode {
        Intake,
        Reverse,
        Idle,
    }

    public static enum ClimbMode {
        Idle,
        Climb,
        Reverse,
    }

    public static enum ShooterMode {

        Idle,
        Rev,
        Shoot,
        Reverse,

    }
      public static enum IntakeDeployMode {
        Deploy,
        Stow,
        ManualUP,
            ManualDOWN,
            Idle
    }
    public static enum HoodMode {

        MoveUp,
        MoveDown,
        Idle

    }
    public InterpolatingDoubleTreeMap distanceToPercent = new InterpolatingDoubleTreeMap();
    private double limelightDist;
    private RawFiducial[] limelightData;

    ShooterMode shooter_mode = ShooterMode.Idle;
    IntakeMode intake_mode = IntakeMode.Idle;
    IntakeDeployMode intake_deploy_mode = IntakeDeployMode.Stow;
    ClimbMode climb_mode = ClimbMode.Idle;
    HoodMode hood_mode = HoodMode.Idle;

    boolean drive_slow = false;
    private double manual_shooter_speed = 0.0;


    // Constructor
    // Here we will be creating private constructor
    // restricted to this class itself
    public Remote() {

        tyToShooterSpeed.put(1.5, 30.0); 
        tyToShooterSpeed.put(3.0, 45.0); 
        tyToShooterSpeed.put(5.0, 60.0); 


    }

    public InterpolatingDoubleTreeMap tyToShooterSpeed = new InterpolatingDoubleTreeMap();


    public ShooterMode getShooterMode() {
        return shooter_mode;
    }

    public IntakeMode getIntakeMode(){
        return intake_mode;
    }
      public IntakeDeployMode getIntakeDeployMode(){
        return intake_deploy_mode;
    }
    public ClimbMode getClimbMode(){
        return climb_mode;
    }

    public HoodMode getHoodMode(){
        return hood_mode;
    }

    public double getManualShooterSpeed() {
        return manual_shooter_speed;
    }

    public static double getLeftY(){
        return operatorJoystickDef.getLeftY();
    }

    public static double getRightTriggerAxis(){
        return operatorJoystickDef.getRightTriggerAxis();
    }

    public static double getLeftTriggerAxis(){
        return operatorJoystickDef.getLeftTriggerAxis();
    }


    double getDriveFactor() {
        if (drive_slow) return 0.2;
        return 0.4;
    }
    double getDriveRotateFactor() {
        if (drive_slow) return 0.4;
        return 0.6;
    }

    private int driveDirection=1;
    public void inverseDriveDirection(){
        if(driverJoystickDef.isConnected()){
            
            if(driverJoystickDef.getYButton()){
                driveDirection*=-1;
            }
        }
    }

    public boolean getHomeButton() {
        if(operatorJoystickDef.isConnected()) {
            return operatorJoystickDef.getStartButton(); // Uses the Start button
        }
        return false;
    }

    public double getDriveX() {
        if (driverJoystick.isConnected()) {
            return -driverJoystick.getLeftY() * getDriveFactor()*driveDirection;
        }
        return 0;
    }
    public double getDriveY() {
        if (driverJoystick.isConnected()) {
            return -driverJoystick.getLeftX() * getDriveFactor()*driveDirection;
        }
        return 0;
        
    }
    public double getDriveRotate() {
        if (driverJoystick.isConnected()) {
            return -driverJoystick.getRightX() * getDriveRotateFactor();
        }
        return 0;
    }

    public void mainloop(RobotContainer robotContainer) {
      
        drive_slow = driverJoystickDef.getRightBumperButton();

        inverseDriveDirection();

     
        if (operatorJoystickDef.isConnected()) {
            if (operatorJoystickDef.getPOV() == 0) { // up = intake roll in
                intake_mode = IntakeMode.Intake;
            } else if (operatorJoystickDef.getPOV() == 180) { // down = intake roll out
                intake_mode = IntakeMode.Reverse;
            } else {
                intake_mode = IntakeMode.Idle;
            }
        }

        if(operatorJoystickDef.getLeftStickButtonPressed()){

            double deployThreshold = robotContainer.m_intake.getStowPosition() * 0.5;
            if(robotContainer.m_intake.intakeDeployer.getPosition().getValueAsDouble() < deployThreshold) {
                intake_deploy_mode = IntakeDeployMode.Deploy; 
            } else {
                intake_deploy_mode = IntakeDeployMode.Stow;  
            }
        }

     
        double rightY = operatorJoystickDef.getLeftY();
        if (rightY < -0.2) { // arm pushing up
            intake_deploy_mode = IntakeDeployMode.ManualUP;
        } else if (rightY > 0.2) { // arm going down
            intake_deploy_mode = IntakeDeployMode.ManualDOWN;
        } else if (intake_deploy_mode != IntakeDeployMode.Deploy && intake_deploy_mode != IntakeDeployMode.Stow) {
            intake_deploy_mode = IntakeDeployMode.Idle;
        }

        if(operatorJoystickDef.isConnected()){
            
   
            if (operatorJoystickDef.getYButton()) { 
                manual_shooter_speed = 50; 
            } else if (operatorJoystickDef.getBButton()) { 
                manual_shooter_speed = 40; 
            } else if (operatorJoystickDef.getAButton()) { 
                manual_shooter_speed = 30; 
            } else if (operatorJoystickDef.getXButton()) { 
                manual_shooter_speed = 20; 
            } else if (Math.abs(operatorJoystickDef.getRightY()) > 0.1) {
        
                manual_shooter_speed = -operatorJoystickDef.getRightY() * 50.0;
            } else {
                manual_shooter_speed = 0.0;
            }

            //shoter state determine
            boolean isPresetPressed = operatorJoystickDef.getYButton() || operatorJoystickDef.getBButton() || 
                                      operatorJoystickDef.getAButton() || operatorJoystickDef.getXButton() ||
                                      Math.abs(operatorJoystickDef.getRightY()) > 0.1;

            if (operatorJoystickDef.getRightStickButton()) {
         
                if (manual_shooter_speed < 0) {
                    shooter_mode = ShooterMode.Reverse;
                } else {
                    shooter_mode = ShooterMode.Shoot; // Normal forward fire
                }
            } 
            else if (isPresetPressed) {
                shooter_mode = ShooterMode.Rev; // spin flywheels at the target speed
            } else {
                shooter_mode = ShooterMode.Idle;
            }
        }


  
    }

    
}
