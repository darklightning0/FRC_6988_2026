package frc.robot.subsystems;

import static frc.robot.Constants.ControllerConstants.driverJoystick;
import static frc.robot.Constants.ControllerConstants.driverJoystickDef;
import static frc.robot.Constants.ControllerConstants.operatorJoystick;
import static frc.robot.Constants.ControllerConstants.operatorJoystickDef;

import frc.robot.Constants;
import frc.robot.LimelightHelpers;
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
    Boolean deployToggle = false;
    ClimbMode climb_mode = ClimbMode.Idle;
    HoodMode hood_mode = HoodMode.Idle;

    boolean drive_slow = false;
    private double manual_shooter_speed = 0.0;


    // Constructor
    // Here we will be creating private constructor
    // restricted to this class itself
    public Remote() {

        tyToShooterSpeed.put(1.5, 30.0); // Subwoofer
        tyToShooterSpeed.put(3.0, 45.0); // Mid-range
        tyToShooterSpeed.put(5.0, 60.0); // Far shot


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

    public void mainloop() {
      
        drive_slow = driverJoystickDef.getRightBumperButton();

        inverseDriveDirection();

        // ==========================================
        // INTAKE ROLLERS (POV)
        // ==========================================
        if (operatorJoystickDef.isConnected()) {
            if (operatorJoystickDef.getPOV() == 0) { // UP on D-Pad
                intake_mode = IntakeMode.Intake;
            } else if (operatorJoystickDef.getPOV() == 180) { // DOWN on D-Pad
                intake_mode = IntakeMode.Reverse;
            } else {
                intake_mode = IntakeMode.Idle;
            }
        }

        // ==========================================
        // INTAKE ARM: AUTOMATIC DEPLOY/STOW (Right Stick Click)
        // ==========================================
        if(operatorJoystickDef.getRightStickButtonPressed() && deployToggle == false){
            intake_deploy_mode = IntakeDeployMode.Deploy;
            deployToggle = true; // Flips the toggle
        } else if(operatorJoystickDef.getRightStickButtonPressed() && deployToggle == true){
            intake_deploy_mode = IntakeDeployMode.Stow;
            deployToggle = false; // Flips the toggle
        }

        // ==========================================
        // INTAKE ARM: MANUAL CONTROL (Now on Right Joystick Y-Axis)
        // ==========================================
        double rightY = operatorJoystickDef.getRightY();
        if (rightY < -0.2) { // Pushing UP on right stick
            intake_deploy_mode = IntakeDeployMode.ManualUP;
        } else if (rightY > 0.2) { // Pulling DOWN on right stick
            intake_deploy_mode = IntakeDeployMode.ManualDOWN;
        } else if (intake_deploy_mode != IntakeDeployMode.Deploy && intake_deploy_mode != IntakeDeployMode.Stow) {
            intake_deploy_mode = IntakeDeployMode.Idle;
        }

        if(operatorJoystickDef.isConnected()){
            
            // 1. Determine Manual Speed
            if (operatorJoystickDef.getYButton()) { 
                manual_shooter_speed = 60; // 60 RPS
            } else if (operatorJoystickDef.getBButton()) { 
                manual_shooter_speed = 50; // 50 RPS
            } else if (operatorJoystickDef.getAButton()) { 
                manual_shooter_speed = 40; // 40 RPS
            } else if (operatorJoystickDef.getXButton()) { 
                manual_shooter_speed = 30; // 30 RPS
            } else if (Math.abs(operatorJoystickDef.getLeftY()) > 0.1) {
                // Manual stick revving (maps -1.0 to 1.0 -> 0 to 60 RPS)
                manual_shooter_speed = Math.abs(operatorJoystickDef.getLeftY()) * 60.0;
            }

            // 2. Determine State
            boolean isPresetPressed = operatorJoystickDef.getYButton() || operatorJoystickDef.getBButton() || 
                                      operatorJoystickDef.getAButton() || operatorJoystickDef.getXButton() ||
                                      Math.abs(operatorJoystickDef.getLeftY()) > 0.1;

            if (operatorJoystickDef.getLeftStickButton()) {
                shooter_mode = ShooterMode.Shoot; // Feed the ball
            } else if (isPresetPressed) {
                shooter_mode = ShooterMode.Rev;   // Just spin flywheels
            } else if (operatorJoystickDef.getRightY() > 0.2) {
                shooter_mode = ShooterMode.Reverse; // Un-jam
            } else {
                shooter_mode = ShooterMode.Idle;
            }
        }


  
    }

    
}
