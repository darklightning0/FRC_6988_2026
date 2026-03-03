package frc.robot.subsystems;

import static frc.robot.Constants.ControllerConstants.driverJoystick;
import static frc.robot.Constants.ControllerConstants.driverJoystickDef;
import static frc.robot.Constants.ControllerConstants.operatorJoystick;
import static frc.robot.Constants.ControllerConstants.operatorJoystickDef;

import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.SubsystemConstants.RemoteOperatorButtons;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.util.DistanceControl;
import frc.robot.util.Util;

// import frc.robot.Constants.SubsystemConstants.RemoteButtons;
// import frc.robot.Constants.SubsystemConstants.RemoteOperatorButtons;
// import frc.robot.util.UltrasonicSensor;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

enum DriveMode {

}

public class Remote {

    public static enum IntakeMode {
        Intake,
        Reverse,
        Idle,
    }
    // Intake kol hareketleri
    public static enum IntakeArmMode {
        Idle,
        Up,
        Down,
    }

    // Intake tekerlek hareket
    public static enum IntakeWheelMode {
        Idle,
        In,
        Shoot,
        Hold,
    }

    public static enum ClimbMode {
        Idle,
        Climb,
        Reverse,
    }

    public static enum ShooterMode {

        Idle,
        Shoot,
        Reverse,

    }
    public InterpolatingDoubleTreeMap distanceToPercent = new InterpolatingDoubleTreeMap();
    private double limelightDist;
    private RawFiducial[] limelightData;

    IntakeArmMode input_armIntake = IntakeArmMode.Idle;
    IntakeWheelMode input_wheelIntake = IntakeWheelMode.Idle;
    ShooterMode shooter_mode = ShooterMode.Idle;
    IntakeMode intake_mode = IntakeMode.Idle;
    ClimbMode climb_mode = ClimbMode.Idle;
    // static UltrasonicSensor ultrasonicSensor = new UltrasonicSensor(0, 0);
    // ElevatorMode input_elevatorMode = ElevatorMode.Idle;
    public double input_innerElevatorTarget = 0;
    public double input_outerElevatorTarget = 0;
    boolean elevator_manual = false;

    boolean elevator_manualHome = false;
    boolean elevator_manualHomePressed = false;

    boolean drive_slow = false;
    private Hood hood;

    // Constructor
    // Here we will be creating private constructor
    // restricted to this class itself
    public Remote() {
    distanceToPercent.put(1.0, 0.35); // distance and hood angle ticks
    distanceToPercent.put(2.0, 0.50);
    distanceToPercent.put(3.0, 0.65);

        tyToHoodTicks.put(-15.0, 18000.0);
        tyToHoodTicks.put(0.0, 10000.0);
        tyToHoodTicks.put(15.0, 2000.0);
    }

    public enum HoodMode {ManualUp, ManualDown, AutoAim, ReturnToZero}
    public HoodMode hood_mode = HoodMode.ReturnToZero;

    public InterpolatingDoubleTreeMap tyToHoodTicks = new InterpolatingDoubleTreeMap();


    public IntakeArmMode getIntakeArmMode() {
        return input_armIntake;
    }

    public ShooterMode getShooterMode() {
        return shooter_mode;
    }

    public IntakeMode getIntakeMode(){
        return intake_mode;
    }

    public ClimbMode getClimbMode(){
        return climb_mode;
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

    public IntakeWheelMode getIntakeWheelMode() {
        return input_wheelIntake;
    }

    public double getInnerElevatorTarget() {
        return input_innerElevatorTarget;
    }
    public double getOuterElevatorTarget() {
        return input_outerElevatorTarget;
    }

    public boolean getElevatorManual() {
        return elevator_manual;
    }

    public void resetTargets() {
        input_innerElevatorTarget = 0.;
		input_outerElevatorTarget = 0;
     
    }

    public boolean getHomeButtonPressed() {
        return elevator_manualHomePressed;
    }

    public boolean getHomeButton() {
        return elevator_manualHome;
    }

    // pos. direction: 0 ~ 1
    // neg. direction: 0 ~ -1
    /*
     * private static double getBoosterValue() {
     * if (!driverJoystickDef.isConnected()) return 0;
     * double value = (1 -
     * Constants.ControllerConstants.driverJoystick.getRawAxis(3))/2;
     * if (value != 0) return getBoosterDirectionInternal() ? value : -value;
     * else return getBoosterDirectionInternal() ? 0.01 : -0.01;
     * }
     * 
     * public static double getDriverBoosterValue() {
     * return driverJoystickDef.isConnected() ? getBoosterValue() : 0.;
     * }
     * 
     * 
     * // true: default (positive)
     * // false: reversed (negative)
     * private static boolean getBoosterDirectionInternal() {
     * return
     * !Constants.ControllerConstants.driverJoystickDef.getRawButton(Constants.
     * SubsystemConstants.RemoteButtons.ShooterRevert);
     * }
     * 
     */

    // private static boolean takeoverEnabled = false;

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
      
       
    

    if (LimelightHelpers.getTV("limelight")==true ){
        RawFiducial[] fiducials = LimelightHelpers.getRawFiducials("limelight");
        RawFiducial fiducial = fiducials[0];
        SmartDashboard.putNumber("TagDist", fiducial.distToCamera);
    




}
  
        // Elevator Manual Toggle
        if (operatorJoystickDef.getRightBumperButtonPressed()) {
            elevator_manual = !elevator_manual;
         
        }

        
        elevator_manualHomePressed = operatorJoystickDef.getRawButtonPressed(RemoteOperatorButtons.home);
        elevator_manualHome = operatorJoystickDef.getRawButton(RemoteOperatorButtons.home);

        if (elevator_manualHomePressed) {
            resetTargets();
        }


        drive_slow = driverJoystickDef.getRightBumperButton();



        // Elevator
        if (elevator_manual) {
         
        } else {
            if (operatorJoystickDef.isConnected()) {
                if (operatorJoystickDef.getAButton()) {
                    input_innerElevatorTarget = Constants.ReefLayers.L1;
                    input_outerElevatorTarget = 0.40;
                } else if (operatorJoystickDef.getBButton()) {
                    input_innerElevatorTarget = Constants.ReefLayers.L2;
                    input_outerElevatorTarget = 0;
                } else if (operatorJoystickDef.getXButton()) {
                    input_innerElevatorTarget = Constants.ReefLayers.L3;
                    input_outerElevatorTarget = 0;
                } else if (operatorJoystickDef.getYButton()) {
                    input_innerElevatorTarget = Constants.ReefLayers.L4;
                    input_outerElevatorTarget = 0;
                }
            }
        }

        inverseDriveDirection();

        // Intake
        if(operatorJoystickDef.isConnected()){
            if(operatorJoystickDef.getRightY() > 0.05){
                intake_mode = IntakeMode.Intake;
                //-hood.setTargetTicks(0);
            } else if(operatorJoystickDef.getRightY() < -0.05){
                intake_mode = IntakeMode.Reverse;

            } else {
                intake_mode = IntakeMode.Idle;
            }
        }
/* 
        if (operatorJoystickDef.isConnected()) {
            if (operatorJoystickDef.getLeftBumperButtonPressed()== true){
                hood.adjustTicks(0.05);
            } else if (operatorJoystickDef.getRightBumperButtonPressed()==true) {
                hood.adjustTicks(-0.05);
            } 
        }
*/
        // Shooter
         if (operatorJoystickDef.getLeftY()> 0.1) {
            shooter_mode = ShooterMode.Reverse;
        } else if (operatorJoystickDef.getLeftY()<-0.1) {
            shooter_mode = ShooterMode.Shoot;
        } else {
            shooter_mode = ShooterMode.Idle;
        } 

        // Climb
        if(driverJoystickDef.getRightBumperButton() == true){
            climb_mode = ClimbMode.Climb;
        } else if (driverJoystickDef.getLeftBumperButton() == true){
            climb_mode = ClimbMode.Reverse;
        }else{
            climb_mode = ClimbMode.Idle;
        }

        // Hood
        if (operatorJoystickDef.getYButton()){
            hood_mode = HoodMode.AutoAim;
            shooter_mode = ShooterMode.Shoot;
        }else if (operatorJoystickDef.getRightBumperButton()){
            hood_mode = HoodMode.ManualUp;
            shooter_mode = ShooterMode.Idle;
        }else if(operatorJoystickDef.getLeftBumperButton()){
            hood_mode = HoodMode.ManualDown;
            shooter_mode = ShooterMode.Idle;
        }else{
            hood_mode = HoodMode.ReturnToZero;
            if (operatorJoystickDef.getLeftY()> 0.1) {
                shooter_mode = ShooterMode.Reverse;
            } else if (operatorJoystickDef.getLeftY()<-0.1) {
                shooter_mode = ShooterMode.Shoot;
            } else {
                shooter_mode = ShooterMode.Idle;
            } 
        }

        // Input -> intake arm command
        if (operatorJoystickDef.isConnected()) {
            if (operatorJoystickDef.getPOV() == 0) {
                input_armIntake = IntakeArmMode.Up;
            } else if (operatorJoystickDef.getPOV() == 180) {
                input_armIntake = IntakeArmMode.Down;
            } else {
                input_armIntake = IntakeArmMode.Idle;
            }
        }
        // input -> intake wheel command
        if (operatorJoystickDef.isConnected()) {
            if (operatorJoystickDef.getPOV() == 270 && !operatorJoystickDef.getLeftBumperButton()) {
                input_wheelIntake = IntakeWheelMode.In;
            } else if (operatorJoystickDef.getPOV() == 90) {
                input_wheelIntake = IntakeWheelMode.Shoot;}
            else if(operatorJoystickDef.getPOV()==270 && operatorJoystickDef.getLeftBumperButton()){
                input_wheelIntake= IntakeWheelMode.Hold;
            
            } else {
                input_wheelIntake = IntakeWheelMode.Idle;
            }
        }

    }
}
