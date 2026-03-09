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
    ClimbMode climb_mode = ClimbMode.Idle;
    HoodMode hood_mode = HoodMode.Idle;

    boolean drive_slow = false;


    // Constructor
    // Here we will be creating private constructor
    // restricted to this class itself
    public Remote() {

        tyToShooterSpeed.put(-15.0, 0.95);
        tyToShooterSpeed.put(0.0, 0.6);
        tyToShooterSpeed.put(15.0, 0.35);


    }

    public InterpolatingDoubleTreeMap tyToShooterSpeed = new InterpolatingDoubleTreeMap();


    public ShooterMode getShooterMode() {
        return shooter_mode;
    }

    public IntakeMode getIntakeMode(){
        return intake_mode;
    }

    public ClimbMode getClimbMode(){
        return climb_mode;
    }

    public HoodMode getHoodMode(){
        return hood_mode;
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
        if(fiducials != null && fiducials.length > 0){
        SmartDashboard.putNumber("FiducalID", LimelightHelpers.getFiducialID("limelight"));
        RawFiducial fiducial = fiducials[0];
        SmartDashboard.putNumber("TagDist", fiducial.distToCamera);
        }
    }


        drive_slow = driverJoystickDef.getRightBumperButton();

        inverseDriveDirection();

        // Intake
        if(operatorJoystickDef.isConnected()){
            if(operatorJoystickDef.getRightY() > 0.05){
                intake_mode = IntakeMode.Intake;

            } else if(operatorJoystickDef.getRightY() < -0.05){
                intake_mode = IntakeMode.Reverse;

            } else {
                intake_mode = IntakeMode.Idle;
            }
        }

   

        if(operatorJoystickDef.isConnected()){
            
            // Check if any of the preset buttons are currently being held
            boolean isPresetPressed = operatorJoystickDef.getYButton() || operatorJoystickDef.getBButton() || 
                                      operatorJoystickDef.getAButton() || operatorJoystickDef.getXButton();

            if (operatorJoystickDef.getLeftStickButton()) {
                // Clicked down (L3): Shoot Forward
                shooter_mode = ShooterMode.Shoot; 
            } else if (isPresetPressed || operatorJoystickDef.getLeftY() < -0.05) {
                // Pushed UP or a Preset is held: Rev Forward
                shooter_mode = ShooterMode.Rev; 
             
            } else if (operatorJoystickDef.getLeftY() > 0.05) {
                // Pulled DOWN (Positive Y): Run everything in Reverse to clear a jam
                shooter_mode = ShooterMode.Reverse;
            } else {
                shooter_mode = ShooterMode.Idle;
            }
        }


        // Climb
        if(driverJoystickDef.getPOV() == 180){
            climb_mode = ClimbMode.Climb;
        } else if (driverJoystickDef.getPOV() == 0){
            climb_mode = ClimbMode.Reverse;
        }else{
            climb_mode = ClimbMode.Idle;
        }

       // Hood
        if(operatorJoystickDef.getPOV() == 180){
            hood_mode = HoodMode.MoveUp;
        } else if (operatorJoystickDef.getPOV() == 0){
            hood_mode = HoodMode.MoveDown;
        }else{
            hood_mode = HoodMode.Idle;
        }

    }

    
}
