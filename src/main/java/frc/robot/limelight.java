package frc.robot;

import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;


public class limelight {

    
    
   
    RawFiducial[] fiducials = LimelightHelpers.getRawFiducials("limelight");{

    for (RawFiducial fiducial : fiducials){
        int id = fiducial.id;
        double txnc = fiducial.txnc;
        double tync = fiducial.tync;
        double distToCamera = fiducial.distToCamera;
        double distToRobot = fiducial.distToRobot;
        double ambiguity = fiducial.ambiguity;



    }

}}
    


