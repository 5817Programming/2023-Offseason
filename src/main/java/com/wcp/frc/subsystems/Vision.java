// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.wcp.frc.subsystems;

import com.wcp.frc.Constants;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.wcp.lib.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  double[] empty = {0.0,0.0,0.0,0.0,0.0,0.0};
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
NetworkTableEntry tx = table.getEntry("tx");
NetworkTableEntry ty = table.getEntry("ty");
NetworkTableEntry ta = table.getEntry("ta");
NetworkTableEntry botpose = table.getEntry("botpose");

NetworkTableEntry tv = table.getEntry("tv");
  public static Vision instance = null;
  public static Vision getInstance() {
    if(instance == null)
        instance = new Vision();
    return instance;
}

public double range;
Swerve swerve;
int index;
  /** Creates a new Vision. */
  public Vision() {
    

//read values periodically

  swerve = Swerve.getInstance();

//post to smart dashboard periodically

  }
  public double x(){  
    double x = tx.getDouble(0.0);
    return x;
  }
  public double y(){
    double y = ty.getDouble(0.0);
    return y;
  }
  public double area(){
    double area = ta.getDouble(0.0);
    return area;
  }
  public double getDistance(){//gets distance to target
    double distanceFromLimelightToGoalInches = (0 - Constants.VisionConstants.LIMELIGHT_LENS_HEIGHT_INCHES)/Math.tan(Math.toRadians(Constants.VisionConstants.LIMELIGHT_MOUNT_ANGLE_DEGREES + y()));
  return distanceFromLimelightToGoalInches>0&&distanceFromLimelightToGoalInches<1000?Units.inchesToMeters(distanceFromLimelightToGoalInches):0;
  }

  public double getDistanceToGroundObject(){//gets distance to target
    double distanceFromLimelightToGoalInches = (0 - Constants.VisionConstants.LIMELIGHT_LENS_HEIGHT_INCHES)/Math.tan(Math.toRadians(Constants.VisionConstants.LIMELIGHT_MOUNT_ANGLE_DEGREES + y()));
  return distanceFromLimelightToGoalInches>0&&distanceFromLimelightToGoalInches<1000?Units.inchesToMeters(distanceFromLimelightToGoalInches):0;
  }


  double Yaw;
  Constants constants = new Constants();
 /*  public void setfinished(boolean isFinishedd){
    
     isFinished = isFinishedd;
  }*/   


  public double getYaw(){
   
    Yaw = tx.getDouble(0.0);
return Yaw;
}
public void   setPipeline(Integer pipeline) {
  if(pipeline<0){
      pipeline = 0;
      throw new IllegalArgumentException("Pipeline can not be less than zero");
  }else if(pipeline>9){
      pipeline = 9;
      throw new IllegalArgumentException("Pipeline can not be greater than nine");
  }
  table.getEntry("pipeline").setValue(pipeline);
}
 public boolean hasTarget(){

  double v = tv.getDouble(0.0);
  if (v == 0.0f){
    return false;
}else {
    return true;
}
 }
 public void setPipelineIndex(int indexer){
  index = indexer;
  

 }

 static double goalHeightInches;



 public double range(double height){
  
return 0;
 }

 public void updatePose(){
  double[] newpose = botpose.getDoubleArray(empty);
  swerve.resetPose(new Pose2d(newpose[0],newpose[1],swerve.getRobotHeading()));
 }


 @Override
 public void periodic() {
   // This method will be called once per scheduler run
   SmartDashboard.putNumber("LimelightX", x());
   SmartDashboard.putNumber("LimelightY", y());
   SmartDashboard.putNumber("LimelightArea", area());
   SmartDashboard.putNumber("ranges", range(0));
 }

}
