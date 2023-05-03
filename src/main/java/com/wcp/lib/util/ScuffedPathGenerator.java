package com.wcp.lib.util;


import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.PathPlannerTrajectory.EventMarker;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.server.PathPlannerServer;
import com.wcp.frc.Constants;
import com.wcp.lib.geometry.Line2d;
import com.wcp.lib.geometry.Pose2d;
import com.wcp.lib.geometry.Rotation2d;
import com.wcp.lib.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.*;

import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

/** Custom PathPlanner version of SwerveControllerCommand */
public class ScuffedPathGenerator extends SubsystemBase {
  boolean noColosions;
  Translation2d previousPose;
  List<Line2d> objectContraints = new ArrayList();

  private static ScuffedPathGenerator instance = null;
  public static ScuffedPathGenerator getInstance() {// if doesnt have an instance will make a new one
    if (instance == null)
        instance = new ScuffedPathGenerator();
    return instance;
}

public PathPlannerTrajectory generateAvoidedPath(
    PathConstraints constraints,
    PathPoint point1,
    PathPoint point2) {
    List<PathPoint> pointsList = new ArrayList<>();
    Line2d testLine = new Line2d();
    pointsList.add(point1);
    pointsList.add(point2);
    PathPlannerTrajectory testTraj = PathPlanner.generatePath(constraints,pointsList);
   for(int i=0;i < testTraj.getTotalTimeSeconds()*100; i++){
    testLine = new Line2d(new Translation2d(testTraj.sample(i).poseMeters.getTranslation().getX(), testTraj.sample(i).poseMeters.getTranslation().getY()),
     new Translation2d(testTraj.sample(i+1).poseMeters.getTranslation().getX(),testTraj.sample(i+1).poseMeters.getTranslation().getY()));
   }
    for(int j = 0; j < objectContraints.size(); j++){
        if(testLine.intersectsWith(objectContraints.get(j))){
            if(objectContraints.get(j).getSlope()>1){
              if(testLine.getTop().y()>objectContraints.get(j).getTop().y()){
                pointsList.add(pointsList.size()-2, new PathPoint(objectContraints.get(j).getTop(), new Rotation2d()));
              }
              else{

              }
            }
        }
    }

      return new PathPlannerTrajectory();
  }
public void insertObject(Line2d line1, Line2d line2, Line2d... lines){
  objectContraints.add(line1.extend(Constants.mRobotHypot));
  objectContraints.add(line2.extend(Constants.mRobotHypot));
  List<Line2d> tempList = new ArrayList<>();
  tempList.addAll(List.of(lines));
  for(int i = 0; i < tempList.size(); i++) {tempList.set(i, tempList.get(i).extend(Constants.mRobotHypot));}
  objectContraints.addAll(tempList);
}

public void removeObject(){
  objectContraints.clear();
}


 
}
