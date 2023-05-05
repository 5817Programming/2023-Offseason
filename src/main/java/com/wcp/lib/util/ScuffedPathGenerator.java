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
  int pointIndex = 0;
  Translation2d previousPose;
  List<Line2d> objectContraints = new ArrayList();
  Translation2d lastSample = new Translation2d();
  double heading;

  private static ScuffedPathGenerator instance = null;
  public static ScuffedPathGenerator getInstance() {// if doesnt have an instance will make a new one
    if (instance == null)
        instance = new ScuffedPathGenerator();
    return instance;
}

public PathPlannerTrajectory generateAvoidedPath(
    PathConstraints constraints,
    Translation2d currentSample = new Translation2d();
    PathPoint point1,
    PathPoint point2) {
    List<PathPoint> pointsList = new ArrayList<>();
    Line2d testLine = new Line2d();
    pointsList.add(point1);
    pointsList.add(point2);
     PathPlannerTrajectory testTraj = PathPlanner.generatePath(constraints,pointsList);
       for(double i=0; i < testTraj.getTotalTimeSeconds()*100; i++){
        currentSample = testTraj.sample(i);
        heading = Math.atan((lastSample.y()-currentSample.y())/lastSample.x()-currentSample.x())*(180/Math.PI);
        testLine = new Line2d(new Translation2d(testTraj.sample(i/100).poseMeters.getTranslation().getX(), testTraj.sample(i/100).poseMeters.getTranslation().getY()),
        new Translation2d(testTraj.sample((i+1)/100).poseMeters.getTranslation().getX(),testTraj.sample((i+1)/100).poseMeters.getTranslation().getY()));
        for(Line2d obj:objectContraints){
          if(testLine.intersectsWith(obj)){
            pointsList.add(new PathPoint(testLine.getInterSection(obj).translateBy(testLine.getDistanceIntersectionToEnd(obj)),new Rotation2d.fromDegrees(heading)));
            testTraj = PathPlanner.generatePath(constraints,pointsList);
          }
        }
        lastSample = currentSample;
       }
      

      return PathPlanner.generatePath(constraints, pointsList);
  }
public void insertObject(Line2d line1, Line2d line2, Line2d... lines){
  objectContraints.add(line1.push(Constants.mRobotHypot).extend(Constants.mRobotHypot));
  objectContraints.add(line2.push(Constants.mRobotHypot).extend(Constants.mRobotHypot));
  List<Line2d> tempList = new ArrayList<>();
  tempList.addAll(List.of(lines));
  for(int i = 0; i < tempList.size(); i++) {tempList.set(i, tempList.get(i).push(Constants.mRobotHypot).extend(Constants.mRobotHypot));}
  objectContraints.addAll(tempList);
}

public void removeObject(){
  objectContraints.clear();
}


 
}
