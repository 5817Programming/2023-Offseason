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
    // while(!noColosions){
       for(double i=0; i < testTraj.getTotalTimeSeconds()*100; i++){
         testLine = new Line2d(new Translation2d(testTraj.sample(i/100).poseMeters.getTranslation().getX(), testTraj.sample(i/100).poseMeters.getTranslation().getY()),
        new Translation2d(testTraj.sample((i+1)/100).poseMeters.getTranslation().getX(),testTraj.sample((i+1)/100).poseMeters.getTranslation().getY()));
        for(Line2d obj:objectContraints){
          if(testLine.intersectsWith(obj)){
            pointsList.add(new PathPoint(testLine.getInterSection().translateBy(testLine.getDistanceIntersectionToEnd(obj,true))));
            testTraj = PathPlanner.generatePath(constraints,pointsList);
          }
        }
       }
  //       for(int j = 0; j < objectContraints.size(); j++){
  //         Line2d lineAcrossPoints = new Line2d(new Translation2d(pointsList.get(pointIndex).position.getX(),pointsList.get(pointIndex).position.getY()), new Translation2d(pointsList.get(pointIndex+1).position.getX(),pointsList.get(pointIndex+1).position.getY()));
  //           if(testLine.intersectsWith(objectContraints.get(j))){
  //               if(objectContraints.get(j).getSlope()>1){
  //                 if(lineAcrossPoints.getTop().y()>objectContraints.get(j).getTop().y()){
  //                   pointsList.add(pointsList.size()-2, new PathPoint(objectContraints.get(j).getTop(), new Rotation2d()));
  //                 }
  //                 else{
  //                   pointsList.add(pointsList.size()-2, new PathPoint(objectContraints.get(j).getBottom(), new Rotation2d()));
  //                 }
  //               }
  //               else{
  //                 if(lineAcrossPoints.getRight().x()>objectContraints.get(j).getRight().x()){
  //                   pointsList.add(pointsList.size()-2, new PathPoint(objectContraints.get(j).getTop(), new Rotation2d()));
  //                 }
  //                 else{
  //                   pointsList.add(pointsList.size()-2, new PathPoint(objectContraints.get(j).getBottom(), new Rotation2d()));
  //                 }
  //               }
  //               pointIndex++;
  //           }
            
  //       }
  //  //   }

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
