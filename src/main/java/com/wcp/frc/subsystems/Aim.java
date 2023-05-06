// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.wcp.frc.subsystems;

import java.lang.annotation.Target;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.wcp.lib.util.ScuffedPathPlanner;
import com.wcp.frc.Constants;
import com.wcp.lib.geometry.Pose2d;
import com.wcp.lib.geometry.Translation2d;
import com.wcp.lib.util.SynchronousPIDF;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Aim extends SubsystemBase {
  Swerve swerve;
  boolean pathStarted;
  double distance;
  int bestScore;
  int offset= 0;
  double xError;
  double yError;
  SynchronousPIDF xPID;
  SynchronousPIDF yPID;
  double lastTimeStamp = 0;
  ScuffedPathPlanner scuffedPathPlanner = ScuffedPathPlanner.getInstance();
  double Roboty;
  double Robotx;
  

  double bestDistance;

  /** Creates a new Aim. */

  public Aim() {
    swerve = Swerve.getInstance();
     xPID = new SynchronousPIDF(.5, 0.0, 0);
     yPID = new SynchronousPIDF(.5, 0.0, 0);

  }
  


  @Override
  public void periodic() {
  Roboty = swerve.getPose().getTranslation().getY();
  Robotx = swerve.getPose().getTranslation().getX();
  //CommandScheduler.getInstance().run();
  swerve = Swerve.getInstance();

    
    
    // This method will be called once per scheduler run
  }

  public void goToGrid(){
    PathPlannerTrajectory toGrid;
    double rotation = 0;
    if(!pathStarted){
      if(Roboty>2.75){
        if(DriverStation.getAlliance() == Alliance.Blue){
        rotation = 180;
      
      toGrid = PathPlanner.generatePath(
        new PathConstraints(4, 3), 
        new PathPoint(swerve.getPose().getTranslation(), Rotation2d.fromDegrees(0), swerve.getPose().getRotation()),
        new PathPoint(new Translation2d(5.35, 4.48), Rotation2d.fromDegrees(0), swerve.getPose().getRotation()),
        new PathPoint(new Translation2d(2.39, 4.48), Rotation2d.fromDegrees(0), swerve.getPose().getRotation())
        );

      }else{
        toGrid = PathPlanner.generatePath(
        new PathConstraints(4, 3), 
        new PathPoint(swerve.getPose().getTranslation(), Rotation2d.fromDegrees(0), swerve.getPose().getRotation()),
        new PathPoint(new Translation2d(11.24, 4.48), Rotation2d.fromDegrees(0), swerve.getPose().getRotation()),
        new PathPoint(new Translation2d(14.12, 4.48), Rotation2d.fromDegrees(0), swerve.getPose().getRotation())
        );
      }
      }else{
    if(DriverStation.getAlliance() == Alliance.Blue){
        rotation = 180;
      
      toGrid = PathPlanner.generatePath(
        new PathConstraints(4, 3), 
        new PathPoint(swerve.getPose().getTranslation(), Rotation2d.fromDegrees(0), swerve.getPose().getRotation()),
        new PathPoint(new Translation2d(5.39, 1.08), Rotation2d.fromDegrees(0), swerve.getPose().getRotation()),
        new PathPoint(new Translation2d(2.39, 1.08), Rotation2d.fromDegrees(0), swerve.getPose().getRotation())
        );
      }else{
        toGrid = PathPlanner.generatePath(
        new PathConstraints(4, 3), 
        new PathPoint(swerve.getPose().getTranslation(), Rotation2d.fromDegrees(0), swerve.getPose().getRotation()),
        new PathPoint(new Translation2d(11.12, 1.06), Rotation2d.fromDegrees(0), swerve.getPose().getRotation()),
        new PathPoint(new Translation2d(14.12, 1.06), Rotation2d.fromDegrees(0), swerve.getPose().getRotation())
        );

      }}
      scuffedPathPlanner.resetTimer();
      scuffedPathPlanner.setTrajectory(toGrid);
    }
    pathStarted = true;
    Translation2d targetTranslation2d = scuffedPathPlanner.getDesiredPose2d(false, 1, swerve.getPose()).getTranslation();
    double currentTime  = Timer.getFPGATimestamp();
    double dt = currentTime-lastTimeStamp;
    xError = xPID.calculate(Robotx-targetTranslation2d.getX(), dt);
    yError = yPID.calculate(Roboty-targetTranslation2d.getY(), dt);
    if(Math.abs(Robotx-targetTranslation2d.getX())>.01&&Math.abs(Roboty-targetTranslation2d.getY())>.01){
      swerve.Aim(new Translation2d(xError, -yError), rotation);
    }
    else{
      pathStarted = false;
      new AutoScore().schedule(); 
    }
}
  
        
  
  public void moveTo(int scoringNode){
    double rotation = 0;
    if(DriverStation.getAlliance() == Alliance.Blue){
      rotation = 180;
    }
    double currentTime  = Timer.getFPGATimestamp();
    double dt = currentTime-lastTimeStamp;

    xError = xPID.calculate(Robotx-1.84, dt);
    yError = yPID.calculate(Roboty-Constants.scoresY.get(scoringNode), dt);
    swerve.Aim(new Translation2d(xError, -yError), rotation);
    lastTimeStamp = currentTime;

  }
  public void aimAtScore(boolean cube,boolean snapDown,boolean snapUp){
   for(int i = 0; i < Constants.scoresY.size(); i++){//finds closest scoring node
    distance = Constants.scoresY.get(i)-Roboty;
    if(bestDistance<distance){//if closer than previous closest
      bestDistance = distance;
      bestScore = i+offset;//sets target to closest plus the user inputed offset
    }
    }

  if(snapUp&& (bestScore+1 < Constants.scoresY.size()-1)){//if wants to move up and isnt at 10 than move up
    offset++;//sets desired scoring station to snap to the next one up
  }else if(snapDown&& ( bestScore- 1> 0)){//if wants to move down and isnt at zero than move down
    offset--;//sets desired scoring station to snap to the next one down
  }
  if(DriverStation.getAlliance() == Alliance.Blue){
    if(Robotx > 2.5){
      moveTo(bestScore);
    }else{
      goToGrid();
    }
      
  }
  else {
    if(Robotx < 14.02){
      moveTo(bestScore);
    }else{
      goToGrid();
  }
  }
  
  }
  
}