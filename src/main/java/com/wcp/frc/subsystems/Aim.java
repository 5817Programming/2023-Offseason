// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.wcp.frc.subsystems;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.wcp.lib.util.PathFollower;
import com.wcp.lib.util.PathGenerator;
import com.wcp.frc.Constants;
import com.wcp.frc.subsystems.gyros.Gyro;
import com.wcp.frc.subsystems.gyros.Pigeon;
import com.wcp.lib.geometry.Rotation2d;
import com.wcp.lib.geometry.Translation2d;
import com.wcp.lib.geometry.HeavilyInspired.Node;
import com.wcp.lib.util.SynchronousPIDF;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
  SynchronousPIDF rPID;

  PIDController thetaController;
  PIDController advanceController;
  double lastTimeStamp = 0;
  PathFollower scuffedPathPlanner = PathFollower.getInstance();
  double Roboty;
  double Robotx;
  Vision vision = Vision.getInstance();
  Gyro pigeon = Pigeon.getInstance();
  double xERROR;
  double yERROR;

  
  public static Aim instance = null;

  public static Aim getInstance(){
    if(instance == null)
      instance = new Aim();
    return instance;
  }

  double bestDistance;

  /** Creates a new Aim. */

  public Aim() {
    swerve = Swerve.getInstance();
     xPID = new SynchronousPIDF(.5, 0.0, 0);
     yPID = new SynchronousPIDF(.5, 0.0, 0);
     rPID = new SynchronousPIDF(.5, 0.0, 0);
     

     thetaController = new PIDController(0.5, 0, 0);
     advanceController = new PIDController(0.5, 0, 0);

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
    if(!pathStarted){
      scuffedPathPlanner.resetTimer();
    }
      if(DriverStation.getAlliance() == Alliance.Blue){

      
      toGrid = PathGenerator.generatePath(new PathConstraints(4, 4), new Node(new Translation2d(2.07,2.85),new Rotation2d(-180)),Constants.FieldConstants.obstacles);

    }else{
      toGrid = PathGenerator.generatePath(new PathConstraints(4, 4), new Node(new Translation2d(14.41,2.85),new Rotation2d(0)),Constants.FieldConstants.obstacles);

    }      
    Logger.getInstance().recordOutput("toGrid", toGrid);
    // scuffedPathPlanner.setTrajectory(toGrid);
    // pathStarted = true;
    // Translation2d targetTranslation2d = scuffedPathPlanner.getDesiredPose2d(false, 1, swerve.getPose()).getTranslation();
    // double currentTime  = Timer.getFPGATimestamp();
    // double dt = currentTime-lastTimeStamp;
    // xError = xPID.calculate(Robotx-targetTranslation2d.getX(), dt);
    // yError = yPID.calculate(Roboty-targetTranslation2d.getY(), dt);

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
    Logger.getInstance().recordOutput("xError",xERROR);
    Logger.getInstance().recordOutput("yError",yERROR);

    swerve.Aim(new Translation2d(xError, -yError), Rotation2d.fromDegrees(rotation));
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
  public void goToObject(){
    //makes sure we have can see a target
    vision.setPipeline(Constants.VisionConstants.CONE_PIPELNE);
    if(!vision.hasTarget()){//if we cant see a cone we will look for a cube
      vision.setPipeline(Constants.VisionConstants.CUBE_PIPELINE);
      if(!vision.hasTarget()){//if we cant see a cube we will exit the function becuase we dont have anywhere to go
        return;
      }
    }
    //gets error
    xERROR = thetaController.calculate(vision.x(),0);
    yERROR= advanceController.calculate(vision.y(),-5);
    //corrects for error
    swerve.Aim(new Translation2d(xERROR,yERROR),0);
    

  }

  public void goToObject(boolean cube){
    Rotation2d heading = Rotation2d.fromDegrees(pigeon.getAngle());
    if(cube){
      vision.setPipeline(Constants.VisionConstants.CUBE_PIPELINE);
    }
    else{
      vision.setPipeline(Constants.VisionConstants.CONE_PIPELNE);
    }
    double xSetPoint = (.1*heading.cos());
    double ySetPoint = (.1*heading.sin());
    double xError = xPID.calculate(vision.getDistanceToGroundObject()*heading.cos(),xSetPoint);
    double yError = yPID.calculate(vision.getDistanceToGroundObject()*heading.sin(),ySetPoint);
    double thetaControl = rPID.calculate(vision.y(), 0);

    swerve.Aim(new Translation2d(xError,yERROR), thetaControl);

  }
}
