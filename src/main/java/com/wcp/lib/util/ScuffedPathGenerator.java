package com.wcp.lib.util;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.EventMarker;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.server.PathPlannerServer;
import com.wcp.lib.geometry.Pose2d;
import com.wcp.lib.geometry.Rotation2d;
import com.wcp.lib.geometry.Translation2d;

import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

/** Custom PathPlanner version of SwerveControllerCommand */
public class ScuffedPathGenerator extends SubsystemBase {
  List<List<Translation2d>> objectContraints = new ArrayList();
  
  public static ScuffedPathGenerator getInstance() {// if doesnt have an instance of swerve will make a new one
    if (instance == null)
        instance = new ScuffedPathGenerator();
    return instance;
}

public static PathPlannerTrajectory generateAvoidedPath(
      PathConstraints constraints,
      PathPoint point1,
      PathPoint point2,
      PathPoint... points) {
    List<PathPoint> pointsList = new ArrayList<>();
    pointsList.add(point1);
    pointsList.add(point2);
    pointsList.addAll(List.of(points));
    for(int i = 0; i < pointsList.size(); i ++){

    }
    return generatePath(constraints, reversed, pointsList);
  }

public static void insertObject(List<Translation2d> objectDimensions){
  objectContraints.add

}
public static void removeObject()


 
}
