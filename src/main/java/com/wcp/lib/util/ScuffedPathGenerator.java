package com.wcp.lib.util;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.EventMarker;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.server.PathPlannerServer;
import com.wcp.lib.geometry.Line2d;
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
  boolean noColosions;
  List<List<Translation2d>> objectContraints = new ArrayList();

  private static ScuffedPathGenerator instance = null;
  public static ScuffedPathGenerator getInstance() {// if doesnt have an instance will make a new one
    if (instance == null)
        instance = new ScuffedPathGenerator();
    return instance;
}

public static PathPlannerTrajectory generateAvoidedPath
      PathConstraints constraints,
      Translation2d point1,
      Translation2d point2,
      Translation2d... points) {
    List<Translation2d> pointsList = new ArrayList<>();
    pointsList.add(point1);
    pointsList.add(point2);
    pointsList.addAll(List.of(points));
    for(int i = 0; i < pointsList.size()-1; i ++){
        Line2d testLine = new Line2d(pointsList.get(i), pointsList.get(i+1));6780¿89''''''''''''''''''''''''''''''''''''''''''''''

    }
    return generatePath(constraints, pointsList);
  }

public static void insertObject(Line2d line1, Line2d line2, Line2d... lines){
  objectContraints.add(line1);
  objectContraints.add(line2);
  objectContraints.add(List.of(points));
}

public static void removeObject(){
  objectContraints.clear();
}
public static void removeObject(){
  
}


 
}
