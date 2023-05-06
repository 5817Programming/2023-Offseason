// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.wcp.frc;

import java.util.Arrays;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.wcp.frc.subsystems.SubsystemManager;
import com.wcp.frc.subsystems.Swerve;
import com.wcp.frc.subsystems.Vision;
import com.wcp.lib.geometry.Rotation2d;
import com.wcp.lib.geometry.Translation2d;
import com.wcp.lib.util.ScuffedPathGenerator;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Robot extends LoggedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  Controls controls;
  ScuffedPathGenerator scuffedPathGenerator;
  SubsystemManager subsystemManager;
  Swerve swerve;
  double yaw;
    
  Vision vision= new Vision();
  @Override
  public void robotInit() {
    Logger.getInstance().recordMetadata("ProjectName", "MyProject"); // Set a metadata value

      // Logger.getInstance().addDataReceiver(new WPILOGWriter("/media/sda1/")); // Log to a USB stick
      Logger.getInstance().addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
      new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
    

    // Logger.getInstance().disableDeterministicTimestamps() // See "Deterministic
    // Timestamps" in the "Understanding Data Flow" page
    Logger.getInstance().start(); 
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    controls = Controls.getInstance();
    swerve = Swerve.getInstance();
    subsystemManager = new SubsystemManager();
    subsystemManager.addSystems(Arrays.asList(
            swerve
    ));
    scuffedPathGenerator.insertObject(
      new Line2d(new Translation2d(2.8,1.52), new Translation2d(4.8,1.52)),
      new Line2d(new Translation2d(2.8,4), new Translation2d(4.8,4)),
      new Line2d(new Translation2d(2.8,4), new Translation2d(2.8,1.52)),
      new Line2d(new Translation2d(4.8,4), new Translation2d(4.8,1.52)));
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    subsystemManager.updateSubsystems();
    subsystemManager.readSystemsPeriodicInputs();
    subsystemManager.writeSubsystemsPeriodicOutputs();
    subsystemManager.outputSystemsTelemetry();
    CommandScheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
   
     System.out.println("run");
    
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

      PathPlannerTrajectory testthing = PathPlanner.generatePath(new PathConstraints(4,4), new PathPoint(Translation2d.toWPI(2.09, 1.18), Rotation2d.fromDegrees(0)) ,  new PathPoint( Translation2d.toWPI(6.17, 4.67), Rotation2d.fromDegrees(0)));

     Logger.getInstance().recordOutput("test thing", testthing);
     PathPlannerTrajectory testTraj = 
     Logger.getInstance().recordOutput("test Trajectory", testTraj);
    controls.update();
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    subsystemManager.stopSubsystems();
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    swerve.zeroModules();
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
