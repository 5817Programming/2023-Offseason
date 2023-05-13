// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.wcp.frc;

import java.util.Arrays;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.wcp.frc.subsystems.SubsystemManager;
import com.wcp.frc.subsystems.Swerve;
import com.wcp.frc.subsystems.Vision;
import com.wcp.lib.geometry.Rotation2d;
import com.wcp.lib.geometry.Translation2d;
import com.wcp.lib.geometry.HeavilyInspired.Node;
import com.wcp.lib.util.PathGenerator;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;


import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import com.wcp.frc.subsystems.SubsystemManager;
import com.wcp.frc.subsystems.Swerve;
import com.wcp.frc.subsystems.Vision;
import com.wcp.frc.subsystems.gyros.Gyro;
import com.wcp.frc.subsystems.gyros.Pigeon;
import com.wcp.frc.subsystems.Lights;
import com.ctre.phoenix.Util;
import com.wcp.frc.subsystems.Arm;
import com.wcp.frc.subsystems.Scores;
import com.wcp.frc.subsystems.Elevator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
//https://github.com/Mechanical-Advantage/AdvantageKit/releases/latest/download/AdvantageKit.json

//https://maven.photonvision.org/repository/internal/org/photonvision/PhotonLib-json/1.0/PhotonLib-json-1.0.json

import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends LoggedRobot {



  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  // private RobotContainer robotContainer = new RobotContainer();

  // RobotContainer robotContainer = new RobotContainer();
  private Command colorChooser;
  Controls controls;
  PathGenerator scuffedPathGenerator;
  SubsystemManager subsystemManager;
  Swerve swerve;
  double yaw;
  Elevator elevator = new Elevator();
  Vision vision;
  Arm arm = new Arm();
  Lights lights;
  Gyro pigeon;

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

    controls = Controls.getInstance();
    swerve = Swerve.getInstance();

    
    // Logger.getInstance().disableDeterministicTimestamps() // See "Deterministic
    // Timestamps" in the "Understanding Data Flow" page
    Logger.getInstance().start(); // Start logging! No more data receivers, replay sources, or metadata values may
                                  // be added.
    swerve = Swerve.getInstance();
    swerve.zeroModules();


    controls = Controls.getInstance();
    swerve = Swerve.getInstance();
    vision = Vision.getInstance();
    

    subsystemManager = new SubsystemManager();
    subsystemManager.addSystems(Arrays.asList(
        swerve));
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and
   * test.
   *
   * 
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    swerve.updateOdometry(defaultPeriodSecs);
    elevator.getEncoder();
    arm.getEncoder();

    subsystemManager.updateSubsystems();
    subsystemManager.readSystemsPeriodicInputs();
    subsystemManager.writeSubsystemsPeriodicOutputs();
    subsystemManager.outputSystemsTelemetry();
    CommandScheduler.getInstance().run();

    swerve.updatePose(Timer.getFPGATimestamp());
   // X = position.getX();
   // Y = position.getY();
    //rotation = position.getRotation().getDegrees();
    //sends states to advantage scope
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different
   * autonomous modes using the dashboard. The sendable chooser code works with
   * the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the
   * chooser code and
   * uncomment the getString line to get the auto name from the text box below the
   * Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure
   * below with additional strings. If using the SendableChooser make sure to add
   * them to the
   * chooser code above as well.
   */

  double startime;

  @Override
  public void autonomousInit() {

    // // schedule the autonomous command (example)
    // if (autoChooser.getSelected() != null) {
    // autoChooser.getSelected().schedule();
    // }
    // new Auto6(swerve).schedule();
    startime = Timer.getFPGATimestamp();
      
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

    // switch (m_autoSelected) {
    // case kCustomAuto:
    // // Put custom auto code here
    // break;
    // case kDefaultAuto:
    // default:
    // // Put default auto code here
    // // break;
    // swerve.updateTrajectory();
    // swerve.startTrjectory();
    // // }
    // swerve.followTrajectory();
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {

   
     System.out.println("run");
    

    // scores.zero();
    swerve = Swerve.getInstance();
    swerve.fieldzeroSwerve();
    swerve.sendInput(0, 0,0);

    // vision.range(1);


  }
  double lastTime;

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    controls.update();

    double currentTime = Timer.getFPGATimestamp();
    if(currentTime - lastTime > 1){
    PathPlannerTrajectory toGrid = PathGenerator.generatePath(new PathConstraints(4, 4), new Node(new Translation2d(14.3,2.85),new Rotation2d(-180)),Constants.FieldConstants.obstacles);
    Logger.getInstance().recordOutput("toGrid", toGrid);
    lastTime = currentTime;
    }

    //swerve.update(Timer.getFPGATimestamp());

  }

  /** This function is called once when the robot is disabled. */

  @Override
  public void disabledInit() {
    subsystemManager.stopSubsystems();
    arm.setPercentOutput(0);
    elevator.my_PercentOutput(0);
 //   scores.setHeight(Constants.ElevatorConstants.ZERO,true);
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    

  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {

  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }
}
