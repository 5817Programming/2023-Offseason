// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.wcp.frc;

import com.wcp.frc.subsystems.Arm;
import com.wcp.frc.subsystems.Elevator;
import com.wcp.frc.subsystems.Intake;
import com.wcp.frc.subsystems.SideElevator;
import com.wcp.frc.subsystems.Swerve;
import com.wcp.frc.subsystems.Vision;
import com.wcp.frc.subsystems.Swerve.State;
import com.wcp.lib.geometry.Pose2d;
import com.wcp.lib.geometry.Translation2d;

import org.littletonrobotics.junction.Logger;

// import com.wcp.frc.subsystems.toHuman;
import com.wcp.frc.subsystems.Aim;
import com.wcp.frc.subsystems.Lights;
import com.wcp.frc.subsystems.Scores;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;


public class Controls {
    Constants constants = new Constants();
    Aim aim = new Aim();
    Scores scores = new Scores();
    SideElevator sideelevator = new SideElevator();
    Elevator elevator = new Elevator();
    Arm arm = new Arm();
    Intake intake = new Intake();
    Lights lights = new Lights();

    double speed;
    
    

    XboxController Driver;
    XboxController CoDriver;
    double yaw;
    Swerve swerve;
    double Acelerator = .6;
    double autorotate = .2;
    double rotate = 0;
    double ignore = 1;

    double toggle = 0;
    Vision vision;
    boolean pick = false;
    boolean pickup = false;
    boolean hooman;
    boolean hoomanactive;
    int pipeline;
    double elevatorstate;
    boolean cube= true;

    Pose2d sim = new Pose2d();

    private static Controls instance = null;
    public static Controls getInstance() {
        if (instance == null)
            instance = new Controls();
        return instance;
    }

    public Controls() {
        Driver = new XboxController(Ports.XBOX_1);
        CoDriver = new XboxController(Ports.XBOX_2);
        swerve = Swerve.getInstance();
        
    }


    public void update() {
        vision = Vision.getInstance();
        //CommandScheduler.getInstance().run();
       // Command toHuman = new toHuman(odometry);
       


        double driverLeftTrigger = Driver.getLeftTriggerAxis();// slow mode
        double driverRightTrigger = Driver.getRightTriggerAxis();// arm down
        double driverLeftXInput = (((Driver.getLeftX())) * Acelerator);
        double driverLeftYInput = -(Driver.getLeftY() * Acelerator);// drive
        double driverRightXInput = -((((Driver.getRightX() * ignore) + rotate) * 2) * Acelerator);// drive
        boolean driverLeftBumperTAP = Driver.getLeftBumperPressed();// do (hold)
        boolean driverRightBumperTAP = Driver.getRightBumperPressed();// succ (toggle)
        boolean driverLeftStickDown = Driver.getLeftStickButtonPressed();
        boolean driverAButton = Driver.getAButton();
        boolean driverXButton = Driver.getXButton();
        boolean driverYButton = Driver.getYButton();
        boolean driverBButton = Driver.getBButton();
        double driverDpad = Driver.getPOV();
        boolean driverRightStickDown = Driver.getRightStickButton();
        boolean driverStartButtonTAP = Driver.getStartButtonPressed();


        boolean coDriverStart = CoDriver.getStartButton();
        boolean coDriverAButton = CoDriver.getAButton();// bottom
        boolean coDriverBButton = CoDriver.getBButton();// middle
        boolean coDriverXButton = CoDriver.getXButton();// top
        boolean coDriverYButton = CoDriver.getYButton();// top
        boolean codriverLeftBumperTAP = CoDriver.getLeftBumperPressed();
        boolean codriverRightBumperTAP = CoDriver.getRightBumperPressed();// Light toggle
        
        boolean codriverLeftBumper = CoDriver.getLeftBumper();
        boolean codriverRightBumper = CoDriver.getRightBumper();// Light toggle
        double coDriverLeftTrigger = CoDriver.getLeftTriggerAxis();
        double coDriverRightTrigger = CoDriver.getRightTriggerAxis();// triggers are for scoring
        double coDriverLeftX = CoDriver.getLeftX();
        double coDriverLeftY = CoDriver.getLeftY();

        double coDriverRightY = CoDriver.getRightY();
        boolean coDriverLeftStickDown = CoDriver.getLeftStickButton();
        boolean coDriverRightStickDown = CoDriver.getRightStickButton();//// press in joystick to hold
        double coDriverRightX = CoDriver.getRightX();
        double codDriverDpad = CoDriver.getPOV(); // jacks jacks
        boolean coDriverBackButton = CoDriver.getBackButton();


        if (codriverLeftBumperTAP) {
            if (pick) {// cube
                pickup = true;
                pick = false;
            } else {// cone
                pickup = true;
                pick = true;
            }
        }
   
        if (driverLeftStickDown) {
            scores.zero();
        }

        if (Driver.getBackButtonPressed()) {
            swerve.zeroSwerve();
        }

        if (Driver.getStartButtonPressed()) {
            swerve.parkMode();
        }

        Acelerator = (1 - (driverLeftTrigger / 2));

        if (coDriverLeftTrigger > .2) {
            if (pick) {
                scores.elevatorOffset = 20000;
            } else {

                scores.elevatorOffset = 15000;
            }

        } else {
            scores.elevatorOffset = 0;

        }

        if (coDriverRightY > .2) {
            scores.armOffset -= 100;
        }

        if (coDriverRightY < -.2) {
            scores.armOffset += 100;
        }
        if (coDriverBackButton){
            intake.setIntake();
        }




        if (coDriverAButton || coDriverBButton || coDriverXButton || coDriverYButton || coDriverBackButton || coDriverLeftStickDown
                || codriverLeftBumperTAP) {

            scores.setHeight(coDriverAButton, coDriverBButton, coDriverXButton, coDriverYButton, coDriverBackButton, coDriverLeftStickDown,
                    codriverLeftBumperTAP, cube);
        }

      
     
        if(coDriverRightTrigger>0.5){
            if(!cube){
                intake.intake(1,false,false);
                //intake.setPercentOutput(1.0);
            }
            else if (cube){
            intake.intake(1,true,false);
            }
                else{
            intake.stop();
        }
        }
        if(codriverRightBumper){
            cube = false;
         }else if (codriverLeftBumper){
            cube = true;
         }
         

        if (driverRightTrigger>.5) {
            aim.aimAtScore(cube, driverLeftBumperTAP, driverRightBumperTAP);
        }else {
            swerve.sendInput(driverLeftYInput, driverLeftXInput, driverRightXInput);
        }
      
       // vision.getHeight(driverLeftTrigger, driverightTrigger);

        // lights.setLights(codriverRightBumperTAtransformByLeftBumperTAP);

        
        sim = Pose2d.fromTranslation(sim.getTranslation().translateBy(new Translation2d(driverLeftXInput/5,driverLeftYInput/5)));
        Logger.getInstance().recordOutput("pose",sim.toWPI());
        swerve.resetPose(sim);
    



}
}
