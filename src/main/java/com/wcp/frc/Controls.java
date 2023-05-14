// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.wcp.frc;

import com.wcp.frc.subsystems.Arm;
import com.wcp.frc.subsystems.Elevator;
import com.wcp.frc.subsystems.Intake;
import com.wcp.frc.subsystems.SideElevator;
import com.wcp.frc.subsystems.SuperStructure;
import com.wcp.frc.subsystems.Swerve;
import com.wcp.frc.subsystems.Vision;
import com.wcp.frc.subsystems.Swerve.State;
import com.wcp.lib.geometry.Pose2d;
import com.wcp.lib.geometry.Translation2d;

import org.littletonrobotics.junction.Logger;

// import com.wcp.frc.subsystems.toHuman;
import com.wcp.frc.subsystems.Aim;
import com.wcp.frc.subsystems.Lights;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;


public class Controls {
    SuperStructure s = SuperStructure.getInstance();

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

        if(driverRightTrigger>.5){
            
        }

}
}
