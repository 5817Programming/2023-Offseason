// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.wcp.frc.subsystems;

import java.util.Arrays;
import java.util.List;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.wcp.frc.Constants;
import com.wcp.frc.Ports;
import com.wcp.frc.subsystems.gyros.Gyro;
import com.wcp.frc.subsystems.gyros.Pigeon;
import com.wcp.lib.HeadingController;
import com.wcp.lib.SwerveInverseKinematics;
import com.wcp.lib.geometry.Pose2d;
import com.wcp.lib.geometry.Rotation2d;
import com.wcp.lib.geometry.Translation2d;
import com.wcp.lib.util.Util;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/** Add your docs here. */
public class Swerve extends Subsystem {

    public static Swerve instance = null;
    public static Swerve getInstance() {
        if(instance == null)
            instance = new Swerve();
        return instance;
    }

    Gyro pigeon;

    SwerveDriveModule frontRightModule, frontLeftModule, rearLeftModule, rearRightModule;
    List<SwerveDriveModule> modules;
    

    Translation2d translationVector;
    Translation2d aimingVector  = new Translation2d();
    public double aimScalar = 0;
    public double snapAngle = 0;
    public double targetHeading;
    public double rotationScalar;

    Pose2d pose = new Pose2d();
    private State currentState = State.MANUAL;
    List<Translation2d> moduleVectors;
    final double translationDeadband = 0.1;
    final double rotationDeadband = 0.1;
    private boolean robotCentric = false;

    SwerveInverseKinematics inverseKinematics = new SwerveInverseKinematics();
    HeadingController headingController = new HeadingController();

    public Swerve() {
        frontRightModule = new SwerveDriveModule(Ports.FRONT_RIGHT_ROTATION, Ports.FRONT_RIGHT_DRIVE, 0, Constants.kFrontRightStartingEncoderPosition, Constants.kFrontRightPosition, true);
        frontLeftModule = new SwerveDriveModule(Ports.FRONT_LEFT_ROTATION, Ports.FRONT_LEFT_DRIVE, 1, Constants.kFrontLeftStartingEncoderPosition, Constants.kFrontLeftPosition, true);
        rearLeftModule = new SwerveDriveModule(Ports.REAR_LEFT_ROTATION, Ports.REAR_LEFT_DRIVE, 2, Constants.kRearLeftStartingEncoderPosition, Constants.kRearLeftPosition, true);
        rearRightModule = new SwerveDriveModule(Ports.REAR_RIGHT_ROTATION, Ports.REAR_RIGHT_DRIVE, 3, Constants.kRearRightStartingEncoderPosition, Constants.kRearRightPosition, true);
        modules = Arrays.asList(frontRightModule, frontLeftModule, rearLeftModule, rearRightModule);

        frontRightModule.invertDriveMotor(TalonFXInvertType.Clockwise);
		frontLeftModule.invertDriveMotor(TalonFXInvertType.CounterClockwise);
		rearLeftModule.invertDriveMotor(TalonFXInvertType.CounterClockwise);
		rearRightModule.invertDriveMotor(TalonFXInvertType.Clockwise);

        pigeon = Pigeon.getInstance();
        pigeon.setAngle(0);

    
    }

    public enum State {
        MANUAL, VECTOR, AIM, SNAP;
    }

   public void sendInput(double x, double y, double rotation) {
        currentState = State.MANUAL;
        translationVector = new Translation2d(x,y);
        if(Math.abs(rotation) <= rotationDeadband) {
            rotation = 0;
        }
        if(rotation == 0 && rotationScalar != 0) {
            headingController.disableHeadingController(true);   
        }

        rotationScalar = rotation;
        final double scaleValue = 1.5;
        double inputMagnitude = translationVector.norm();
        inputMagnitude = Math.pow(inputMagnitude, scaleValue);
        inputMagnitude = Util.deadband(translationDeadband, inputMagnitude);
        if(translationVector.norm() <= translationDeadband) {
            translationVector = new Translation2d();
        }  
        rotationScalar *= 0.01;
        if(translationVector.norm() <= translationDeadband&& Math.abs(rotation)<= rotationDeadband){
           // rotationScalar=.5;
          //rotationScalar *= 0.01;
          //this.update(Timer.getFPGATimestamp());
            this.commandModuleDrivePowers(0);
        }
        else{
            this.update(Timer.getFPGATimestamp());

        }

    }
    public Pose2d getPose(){
        return new Pose2d();
    }
   
    public void parkMode(){
         rotationScalar=.5;
          rotationScalar *= 0.01;
          this.update(Timer.getFPGATimestamp());
          this.commandModuleDrivePowers(0);
    }
    public void snap(double angle){
        currentState = State.SNAP;
        targetHeading = angle;
    }
    
    public void setState(State state){
        currentState = state;
    }
   

      
  
    public void commandModules(List<Translation2d> moduleVectors) {
        this.moduleVectors = moduleVectors;
        for(int i = 0; i < moduleVectors.size(); i++) {
            if(Util.shouldReverse(moduleVectors.get(i).direction(), Rotation2d.fromDegrees(modules.get(i).getModuleAngle()))) {
                modules.get(i).setModuleAngle(moduleVectors.get(i).direction().getDegrees() + 180);
                modules.get(i).setDriveOpenLoop(-moduleVectors.get(i).norm());
            } else {
                modules.get(i).setModuleAngle(moduleVectors.get(i).direction().getDegrees());
                modules.get(i).setDriveOpenLoop(moduleVectors.get(i).norm());
            }
        }
    }
    public void Aim(Translation2d aimingVector,double rotation){
        currentState = State.AIM;
        this.aimingVector = aimingVector;
        targetHeading = rotation;
    }



    public void commandModuleDrivePowers(double power) {
        for(int i = 0; i < modules.size(); i++) {
            modules.get(i).setDriveOpenLoop(power);
        }
    }



    public void zeroModules() {
        modules.forEach((m) -> {m.resetModulePositionToAbsolute();});
    }
    public Rotation2d getRobotHeading() {
        return Rotation2d.fromDegrees(pigeon.getAngle());
    }
    
    public void update(double timestamp) {
        pose = Pose2d.fromRotaiton(getRobotHeading());
        if(currentState == State.MANUAL ){
            
            double rotationCorrection =  headingController.updateRotationCorrection(pose.getRotation(), timestamp);
        if(translationVector.norm() == 0 || rotationScalar != 0) {
            rotationCorrection = 0;
        }
        SmartDashboard.putNumber("Swerve Heading Correctiomm    33  33   /n", rotationCorrection);
        commandModules(inverseKinematics.updateDriveVectors(translationVector, rotationScalar + rotationCorrection, pose, robotCentric));
        }
        if(currentState == State.AIM){
            headingController.setTargetHeading(Rotation2d.fromDegrees(targetHeading));
            double rotationCorrection = headingController.getRotationCorrection(getRobotHeading(), timestamp);
            if(translationVector.norm() == 0 || rotationScalar != 0) {
                rotationCorrection = 0;
            }
            SmartDashboard.putNumber("Swerve Heading Correctiomm    33  33   /n", rotationCorrection);
            commandModules(inverseKinematics.updateDriveVectors(aimingVector, rotationCorrection, pose, robotCentric));
        }
        if(currentState == State.SNAP){
            headingController.setTargetHeading(Rotation2d.fromDegrees(targetHeading));
            double rotationCorrection =  headingController.getRotationCorrection(pose.getRotation(), timestamp);
        
        SmartDashboard.putNumber("Swerve Heading Correctiomm    33  33   /n", rotationCorrection);
        commandModules(inverseKinematics.updateDriveVectors(translationVector, rotationCorrection, pose, robotCentric));
        
        }
    }

    public void resetPose(Pose2d newPose){

        
    }

    

    @Override
    public void readPeriodicInputs() {
        modules.forEach((m) -> {m.readPeriodicInputs();});
    }
    @Override
    public void writePeriodicOutputs() {
        modules.forEach((m) -> {m.writePeriodicOutputs();});
    }
    public void zeroSwerve() {
        pose = new Pose2d();
        headingController.setTargetHeading(Rotation2d.fromDegrees(0));
        pigeon.setAngle(0);
    }

    @Override
    public void outputTelemetry() {
        modules.forEach((m) -> {m.outputTelemetry();});
        SmartDashboard.putNumber("Robot Heading", getRobotHeading().getDegrees());
    }

    @Override
    public void stop() {
        translationVector=new Translation2d();
        rotationScalar=0;
        update(Timer.getFPGATimestamp());
        commandModuleDrivePowers(0);
    }

}
