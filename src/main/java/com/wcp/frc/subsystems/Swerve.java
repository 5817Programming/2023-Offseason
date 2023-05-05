// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.wcp.frc.subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.littletonrobotics.junction.Logger;

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

import edu.wpi.first.math.util.Units;
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
    Pose2d drivingpose = new Pose2d();

    public double aimScalar = 0;
    public double snapAngle = 0;
    public double targetHeading;
    public double rotationScalar;
    double distanceTraveled;
	double currentVelocity = 0;
	double lastUpdateTimestamp = 0;

    Pose2d pose = new Pose2d();
    private State currentState = State.MANUAL;
    List<Translation2d> moduleVectors;
    final double translationDeadband = 0.1;
    final double rotationDeadband = 0.1;
    private boolean robotCentric = false;
    List<SwerveDriveModule> positionModules;

    SwerveInverseKinematics inverseKinematics = new SwerveInverseKinematics();
    HeadingController headingController = new HeadingController();

    public Swerve() {
        frontRightModule = new SwerveDriveModule(Ports.FRONT_RIGHT_ROTATION, Ports.FRONT_RIGHT_DRIVE, 0,
        Constants.kFrontRightStartingEncoderPosition, Constants.kFrontRightPosition, true,Constants.mFrontRightPosition);
frontLeftModule = new SwerveDriveModule(Ports.FRONT_LEFT_ROTATION, Ports.FRONT_LEFT_DRIVE, 1,
        Constants.kFrontLeftStartingEncoderPosition, Constants.kFrontLeftPosition, true,Constants.mFrontLeftPosition);
rearLeftModule = new SwerveDriveModule(Ports.REAR_LEFT_ROTATION, Ports.REAR_LEFT_DRIVE, 2,
        Constants.kRearLeftStartingEncoderPosition, Constants.kRearLeftPosition, true,Constants.mRearLeftPosition);
rearRightModule = new SwerveDriveModule(Ports.REAR_RIGHT_ROTATION, Ports.REAR_RIGHT_DRIVE, 3,
        Constants.kRearRightStartingEncoderPosition, Constants.kRearRightPosition, true,Constants.mRearRightPosition);
modules = Arrays.asList(frontRightModule, frontLeftModule, rearLeftModule, rearRightModule);
        frontRightModule.invertDriveMotor(TalonFXInvertType.Clockwise);
		frontLeftModule.invertDriveMotor(TalonFXInvertType.CounterClockwise);
		rearLeftModule.invertDriveMotor(TalonFXInvertType.CounterClockwise);
		rearRightModule.invertDriveMotor(TalonFXInvertType.Clockwise);
        positionModules = Arrays.asList(frontRightModule, frontLeftModule, rearLeftModule, rearRightModule);

        pigeon = Pigeon.getInstance();
        pigeon.setAngle(0);

        modules.forEach((m) -> m.resetPose(new Pose2d(new Translation2d(5,5),new Rotation2d())));

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
        return pose;
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
    public void updateOdometry(double timestamp) {// uses sent input to commad modules and correct for rotatinol drift

        lastUpdateTimestamp = timestamp;
    
        }



    public void zeroModules() {
        modules.forEach((m) -> {m.resetModulePositionToAbsolute();});
    }
    public Rotation2d getRobotHeading() {
        return Rotation2d.fromDegrees(pigeon.getAngle());
    }
    public void fieldzeroSwerve() {// starts the zero 180 off
        headingController.setTargetHeading(Rotation2d.fromDegrees(-180));
        pigeon.setAngle(-180);
    }
    
    public void update(double timestamp) {
        drivingpose = Pose2d.fromRotaiton(getRobotHeading());
        // if(currentState == State.MANUAL ){
            
            double rotationCorrection =  headingController.updateRotationCorrection(drivingpose.getRotation(), timestamp);
        if(translationVector.norm() == 0 || rotationScalar != 0) {
            rotationCorrection = 0;
        }
        SmartDashboard.putNumber("Swerve Heading Correctiomm    33  33   /n", rotationCorrection);
        commandModules(inverseKinematics.updateDriveVectors(translationVector, rotationScalar + rotationCorrection, drivingpose, robotCentric));
        // }
        // if(currentState == State.AIM){
        //     headingController.setTargetHeading(Rotation2d.fromDegrees(targetHeading));
        //     double rotationCorrection = headingController.getRotationCorrection(getRobotHeading(), timestamp);
        //     if(translationVector.norm() == 0 || rotationScalar != 0) {
        //         rotationCorrection = 0;
        //     }
        //     SmartDashboard.putNumber("Swerve Heading Correctiomm    33  33   /n", rotationCorrection);
        //     commandModules(inverseKinematics.updateDriveVectors(aimingVector, rotationCorrection, drivingpose, robotCentric));
        // }
        // if(currentState == State.SNAP){
        //     headingController.setTargetHeading(Rotation2d.fromDegrees(targetHeading));
        //     double rotationCorrection =  headingController.getRotationCorrection(drivingpose.getRotation(), timestamp);
        
        // SmartDashboard.putNumber("Swerve Heading Correctiomm    33  33   /n", rotationCorrection);
        // commandModules(inverseKinematics.updateDriveVectors(translationVector, rotationCorrection, drivingpose, robotCentric));
        
        // }
    }
    /** The tried and true algorithm for keeping track of position */
	public synchronized void updatePose(double timestamp){
		double x = 0.0;
		double y = 0.0;
		Rotation2d heading = getRobotHeading();
		
		double averageDistance = 0.0;
		double[] distances = new double[4];
		for(SwerveDriveModule m : positionModules){
			m.updatePose(heading);
			double distance = m.getEstimatedRobotPose().getTranslation().translateBy(pose.getTranslation().inverse()).norm();
			distances[m.moduleID] = distance;
			averageDistance += distance;
		}
		averageDistance /= positionModules.size();
		
		int minDevianceIndex = 0;
		double minDeviance = Units.inchesToMeters(100);
		List<SwerveDriveModule> modulesToUse = new ArrayList<>();
		for(SwerveDriveModule m : positionModules){
				double deviance = Math.abs(distances[m.moduleID] - averageDistance);
				if(deviance < minDeviance){
					minDeviance = deviance;
					minDevianceIndex = m.moduleID;
				}
				if(deviance <= 0.05){
					modulesToUse.add(m);
				}
			}
		
		if(modulesToUse.isEmpty()){
			modulesToUse.add(modules.get(minDevianceIndex));
		}
		


		//SmartDashboard.putNumber("Modules Used", modulesToUse.size());
		
		for(SwerveDriveModule m : modulesToUse){
			x += m.getEstimatedRobotPose().getTranslation().x();
			y += m.getEstimatedRobotPose().getTranslation().y();
		}
		Pose2d updatedPose = new Pose2d(new Translation2d(x / modulesToUse.size(), y / modulesToUse.size()), heading);
		double deltaPos = updatedPose.getTranslation().translateBy(pose.getTranslation().inverse()).norm();
        Logger.getInstance().recordOutput("delta pose", deltaPos);
		distanceTraveled += deltaPos;
		currentVelocity = deltaPos / (timestamp - lastUpdateTimestamp);
		pose = updatedPose;
		modules.forEach((m) -> m.resetPose(pose));
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
        edu.wpi.first.math.geometry.Pose2d pose2d = new edu.wpi.first.math.geometry.Pose2d(pose.getTranslation().x(),pose.getTranslation().y(),Rotation2d.fromDegrees(pigeon.getAngle()));
        modules.forEach((m) -> {m.outputTelemetry();});
        SmartDashboard.putNumber("Robot Heading", getRobotHeading().getDegrees());
        Logger.getInstance().recordOutput("Odometry",pose2d);

    }

    @Override
    public void stop() {
        translationVector=new Translation2d();
        rotationScalar=0;
        update(Timer.getFPGATimestamp());
        commandModuleDrivePowers(0);
    }

}
