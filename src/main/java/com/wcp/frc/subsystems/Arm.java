// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.wcp.frc.subsystems;

import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.wcp.frc.Constants;
import com.wcp.frc.Ports;

public class Arm extends Subsystem {
	PeriodicIO mPeriodicIO = new PeriodicIO();

	/* Setpoints */
	double mTargetMin = -32000;// 500
	double mTargetMax = -500;// 78000
	double mtargetmid = -23556;
	double targetPos;
	public double velocity;

	/* Hardware */
	TalonFX arm = new TalonFX(Ports.kArmPort);

	/* Used to build string throughout loop */
	StringBuilder _sb = new StringBuilder();

	/** How much smoothing [0,8] to use during MotionMagic */
	int _smoothing = 0;

	/** save the last Point Of View / D-pad value */
	int _pov = -1;

	public Arm() {
	/* Factory 
		default hardware to prevent unexpected behavior */
		arm.configFactoryDefault();
		// mRightElevator.configFactoryDefault();

		/* Set to Brake Mode */
		arm.setNeutralMode(NeutralMode.Brake);
		// mRightElevator.setNeutralMode(NeutralMode.Brake);

		/* Will follow so no need to config further */
		// mRightElevator.follow(mLeftElevator);
		// mRightElevator.setInverted(TalonFXInvertType.FollowMaster);

		/* Configure Sensor Source for Pirmary PID */
		arm.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.kPIDLoopIdx,
				Constants.TIMEOUT_MILLISECONDS);

		/* set deadband to super small 0.001 (0.1 %).
			The default deadband is 0.04 (4 %) */
		arm.configNeutralDeadband(0.001, Constants.TIMEOUT_MILLISECONDS);

		/**
		 * Configure Talon FX Output and Sensor direction accordingly Invert Motor to
		 * have green LEDs when driving Talon Forward / Requesting Postiive Output Phase
		 * sensor to have positive increment when driving Talon Forward (Green LED)
		 */
		arm.setSensorPhase(false);
		arm.setInverted(false);

		/*
		 * Talon FX does not need sensor phase set for its integrated sensor
		 * This is because it will always be correct if the selected feedback device is integrated sensor (default value)
		 * and the user calls getSelectedSensor* to get the sensor's position/velocity.
		 * 
		 * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#sensor-phase
		 */
        // mLeftElevator.setSensorPhase(true);

		/* Set relevant frame periods to be at least as fast as periodic rate */
		arm.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.TIMEOUT_MILLISECONDS);
		arm.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.TIMEOUT_MILLISECONDS);

		/* Set the peak and nominal outputs */
		arm.configNominalOutputForward(0, Constants.TIMEOUT_MILLISECONDS);
		arm.configNominalOutputReverse(0, Constants.TIMEOUT_MILLISECONDS);
		arm.configPeakOutputForward(1, Constants.TIMEOUT_MILLISECONDS);
		arm.configPeakOutputReverse(-1, Constants.TIMEOUT_MILLISECONDS);
		
		/* Set Motion Magic gains in slot0 - see documentation */
		arm.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
		arm.config_kF(Constants.kSlotIdx, 0.07856539436, Constants.TIMEOUT_MILLISECONDS);//0.0649
		arm.config_kP(Constants.kSlotIdx, 0.03100469768, Constants.TIMEOUT_MILLISECONDS);//0.7161
		arm.config_kI(Constants.kSlotIdx, 0.00, Constants.TIMEOUT_MILLISECONDS);//0.001
		arm.config_kD(Constants.kSlotIdx, .3100469768, Constants.TIMEOUT_MILLISECONDS);//P value * ten

		/*mLeftElevator.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
		mLeftElevator.config_kF(Constants.kSlotIdx, 0.05757217626, Constants.kTimeoutMs);//0.0649
		mLeftElevator.config_kP(Constants.kSlotIdx, 0, Constants.kTimeoutMs);//0.7161
		mLeftElevator.config_kI(Constants.kSlotIdx, 0, Constants.kTimeoutMs);//0.001
		mLeftElevator.config_kD(Constants.kSlotIdx, 0, Constants.kTimeoutMs);//P value * ten
*/
		/* Set acceleration and vcruise velocity - see documentation */
		arm.configMotionCruiseVelocity(10417, Constants.TIMEOUT_MILLISECONDS);
		arm.configMotionAcceleration(10417, Constants.TIMEOUT_MILLISECONDS);

		/* Zero the sensor once on robot boot up */
		arm.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.TIMEOUT_MILLISECONDS);
	}



	public void setMotionMagic(double targetPos) {
		mPeriodicIO.driveControlMode = ControlMode.MotionMagic;
		mPeriodicIO.driveDemand = targetPos;

	}

	public void setPercentOutput(double speed) {
		mPeriodicIO.driveControlMode = ControlMode.PercentOutput;
		mPeriodicIO.driveDemand = speed;
	}

	public void configVelocity(double velocity) {
		arm.configMotionCruiseVelocity(velocity, Constants.TIMEOUT_MILLISECONDS);

	}
	public double getEncoder(){
		Logger.getInstance().recordOutput("side elevator", mPeriodicIO.drivePosition);
		return mPeriodicIO.drivePosition;
	  }
	@Override
    public void writePeriodicOutputs() {
        mPeriodicIO.drivePosition = arm.getSelectedSensorPosition();
        mPeriodicIO.velocity = arm.getSelectedSensorVelocity();
    }
    @Override
    public void readPeriodicInputs() {
        arm.set(mPeriodicIO.driveControlMode, mPeriodicIO.driveDemand);
    }
	public static class PeriodicIO  {
        double drivePosition = 0;
        double velocity = 0;

        ControlMode driveControlMode = ControlMode.MotionMagic;
        double rotationDemand = 0.0;
        double driveDemand = 0.0;
    }
	@Override
	public void outputTelemetry() {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void stop() {
		// TODO Auto-generated method stub
		
	}

}
