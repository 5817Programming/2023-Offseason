// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.wcp.frc.subsystems;

import org.littletonrobotics.junction.Logger;
import com.wcp.frc.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Scores extends SubsystemBase {
   State currentState = State.ZERO;
  public double armOffset = 0;
  public double elevatorOffset =  0;
  public boolean CUBE= false;
  public double elevatorPercent;
  public double armPercent;
  public double elevatorDesired=Constants.ElevatorConstants.ZERO;
  public double sideElevatorDesired=Constants.SideElevatorConstants.ZERO;
  public double armDesired=Constants.ArmConstants.ZERO;
  public double elevatorSetPoint=Constants.ElevatorConstants.ZERO;
  public double sideElevatorSetPoint=Constants.SideElevatorConstants.ZERO;
  public double armSetPoint=Constants.ArmConstants.ZERO;
  
  
  

  /** Creates a new Scores. */
  public  Scores() {

  }

  @Override
  public void periodic() {
    
    // This method will be called once per scheduler run
  }

  // Vision vision;
  Elevator elevator = new Elevator();
  SideElevator sideElevator = new SideElevator();
  Arm arm = new Arm();

  public enum State{
    HIGH, MID, LOW, ZERO, CHUTE, HUMAN;
  }

  public void setHeight(State newState){
      currentState = newState;
  }

  public void setHeight(boolean coDriverA, boolean coDriverB, boolean coDriverX, boolean coDriverY, 
  boolean coDriverBackButton, boolean coDriverLeftStickDown, boolean coDriverLeftBumperTAP, boolean Cube) {
  
    if (coDriverA) {
      currentState = State.LOW;
    } else if (coDriverB) {
      currentState = State.MID;
    } else if (coDriverX) {
      currentState = State.HUMAN;
    } else if (coDriverY) {
      currentState = State.HIGH;
    } else if (coDriverBackButton) {
      currentState = State.CHUTE;
    } else if (coDriverLeftStickDown) {
      currentState = State.ZERO;
    }
    this.CUBE = Cube;
  }

  public void selectPreset(){
    if(currentState == State.HIGH&&!CUBE){
      scoreHighCone();
    }
    if(currentState == State.MID&&!CUBE){
      scoreMidCone();
    }
    if(currentState == State.LOW&&!CUBE){
      scoreLowCone();
    }
    if(currentState == State.HIGH&&CUBE){
      scoreHighCube();
    } 
    if(currentState == State.MID&&CUBE){
      scoreMidCube();
    }
    if(currentState == State.LOW&&CUBE){
      scoreLowCube();
    }
    if(currentState == State.CHUTE){
      pickup();
    }
    if(currentState == State.ZERO){
      zero();
    }
    if(currentState == State.HUMAN&&!CUBE){
      hoomanCone();
    }
    if(currentState == State.HUMAN&&CUBE){
      hoomanCube();
    }
  }
  public void setElevator(State state){
    currentState = state;
    setElevator();
  }
  public void setElevator(){
    selectPreset();
    goToPreset();
  }
  public void goToPreset(){

    if ((arm.getEncoder()/armDesired)<1.1){
      armPercent = arm.getEncoder()/armDesired;
    }
    else if ((elevator.getEncoder()/elevatorDesired)>1.1){
      armPercent = armDesired/arm.getEncoder();
    }
    if(elevatorDesired>elevator.getEncoder()){
      elevatorSetPoint = elevatorDesired;
      sideElevatorSetPoint = sideElevatorDesired;
      armSetPoint = (Math.pow(elevatorPercent, 4))*armDesired;
     
    }else{
      elevatorSetPoint = elevatorDesired;
      sideElevatorSetPoint = sideElevatorDesired;
      armSetPoint = armDesired/(Math.pow(elevatorPercent, 4));
    }

    elevator.elevator(elevatorSetPoint);
    sideElevator.elevator(sideElevatorSetPoint);
    arm.setMotionMagic(armSetPoint);
  }

 public void hold() {
    sideElevator.elevator(0);
    elevator.elevator(Constants.ElevatorConstants.MAX_DOWN);
    armDesired = (Constants.ArmConstants.HOLD- armOffset);
    // armOffset=0;
  }

  public void hoomanCube() {
    sideElevatorDesired = (Constants.SideElevatorConstants.HOOMAN);
    elevatorDesired = (Constants.ElevatorConstants.HOOMAN_CUBE);
    armDesired = (Constants.ArmConstants.HOOMAN-armOffset);
  }  
  
  public void hoomanCone() {
    sideElevatorDesired = (Constants.SideElevatorConstants.HOOMAN);
    elevatorDesired = (Constants.ElevatorConstants.HOOMAN_CONE);
    armDesired = (Constants.ArmConstants.HOOMAN-armOffset);
  }


  public void scoreLowCone() {
    sideElevatorDesired  = (Constants.SideElevatorConstants.MAX_DOWN);
    elevatorDesired = (Constants.ElevatorConstants.LOW_CONE);
    armDesired = (Constants.ArmConstants.LOW_SCORE_CONE - armOffset);
  }
 

  public void scoreMidCone() {
    sideElevatorDesired  = (Constants.SideElevatorConstants.MAX_DOWN);
    elevatorDesired = (Constants.ElevatorConstants.MID_CONE);
    armDesired = (Constants.ArmConstants.MIDDLE_CONE - armOffset);
  }

  public void scoreHighCone() {
    sideElevatorDesired  = (Constants.SideElevatorConstants.HIGH_CONE);
    elevatorDesired = (Constants.ElevatorConstants.HIGH_CONE);
    armDesired = (Constants.ArmConstants.HIGH_CONE - armOffset);
  }

  public void scoreLowCube() {
    sideElevatorDesired  = (Constants.SideElevatorConstants.MAX_DOWN);
    elevatorDesired = (Constants.ElevatorConstants.LOW_CUBE);
    armDesired = (Constants.ArmConstants.LOW_SCORE_CUBE - armOffset);
  }
 

  public void scoreMidCube() {
    sideElevatorDesired  = (Constants.SideElevatorConstants.MAX_DOWN);
    elevatorDesired = (Constants.ElevatorConstants.MID_CUBE);
    armDesired = (Constants.ArmConstants.MIDDLE_CUBE - armOffset);
  }

  public void scoreHighCube() {
    sideElevatorDesired  = (Constants.SideElevatorConstants.HIGH_CUBE);
    elevatorDesired = (Constants.ElevatorConstants.HIGH_CUBE);
    armDesired = (Constants.ArmConstants.HIGH_CUBE - armOffset);
  }

  public void pickup() {

    sideElevatorDesired  = (Constants.SideElevatorConstants.PICKUP);
    elevatorDesired = (Constants.ElevatorConstants.PICKUP - elevatorOffset);
    armDesired = (Constants.ArmConstants.PICKUP);

  }

  public void zero() {

    sideElevatorDesired  = (Constants.SideElevatorConstants.ZERO);
    elevatorDesired = (Constants.ElevatorConstants.ZERO);
    armDesired = (Constants.ArmConstants.ZERO+500);

  }
}
