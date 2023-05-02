// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.wcp.frc.subsystems;

import org.littletonrobotics.junction.Logger;

import com.wcp.frc.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Scores extends SubsystemBase {
  private State currentState = State.ZERO;
  public double armOffset = 0;
  public double sideElvator = 0;

  public double elevatorOffset =  0;
  public boolean Cube= true;
  

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

  double height = Constants.zero;
  Constants constants;
  public void setHeight(double height, boolean Cube){
    this.height = height;
    this.Cube = Cube;
    Logger.getInstance().recordOutput("heighst", height);

  }
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
      currentState = State.LOW;
    } else if (coDriverX) {
      currentState = State.HOOMAN;
    } else if (coDriverY) {
      currentState = State.HIGH;
    } else if (coDriverBackButton) {
      currentState = State.CHUTE;
    } else if (coDriverLeftStickDown) {
      currentState = State.ZERO;
    }
    this.Cube = Cube;
  }

  public static void selectPreset(){
    if(curentState == State.HIGH&&!CUBE){
      
    }
    if(curentState == State.MID&&!CUBE){
      
    }
    if(curentState == State.LOW&&!CUBE){
      
    }
    if(curentState == State.HIGH&&CUBE){
      
    }
    if(curentState == State.MID&&CUBE){
      
    }
    if(curentState == State.LOW&&CUBE){
      
    }
    if(currentState == State.CHUTE){

    }
    if(currentState == )
  }
  public static void setElevator(State state){
    currentState = state;
    selectPreset();

  }
  public static void goToPreset(){

  }

 public void hold() {
    sideElevator.elevator(0);
    elevator.elevator(Constants.ElevatorConstants.MAX_DOWN);
    armDesired = (Constants.ArmConstants.HOLD- armOffset);
    // armOffset=0;
  }

  public void hooman() {
    sideElevatorDesired = (Constants.SideElevatorConstants.HOOMAN);// go to set offset based off vision
    elevatorDesired = (Constants.ElevatorConstants.HOOMAN);
      armDesired = (Constants.ArmConstants.HOOMAN-armOffset);
   

  }

  public void scoreLowCone() {
    sideElevatorDesired  = (Constants.SideElevatorConstants.MAX_DOWN);// go to set offset based off vision
    elevatorDesired = (Constants.ElevatorConstants.LOW_CONE);
    armDesired = (Constants.ArmConstants.LOW_SCORE_CONE - armOffset);

  }
 

  public void scoreMidCone() {
    sideElevatorDesired  = (Constants.SideElevatorConstants.MAX_DOWN);// go to set offset based off vision
    elevatorDesired = (Constants.ElevatorConstants.MID_CONE);
    armDesired = (Constants.ArmConstants.MIDDLE_CONE - armOffset);

  }

  // (((vision.range)/12)-3)*(2048/Constants.sideelevatorgearratio)*Constants.sideelevatorrotationsperfoot
  public void scoreHighCone() {
    sideElevatorDesired  = (Constants.SideElevatorConstants.HIGH_CONE);// goto set offset based off vision
    elevatorDesired = (Constants.ElevatorConstants.HIGH_CONE);

    armDesired = (Constants.ArmConstants.HIGH_CONE - armOffset);

  }
  public void scoreLowCube() {
    sideElevatorDesired  = (Constants.SideElevatorConstants.MAX_DOWN);// go to set offset based off vision
    elevatorDesired = (Constants.ElevatorConstants.LOW_CUBE);
    armDesired = (Constants.ArmConstants.LOW_SCORE_CUBE - armOffset);

  }
 

  public void scoreMidCube() {
    sideElevatorDesired  = (Constants.SideElevatorConstants.MAX_DOWN);// go to set offset based off vision
    elevatorDesired = (Constants.ElevatorConstants.MID_CUBE);
    armDesired = (Constants.ArmConstants.MIDDLE_CUBE - armOffset);

  }

  // (((vision.range)/12)-3)*(2048/Constants.sideelevatorgearratio)*Constants.sideelevatorrotationsperfoot
  public void scoreHighCube() {
    sideElevatorDesired  = (Constants.SideElevatorConstants.HIGH_CUBE);// goto set offset based off vision
    elevatorDesired = (Constants.ElevatorConstants.HIGH_CUBE);

    armDesired = (Constants.ArmConstants.HIGH_CUBE - armOffset);

  }

  public void pickup() {

    sideElevatorDesired  = (Constants.SideElevatorConstants.PICKUP);
    elevatorDesired = (Constants.ElevatorConstants.PICKUP - elevatorOffset);
    armDesired = (Constants.ArmConstants.PICKUP);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  public void zero() {

    sideElevatorDesired  = (Constants.SideElevatorConstants.ZERO);
    elevatorDesired = (Constants.ElevatorConstants.ZERO);
    armDesired = (Constants.ArmConstants.ZERO+500);

    // Use addRequirements() here to declare subsystem dependencies.
  }
}
