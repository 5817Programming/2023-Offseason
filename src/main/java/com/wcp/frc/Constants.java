// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.wcp.frc;

import java.util.Arrays;
import java.util.List;
import java.util.Map;

import com.wcp.lib.geometry.Pose2d;
import com.wcp.lib.geometry.Rotation2d;
import com.wcp.lib.geometry.Translation2d;
import com.wcp.lib.geometry.HeavilyInspired.Obstacle;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Add your docs here. */
public class Constants {
    public static final int kCANTimeoutMs = 20;//The refresh rate of the periodic looper


    public static final double kRobotBaseWidth = 32.0; //The Robot Wheel Base Width
    public static final double kRobotBaseLength = 32.0; //The Robot Wheel Base Length
    public static final double mRobotBaseWidth = Units.inchesToMeters(29); //The Robot Wheel Base Width
    public static final double mRobotBaseLength = Units.inchesToMeters(29); //The Robot Wheel Base Length

    public static final double mRobotHypot = Math.hypot(mRobotBaseWidth,mRobotBaseLength)*2;

    public static final double kOuterWheelDriveDiameter = 4.0;

    ///-------Swerve Values-------///
    //The positions of the modules, relative to the robot's center
    public static final Translation2d kFrontRightPosition = new Translation2d(kRobotBaseWidth / 2, kRobotBaseLength / 2);
    public static final Translation2d kFrontLeftPosition = new Translation2d(kRobotBaseWidth / 2, -kRobotBaseLength / 2);
    public static final Translation2d kRearLeftPosition = new Translation2d(-kRobotBaseWidth / 2, -kRobotBaseLength / 2);
    public static final Translation2d kRearRightPosition = new Translation2d(-kRobotBaseWidth / 2, kRobotBaseLength / 2);
    public static final Translation2d mFrontRightPosition = new Translation2d(mRobotBaseWidth / 2, mRobotBaseLength / 2);
    public static final Translation2d mFrontLeftPosition = new Translation2d(mRobotBaseWidth / 2, -mRobotBaseLength / 2);
    public static final Translation2d mRearLeftPosition = new Translation2d(-mRobotBaseWidth / 2, -mRobotBaseLength / 2);
    public static final Translation2d mRearRightPosition = new Translation2d(-mRobotBaseWidth / 2, mRobotBaseLength / 2);

    public static final List<Translation2d> kModulePositions = Arrays.asList(kFrontRightPosition, kFrontLeftPosition, kRearLeftPosition, kRearRightPosition); 

    public static  double kSwerveRotationMaxSpeed = 12720.0 * 0.8;//trigger changes this in controls kinda ghetto ngl
    public static final double kSwerveDriveMaxSpeed = 22000.0; //The theoretical max speed(For the Falcon 500s)
    public static final double kSwerveRotation10VoltMaxSpeed = 1350.0;
    public static final double kSwerveRotationReduction = Options.rotationRatio; //The Module to Motor Ratio(i.e, amount the rotation motor rotates, for every one rotation for the module)
    public static final double kSwerveWheelReduction = Options.driveRatio; //The Wheel to Motor Ratio(i.e, amount the drive motor rotates, for every one rotation for the wheel)
    public static final double kSwerveRotationEncoderResolution = 2048.0;
    public static final double kSwerveDriveEncoderResolution = 2048.0; 


    ///The absolute starting postion for each module
    //originally +180 to each
    public static final double kFrontRightStartingEncoderPosition = -132; //-354.950352
    public static final double kFrontLeftStartingEncoderPosition = -355.170825; //-263.094811
    public static final double kRearLeftStartingEncoderPosition =  -354.950352; //-121.094031
    public static final double kRearRightStartingEncoderPosition = -262; //-355.170825    



    public static final double kFrontRightParkEncoderPosition = -8; //-354.950352
    public static final double kFrontLeftParkEncoderPosition = -265; //-263.094811
    public static final double kRearLeftParkEncoderPosition =  -265; //-121.094031
    public static final double kRearRightParkEncoderPosition = -262; //-355.170825    
    
    public static final List<Double> parkPositions = Arrays.asList(kFrontRightParkEncoderPosition,kFrontLeftParkEncoderPosition,kRearLeftParkEncoderPosition,kRearRightParkEncoderPosition);


    public final int hold = 0;
    public static final double kWheelCircumference = Units.inchesToMeters(kOuterWheelDriveDiameter*Math.PI);


    public final double mid_score = 0;


    public final double high_score = 0;

    	//Scrub Factors
        public static final boolean kSimulateReversedCarpet = false;
        public static final double[] kWheelScrubFactors = new double[]{1.0, 1.0, 1.0, 1.0};
        public static final double kXScrubFactor = 1.0 / (1.0 - (9549.0 / 293093.0));
        public static final double kYScrubFactor = 1.0 / (1.0 - (4.4736 / 119.9336));
    public final double low_score = 0;


    public final double YawInRange = 1;

    //photonvison
    public final double CAMERA_HEIGHT_METERS = Units.feetToMeters(1);
    public final double TARGET_HEIGHT_METERS = Units.feetToMeters(1);
    public final double CAMERA_PITCH_RADIANS = 0;
    public final int Cube = 0;
    public final int Cone =  2;
    public final int Limelight= 5;
    public final int April =  3;
        
    public static final int kSlotIdx = 0;

	/**
	 * Talon FX supports multiple (cascaded) PID loops. For
	 * now we just want the primary one.
	 */
	public static final int kPIDLoopIdx = 0;

	/**
	 * set to zero to skip waiting for confirmation, set to nonzero to wait and
	 * report to DS if action fails.
	 */
	public static final int kTimeoutMs = 30;

    public static final List<Double> scoresY =  Arrays.asList(
        0.46,
        1.04,
        1.57,
        2.16,
        2.72,
        3.25,
        3.84,
        4.39,
        4.94
    );


  

	/////----- Ports -----/////
	public static final int kLeftElevatorPort = 13;
	public static final int kRightElevatorPort = 12;
    public static final class ElevatorConstants {

        //IN TIK
        public static final double MAX_UP = 78000;
        public static final double HOOMAN_CONE = 59700;
        public static final double HOOMAN_CUBE = 57000;
        public static final double MAX_DOWN = 800;

        public static final double LOW_CONE = MAX_UP * .1 ;
        public static final double MID_CONE = 27000;
        public static final double HIGH_CONE = 67000;
        public static final double LOW_CUBE = MAX_UP * .1 ;
        public static final double MID_CUBE = 1100;
        public static final double HIGH_CUBE = 65000;


        public static final double PICKUP = MAX_DOWN+5;
        public static final double ZERO = 801;
        public static final double HOLD = MAX_DOWN;//800
       
    }
    public static final double fieldLength = Units.inchesToMeters(651.25);


    
    public static final class SideElevatorConstants{

        // IN TICKS 
        
        public static final double MAX_UP = 44000;

        public static final double MAX_DOWN = 0;

        public static final double LOW_CONE = MAX_UP * .1;
        
        public static final double MID_CONE =  1000;
        
        public static final double HIGH_CONE = 40000;

        public static final double LOW_CUBE = MAX_DOWN;
        
        public static final double MID_CUBE =  0;
        
        public static final double HIGH_CUBE = 44000;

     

        
        public static final double HOLD = 320;
        
        public static final double ZERO = MAX_DOWN;

        public static final double HOOMAN = 10;

        public static final double PICKUP = 0;

        public final static int ROTATION_PER_FOOT = 6;

    }

    public static final class ArmConstants {

        // IN TICKS

        public static final double MAX_UP_TICKS = 0;
        
        public static final double MAX_DOWN_TICKS = -63605;
        
        public static final double MIDDLE_CONE = -26000;

        public static final double HIGH_CONE = -30000;

        public static final double HOOMAN = -29000  ;

        public static final double LOW_SCORE_CONE = MIDDLE_CONE;

        public static final double MIDDLE_CUBE = -19624;

        public static final double HIGH_CUBE = -21018;

        public static final double LOW_SCORE_CUBE = 0;

        public static final double PICKUP = -60605;
        
        public static final double HOLD = -5000;//MAX_UP_TICKS-2000

        public static final double ZERO = MAX_UP_TICKS;
        
    }

    public static final class VisionConstants {
        
        public final static int CUBE_PIPELINE = 3;
        public final static int CONE_PIPELNE = 2;
        public final static int LOW_RETRO_PIPLINE = 3;
        public final static int APRIL_PIPLINE = 0;

        public final static double APRIL_HEIGHT_INCHES = 18.467;
        public final static double RETRO_HEIGHT_INCHES = 26.063;
        public final static double PICKUP_HEIGHT_INCHES = 0;
        public final static double HOOMAN_HEIGHT_INCHES = 27;
        public final static List<Double> HEIGHTS = Arrays.asList(APRIL_HEIGHT_INCHES, RETRO_HEIGHT_INCHES, HOOMAN_HEIGHT_INCHES,
            PICKUP_HEIGHT_INCHES);

        public final static double APRIL_OFFSET = 0;
        public final static double RETRO_OFFSET = .3;
        public final static double PICKUP_OFFSET = 0;
        public final static double HOOMAN_OFFSET = .4;
        public final static List<Double> OFFSETS = Arrays.asList(APRIL_OFFSET, RETRO_OFFSET, HOOMAN_OFFSET,
            PICKUP_OFFSET);

        // how many degrees back is your limelight rotated from perfectly vertical?
        public final static double LIMELIGHT_MOUNT_ANGLE_DEGREES = 1.4;

        // distance from the center of the Limelight lens to the floor
        public final static double LIMELIGHT_LENS_HEIGHT_INCHES = 28.6;
    }

    public static final class LightConstants {
        public final static double NORMAL_LIGHT = -0.99;//rainbow light

        public final static double CUBE_LIGHT = 0.57;//purple light

        public final static double CONE_LIGHT = 0.65;//yellow light
    }
    public static final class FieldConstants {
        public static final double fieldLength = Units.inchesToMeters(651.25);
        public static final double fieldWidth = Units.inchesToMeters(315.5);
        public static final double tapeWidth = Units.inchesToMeters(2.0);
    
        // Dimensions for community and charging station, including the tape.
        public static final class Community {
            // Region dimensions
            public static final double innerX = 0.0;
            public static final double midX = Units.inchesToMeters(132.375); // Tape to the left of charging station
            public static final double outerX = Units.inchesToMeters(193.25); // Tape to the right of charging station
            public static final double leftY = Units.feetToMeters(18.0);
            public static final double midY = leftY - Units.inchesToMeters(59.39) + tapeWidth;
            public static final double rightY = 0.0;
            public static final Translation2d[] regionCorners = new Translation2d[] {
                    new Translation2d(innerX, rightY),
                    new Translation2d(innerX, leftY),
                    new Translation2d(midX, leftY),
                    new Translation2d(midX, midY),
                    new Translation2d(outerX, midY),
                    new Translation2d(outerX, rightY),
            };
    
            // Charging station dimensions
            public static final double chargingStationLength = Units.inchesToMeters(76.125);
            public static final double chargingStationWidth = Units.inchesToMeters(97.25);
            public static final double chargingStationOuterX = outerX - tapeWidth;
            public static final double chargingStationInnerX = chargingStationOuterX - chargingStationLength;
            public static final double chargingStationLeftY = midY - tapeWidth;
            public static final double chargingStationRightY = chargingStationLeftY - chargingStationWidth;
            public static final Translation2d[] chargingStationCorners = new Translation2d[] {
                    new Translation2d(chargingStationInnerX, chargingStationRightY),
                    new Translation2d(chargingStationInnerX, chargingStationLeftY),
                    new Translation2d(chargingStationOuterX, chargingStationRightY),
                    new Translation2d(chargingStationOuterX, chargingStationLeftY)
            };
            public static final Translation2d[] wallCorners = new Translation2d[] {
                new Translation2d(3.5, 5.34),
                new Translation2d(3.5, 5.70),
                new Translation2d(-100, 5.34),
                new Translation2d(-100, 5.70)
        }; 
    
            // Cable bump
            public static final double cableBumpInnerX = innerX + Grids.outerX + Units.inchesToMeters(95.25);
            public static final double cableBumpOuterX = cableBumpInnerX + Units.inchesToMeters(7);
            public static final Translation2d[] cableBumpCorners = new Translation2d[] {
                    new Translation2d(cableBumpInnerX, 0.0),
                    new Translation2d(cableBumpInnerX, chargingStationRightY),
                    new Translation2d(cableBumpOuterX, 0.0),
                    new Translation2d(cableBumpOuterX, chargingStationRightY)
            };
        }
    
        // Dimensions for grids and nodes
        public static final class Grids {
            // X layout
            public static final double outerX = Units.inchesToMeters(54.25);
            public static final double lowX = outerX - (Units.inchesToMeters(14.25) / 2.0); // Centered when under cube
                                                                                            // nodes
            public static final double midX = outerX - Units.inchesToMeters(22.75);
            public static final double highX = outerX - Units.inchesToMeters(39.75);
    
            // Y layout
            public static final int nodeRowCount = 9;
            public static final double nodeFirstY = Units.inchesToMeters(20.19);
            public static final double nodeSeparationY = Units.inchesToMeters(22.0);
    
            // Z layout
            public static final double cubeEdgeHigh = Units.inchesToMeters(3.0);
            public static final double highCubeZ = Units.inchesToMeters(35.5) - cubeEdgeHigh;
            public static final double midCubeZ = Units.inchesToMeters(23.5) - cubeEdgeHigh;
            public static final double highConeZ = Units.inchesToMeters(46.0);
            public static final double midConeZ = Units.inchesToMeters(34.0);
    
            // Translations (all nodes in the same column/row have the same X/Y coordinate)
            public static final Translation2d[] lowTranslations = new Translation2d[nodeRowCount];
            public static final Translation2d[] midTranslations = new Translation2d[nodeRowCount];
            public static final Translation3d[] mid3dTranslations = new Translation3d[nodeRowCount];
            public static final Translation2d[] highTranslations = new Translation2d[nodeRowCount];
            public static final Translation3d[] high3dTranslations = new Translation3d[nodeRowCount];
    
            static {
                for (int i = 0; i < nodeRowCount; i++) {
                    boolean isCube = i == 1 || i == 4 || i == 7;
                    lowTranslations[i] = new Translation2d(lowX, nodeFirstY + nodeSeparationY * i);
                    midTranslations[i] = new Translation2d(midX, nodeFirstY + nodeSeparationY * i);
                    mid3dTranslations[i] = new Translation3d(midX, nodeFirstY + nodeSeparationY * i,
                            isCube ? midCubeZ : midConeZ);
                    high3dTranslations[i] = new Translation3d(
                            highX, nodeFirstY + nodeSeparationY * i, isCube ? highCubeZ : highConeZ);
                    highTranslations[i] = new Translation2d(highX, nodeFirstY + nodeSeparationY * i);
                }
            }
    
            // Complex low layout (shifted to account for cube vs cone rows and wide edge
            // nodes)
            public static final double complexLowXCones = outerX - Units.inchesToMeters(16.0) / 2.0; // Centered X under
                                                                                                     // cone nodes
            public static final double complexLowXCubes = lowX; // Centered X under cube nodes
            public static final double complexLowOuterYOffset = nodeFirstY - Units.inchesToMeters(3.0)
                    - (Units.inchesToMeters(25.75) / 2.0);
    
            public static final Translation2d[] complexLowTranslations = new Translation2d[] {
                    new Translation2d(complexLowXCones, nodeFirstY - complexLowOuterYOffset),
                    new Translation2d(complexLowXCubes, nodeFirstY + nodeSeparationY * 1),
                    new Translation2d(complexLowXCones, nodeFirstY + nodeSeparationY * 2),
                    new Translation2d(complexLowXCones, nodeFirstY + nodeSeparationY * 3),
                    new Translation2d(complexLowXCubes, nodeFirstY + nodeSeparationY * 4),
                    new Translation2d(complexLowXCones, nodeFirstY + nodeSeparationY * 5),
                    new Translation2d(complexLowXCones, nodeFirstY + nodeSeparationY * 6),
                    new Translation2d(complexLowXCubes, nodeFirstY + nodeSeparationY * 7),
                    new Translation2d(
                            complexLowXCones, nodeFirstY + nodeSeparationY * 8 + complexLowOuterYOffset),
            };
        }
    
        // Dimensions for loading zone and substations, including the tape
        public static final class LoadingZone {
            // Region dimensions
            public static final double width = Units.inchesToMeters(99.0);
            public static final double innerX = FieldConstants.fieldLength;
            public static final double midX = fieldLength - Units.inchesToMeters(132.25);
            public static final double outerX = fieldLength - Units.inchesToMeters(264.25);
            public static final double leftY = FieldConstants.fieldWidth;
            public static final double midY = leftY - Units.inchesToMeters(50.5);
            public static final double rightY = leftY - width;
            public static final Translation2d[] regionCorners = new Translation2d[] {
                    new Translation2d(
                            midX, rightY), // Start at lower left next to border with opponent community
                    new Translation2d(midX, midY),
                    new Translation2d(outerX, midY),
                    new Translation2d(outerX, leftY),
                    new Translation2d(innerX, leftY),
                    new Translation2d(innerX, rightY),
            };
    
            // Double substation dimensions
            public static final double doubleSubstationLength = Units.inchesToMeters(14.0);
            public static final double doubleSubstationX = innerX - doubleSubstationLength;
            public static final double doubleSubstationShelfZ = Units.inchesToMeters(37.375);
    
            // Single substation dimensions
            public static final double singleSubstationWidth = Units.inchesToMeters(22.75);
            public static final double singleSubstationLeftX = FieldConstants.fieldLength - doubleSubstationLength
                    - Units.inchesToMeters(88.77);
            public static final double singleSubstationCenterX = singleSubstationLeftX + (singleSubstationWidth / 2.0);
            public static final double singleSubstationRightX = singleSubstationLeftX + singleSubstationWidth;
            public static final Translation2d singleSubstationTranslation = new Translation2d(singleSubstationCenterX,
                    leftY);
    
            public static final double singleSubstationHeight = Units.inchesToMeters(18.0);
            public static final double singleSubstationLowZ = Units.inchesToMeters(27.125);
            public static final double singleSubstationCenterZ = singleSubstationLowZ + (singleSubstationHeight / 2.0);
            public static final double singleSubstationHighZ = singleSubstationLowZ + singleSubstationHeight;
        }
    
        // Locations of staged game pieces
        public static final class StagingLocations {
            public static final double centerOffsetX = Units.inchesToMeters(47.36);
            public static final double positionX = fieldLength / 2.0 - Units.inchesToMeters(47.36);
            public static final double firstY = Units.inchesToMeters(36.19);
            public static final double separationY = Units.inchesToMeters(48.0);
            public static final Translation2d[] translations = new Translation2d[4];
    
            static {
                for (int i = 0; i < translations.length; i++) {
                    translations[i] = new Translation2d(positionX, firstY + (i * separationY));
                }
            }
        }
    
        // AprilTag locations (do not flip for red alliance)
        public static final Map<Integer, Pose3d> aprilTags = Map.of(
                1,
                new Pose3d(
                        Units.inchesToMeters(610.77),
                        Units.inchesToMeters(42.19),
                        Units.inchesToMeters(18.22),
                        new Rotation3d(0.0, 0.0, Math.PI)),
                2,
                new Pose3d(
                        Units.inchesToMeters(610.77),
                        Units.inchesToMeters(108.19),
                        Units.inchesToMeters(18.22),
                        new Rotation3d(0.0, 0.0, Math.PI)),
                3,
                new Pose3d(
                        Units.inchesToMeters(610.77),
                        Units.inchesToMeters(174.19), // FIRST's diagram has a typo (it says 147.19)
                        Units.inchesToMeters(18.22),
                        new Rotation3d(0.0, 0.0, Math.PI)),
                4,
                new Pose3d(
                        Units.inchesToMeters(636.96),
                        Units.inchesToMeters(265.74),
                        Units.inchesToMeters(27.38),
                        new Rotation3d(0.0, 0.0, Math.PI)),
                5,
                new Pose3d(
                        Units.inchesToMeters(14.25),
                        Units.inchesToMeters(265.74),
                        Units.inchesToMeters(27.38),
                        new Rotation3d()),
                6,
                new Pose3d(
                        Units.inchesToMeters(40.45),
                        Units.inchesToMeters(174.19), // FIRST's diagram has a typo (it says 147.19)
                        Units.inchesToMeters(18.22),
                        new Rotation3d()),
                7,
                new Pose3d(
                        Units.inchesToMeters(40.45),
                        Units.inchesToMeters(108.19),
                        Units.inchesToMeters(18.22),
                        new Rotation3d()),
                8,
                new Pose3d(
                        Units.inchesToMeters(40.45),
                        Units.inchesToMeters(42.19),
                        Units.inchesToMeters(18.22),
                        new Rotation3d()));
    
        
        
    
        public static List<Obstacle> obstacles = List.of(
                // Blue Charging Station
                new Obstacle(new double[] {
                        FieldConstants.Community.chargingStationCorners[0].getX(),
                        FieldConstants.Community.chargingStationCorners[1].getX(),
                        FieldConstants.Community.chargingStationCorners[3].getX(),
                        FieldConstants.Community.chargingStationCorners[2].getX(),
                }, new double[] {
                        FieldConstants.Community.chargingStationCorners[0].getY(),
                        FieldConstants.Community.chargingStationCorners[1].getY(),
                        FieldConstants.Community.chargingStationCorners[3].getY(),
                        FieldConstants.Community.chargingStationCorners[2].getY()
                }).offset(0.47)
                ,
                // Red Charging Station
                new Obstacle(new double[] {
                        allianceFlip(FieldConstants.Community.chargingStationCorners[2]).getX(),
                        allianceFlip(FieldConstants.Community.chargingStationCorners[3]).getX(),
                        allianceFlip(FieldConstants.Community.chargingStationCorners[1]).getX(),
                        allianceFlip(FieldConstants.Community.chargingStationCorners[0]).getX(),
                }, new double[] {
                        allianceFlip(FieldConstants.Community.chargingStationCorners[2]).getY(),
                        allianceFlip(FieldConstants.Community.chargingStationCorners[3]).getY(),
                        allianceFlip(FieldConstants.Community.chargingStationCorners[1]).getY(),
                        allianceFlip(FieldConstants.Community.chargingStationCorners[0]).getY()
                }).offset(0.3),
                new Obstacle(new double[] {
                    FieldConstants.Community.wallCorners[0].getX(),
                    FieldConstants.Community.wallCorners[1].getX(),
                    FieldConstants.Community.wallCorners[3].getX(),
                    FieldConstants.Community.wallCorners[2].getX(),
                }, new double[] {
                    FieldConstants.Community.wallCorners[0].getY(),
                    FieldConstants.Community.wallCorners[1].getY(),
                    FieldConstants.Community.wallCorners[3].getY(),
                    FieldConstants.Community.wallCorners[2].getY()
                }).offset(0.6 ),
                new Obstacle(new double[] {
                    allianceFlip(FieldConstants.Community.wallCorners[2]).getX(),
                    allianceFlip(FieldConstants.Community.wallCorners[3]).getX(),
                    allianceFlip(FieldConstants.Community.wallCorners[1]).getX(),
                    allianceFlip(FieldConstants.Community.wallCorners[0]).getX(),
                }, new double[] {
                    allianceFlip(FieldConstants.Community.wallCorners[2]).getY(),
                    allianceFlip(FieldConstants.Community.wallCorners[3]).getY(),
                    allianceFlip(FieldConstants.Community.wallCorners[1]).getY(),
                    allianceFlip(FieldConstants.Community.wallCorners[0]).getY()
                }).offset(0.6)
                );

                
    
        /**
         * Flips a translation to the correct side of the field based on the current
         * alliance color. By
         * default, all translations and poses in {@link FieldConstants} are stored with
         * the origin at the
         * rightmost point on the BLUE ALLIANCE wall.
         */
        public static Translation2d allianceFlip(Translation2d translation) {
            
            if (DriverStation.getAlliance() == Alliance.Red) {
                return new Translation2d(fieldLength - translation.getX(), translation.getY());
            } else {
                return translation;
            }
            
        }
    
        /**
         * Flips a pose to the correct side of the field based on the current alliance
         * color. By default,
         * all translations and poses in {@link FieldConstants} are stored with the
         * origin at the
         * rightmost point on the BLUE ALLIANCE wall.
         */
        public static Pose2d allianceFlip(Pose2d pose) {
            if (DriverStation.getAlliance() == Alliance.Red) {
                return new Pose2d(
                        fieldLength - pose.getX(),
                        pose.getY(),
                        new Rotation2d(-pose.getRotation().getCos(), pose.getRotation().getSin()));
            } else {
                return pose;
            }
        }
      }

    /**
     * Flips a translation to the correct side of the field based on the current
     * alliance color. By
     * default, all translations and poses in {@link FieldConstants} are stored with
     * the origin at the
     * rightmost point on the BLUE ALLIANCE wall.
     */
    public static Translation2d allianceFlip(Translation2d translation) {
        if (DriverStation.getAlliance() == Alliance.Red) {
            return new Translation2d(fieldLength - translation.getX(), translation.getY());
        } else {
            return translation;
        }
    }


	public static final int TIMEOUT_MILLISECONDS = 30;

    public static double zero=19241;
    public static double pickup =83492;

}
