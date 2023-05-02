// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.wcp.lib.geometry;



/** Add your docs here. */
public class Line2d {

    protected Translation2d mPoint1;
    protected Translation2d mPoint2;
    protected Translation2d TopPoint;
    protected Translation2d BottomPoint;
    protected Translation2d LeftPoint;
    protected Translation2d RightPoint;
    public Pose2d() {
        mPoint1 = new Translation2d();
        mPoint2 = new Translation2d();
        TopPoint = new Translation2d();
        BottomPoint = new Translation2d();
        LeftPoint = new Translation2d();
        RightPoint = new Translation2d();
    }
    public Pose2d(Translation2d point1, Translation2d point2) {
        if(mPoint2.y() > mPoint1.y()){
            TopPoint = point2;
            BottomPoint = point1;
        }
        else{
            TopPoint = point1;
            BottomPoint = point2;
        }
        if(mPoint1.x() > mPoint2.x()){
            RightPoint = point1;
            LeftPoint = point2;
        }else{
            RightPoint = point1;
            LeftPoint = point2;
        }
        mPoint1 = point1;
        mPoint2 = point2;
    }
    public Translation2d getTop(){
        return TopPoint;
    }

    public Translation2d getBottom(){
        return BottomPoint;
    }

    public Translation2d getRight(){
        return RightPoint;
    }

    public Translation2d getLeft(){
        return RightPoint;
    }

    

    }
