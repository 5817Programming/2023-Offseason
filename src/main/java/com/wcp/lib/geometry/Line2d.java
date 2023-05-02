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

    protected double x1;
    protected double x2;
    protected double y1;
    protected double y2;

    public Pose2d() {
        mPoint1 = new Translation2d();
        mPoint2 = new Translation2d();
        mTopPoint = new Translation2d();
        mBottomPoint = new Translation2d();
        mLeftPoint = new Translation2d();
        mRightPoint = new Translation2d();
        mX1 = 0;
        mX2 = 0;
        mY1 = 0;
        mY2 = 0;
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
        mX1 = point1.x();
        mY1 = point1.x();

        mPoint2 = point2;
        mX2 = point2.y();
        mY2 = point2.y();
        

    }
    public Translation2d getPoint1(){
        return mPoint1;
    }
    public Translation2d getPoint2(){
        return mPoint2;
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
    public double x1(){
        return mX1;
    }
    public double y1(){
        return mY1;
    }
    public double x2(){
        return mX2;
    }
    public double y2(){
        return mY2;
    }
    
    public boolean intersectsWith(Line otherLine){
        double a1 = mY2 - mY1;
        double b1 = mX1 - mX2;
        double c1 = (a1*mX1) + (b1*mY1);
        double a2 = otherLine.y2() - otherLine.y1(); 
        double b2 = otherLine.x1() - otherLine.x2();
        double c2 =  (a2*otherLine.x1()) + (b2*otherLine.y1());

        double det = (a1 * b2) - (a2 * b1)
        if(det == 0) {
            return false;
        } else{
            double x = (b2 * c1 - b1 * c2) / det;
            double y = (a1 * c2 - a2 * c1) / det;
            if(x>=RightPoint.x() && x <= LeftPoint.x() && y >= bottomPoint.y() && y <= TopPoint.y()){
                return true;
            }else{
                return false;
            }
        }
    }
public Translation2d getInterSection(Line otherLine){
        double a1 = mY2 - mY1;
        double b1 = mX1 - mX2;
        double c1 = (a1*mX1) + (b1*mY1);
        double a2 = otherLine.y2() - otherLine.y1(); 
        double b2 = otherLine.x1() - otherLine.x2();
        double c2 =  (a2*otherLine.x1()) + (b2*otherLine.y1());

        double det = (a1 * b2) - (a2 * b1);

            double x = (b2 * c1 - b1 * c2) / det;
            double y = (a1 * c2 - a2 * c1) / det;
            return new Translation2d(x,y);
        }

public double midPointX(){
    return ((RightPoint.x() - LeftPoint.x())/2) + LeftPoint.x();
}
public double midPointY(){
    return ((TopPoint.y() - BottomPoint.y())/2) + BottomPoint.y();
}
    }

    

    
