// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.wcp.lib.geometry;



/** Add your docs here. */
public class Line2d {

    protected Translation2d mPoint1;
    protected Translation2d mPoint2;
    protected Translation2d mTopPoint;
    protected Translation2d mBottomPoint;
    protected Translation2d mLeftPoint;
    protected Translation2d mRightPoint;

    protected double mX1;
    protected double mX2;
    protected double mY1;
    protected double mY2;

    public Line2d() {
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
    public Line2d(Translation2d point1, Translation2d point2) {
        
        mPoint1 = point1;
        mX1 = point1.getX();
        mY1 = point1.getY();

        mPoint2 = point2;
        mX2 = point2.getX();
        mY2 = point2.getY();
        
        if(mPoint2.getY() > mPoint1.getY()){
            mTopPoint = point2;
            mBottomPoint = point1;
        }
        else{
            mTopPoint = point1;
            mBottomPoint = point2;
        }
        if(mPoint1.getX() > mPoint2.getX()){
            mRightPoint = point1;
            mLeftPoint = point2;
        }else{
            mRightPoint = point1;
            mLeftPoint = point2;
        }

    }
    public Translation2d getPoint1(){
        return mPoint1;
    }
    public Translation2d getPoint2(){
        return mPoint2;
    }

    public Translation2d getTop(){
        return mTopPoint;
    }

    public Translation2d getBottom(){
        return mBottomPoint;
    }

    public Translation2d getRight(){
        return mRightPoint;
    }

    public Translation2d getLeft(){
        return mRightPoint;
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
    
    public boolean intersectsWith(Line2d otherLine){
        double a1 = mY2 - mY1;
        double b1 = mX1 - mX2;
        double c1 = (a1*mX1) + (b1*mY1);
        double a2 = otherLine.y2() - otherLine.y1(); 
        double b2 = otherLine.x1() - otherLine.x2();
        double c2 =  (a2*otherLine.x1()) + (b2*otherLine.y1());

        double det = (a1 * b2) - (a2 * b1);
        if(det == 0) {
            return false;
        } else{
            double x = (b2 * c1 - b1 * c2) / det;
            double y = (a1 * c2 - a2 * c1) / det;
            if(x>=mRightPoint.getX() && x <= mLeftPoint.getX() && y >= mBottomPoint.getY() && y <= mTopPoint.getY()){
                return true;
            }else{
                return false;
            }
        }
    }
    public Line2d extend(double extend){
        
        double k = Math.sqrt(Math.pow(mX2-mX1, 2) + Math.pow(mY2-mY1, 2));
        double nTopx = mTopPoint.getX() +(mX2-mX1)/(extend*k);
        double nTopy = mTopPoint.getY() +(mY2-mY1)/(extend*k);
        double nBottomx = mBottomPoint.getX() - (mX2-mX1)/(extend*k);
        double nBottomy = mBottomPoint.getY() +(mY2-mY1)/(extend*k);

        
        return new Line2d(new Translation2d(nTopx, nTopy), new Translation2d(nBottomx, nBottomy));
    }
    public Translation2d getInterSection(Line2d otherLine){
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
    public Translation2d getDistanceIntersectionToEnd(Line2d otherLine){
        Translation2d intersection = getInterSection(otherLine);
        if(getside(otherLine)){
            return new Translation2d(intersection.getX()-mX1,intersection.getY()-mY1);
        }
        return new Translation2d(intersection.getX()-mX2,intersection.getY()-mY1);
    }
    public double getLength(){
        return Math.sqrt(Math.pow((mX2-mX1),2)+(Math.pow(mY2-mY1,2)));
    }
    public boolean getside(Line2d otherLine){
        Translation2d intersection = getInterSection(otherLine);
        double left = Math.sqrt((intersection.getX()-mX1) + Math.pow((intersection.getY()-mY1),2));
        double right = Math.sqrt((intersection.getX()-mX2) + Math.pow((intersection.getY()-mY2),2));
        return left>right;
    }

public double getSlope(){
    if(mX2 == mX1){
        return 1000000000;
    }
    else{
        return (mY2-mY1)/(mX2 - mX1);
    }
   
}
public Line2d push(double hypt){
    double normSlope=-1/(getSlope());
    double theta = Math.atan(normSlope)*180/Math.PI;
    return new Line2d(new Translation2d(mX1+(hypt*Math.cos(theta)),mY1+(hypt*Math.sin(theta))),new Translation2d(mX2+(hypt*Math.cos(theta)),mY2+(hypt*Math.sin(theta))));
}

public double midPointX(){
    return ((mRightPoint.getX() - mLeftPoint.getX())/2) + mLeftPoint.getX();
}
public double midPointY(){
    return ((mTopPoint.getY() - mBottomPoint.getY())/2) + mBottomPoint.getY();
}
    

public boolean intersectUpper(Line2d otherLine){
    if(intersectsWith(otherLine)){
        if(getInterSection(otherLine).getY()>midPointY()){
        return true;
        }
        else{
          return false;
        }
    }
    else{
        return false;
    }
    
}
public boolean intersectRighter(Line2d otherLine){
    if(intersectsWith(otherLine)){
            if(getInterSection(otherLine).getX()>midPointX()){
            return true;
        }
        else{
            return false;
        }
    }
    else{
        return false;
    }
}
}
    

    
