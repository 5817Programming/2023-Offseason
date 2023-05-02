// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.wcp.lib.geometry;

import static com.wcp.lib.util.Util.kEpsilon;
/** Add your docs here. */
public class Rotation2d extends edu.wpi.first.math.geometry.Rotation2d{
    
    protected double mCos;
    protected double mSin;

    public Rotation2d() {
        this(1,0, false);
    }
    
    public Rotation2d(double x, double y, boolean normalize) {
        if(normalize) {
            double magnitude = Math.hypot(x, y);
            if (magnitude > kEpsilon) {
                mCos = x / magnitude;
                mSin = y / magnitude;
            } else {
                mCos = 1;
                mSin = 0;
            }
        } else {
            mCos = x;
            mSin = y;
        }
    }
    
    public Rotation2d(final Rotation2d otherRotation) {
        mCos = otherRotation.mCos;
        mSin = otherRotation.mSin;
    }
    
    public Rotation2d(double thetaDegrees) {
        mCos = Math.cos(Math.toRadians(thetaDegrees));
        mSin = Math.sin(Math.toRadians(thetaDegrees));
    }
    
    public static Rotation2d fromDegrees(double angle) {
        return new Rotation2d(angle);
    }
    
    public double cos() {
        return mCos;
    }
    public double sin() {
        return mSin;
    }
    @Override
    public double getRadians() {
        return Math.atan2(mSin, mCos);
    }
    @Override
    public double getDegrees() {
        return Math.toDegrees(getRadians());
    }
    
    public Rotation2d rotateBy(final Rotation2d other) {
        return new Rotation2d(mCos * other.mCos - mSin * other.mSin, mCos * other.mSin + mSin * other.mCos, true);
    }
    
    public double distance(Rotation2d other) {
        return this.rotateBy(other.inverse()).getRadians();
    }
    
    public Rotation2d inverse() {
        return new Rotation2d(mCos, -mSin, false);
    }
    
    public Translation2d toVector2d() {
        return new Translation2d(mCos, mSin);
    }
}
