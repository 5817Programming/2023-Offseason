// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.wcp.lib.geometry;


/** Add your docs here. */
public class Translation2d extends edu.wpi.first.math.geometry.Translation2d{

    protected double mX;
    protected double mY;
    public Translation2d() {
        this(0,0);
    }
    public Translation2d(double x, double y) {
        mX = x;
        mY = y;
    }
    public Translation2d(final Translation2d otherVec) {
        mX = otherVec.mX;
        mY = otherVec.mY;
    }

    public static Translation2d fromPolar(Rotation2d direction, double magnitude) {
        return new Translation2d(direction.mCos * magnitude, direction.mSin * magnitude);
    }

    public double norm() {
        return Math.hypot(mX, mY);
    }
    
    public double x() {
        return mX;
    }
    public double y() {
        return mY;
    }

    public Translation2d translateBy(final Translation2d other) {
        return new Translation2d(mX + other.mX, mY + other.mY);
    }
    
    public Translation2d rotateBy(final Rotation2d rotation) {
        return new Translation2d(mX * rotation.cos() - mY * rotation.sin(), mX * rotation.sin() + mY * rotation.cos());
    }

    public Rotation2d direction() {
        return new Rotation2d(mX, mY, true);
    }

    public Translation2d inverse() {
        return new Translation2d(-mX, -mY);
    }
    
    public Translation2d scale(double scaleFactor) {
        return new Translation2d(mX * scaleFactor, mY * scaleFactor);
    }

    public double distance(final Translation2d other) {
        return this.translateBy(other.inverse()).norm();
    }
}
