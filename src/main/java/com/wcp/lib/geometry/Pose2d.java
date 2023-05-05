// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.wcp.lib.geometry;



/** Add your docs here. */
public class Pose2d extends edu.wpi.first.math.geometry.Pose2d {

    protected Translation2d mTranslation;
    protected Rotation2d mRotation;
    public Pose2d() {
        mTranslation = new Translation2d();
        mRotation = new Rotation2d();
    }
    public Pose2d(Translation2d translation, Rotation2d rotation) {
        mTranslation = translation;
        mRotation = rotation;
    }
        public Pose2d(double x, double y, Rotation2d rotation) {
        mTranslation = new Translation2d(x,y);
        mRotation = rotation;
    }
    public static Pose2d fromTranslation(final Translation2d translation) {
        return new Pose2d(translation, new Rotation2d());
    }

    public static Pose2d fromRotaiton(final Rotation2d rotation) {
        return new Pose2d(new Translation2d(), rotation);
    }
            /**
     * The inverse of this transform "undoes" the effect of translating by this transform.
     *
     * @return The opposite of this transform.
     */
    public Pose2d inverse() {
        Rotation2d rotation_inverted = mRotation.inverse();
        return new Pose2d(mTranslation.inverse().rotateBy(rotation_inverted), rotation_inverted);
    }
    @Override
    public Rotation2d getRotation() {
        return this.mRotation;
    }
    
    @Override
    public Translation2d getTranslation() {
        return this.mTranslation;
    }
        /**
     * Transforming this RigidTransform2d means first translating by other.translation and then rotating by
     * other.rotation
     *
     * @param other The other transform.
     * @return This transform * other
     */
    public Pose2d transformBy(final Pose2d other) {
        return new Pose2d(mTranslation.translateBy(other.mTranslation.rotateBy(mRotation)),
                mRotation.rotateBy(other.mRotation));
    }

}
