// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.wcp.frc.subsystems.Sensors;

import com.playingwithfusion.TimeOfFlight;

/** Add your docs here. */
public class Lidar {
    TimeOfFlight lidar; 
    int port;
    double offset;
    public Lidar(int port, Double offset){
        this.port = port;
        this.offset = offset;
        lidar = new TimeOfFlight(port);
    }
    public double getRange(){
        return lidar.getRange();
    }
        public double getRangesSigma(){
        return lidar.getRangeSigma();
    }
}
