// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.wcp.frc.subsystems.Sensors;

import java.sql.Time;
import java.util.ArrayList;
import java.util.List;

import com.playingwithfusion.TimeOfFlight;

/** Add your docs here. */
public class LidarGroup {
    List<Lidar> lidars = new ArrayList<>();
    double best = 12904890;
    int bestModule = 0;

    public LidarGroup(int[] ports, double[] offsets){

        for(int i = 0; i < ports.length; i++){
            lidars.add(new Lidar(ports[i],offsets[i]));
        }

  

    }

    public double getDistanceMeter(){
        for(int i = 0; i<lidars.size();i++){
            if(lidars.get(i).getRange()<best){
                bestModule = i;
            }
        }

        return lidars.get(bestModule).getRange();
    }
    
    public double[] getDistanceAll(){
        double[] distances = new double[lidars.size()];
        for(int i = 0; i < lidars.size(); i++){
            distances[i] = lidars.get(i).getRange()/1000;
        }
        return distances;
    }

}

