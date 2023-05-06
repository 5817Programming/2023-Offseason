package com.wcp.lib.util;


import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.wcp.frc.Constants;
import com.wcp.frc.subsystems.Swerve;
import com.wcp.lib.geometry.Pose2d;
import com.wcp.lib.geometry.Rotation2d;
import com.wcp.lib.geometry.Translation2d;
import com.wcp.lib.geometry.HeavilyInspired.Edge;
import com.wcp.lib.geometry.HeavilyInspired.Node;
import com.wcp.lib.geometry.HeavilyInspired.Obstacle;
import com.wcp.lib.geometry.HeavilyInspired.VisGraph;

import edu.wpi.first.wpilibj2.command.*;

import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

/** Custom PathPlanner version of SwerveControllerCommand */
public class PathGenerator {
        List<Node> fullPath = new ArrayList<Node>();
    static PathPlannerTrajectory generatePath(PathConstraints constraints,Node endTarget, List<Obstacle> obstacles){
        
        Node start = new Node(Swerve.getInstance());
        VisGraph aStar = VisGraph.getInstance();
        List<Node> fullPath = new ArrayList<Node>();
        List<PathPoint> fullPathPoints = new ArrayList<PathPoint>();

        if(aStar.addEdge(new Edge(start, endTarget), obstacles)){
            fullPath.add(0,start);
            fullPath.add(1,endTarget);

        }else{
            for(int i = 0; i<aStar.getNodeSize();i++){
                Node end = aStar.getNode(i);
                aStar.addEdge(new Edge(start,end), obstacles);
            }
            fullPath = aStar.findPath(start, endTarget);
        }
        Rotation2d Heading = new Rotation2d(fullPath.get(1).getX()-start.getX(),fullPath.get(1).getY()-start.getY());

        for(int i = 0; i < fullPath.size(); i++){
            if(i == 0){
                fullPathPoints.add(new PathPoint(start.getTranslation(), Heading ,start.getHolRot()));
            }
            else if( i + 1 == fullPath.size()){
                Translation2d translation = new Translation2d(endTarget.getTranslation());
                Rotation2d heading = new Rotation2d(fullPath.get(i).getX() - fullPath.get(i - 1).getX(), endTarget.getY() - fullPath.get(i - 1).getY());
                fullPathPoints.add(i,new PathPoint(translation, heading, endTarget.getHolRot()));
            }
            else{
                Translation2d translation = new Translation2d(fullPath.get(i).getTranslation().getX(),fullPath.get(i).getTranslation().getY());
                Rotation2d heading = new Rotation2d(fullPath.get(i).getX() - fullPath.get(i - 1).getX(), fullPath.get(i).getY() - fullPath.get(i - 1).getY());
                fullPathPoints.add(i,new PathPoint(translation, heading, endTarget.getHolRot()));
            }
            
        }
        return PathPlanner.generatePath(constraints,fullPathPoints);
    }
   
    
}
