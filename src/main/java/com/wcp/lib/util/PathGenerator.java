package com.wcp.lib.util;


import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.wcp.frc.subsystems.Swerve;
import com.wcp.lib.geometry.Pose2d;
import com.wcp.lib.geometry.Rotation2d;
import com.wcp.lib.geometry.Translation2d;
import com.wcp.lib.geometry.HeavilyInspired.Edge;
import com.wcp.lib.geometry.HeavilyInspired.Node;
import com.wcp.lib.geometry.HeavilyInspired.Obstacle;
import com.wcp.lib.geometry.HeavilyInspired.VisGraph;
import org.littletonrobotics.junction.Logger;


import java.util.ArrayList;
import java.util.List;

/** Custom PathPlanner version of SwerveControllerCommand */
public class PathGenerator {
    public static boolean  ran = false;
    public static PathPlannerTrajectory generatePath(PathConstraints constraints,Node endTarget, List<Obstacle> obstacles){
        List<Node> fullPath = new ArrayList<Node>();

        Node start = new Node(Swerve.getInstance());
        VisGraph aStar = VisGraph.getInstance();
        List<PathPoint> fullPathPoints = new ArrayList<PathPoint>();


    aStar.addNode(endTarget);

        if(!ran){
        aStar.addNode(new Node(2.92-.6,1.51-.6));
        aStar.addNode(new Node(2.92-0.6,3.98+0.6));
        aStar.addNode(new Node(4.86+0.6,3.98+0.6));
        aStar.addNode(new Node(4.86+0.6,1.51-.6));

        aStar.addNode(new Node(11.68-0.6,1.51-0.6));
        aStar.addNode(new Node(11.68-0.6,4.4));
        aStar.addNode(new Node(14.2,4.4));
        aStar.addNode(new Node(14.2,1.51-0.6));

        aStar.addNode(new Node(1.22-0.5, 5.34-0.5));
        aStar.addNode(new Node(1.22-0.5, 5.7+0.5));
        aStar.addNode(new Node(3.26+0.8, 5.34+0.5));
        aStar.addNode(new Node(3.26+0.8, 5.7-0.5));

        aStar.addNode(new Node(13, 5.34-0.6));
        aStar.addNode(new Node(13, 5.70+0.6));
        aStar.addNode(new Node(17, 5.34+0.6));
        aStar.addNode(new Node(17, 5.7-0.6));}

    for(int i = 0; i<aStar.getNodeSize();i++){
      Node startNode = aStar.getNode(i);
      //System.out.println(""+startNode.getX()+","+startNode.getY());
      for(int j = 0; j<aStar.getNodeSize(); j++){
        aStar.addEdge(new Edge(startNode, aStar.getNode(j)), obstacles);
      }
    }

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

        if(fullPath != null){
            edu.wpi.first.math.geometry.Rotation2d Heading = new Rotation2d(fullPath.get(1).getX()-start.getX(),fullPath.get(1).getY()-start.getY()).toWPI();

        for(int i = 0; i < fullPath.size(); i++){
            if(i == 0){
                fullPathPoints.add(new PathPoint(start.getTranslation().toWPI(), Heading ,start.getHolRot()));
            }
            else if(i + 1 == fullPath.size()){
                edu.wpi.first.math.geometry.Translation2d translation = new Translation2d(endTarget.getTranslation()).toWPI();
                edu.wpi.first.math.geometry.Rotation2d heading = new Rotation2d(fullPath.get(i).getX() - fullPath.get(i - 1).getX(), endTarget.getY() - fullPath.get(i - 1).getY()).toWPI();
                fullPathPoints.add(i,new PathPoint(translation, heading, endTarget.getHolRot()));
            }
            else{
                edu.wpi.first.math.geometry.Translation2d translation = new Translation2d(fullPath.get(i).getTranslation().getX(),fullPath.get(i).getTranslation().getY()).toWPI();
                edu.wpi.first.math.geometry.Rotation2d heading = fullPath.get(i+1).getTranslation().translateBy(fullPath.get(i).getTranslation().inverse()).getAngle().toWPI();
                fullPathPoints.add(i,new PathPoint(translation, heading, endTarget.getHolRot()).withControlLengths(.8,.8));
            }
            
        }
        ran = true;
        for(int i = 0; i < fullPath.size(); i++){
            Logger.getInstance().recordOutput("point"+i,Pose2d.fromTranslation(fullPath.get(i).getTranslation()).toWPI());
        }
        return PathPlanner.generatePath(constraints,fullPathPoints);
        }else{
            return new PathPlannerTrajectory();
        }
        
    }
    
}
