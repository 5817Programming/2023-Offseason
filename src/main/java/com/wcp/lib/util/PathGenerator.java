package com.wcp.lib.util;


import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.wcp.frc.Constants.FieldConstants;
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
        List<Obstacle> totalObstacles = new ArrayList<>();
        double x = Swerve.getInstance().getPose().getTranslation().getX()+1;
        Logger.getInstance().recordOutput("xob", x);
  
for(int i = 0; i < obstacles.size(); i ++){
                totalObstacles.add(obstacles.get(i));
            }

    aStar.addNode(endTarget);

        if(!ran){
        for(double i = 6; i < 45; i++){
            for( double j = 0; j < 27; j++){
                aStar.addNode(new Node(i/3,j/3));
            }
        }
        }

    for(int i = 0; i<aStar.getNodeSize();i++){
      Node startNode = aStar.getNode(i);
      //System.out.println(""+startNode.getX()+","+startNode.getY());
      for(int j = i; j<aStar.getNodeSize(); j++){
        aStar.addEdge(new Edge(startNode, aStar.getNode(j)), totalObstacles);
      }
    }
    

        if(aStar.addEdge(new Edge(start, endTarget), totalObstacles)){
            fullPath.add(0,start);
            fullPath.add(1,endTarget);

        }else{
            for(int i = 0; i<aStar.getNodeSize();i++){
                Node end = aStar.getNode(i);
                aStar.addEdge(new Edge(start,end), totalObstacles);
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
                fullPathPoints.add(i,new PathPoint(translation, heading, endTarget.getHolRot()).withControlLengths(.3,.3));
            }
            
        }
        ran = true;
        for(int i = 0; i < fullPath.size(); i++){
            //Logger.getInstance().recordOutput("point"+i,Pose2d.fromTranslation(fullPath.get(i).getTranslation()).toWPI());
        }
        return PathPlanner.generatePath(constraints,fullPathPoints);
        }else{
            return new PathPlannerTrajectory();
        }
        
    }
    
}
