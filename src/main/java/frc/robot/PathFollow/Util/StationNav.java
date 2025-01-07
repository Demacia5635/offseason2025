// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.PathFollow.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import static frc.robot.PathFollow.Util.PathsConstants.STATIONS;

import java.nio.file.Paths;
import java.util.ArrayList;

import static frc.robot.PathFollow.Util.PathFollow.*;

/** Add your docs here. */
public class StationNav {
    private static Translation2d getEntryPoint(Translation2d curPose){
        double minDistance = Integer.MAX_VALUE;
        int minIndex = -1;
        for (int i = 0; i < PathsConstants.STATIONS.length; i++) {
            Pose2d curStation = PathsConstants.STATIONS[i];
            if(curPose.getDistance(curStation.getTranslation()) < minDistance){
                minDistance = curPose.getDistance(curStation.getTranslation());
                minIndex = i;
            }
        }
        return PathsConstants.STATIONS[minIndex].getTranslation();
    }
    private static int getIndex(Translation2d pos){
        
        for(int i = 0; i < PathsConstants.STATIONS.length; i++){
            if(pos == PathsConstants.STATIONS[i].getTranslation()) return i;
        }
        return -1;
    }

    private static void reverseArray(Translation2d[] array){
        for (int i = 0; i < array.length / 2; i++) {
            Translation2d t = array[i];
            array[i] = array[array.length - 1 - i];
            array[array.length - 1 - i] = t;
        }


    }
    private static Translation2d[] calcPositionPoints(Pose2d curPose, Translation2d finalPoint){
        Translation2d entryPoint = getEntryPoint(curPose.getTranslation());
        int entryPointIndex = getIndex(entryPoint);
        int finalPointIndex = getIndex(finalPoint);
        
        ArrayList<Translation2d> points = new ArrayList<>();
        Translation2d[] pointsArray;

        if(entryPointIndex < finalPointIndex){
            for(int i = entryPointIndex; i < finalPointIndex; i++){
                points.add(PathsConstants.STATIONS[i].getTranslation());
            }
            pointsArray = (Translation2d[]) points.toArray();
        }
        else{
            for(int i = finalPointIndex; i < entryPointIndex; i++){
                points.add(PathsConstants.STATIONS[i].getTranslation());
            }
            pointsArray = (Translation2d[]) points.toArray();
            reverseArray(pointsArray);
        }

        return pointsArray;
        
    }

    



    public static PathFollow genLineByDis(Translation2d initial,Pose2d fin,double velocity){
        
        int closeInit = 0;
        int closeFin = 0;
        
        for(int i = 1; i < STATIONS.length; i++)
        {
            Pose2d station = STATIONS[i];

            double d_init = station.getTranslation().minus(initial).getNorm();
            double d_initclose = station.getTranslation().minus(
                STATIONS[closeInit].getTranslation()).getNorm();

            double d_fin = station.getTranslation().minus(fin.getTranslation()).getNorm();
            double d_finclose = station.getTranslation().minus(
                STATIONS[closeFin].getTranslation()).getNorm();
            
            if (d_init < d_initclose)
                closeInit = i;
            if(d_fin < d_finclose)
                closeFin = i;
        }

        int cClock = 0;
        int cCounter = 0;
        int iter = closeInit;

        for(int i = 0; i < STATIONS.length && iter % STATIONS.length != closeFin; i++)
        {
            cClock++;
            iter++;
            iter = iter % STATIONS.length;
        }
        iter = 0;
        for(int i = 0; i < STATIONS.length && Math.abs(iter) % STATIONS.length != closeFin; i++)
        {
            cCounter++;
            iter--;
            iter = iter % STATIONS.length;
        }

        int cPoints = Math.abs(closeFin-closeInit) + 2;

        if(cCounter < cClock)
        {
            pathPoint[] points = new pathPoint[cPoints];

            points[0] = new pathPoint(initial,fin.getRotation());

            for(int i = closeInit,j = 1; i != -1;i--,j++)
            {
                points[j] = new pathPoint(STATIONS[i].getTranslation(), fin.getRotation());
            }

            for(int i = cPoints - closeInit,j = 1 + closeInit; i >= closeFin;i--,j++)
            {
                points[j] = new pathPoint(STATIONS[i].getTranslation(), fin.getRotation());
            }

            points[cPoints - 1] = new pathPoint(fin.getTranslation(), fin.getRotation());

            return new PathFollow(points);
        }
        else
        {
            pathPoint[] points = new pathPoint[cPoints];

            points[0] = new pathPoint(initial,fin.getRotation());

            for(int i = closeInit,j = 1; i != closeFin;i++,j++)
            {
                points[j] = new pathPoint(STATIONS[i%STATIONS.length].getTranslation(), fin.getRotation());
            }

            points[cPoints - 1] = new pathPoint(fin.getTranslation(), fin.getRotation());

            return new PathFollow(points);
        }

        
    }

    public static void genLineClockwise(Translation2d initial,Pose2d fin,double velocity)
    {

    }

    public static void genLineCounter(Translation2d initial,Pose2d fin,double velocity)
    {
        
    }
}
