// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.PathFollow.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import static frc.robot.PathFollow.Util.PathsConstants.STATIONS;
import static frc.robot.PathFollow.Util.PathsConstants.STATION_RADIUS;

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

    public static PathFollow goToScore(Pose2d curPose, Pose2d scorePose){
        Rotation2d angleToScore = scorePose.getRotation();
        Translation2d[] points = calcPositionPoints(curPose, scorePose.getTranslation());
        pathPoint[] pathPoints = new pathPoint[points.length];
        for(int i = 0; i < pathPoints.length; i++){
            pathPoints[i] = new pathPoint(points[i], angleToScore);
        }
        return new PathFollow(pathPoints);
    }

    



    private static PathFollow bridgeClock(int closeInit,int closeFin, Translation2d init, Pose2d fin)
    {
        int diff = Math.abs(closeFin-closeInit);
        int cPoints = closeFin > closeInit ? diff + 3 : STATIONS.length - diff + 3; 
        pathPoint[] points = new pathPoint[cPoints];
        points[0] = new pathPoint(init,fin.getRotation());

        for(int i = 1,j=closeInit; i < cPoints - 2; i++,j++){
            // System.out.println("index : " + j%(STATIONS.length));
            points[i] = new pathPoint(STATIONS[j%(STATIONS.length)].getTranslation(), fin.getRotation());
        }
        
        points[cPoints - 2] = new pathPoint(STATIONS[closeFin].getTranslation(), fin.getRotation());
        points[cPoints - 1] = new pathPoint(fin.getTranslation(), fin.getRotation());

        return new PathFollow(points);

    }


    private static PathFollow bridgeCounter(int closeInit,int closeFin, Translation2d init, Pose2d fin)
    {
        int diff = Math.abs(closeFin-closeInit);
        int cPoints = closeInit > closeFin ? diff + 3: STATIONS.length - diff + 3;// stations - diff + initclose + fin+ init
        pathPoint[] points = new pathPoint[cPoints];
        points[0] = new pathPoint(init,fin.getRotation());

        for(int i = 1,j=closeInit; i < cPoints - 2; i++,j--)
        {
            // System.out.println("index : " + j%(STATIONS.length));
            points[i] = new pathPoint(STATIONS[j].getTranslation(), fin.getRotation());
            if(j == 0)
                j = STATIONS.length;
        }

        points[cPoints - 2] = new pathPoint(STATIONS[closeFin].getTranslation(), fin.getRotation());
        points[cPoints - 1] = new pathPoint(fin.getTranslation(), fin.getRotation());

        return new PathFollow(points);
    }

    private static int shiftClock(int tan,int close,Translation2d dest)
    {
        int station = tan;
        
        //intersections counter
        int inter = PathsConstants.cSegCircleInter(STATIONS[station].getTranslation(), dest, PathsConstants.STATION_CENTER, PathsConstants.STATION_RADIUS);
        while(station != close && (inter != 1 || inter != 0)){
            //move clockwise
            station = (station + 1) % STATIONS.length;
            inter = PathsConstants.cSegCircleInter(STATIONS[station].getTranslation(), dest, PathsConstants.STATION_CENTER, PathsConstants.STATION_RADIUS);
        } 

        return station;
    }

    private static int shiftCounter(int tan,int close,Translation2d dest)
    {
        int station = tan;
        
        //intersections counter
        int inter = PathsConstants.cSegCircleInter(STATIONS[station].getTranslation(), dest, PathsConstants.STATION_CENTER, PathsConstants.STATION_RADIUS);
        while(station != close && (inter != 1 || inter != 0)){
            //move counter-clockwise
            if(station - 1 >= 0)
                station--;
            else
                station = STATIONS.length - 1;
            inter = PathsConstants.cSegCircleInter(STATIONS[station].getTranslation(), dest, PathsConstants.STATION_CENTER, PathsConstants.STATION_RADIUS);
        } 

        return station;
    }

    public static PathFollow genLineByDis(Translation2d initial,Pose2d fin,double velocity){
        
        int closeInit = 0;
        int closeFin = 0;

        int tanInit = 0;
        int tanFin = 0;
        
        for(int i = 1; i < STATIONS.length; i++)
        {
            Pose2d station = STATIONS[i];//TODO:optimize this loop

            double d_init = station.getTranslation().minus(initial).getNorm();
            double d_initclose = initial.minus(
                STATIONS[closeInit].getTranslation()).getNorm();

            double d_fin = station.getTranslation().minus(fin.getTranslation()).getNorm();
            double d_finclose = fin.getTranslation().minus(
                STATIONS[closeFin].getTranslation()).getNorm();

            double dot_init = PathsConstants.cross_prod(initial, station.getTranslation());
            double dot_initclose = PathsConstants.cross_prod(initial, STATIONS[tanInit].getTranslation());

            double dot_fin = PathsConstants.cross_prod(fin.getTranslation(), station.getTranslation());
            double dot_finclose = PathsConstants.cross_prod(fin.getTranslation(), STATIONS[tanFin].getTranslation());
            
            if(Math.abs(dot_init) < Math.abs(dot_initclose))
                tanInit = i;
            if(Math.abs(dot_fin) < Math.abs(dot_finclose))
                tanFin = i;

            if (d_init < d_initclose)
                closeInit = i;
            if(d_fin < d_finclose)
                closeFin = i;
        }

        System.out.println("tanInit : " + tanInit);
        System.out.println("tanFin : " + tanFin);

        int diff = Math.abs(tanFin-tanInit);
        if (tanInit < tanFin)
        {
            if(diff < STATIONS.length/2)
            {
                System.out.println("small diff, init - fin");

                int exitStation = tanFin;
                int enterStation = tanInit;
                
                //intersections counter
                exitStation = shiftClock(tanFin,closeFin,fin.getTranslation());

                enterStation = shiftCounter(tanInit, closeInit, initial);

                return bridgeClock(enterStation, exitStation, initial, fin);

            }
            else
            {
                System.out.println("big diff, init - fin");
                int exitStation = tanFin;
                int enterStation = tanInit;
                
                //intersections counter
                exitStation = shiftCounter(tanFin,closeFin,fin.getTranslation());

                enterStation = shiftClock(tanInit, closeInit, initial);

                return bridgeCounter(enterStation, exitStation, initial, fin);
            }
        }
        else{
            if(tanFin == tanInit)
            {
                pathPoint[] points = new pathPoint[3];
                points[0] = new pathPoint(initial, fin.getRotation());
                points[1] = new pathPoint(STATIONS[closeInit].getTranslation(),fin.getRotation());
                points[2] = new pathPoint(fin.getTranslation(), fin.getRotation());
                return new PathFollow(points);
            }
            else{

                if(diff < STATIONS.length/2)
                {
                    System.out.println("small diff, fin - init");

                    int exitStation = tanFin;
                    int enterStation = tanInit;
                    
                    //intersections counter
                    exitStation = shiftCounter(tanFin,closeFin,fin.getTranslation());

                    enterStation = shiftClock(tanInit, closeInit, initial);

                    return bridgeCounter(enterStation, exitStation, initial, fin);

                }
                else
                {
                    System.out.println("big diff, fin - init");
                    int exitStation = tanFin;
                    int enterStation = tanInit;
                
                    //intersections counter
                    exitStation = shiftClock(tanFin,closeFin,fin.getTranslation());

                    enterStation = shiftCounter(tanInit, closeInit, initial);

                    return bridgeClock(enterStation, exitStation, initial, fin);
                }
            }

           
        }
    }

    public static void genLineClockwise(Translation2d initial,Pose2d fin,double velocity)
    {

    }

    public static void genLineCounter(Translation2d initial,Pose2d fin,double velocity)
    {
        
    }
}
