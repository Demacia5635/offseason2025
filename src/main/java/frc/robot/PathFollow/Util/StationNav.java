// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.PathFollow.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import static frc.robot.PathFollow.Util.PathsConstants.STATIONS;
import static frc.robot.PathFollow.Util.PathFollow.*;

/** Add your docs here. */
public class StationNav {
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
