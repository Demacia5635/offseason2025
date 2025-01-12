// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.PathFollow.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/** Add your docs here. */
public class PathsConstants {
    public final static double FIELD_LENGTH = 16.54; // in meters
    public final static double FIELD_HEIGH = 8.21; // in meters
    public final static double DISTANCE_OFFSET = 0.01;;
    public final static double MAX_VELOCITY = 4.1;
    public final static double ACCEL = 8;
    public final static double MAX_ROTATION_VELOCITY = 180;
    public final static double ROTATION_ACCEL = 360;
    public final static double FINISH_OFFSET = 0.05;
    public final static double MIN_SEGMENT_LENGTH = 0.15;
    public final static double MAX_RADIAL_ACCEL = -1;


    public final static Translation2d STATION_CENTER = new Translation2d(4,4);
    public final static double STATION_RADIUS = 4;
    public final static Pose2d[] STATIONS = new Pose2d[]{
        new Pose2d(new Translation2d(1,0).times(STATION_RADIUS).plus(STATION_CENTER),new Rotation2d(0)),
        new Pose2d(new Translation2d(0.866,-0.5).times(STATION_RADIUS).plus(STATION_CENTER),new Rotation2d(((Math.PI*2)/12) - Math.PI)),
        new Pose2d(new Translation2d(0.5,-0.866).times(STATION_RADIUS).plus(STATION_CENTER),new Rotation2d(((Math.PI*2)/12)*2 - Math.PI)),
        new Pose2d(new Translation2d(0,-1).times(STATION_RADIUS).plus(STATION_CENTER),new Rotation2d(((Math.PI*2)/12)*3 - Math.PI)),
        new Pose2d(new Translation2d(-0.5,-0.866).times(STATION_RADIUS).plus(STATION_CENTER),new Rotation2d(((Math.PI*2)/12)*4 - Math.PI)),
        new Pose2d(new Translation2d(-0.866,-0.5).times(STATION_RADIUS).plus(STATION_CENTER),new Rotation2d(((Math.PI*2)/12)*5 - Math.PI)),
        new Pose2d(new Translation2d(-1,0).times(STATION_RADIUS).plus(STATION_CENTER),new Rotation2d(((Math.PI*2)/12)*6 - Math.PI)),
        new Pose2d(new Translation2d(-0.866,0.5).times(STATION_RADIUS).plus(STATION_CENTER),new Rotation2d(((Math.PI*2)/12)*7 - Math.PI)),
        new Pose2d(new Translation2d(-0.5,0.866).times(STATION_RADIUS).plus(STATION_CENTER),new Rotation2d(((Math.PI*2)/12)*8 - Math.PI)),
        new Pose2d(new Translation2d(0,1).times(STATION_RADIUS).plus(STATION_CENTER),new Rotation2d(((Math.PI*2)/12)*9 - Math.PI)),
        new Pose2d(new Translation2d(0.5,0.866).times(STATION_RADIUS).plus(STATION_CENTER),new Rotation2d(((Math.PI*2)/12)*10 - Math.PI)),
        new Pose2d(new Translation2d(0.866,0.5).times(STATION_RADIUS).plus(STATION_CENTER),new Rotation2d(((Math.PI*2)/12)*11 - Math.PI))
    }; //please arrange it clockwise

}
