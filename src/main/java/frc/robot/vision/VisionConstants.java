// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Constants and configuration values for AprilTag vision processing.
 * This class contains the physical layout and properties of AprilTags on the field,
 * as well as camera mounting parameters.
 */
public class VisionConstants {

    // Heights of different AprilTag groups from the ground
    private static double MID_TAG_HIGHT = 1.355852;//group 1   *
    private static double HIGH_TAG_HIGHT = 1.451102;// gorup 2   #
    private static double LOW_TAG_HIGHT =1.3208;// group 3 !

    /**
     * Array of AprilTag positions relative to field origin (0,0).
     * Each Translation2d represents X,Y coordinates in meters.
     * Index corresponds to AprilTag ID (0 is unused).
     * Coordinates are converted from inches to meters using inchToMeter().
     */
    public static Translation2d[] O_TO_TAG = {null,//0
        new Translation2d(inchToMeter(593.68), inchToMeter(9.68)),//1
        new Translation2d(inchToMeter(637.21), inchToMeter(34.79)),//2
        new Translation2d(inchToMeter(652.73), inchToMeter(196.17)),//3
        new Translation2d(inchToMeter(652.73), inchToMeter(218.42)),//4
        new Translation2d(inchToMeter(578.77), inchToMeter(323.00)),//5
        new Translation2d(inchToMeter(72.5), inchToMeter(323.00)),//6
        new Translation2d(inchToMeter(-1.50), inchToMeter(218.42)),//7
        new Translation2d(inchToMeter(-1.50), inchToMeter(196.17)),//8
        new Translation2d(inchToMeter(14.02), inchToMeter(34.79)),//9
        new Translation2d(inchToMeter(57.54), inchToMeter(9.68)),//10
        new Translation2d(inchToMeter(468.69), inchToMeter(146.19)),//11
        new Translation2d(inchToMeter(468.69), inchToMeter(177.10)),//12
        new Translation2d(inchToMeter(441.74), inchToMeter(161.62)),//13
        new Translation2d(inchToMeter(209.48), inchToMeter(161.62)),//14
        new Translation2d(inchToMeter(182.73), inchToMeter(177.10)),//15
        new Translation2d(inchToMeter(182.73),inchToMeter(146.19))//16
    };

    /**
     * Array of AprilTag rotations relative to field.
     * Each Rotation2d represents the tag's facing direction in degrees.
     * Index corresponds to AprilTag ID (0 is unused).
     */
    public static Rotation2d[] TAG_ANGLE = {Rotation2d.fromDegrees(0.0),//0
        Rotation2d.fromDegrees(60.0),//1
        Rotation2d.fromDegrees(60.0),//2
        Rotation2d.fromDegrees(0.0),//3
        Rotation2d.fromDegrees(0.0),//4
        Rotation2d.fromDegrees(270.0),//5
        Rotation2d.fromDegrees(270.0),//6
        Rotation2d.fromDegrees(0.0),//7
        Rotation2d.fromDegrees(0.0),//8
        Rotation2d.fromDegrees(60.0),//9
        Rotation2d.fromDegrees(60.0),//10
        Rotation2d.fromDegrees(300.0),//11
        Rotation2d.fromDegrees(60.0),//12
        Rotation2d.fromDegrees(0.0),//13
        Rotation2d.fromDegrees(0.0),//14
        Rotation2d.fromDegrees(300.0),//15
        Rotation2d.fromDegrees(60.0)//16
    };

    /**
     * Array of AprilTag heights from the ground.
     * Each value corresponds to either LOW, MID, or HIGH mounting position.
     * Index corresponds to AprilTag ID (0 is unused).
     */
    public static double[] TAG_HIGHT = {0,//0
        MID_TAG_HIGHT,//1
        MID_TAG_HIGHT,//2
        HIGH_TAG_HIGHT,//3
        HIGH_TAG_HIGHT,//4
        MID_TAG_HIGHT,//5
        MID_TAG_HIGHT,//6
        HIGH_TAG_HIGHT,//7
        HIGH_TAG_HIGHT,//8
        MID_TAG_HIGHT,//9
        MID_TAG_HIGHT,//10
        LOW_TAG_HIGHT,//11
        LOW_TAG_HIGHT,//12
        LOW_TAG_HIGHT,//13
        LOW_TAG_HIGHT,//14
        LOW_TAG_HIGHT,//15
        LOW_TAG_HIGHT//16
    };

    /**
     * Converts a measurement from inches to meters
     * @param inch Value in inches
     * @return Value in meters
     */
    public static double inchToMeter(double inch){
    return inch*0.0254;
    }

     // Camera mounting configuration
    public static final double TAG_CAM_HIGHT = 0.220;
    public static final double TAG_CAM_ANGLE = 34;

    // Camera position relative to robot center
    public static final Translation2d ROBOT_TO_CAM = new Translation2d(0.23,-0.14);

    // NetworkTables key for AprilTag vision data
    public static final String TAG_TABLE = "limelight-tag";
}
