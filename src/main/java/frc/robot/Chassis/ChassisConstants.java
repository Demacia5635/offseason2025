// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Chassis;


/** Add your docs here. */
public class ChassisConstants {
    public static class configConstants{
        public static final int DRIVE_ID = 0;
        public static final String DRIVE_CANBUS = "rio";
        public static final String DRIVE_NAME = "drive motor";
        public static final double DRIVE_RADIOS = 0;
        public static final double DRIVE_circonference = DRIVE_RADIOS * 2 * Math.PI;
        public static final double DRIVE_KP = 0;
        public static final double DRIVE_KI = 0;
        public static final double DRIVE_KD = 0;
        public static final double DRIVE_KS = 0;
        public static final double DRIVE_KV = 0;
        public static final double DRIVE_KA = 0;
        public static final double DRIVE_KG = 0;
        public static final boolean DRIVE_IS_INVERT = false;
        public static final boolean DRIVE_IS_BRAKE = false;

        public static final int STEER_ID = 0;
        public static final String STEER_CANBUS = "rio";
        public static final String STEER_NAME = "steer motor";
        public static final int STEER_KP = 0;
        public static final int STEER_KI = 0;
        public static final int STEER_KD = 0;
        public static final int STEER_KS = 0;
        public static final int STEER_KV = 0;
        public static final int STEER_KA = 0;
        public static final int STEER_KG = 0;
        public static final int STEER_MOTION_MAGIC_VELOCITY = 0;
        public static final int STEER_MOTION_MAGIC_ACCELERATION = 0;
        public static final int STEER_MOTION_MAGIC_JERK = 0;
        public static final boolean STEER_IS_INVERT = false;
        public static final boolean STEER_IS_BRAKE = false;

        public static final int CANCODER_ID = 0;
        public static final String CANCODER_CANBUS = "rio";
        public static final String CANCODER_NAME = "cancoder motor";
        public static final double CANCODER_OFFSET = 0;
        public static final boolean CANCODER_IS_INVERT = false;
    }
}
