package frc.robot.subsystems.chassis;

import frc.robot.utils.TalonConfig;

public class ChassisConstants {
    public static final int GYRO_ID = 14;
    public static final String BUS = "canivore";
    
    public static class ModuleConstants {
        public static final TalonConfig FRONT_LEFT_STEER = new TalonConfig(5, BUS, "Front Left Steer")
            .withPID(10, 0, 0, 0.02, 0.12, 0, 0)
            .withMotionMagic(6 * 2 * Math.PI, 30 * 2 * Math.PI, 80 * 2 * Math.PI);
        public static final TalonConfig FRONT_RIGHT_STEER = new TalonConfig(2, BUS, "Front Right Steer")
            .withPID(10, 0, 0, 0.02, 0.12, 0, 0)
            .withMotionMagic(4 * 2 * Math.PI, 12 * 2 * Math.PI, 16 * 2 * Math.PI);
        public static final TalonConfig BACK_LEFT_STEER = new TalonConfig(11, BUS, "Back Left Steer")
            .withPID(10, 0, 0, 0.02, 0.12, 0, 0)
            .withMotionMagic(4 * 2 * Math.PI, 12 * 2 * Math.PI, 16 * 2 * Math.PI);
        public static final TalonConfig BACK_RIGHT_STEER = new TalonConfig(8, BUS, "Back Right Steer")
            .withPID(10, 0, 0, 0.02, 0.12, 0, 0)
            .withMotionMagic(4 * 2 * Math.PI, 12 * 2 * Math.PI, 16 * 2 * Math.PI);

        public static final TalonConfig FRONT_LEFT_DRIVE = new TalonConfig(4, BUS, "Front Left Drive");
        public static final TalonConfig FRONT_RIGHT_DRIVE = new TalonConfig(1, BUS, "Front Right Drive");
        public static final TalonConfig BACK_LEFT_DRIVE = new TalonConfig(10, BUS, "Back Left Drive");
        public static final TalonConfig BACK_RIGHT_DRIVE = new TalonConfig(7, BUS, "Back Right Drive");
    }

}
