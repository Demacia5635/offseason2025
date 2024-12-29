package frc.robot.subsystems.chassis;

import frc.robot.utils.TalonConfig;

public class ChassisConstants {
    public static TalonConfig FRONT_LEFT_STEER = new TalonConfig(5, "canivore", "Front Left Steer")
        .withPID(10, 0, 0, 0.02, 0.12, 0, 0)
        .withMotionMagic(4 * 2 * Math.PI, 12 * 2 * Math.PI, 16 * 2 * Math.PI);
    public static TalonConfig FRONT_RIGHT_STEER = new TalonConfig(2, "canivore", "Front Right Steer")
        .withPID(10, 0, 0, 0.02, 0.12, 0, 0)
        .withMotionMagic(4 * 2 * Math.PI, 12 * 2 * Math.PI, 16 * 2 * Math.PI);
    public static TalonConfig BACK_LEFT_STEER = new TalonConfig(11, "canivore", "Back Left Steer")
        .withPID(10, 0, 0, 0.02, 0.12, 0, 0)
        .withMotionMagic(4 * 2 * Math.PI, 12 * 2 * Math.PI, 16 * 2 * Math.PI);
    public static TalonConfig BACK_RIGHT_STEER = new TalonConfig(8, "canivore", "Back Right Steer")
        .withPID(10, 0, 0, 0.02, 0.12, 0, 0)
        .withMotionMagic(4 * 2 * Math.PI, 12 * 2 * Math.PI, 16 * 2 * Math.PI);

    public static TalonConfig FRONT_LEFT_DRIVE = new TalonConfig(4, "canivore", "Front Left Drive");
    public static TalonConfig FRONT_RIGHT_DRIVE = new TalonConfig(1, "canivore", "Front Right Drive");
    public static TalonConfig BACK_LEFT_DRIVE = new TalonConfig(10, "canivore", "Back Left Drive");
    public static TalonConfig BACK_RIGHT_DRIVE = new TalonConfig(7, "canivore", "Back Right Drive");

}
