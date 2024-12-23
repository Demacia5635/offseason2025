package frc.robot.subsystems.chassis;

import frc.robot.utils.TalonConfig;

public class ChassisConstants {
    public static TalonConfig FRONT_LEFT_STEER = new TalonConfig(2, "rio", "Front Left Steer")
        .withPID(0.02, 0, 0, 0.04, 0.13, 0, 0);
    public static TalonConfig FRONT_RIGHT_STEER = new TalonConfig(5, "rio", "Front Right Steer");
    public static TalonConfig BACK_LEFT_STEER = new TalonConfig(8, "rio", "Back Left Steer");
    public static TalonConfig BACK_RIGHT_STEER = new TalonConfig(11, "rio", "Back Right Steer");

    public static TalonConfig FRONT_LEFT_DRIVE = new TalonConfig(1, "rio", "Front Left Drive");
    public static TalonConfig FRONT_RIGHT_DRIVE = new TalonConfig(4, "rio", "Front Right Drive");
    public static TalonConfig BACK_LEFT_DRIVE = new TalonConfig(7, "rio", "Back Left Drive");
    public static TalonConfig BACK_RIGHT_DRIVE = new TalonConfig(10, "rio", "Back Right Drive");

}
