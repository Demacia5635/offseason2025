package frc.robot.subsystems.chassis;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.utils.CancoderConfig;
import frc.robot.utils.TalonConfig;

public class ChassisConstants {
    public static final int GYRO_ID = 14;
    public static final String BUS = "canivore";
    public static final double WHEEL_DIAMETER = 4 * 0.0254; // 4 inch
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
    
    public static class SwerveModuleConfigs {
        public final TalonConfig STEER_CONFIG;
        public final TalonConfig DRIVE_CONFIG;
        public final CancoderConfig CANCODER_CONFIG;
        public final Translation2d POSITION;
        public final double STEER_OFFSET;

        public SwerveModuleConfigs(TalonConfig steerConfig, TalonConfig driveConfig, CancoderConfig cancoderConfig, Translation2d position, double steerOffset) {
            STEER_CONFIG = steerConfig;
            DRIVE_CONFIG = driveConfig;
            CANCODER_CONFIG = cancoderConfig;
            POSITION = position;
            STEER_OFFSET = steerOffset;
        }
    }
    
    public static final double STEER_KP = 12;
    public static final double STEER_KI = 0;
    public static final double STEER_KD = 0;
    public static final double STEER_KS = 0.02;
    public static final double STEER_KV = 0.12;

    public static final double DRIVE_KP = 0.06;
    public static final double DRIVE_KI = 0;
    public static final double DRIVE_KD = 0.004;
    public static final double DRIVE_KS = 0;
    public static final double DRIVE_KV = 0.45;

    public static final SwerveModuleConfigs FRONT_LEFT = new SwerveModuleConfigs(
        new TalonConfig(5, BUS, "Front Left Steer")
            .withPID(STEER_KP, STEER_KI, STEER_KD, STEER_KS, STEER_KV, 0, 0)
            .withMotionMagic(6 * 2 * Math.PI, 30 * 2 * Math.PI, 80 * 2 * Math.PI)
            .withBrake(true),
        new TalonConfig(4, BUS, "Front Left Drive")
            .withMeterMotor(WHEEL_CIRCUMFERENCE)
            .withPID(DRIVE_KP, DRIVE_KI, DRIVE_KD, DRIVE_KS, DRIVE_KV, 0, 0)
            .withBrake(true)
            .withInvert(true),
        new CancoderConfig(6, BUS, "Front Left Cancoder"),
        new Translation2d(0.266, 0.249),
        -0.18407769454627693
    );
    public static final SwerveModuleConfigs FRONT_RIGHT = new SwerveModuleConfigs(
        new TalonConfig(2, BUS, "Front Right Steer")
            .withPID(STEER_KP, STEER_KI, STEER_KD, STEER_KS, STEER_KV, 0, 0)
            .withMotionMagic(4 * 2 * Math.PI, 12 * 2 * Math.PI, 16 * 2 * Math.PI)
            .withBrake(true),
        new TalonConfig(1, BUS, "Front Right Drive")
            .withMeterMotor(WHEEL_CIRCUMFERENCE)
            .withPID(DRIVE_KP, DRIVE_KI, DRIVE_KD, DRIVE_KS, DRIVE_KV, 0, 0)
            .withBrake(true)
            .withInvert(true),
        new CancoderConfig(3, BUS, "Front Right Cancoder"),
        new Translation2d(0.266, -0.249),
        1.9542915237663068
    );
    public static final SwerveModuleConfigs BACK_LEFT = new SwerveModuleConfigs(
        new TalonConfig(11, BUS, "Back Left Steer")
            .withPID(STEER_KP, STEER_KI, STEER_KD, STEER_KS, STEER_KV, 0, 0)
            .withMotionMagic(4 * 2 * Math.PI, 12 * 2 * Math.PI, 16 * 2 * Math.PI)
            .withBrake(true),
        new TalonConfig(10, BUS, "Back Left Drive")
            .withMeterMotor(WHEEL_CIRCUMFERENCE)
            .withPID(DRIVE_KP, DRIVE_KI, DRIVE_KD, DRIVE_KS, DRIVE_KV, 0, 0)
            .withBrake(true)
            .withInvert(true),
        new CancoderConfig(12, BUS, "Back Left Cancoder"),
        new Translation2d(-0.266, 0.249),
        0.983281685034696
    );
    public static final SwerveModuleConfigs BACK_RIGHT = new SwerveModuleConfigs(
        new TalonConfig(8, BUS, "Back Right Steer")
            .withPID(STEER_KP, STEER_KI, STEER_KD, STEER_KS, STEER_KV, 0, 0)
            .withMotionMagic(4 * 2 * Math.PI, 12 * 2 * Math.PI, 16 * 2 * Math.PI)
            .withBrake(true),
        new TalonConfig(7, BUS, "Back Right Drive")
            .withMeterMotor(WHEEL_CIRCUMFERENCE)
            .withPID(DRIVE_KP, DRIVE_KI, DRIVE_KD, DRIVE_KS, DRIVE_KV, 0, 0)
            .withBrake(true)
            .withInvert(true),
        new CancoderConfig(9, BUS, "Back Right Cancoder"),
        new Translation2d(-0.266, -0.249),
        0.9602719732164113
    );
}
