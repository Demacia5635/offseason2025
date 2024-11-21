package frc.robot.chassis;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.utils.CancoderConfig;
import frc.robot.utils.TalonConfig;

/**
 * Constants and configuration for a Swerve Drive robot chassis.
 * Contains hardware IDs, physical measurements, and module-specific configurations.
 */
public class SwerveConstants {
    // Gyro Configuration
    public static final int GYRO_ID = 14;

    // Wheel Properties
    public static final double WHEEL_DIAMETER = 4 * 0.0254;  // 4-inch wheel converted to meters
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

    // Gear Ratios
    public static final double MOVE_GEAR_RATIO = 8.14;
    public static final double FRONT_STEER_RATIO = 151.0 / 7.0;
    public static final double BACK_STEER_RATIO = 12.8;

    // Motion Profile Parameters
    public static final double DRIVE_VELOCITY = 0;
    public static final double DRIVE_ACCELERATION = 0;
    public static final double DRIVE_JERK = 0;
    public static final double STEER_VELOCITY = 0;
    public static final double STEER_ACCELERATION = 0;
    public static final double STEER_JERK = 0;

    // CAN Bus Hardware IDs for Drive Motors, Steer Motors and CANCoders
    public static final int 
        FRONT_LEFT_DRIVE_ID = 1,
        FRONT_LEFT_STEER_ID = 2,
        FRONT_LEFT_CANCODER_ID = 3,
        FRONT_RIGHT_DRIVE_ID = 4,
        FRONT_RIGHT_STEER_ID = 5,
        FRONT_RIGHT_CANCODER_ID = 6,
        BACK_LEFT_DRIVE_ID = 7,
        BACK_LEFT_STEER_ID = 8,
        BACK_LEFT_CANCODER_ID = 9,
        BACK_RIGHT_DRIVE_ID = 10,
        BACK_RIGHT_STEER_ID = 11,
        BACK_RIGHT_CANCODER_ID = 12;

    // PID and Feed-Forward Constants
    public static final Control_Constants 
        MOVE_PID = new Control_Constants(0, 0, 0, 0, 0, 0, 0),
        FRONT_STEER_PID = new Control_Constants(0, 0, 0, 0, 0, 0, 0),
        BACK_STEER_PID = new Control_Constants(0, 0, 0, 0, 0, 0, 0);

    public final static SwerveModuleConstants FRONT_LEFT = new SwerveModuleConstants("frontLeft",
            new TalonConfig(FRONT_LEFT_DRIVE_ID, "rio", "frontLeft/Drive")
                .withPID(MOVE_PID.KP, MOVE_PID.KI, MOVE_PID.KD, MOVE_PID.KS, MOVE_PID.KV, MOVE_PID.KA, MOVE_PID.KG)
                .withBrake(true)
                .withInvert(false)
                .withMotorRatio(MOVE_GEAR_RATIO)
                .withMeterMotor(WHEEL_CIRCUMFERENCE)
                .withMotionMagic(DRIVE_VELOCITY, DRIVE_ACCELERATION, DRIVE_JERK),
            
            new TalonConfig(FRONT_LEFT_STEER_ID, "rio", "frontLeft/Steer")
                .withPID(FRONT_STEER_PID.KP, FRONT_STEER_PID.KI, FRONT_STEER_PID.KD, FRONT_STEER_PID.KS, FRONT_STEER_PID.KV, FRONT_STEER_PID.KA, FRONT_STEER_PID.KG)
                .withBrake(true)
                .withInvert(false)
                .withMotorRatio(FRONT_STEER_RATIO)
                .withMeterMotor(WHEEL_CIRCUMFERENCE)
                .withMotionMagic(STEER_VELOCITY, STEER_ACCELERATION, STEER_JERK),

            new CancoderConfig(FRONT_LEFT_CANCODER_ID, "rio", "frontLeft/CanCoder").withInvert(false),
            new Translation2d(0.332, 0.277),
            Rotation2d.fromDegrees(0)
        );

    public final static SwerveModuleConstants FRONT_RIGHT = new SwerveModuleConstants("frontLeft",
            new TalonConfig(FRONT_LEFT_DRIVE_ID, "rio", "frontLeft/Drive")
                .withPID(MOVE_PID.KP, MOVE_PID.KI, MOVE_PID.KD, MOVE_PID.KS, MOVE_PID.KV, MOVE_PID.KA, MOVE_PID.KG)
                .withBrake(true)
                .withInvert(false)
                .withMotorRatio(MOVE_GEAR_RATIO)
                .withMeterMotor(WHEEL_CIRCUMFERENCE)
                .withMotionMagic(DRIVE_VELOCITY, DRIVE_ACCELERATION, DRIVE_JERK),
            
            new TalonConfig(FRONT_LEFT_STEER_ID, "rio", "frontLeft/Steer")
                .withPID(FRONT_STEER_PID.KP, FRONT_STEER_PID.KI, FRONT_STEER_PID.KD, FRONT_STEER_PID.KS, FRONT_STEER_PID.KV, FRONT_STEER_PID.KA, FRONT_STEER_PID.KG)
                .withBrake(true)
                .withInvert(false)
                .withMotorRatio(FRONT_STEER_RATIO)
                .withMeterMotor(WHEEL_CIRCUMFERENCE)
                .withMotionMagic(STEER_VELOCITY, STEER_ACCELERATION, STEER_JERK),

            new CancoderConfig(FRONT_LEFT_CANCODER_ID, "rio", "frontLeft/CanCoder").withInvert(false),
            new Translation2d(0.332, 0.277),
            Rotation2d.fromDegrees(0)
        );
    
    public final static SwerveModuleConstants BACK_LEFT = new SwerveModuleConstants("frontLeft",
            new TalonConfig(FRONT_LEFT_DRIVE_ID, "rio", "frontLeft/Drive")
                .withPID(MOVE_PID.KP, MOVE_PID.KI, MOVE_PID.KD, MOVE_PID.KS, MOVE_PID.KV, MOVE_PID.KA, MOVE_PID.KG)
                .withBrake(true)
                .withInvert(false)
                .withMotorRatio(MOVE_GEAR_RATIO)
                .withMeterMotor(WHEEL_CIRCUMFERENCE)
                .withMotionMagic(DRIVE_VELOCITY, DRIVE_ACCELERATION, DRIVE_JERK),
            
            new TalonConfig(FRONT_LEFT_STEER_ID, "rio", "frontLeft/Steer")
                .withPID(FRONT_STEER_PID.KP, FRONT_STEER_PID.KI, FRONT_STEER_PID.KD, FRONT_STEER_PID.KS, FRONT_STEER_PID.KV, FRONT_STEER_PID.KA, FRONT_STEER_PID.KG)
                .withBrake(true)
                .withInvert(false)
                .withMotorRatio(FRONT_STEER_RATIO)
                .withMeterMotor(WHEEL_CIRCUMFERENCE)
                .withMotionMagic(STEER_VELOCITY, STEER_ACCELERATION, STEER_JERK),

            new CancoderConfig(FRONT_LEFT_CANCODER_ID, "rio", "frontLeft/CanCoder").withInvert(false),
            new Translation2d(0.332, 0.277),
            Rotation2d.fromDegrees(0)
        );
    
    public final static SwerveModuleConstants BACK_RIGHT = new SwerveModuleConstants("frontLeft",
            new TalonConfig(FRONT_LEFT_DRIVE_ID, "rio", "frontLeft/Drive")
                .withPID(MOVE_PID.KP, MOVE_PID.KI, MOVE_PID.KD, MOVE_PID.KS, MOVE_PID.KV, MOVE_PID.KA, MOVE_PID.KG)
                .withBrake(true)
                .withInvert(false)
                .withMotorRatio(MOVE_GEAR_RATIO)
                .withMeterMotor(WHEEL_CIRCUMFERENCE)
                .withMotionMagic(DRIVE_VELOCITY, DRIVE_ACCELERATION, DRIVE_JERK),
            
            new TalonConfig(FRONT_LEFT_STEER_ID, "rio", "frontLeft/Steer")
                .withPID(FRONT_STEER_PID.KP, FRONT_STEER_PID.KI, FRONT_STEER_PID.KD, FRONT_STEER_PID.KS, FRONT_STEER_PID.KV, FRONT_STEER_PID.KA, FRONT_STEER_PID.KG)
                .withBrake(true)
                .withInvert(false)
                .withMotorRatio(FRONT_STEER_RATIO)
                .withMeterMotor(WHEEL_CIRCUMFERENCE)
                .withMotionMagic(STEER_VELOCITY, STEER_ACCELERATION, STEER_JERK),

            new CancoderConfig(FRONT_LEFT_CANCODER_ID, "rio", "frontLeft/CanCoder").withInvert(false),
            new Translation2d(0.332, 0.277),
            Rotation2d.fromDegrees(0)
        );

    // Swerve Drive Kinematics for Odometry and Movement Calculations
    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
        FRONT_LEFT.moduleTranslationOffset,
        FRONT_RIGHT.moduleTranslationOffset,
        BACK_LEFT.moduleTranslationOffset,
        BACK_RIGHT.moduleTranslationOffset
    );


    /**
     * Inner class to hold PID and Feed-Forward control constants
     */
    public static class Control_Constants {
        public final double KP, KI, KD, KS, KV, KA, KG;

        Control_Constants(double KP, double KI, double KD, double KS, double KV, double KA, double KG) {
            this.KP = KP;   // Proportional gain
            this.KI = KI;   // Integral gain
            this.KD = KD;   // Derivative gain
            this.KS = KS;   // Static gain (friction compensation)
            this.KV = KV;   // Velocity gain
            this.KA = KA;   // Acceleration gain
            this.KG = KG;   // Gravity compensation gain
        }
    }

    /**
     * Inner class to encapsulate Swerve Module Configuration
     */
    public static class SwerveModuleConstants {
        String name;
        public TalonConfig driveConfig;
        public TalonConfig steerConfig;
        public CancoderConfig cancoderConfig;
        public final Translation2d moduleTranslationOffset;
        public final Rotation2d steerOffset;

        public SwerveModuleConstants(String name, TalonConfig drive, TalonConfig steer, 
                                     CancoderConfig cancoderConfig, Translation2d offset, 
                                     Rotation2d steerOffset) {
            this.name = name;
            this.driveConfig = drive;
            this.steerConfig = steer;
            this.cancoderConfig = cancoderConfig;
            this.moduleTranslationOffset = offset;
            this.steerOffset = steerOffset;
        }
    }
}