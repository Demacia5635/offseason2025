package frc.robot.chassis;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.utils.CancoderConfig;
import frc.robot.utils.TalonConfig;

/**
 * Constants and configuration for a Swerve Drive robot chassis.
 * Contains hardware IDs, physical measurements, and module-specific configurations.
 */
public class SwerveConstants {
    // max drive and speen velocity
    public static final double MAX_DRIVE_VELOCITY = 4.1;
    public static final double MAX_OMEGA_VELOCITY = Math.toRadians(360);
    // canBas name
    public static final String CanBas= "rio";
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
    public static final double STEER_VELOCITY = Math.PI * 6;
    public static final double STEER_ACCELERATION = Math.PI * 12;
    public static final double STEER_JERK = 0   ;

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
        MOVE_PID = new Control_Constants(0, 0, 0, 0.4, 2.7, 0.03, 0),
        FRONT_STEER_PID = new Control_Constants(0.8, 4, 0, 0.3295, 0.238, 0.003, 0),
        BACK_STEER_PID = new Control_Constants(0.8, 4, 0, 0.3295, 0.238, 0.003, 0);

    public final static SwerveModuleConstants FRONT_LEFT = new SwerveModuleConstants("frontLeft",
            new TalonConfig(FRONT_LEFT_DRIVE_ID, CanBas, "frontLeft/Drive")
                .withPID(MOVE_PID.KP, MOVE_PID.KI, MOVE_PID.KD, MOVE_PID.KS, MOVE_PID.KV, MOVE_PID.KA, MOVE_PID.KG)
                .withBrake(true)
                .withInvert(false)
                .withMotorRatio(MOVE_GEAR_RATIO)
                .withMeterMotor(WHEEL_CIRCUMFERENCE)
                .withMotionMagic(DRIVE_VELOCITY, DRIVE_ACCELERATION, DRIVE_JERK),
            
            new TalonConfig(FRONT_LEFT_STEER_ID, CanBas, "frontLeft/Steer")
                .withPID(FRONT_STEER_PID.KP, FRONT_STEER_PID.KI, FRONT_STEER_PID.KD, FRONT_STEER_PID.KS, FRONT_STEER_PID.KV, FRONT_STEER_PID.KA, FRONT_STEER_PID.KG)
                .withBrake(true)
                .withInvert(false)
                .withMotorRatio(FRONT_STEER_RATIO)
                .withRadiansMotor()
                .withMotionMagic(STEER_VELOCITY, STEER_ACCELERATION, STEER_JERK),

            new CancoderConfig(FRONT_LEFT_CANCODER_ID, CanBas, "frontLeft/CanCoder").withInvert(false),
            new Translation2d(0.332, 0.277),
            0.670166015625
        );

    public final static SwerveModuleConstants FRONT_RIGHT = new SwerveModuleConstants("frontRight",
            new TalonConfig(FRONT_RIGHT_DRIVE_ID, CanBas, "frontRight/Drive")
                .withPID(MOVE_PID.KP, MOVE_PID.KI, MOVE_PID.KD, MOVE_PID.KS, MOVE_PID.KV, MOVE_PID.KA, MOVE_PID.KG)
                .withBrake(true)
                .withInvert(false)
                .withMotorRatio(MOVE_GEAR_RATIO)
                .withMeterMotor(WHEEL_CIRCUMFERENCE)
                .withMotionMagic(DRIVE_VELOCITY, DRIVE_ACCELERATION, DRIVE_JERK),
            
            new TalonConfig(FRONT_RIGHT_STEER_ID, CanBas, "frontRight/Steer")
                .withPID(FRONT_STEER_PID.KP, FRONT_STEER_PID.KI, FRONT_STEER_PID.KD, FRONT_STEER_PID.KS, FRONT_STEER_PID.KV, FRONT_STEER_PID.KA, FRONT_STEER_PID.KG)
                .withBrake(true)
                .withInvert(false)
                .withMotorRatio(FRONT_STEER_RATIO)
                .withRadiansMotor()
                .withMotionMagic(STEER_VELOCITY, STEER_ACCELERATION, STEER_JERK),

            new CancoderConfig(FRONT_LEFT_CANCODER_ID, CanBas, "frontRight/CanCoder").withInvert(false),
            new Translation2d(0.332, -0.277),
            0.67041015625-2*Math.PI
        );
    
    public final static SwerveModuleConstants BACK_LEFT = new SwerveModuleConstants("backLeft",
            new TalonConfig(BACK_LEFT_DRIVE_ID, CanBas, "backLeft/Drive")
                .withPID(MOVE_PID.KP, MOVE_PID.KI, MOVE_PID.KD, MOVE_PID.KS, MOVE_PID.KV, MOVE_PID.KA, MOVE_PID.KG)
                .withBrake(true)
                .withInvert(false)
                .withMotorRatio(MOVE_GEAR_RATIO)
                .withMeterMotor(WHEEL_CIRCUMFERENCE)
                .withMotionMagic(DRIVE_VELOCITY, DRIVE_ACCELERATION, DRIVE_JERK),
            
            new TalonConfig(BACK_LEFT_STEER_ID, CanBas, "backLeft/Steer")
                .withPID(BACK_STEER_PID.KP, BACK_STEER_PID.KI, BACK_STEER_PID.KD, BACK_STEER_PID.KS, BACK_STEER_PID.KV, BACK_STEER_PID.KA, BACK_STEER_PID.KG)
                .withBrake(true)
                .withInvert(true)
                .withMotorRatio(BACK_STEER_RATIO)
                .withRadiansMotor()
                .withMotionMagic(STEER_VELOCITY, STEER_ACCELERATION, STEER_JERK),

            new CancoderConfig(BACK_LEFT_CANCODER_ID, CanBas, "backLeft/CanCoder").withInvert(false),
            new Translation2d(-0.332, 0.277),
            2.41259765625
        );
    
    public final static SwerveModuleConstants BACK_RIGHT = new SwerveModuleConstants("backRight",
            new TalonConfig(BACK_RIGHT_DRIVE_ID, CanBas, "backRight/Drive")
                .withPID(MOVE_PID.KP, MOVE_PID.KI, MOVE_PID.KD, MOVE_PID.KS, MOVE_PID.KV, MOVE_PID.KA, MOVE_PID.KG)
                .withBrake(true)
                .withInvert(false)
                .withMotorRatio(MOVE_GEAR_RATIO)
                .withMeterMotor(WHEEL_CIRCUMFERENCE)
                .withMotionMagic(DRIVE_VELOCITY, DRIVE_ACCELERATION, DRIVE_JERK),
            
            new TalonConfig(BACK_RIGHT_STEER_ID, CanBas, "backRight/Steer")
                .withPID(BACK_STEER_PID.KP, BACK_STEER_PID.KI, BACK_STEER_PID.KD, BACK_STEER_PID.KS, BACK_STEER_PID.KV, BACK_STEER_PID.KA, BACK_STEER_PID.KG)
                .withBrake(true)
                .withInvert(true)
                .withMotorRatio(BACK_STEER_RATIO)
                .withRadiansMotor()
                .withMotionMagic(STEER_VELOCITY, STEER_ACCELERATION, STEER_JERK),

            new CancoderConfig(BACK_RIGHT_CANCODER_ID, CanBas, "backRight/CanCoder").withInvert(false),
            new Translation2d(-0.332, -0.277),
            0.091796875
            
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
        public String name;
        public TalonConfig driveConfig;
        public TalonConfig steerConfig;
        public CancoderConfig cancoderConfig;
        public final Translation2d moduleTranslationOffset;
        public final double steerOffset;

        public SwerveModuleConstants(String name, TalonConfig drive, TalonConfig steer, 
                                     CancoderConfig cancoderConfig, Translation2d offset, 
                                     double steerOffset) {
            this.name = name;
            this.driveConfig = drive;
            this.steerConfig = steer;
            this.cancoderConfig = cancoderConfig;
            this.moduleTranslationOffset = offset;
            this.steerOffset = steerOffset;
        }
    }
}