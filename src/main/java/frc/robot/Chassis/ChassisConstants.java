// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Chassis;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Chassis.utils.ModuleConstants;
import frc.robot.utils.CancoderConfig;
import frc.robot.utils.TalonConfig;

/** Add your docs here. */
public class ChassisConstants {
    public final static int MAX_DRIVE_VELOCITY = 4;
    public final static int MAX_OMEGA_VELOCITY = 4;
    public final static int CONTROLLER_PORT = 0;
    public class GYRO {
        public final static int GYRO_ID = 14;
        public final static String GYRO_CANBUS = "rio";
    }
    public class MODULES{
        public static ModuleConstants LEFT_FRONT = new ModuleConstants(
            "FrontLeft", 
            new TalonConfig(4,"rio","FrontLeftDrive")
                .withPID(0.1, 0, 0, 0, 0, 0, 0)
                .withInvert(false)
                .withMeterMotor(1)
                .withMotorRatio(1)
                .withBrake(false),
            new TalonConfig(5, "rio", "FrontLeftSteer")
                .withPID(0.1, 0, 0, 0, 0, 0, 0)
                .withRadiansMotor()
                .withInvert(false)
                .withMotionMagic(25, 40, 130)
                .withMotorRatio(1)
                .withBrake(false),
            new CancoderConfig(6, "rio", "FrontLeftCancoder")
                .withInvert(false)
                .withOffset(0),
            new Translation2d(0,0));
        public static ModuleConstants RIGHT_FRONT = new ModuleConstants(
            "FrontRight", 
            new TalonConfig(1,"rio","FrontRightDrive")
                .withPID(0.1, 0, 0, 0, 0, 0, 0)
                .withInvert(false)
                .withMeterMotor(1)
                .withMotorRatio(1)
                .withBrake(false),
            new TalonConfig(2, "rio", "FrontRightSteer")
                .withPID(0.1, 0, 0, 0, 0, 0, 0)
                .withRadiansMotor()
                .withInvert(false)
                .withMotionMagic(25, 40, 130)
                .withMotorRatio(1)
                .withBrake(false),
            new CancoderConfig(3, "rio", "FrontRightCancoder")
                .withInvert(false)
                .withOffset(0),
            new Translation2d(0,0));
        public static ModuleConstants LEFT_BACK = new ModuleConstants(
            "BackLeft", 
            new TalonConfig(7,"rio","BackLeftDrive")
                .withPID(0, 0, 0, 0, 0, 0, 0)
                .withInvert(false)
                .withMeterMotor(1)
                .withMotorRatio(1)
                .withBrake(false),
            new TalonConfig(8, "rio", "BackLeftSteer")
                .withPID(0, 0, 0, 0, 0, 0, 0)
                .withRadiansMotor()
                .withInvert(false)
                .withMotionMagic(25, 40, 130)
                .withMotorRatio(1)
                .withBrake(false),
            new CancoderConfig(9, "rio", "BackLeftCancoder")
                .withInvert(false)
                .withOffset(0),
            new Translation2d(0,0));
        public static ModuleConstants RIGHT_BACK = new ModuleConstants(
            "RightBack", 
            new TalonConfig(10,"rio","RightBackDrive")
                .withPID(0, 0, 0, 0, 0, 0, 0)
                .withInvert(false)
                .withMeterMotor(1)
                .withMotorRatio(1)
                .withBrake(false),
            new TalonConfig(11, "rio", "RightBackSteer")
                .withPID(0, 0, 0, 0, 0, 0, 0)
                .withRadiansMotor()
                .withInvert(false)
                .withMotionMagic(25, 40, 130)
                .withMotorRatio(1)
                .withBrake(false),
            new CancoderConfig(12, "rio", "rightBackeCancoder")
                .withInvert(false)
                .withOffset(0),
            new Translation2d(0,0));
    public final static Translation2d FRONT_LEFT_LOCATION = new Translation2d(0, 0);
    public final static Translation2d FRONT_RIGHT_LOCATION = new Translation2d(0, 0);
    public final static Translation2d BACK_LEFT_LOCATION = new Translation2d(0, 0);
    public final static Translation2d BACK_RIGHT_LOCATION = new Translation2d(0, 0);
    }
}
