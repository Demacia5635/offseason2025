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
    public class MODULES{
        public static ModuleConstants RIGHT_FRONT = new ModuleConstants(
            "FrontRight", 
            new TalonConfig(0,"rio","FrontRightDrive")
                .withPID(0, 0, 0, 0, 0, 0, 0)
                .withInvert(false)
                .withMeterMotor(1)
                .withMotorRatio(1)
                .withBrake(false),
            new TalonConfig(0, "rio", "FrontRightSteer")
                .withPID(0, 0, 0, 0, 0, 0, 0)
                .withRadiansMotor()
                .withInvert(false)
                .withMotionMagic(0, 0, 0)
                .withMotorRatio(1)
                .withBrake(false),
            new CancoderConfig(0, "rio", "FrontRightCancoder")
                .withInvert(false)
                .withOffset(0),
            new Translation2d(0,0));
        public static ModuleConstants RIGHT_BACK = new ModuleConstants(
            "FrontRack", 
            new TalonConfig(0,"rio","FrontRackDrive")
                .withPID(0, 0, 0, 0, 0, 0, 0)
                .withInvert(false)
                .withMeterMotor(1)
                .withMotorRatio(1)
                .withBrake(false),
            new TalonConfig(0, "rio", "FrontBackSteer")
                .withPID(0, 0, 0, 0, 0, 0, 0)
                .withRadiansMotor()
                .withInvert(false)
                .withMotionMagic(0, 0, 0)
                .withMotorRatio(1)
                .withBrake(false),
            new CancoderConfig(0, "rio", "FrontRightCancoder")
                .withInvert(false)
                .withOffset(0),
            new Translation2d(0,0));
        public static ModuleConstants LEFT_FRONT = new ModuleConstants(
            "FrontLeft", 
            new TalonConfig(0,"rio","FrontLeftDrive")
                .withPID(0, 0, 0, 0, 0, 0, 0)
                .withInvert(false)
                .withMeterMotor(1)
                .withMotorRatio(1)
                .withBrake(false),
            new TalonConfig(0, "rio", "FrontLeftSteer")
                .withPID(0, 0, 0, 0, 0, 0, 0)
                .withRadiansMotor()
                .withInvert(false)
                .withMotionMagic(0, 0, 0)
                .withMotorRatio(1)
                .withBrake(false),
            new CancoderConfig(0, "rio", "FrontRightCancoder")
                .withInvert(false)
                .withOffset(0),
            new Translation2d(0,0));
        public static ModuleConstants LEFT_BACK = new ModuleConstants(
            "BackLeft", 
            new TalonConfig(0,"rio","BackLeftDrive")
                .withPID(0, 0, 0, 0, 0, 0, 0)
                .withInvert(false)
                .withMeterMotor(1)
                .withMotorRatio(1)
                .withBrake(false),
            new TalonConfig(0, "rio", "BackLeftSteer")
                .withPID(0, 0, 0, 0, 0, 0, 0)
                .withRadiansMotor()
                .withInvert(false)
                .withMotionMagic(0, 0, 0)
                .withMotorRatio(1)
                .withBrake(false),
            new CancoderConfig(0, "rio", "FrontRightCancoder")
                .withInvert(false)
                .withOffset(0),
            new Translation2d(0,0));
    }
    public class GYRO {
        public final static int GYRO_ID = 0;
        public final static String GYRO_CANBUS = "rio";
    }
}
