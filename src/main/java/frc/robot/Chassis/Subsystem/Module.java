// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Chassis.Subsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Chassis.ChassisConstants.configConstants.*;
import frc.robot.utils.Cancoder;
import frc.robot.utils.CancoderConfig;
import frc.robot.utils.TalonConfig;
import frc.robot.utils.TalonMotor;

public class Module extends SubsystemBase {
  TalonMotor driveMotor;
  TalonConfig driveConfig;
  TalonMotor steerMotor;
  TalonConfig steerConfig;
  Cancoder cancoder;
  CancoderConfig cancoderConfig;


  /** Creates a new Module. */
  public Module() {
    driveConfig = new TalonConfig(DRIVE_ID, DRIVE_CANBUS, DRIVE_NAME)
    .withMeterMotor(DRIVE_circonference)
    .withPID(DRIVE_KP, DRIVE_KI, DRIVE_KD, DRIVE_KS, DRIVE_KV, DRIVE_KA, DRIVE_KG)
    .withInvert(DRIVE_IS_INVERT)
    .withBrake(DRIVE_IS_BRAKE);
    driveMotor = new TalonMotor(driveConfig);

    steerConfig = new TalonConfig(STEER_ID, STEER_CANBUS, STEER_NAME)
    .withRadiansMotor()
    .withPID(STEER_KP, STEER_KI, STEER_KD, STEER_KS, STEER_KV, STEER_KA, STEER_KG)
    .withMotionMagic(STEER_MOTION_MAGIC_VELOCITY, STEER_MOTION_MAGIC_ACCELERATION, STEER_MOTION_MAGIC_JERK)
    .withInvert(STEER_IS_INVERT)
    .withBrake(STEER_IS_BRAKE);
    steerMotor = new TalonMotor(driveConfig);

    cancoderConfig = new CancoderConfig(CANCODER_ID, CANCODER_CANBUS, CANCODER_NAME)
    .withInvert(DRIVE_IS_INVERT)
    .withOffset(CANCODER_OFFSET);
    cancoder = new Cancoder(cancoderConfig);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setDriveMotorPower(double power){
    driveMotor.setDuty(power);
  }
  
  public void setDriveMotorVelocity(double velocity){
    driveMotor.setVelocity(velocity);
  }

  public void setDriveMotorBrake(boolean isBrake){
    driveMotor.setBrake(isBrake);
  }

  public void setSteerMotorPower(double power){
    driveMotor.setDuty(power);
  }
  
  public void setSteerMotorVelocity(double velocity){
    driveMotor.setVelocity(velocity);
  }

  public void setSteerMotorMotionMagic(double position){
    driveMotor.setMotionMagic(position);
  }

  public void setSteerMotorBrake(boolean isBrake){
    driveMotor.setBrake(isBrake);
  }

  public double getPosition(){
    return cancoder.getPositionRadians();
  }
  
  public double getAbsPosition(){
    return cancoder.getAbsPositionRadians();
  }

  public double getVelocity(){
    return cancoder.getVelocityRadiansPerSec();
  }
}
