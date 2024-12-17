// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Chassis.Subsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Cancoder;
import frc.robot.utils.CancoderConfig;
import frc.robot.utils.TalonConfig;
import frc.robot.utils.TalonMotor;
import frc.robot.Chassis.utils.ModuleConstants;

public class Module extends SubsystemBase {
  TalonMotor driveMotor;
  TalonConfig driveConfig;
  TalonMotor steerMotor;
  TalonConfig steerConfig;
  Cancoder cancoder;
  CancoderConfig cancoderConfig;


  /** Creates a new Module. */
  public Module(ModuleConstants moduleConstants) {
    driveConfig = moduleConstants.driveConfig;
    driveMotor = new TalonMotor(driveConfig);

    steerConfig = moduleConstants.steerConfig;
    steerMotor = new TalonMotor(driveConfig);

    cancoderConfig = moduleConstants.cancoderConfig;
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

  public void setDriveMotorPosition(double pos){
    driveMotor.setPosition(pos);
  }

  public double getDriveMotorVelocity(){
    return driveMotor.getCurrentVelocity();
  }

  public double getDrivePosition(){
    return driveMotor.getCurrentPosition();
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

  public void setSteerMotorPosition(double pos) {
    steerMotor.setPosition(pos);
    cancoder.setPosition(pos);
  }

  public double getSteerVelocity(){
    return cancoder.getPositionRadians();
  }

  public double getSteerPosition(){
    return cancoder.getPositionRadians();
  }
  
  public double getSteerAbsPosition(){
    return cancoder.getAbsPositionRadians();
  }

  public void setStop(){
    setDriveMotorPower(0);
    setSteerMotorPower(0);
  }
}