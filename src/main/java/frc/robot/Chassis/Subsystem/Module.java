// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Chassis.Subsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
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
  public Module(TalonConfig driveConfig, TalonConfig steerConfig, CancoderConfig cancoderConfig) {
    driveConfig = this.driveConfig;
    driveMotor = new TalonMotor(driveConfig);

    steerConfig = this.steerConfig;
    steerMotor = new TalonMotor(driveConfig);

    cancoderConfig = this.cancoderConfig;
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

  public double getDriveMotorPower(){
    return driveMotor.getCurrentPosition();
  }

  public double getDriveMotorVelocity(){
    return driveMotor.getCurrentVelocity();
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
