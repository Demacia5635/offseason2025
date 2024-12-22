// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Chassis.Subsystem;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    steerMotor = new TalonMotor(steerConfig);

    cancoderConfig = moduleConstants.cancoderConfig;
    cancoder = new Cancoder(cancoderConfig);

     SmartDashboard.putData(this);
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
    steerMotor.setDuty(power);
  }
  
  public void setSteerMotorVelocity(double velocity){
    steerMotor.setVelocity(velocity);
  }

  public void setSteerMotorMotionMagic(double position){
    steerMotor.setMotionMagic(position);
  }

  public void setSteerMotorBrake(boolean isBrake){
    steerMotor.setBrake(isBrake);
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

  public SwerveModulePosition getSwerveModulePosition(){
    return new SwerveModulePosition(getDrivePosition(),Rotation2d.fromRadians(getSteerPosition()));
  }

  public void setState(SwerveModuleState state){
    double currPos = steerMotor.getCurrentPosition();
    state = SwerveModuleState.optimize(state, Rotation2d.fromRadians(currPos));
    double stateRadians = state.angle.getRadians();
    double vel = state.speedMetersPerSecond;
    setSteerMotorMotionMagic(currPos+stateRadians);
    setDriveMotorVelocity(vel);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveMotorVelocity(), Rotation2d.fromRadians(getSteerAbsPosition()));
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("drive position", () -> getDrivePosition(), null);
    builder.addDoubleProperty("drive velocity", () -> getDriveMotorVelocity(), null);
    builder.addDoubleProperty("steer position", () -> getSteerPosition(), null);
    builder.addDoubleProperty("steer velocity", () -> getSteerVelocity(), null);
    builder.addDoubleProperty("steer abs position", () -> getSteerAbsPosition(), null);
  }
}