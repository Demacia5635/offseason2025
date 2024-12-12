// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Chassis.Subsystem;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;

import frc.robot.Chassis.ChassisConstants.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Chassis extends SubsystemBase {
  /** Creates a new Chassis. */
  private final Module[] modules;
  public final Pigeon2 gyro;
  public Chassis() {
    modules = new Module[]{
      new Module(MODULES.LEFT_FRONT),
      new Module(MODULES.RIGHT_FRONT),
      new Module(MODULES.LEFT_BACK),
      new Module(MODULES.RIGHT_BACK)
    };
    gyro = new Pigeon2(GYRO.GYRO_ID,GYRO.GYRO_CANBUS);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setModulesDrivePower(double pow) {
    for (var module : modules) {
      module.setDriveMotorPower(pow);
    }
  }

  public void setModulerDrivePower(int pow, int index){
    modules[index].setDriveMotorPower(pow);
  }

  public void setModulesDriveVelocity(double vel) {
    for (var module : modules) {
      module.setDriveMotorPower(vel);
    }
  }

  public void setModuleDriveVelocity(int vel, int index){
    modules[index].setDriveMotorVelocity(vel);
  }

  public void setModulesSteerPower(double pow) {
    for (var module : modules) {
      module.setSteerMotorPower(pow);
    }
  }

  public void setModuleSteerPower(int pow, int index){
    modules[index].setSteerMotorPower(pow);
  }

  public void setModulesSteerVelocity(double vel) {
    for (var module : modules) {
      module.setDriveMotorPower(vel);
    }
  }
  
  public void setModuleSteerVelocity(int vel, int index){
    modules[index].setSteerMotorVelocity(vel);
  }

  public void setModulesSteeMotionMagic(double position) {
    for (var module : modules) {
      module.setDriveMotorPower(position);   
    }
  }

  public void setModuleSteerMotionMagic(int position, int index){
    modules[index].setSteerMotorMotionMagic(position);
  }

}
