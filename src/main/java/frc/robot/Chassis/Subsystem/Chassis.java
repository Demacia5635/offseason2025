// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Chassis.Subsystem;

import com.ctre.phoenix6.hardware.Pigeon2;

import frc.robot.Chassis.ChassisConstants.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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

  public double getGyroAngle(){
    return gyro.getAngle();
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
      module.setDriveMotorVelocity(vel);
    }
  }

  public void setModuleDriveVelocity(int vel, int index){
    modules[index].setDriveMotorVelocity(vel);
  }

  public void setModulesDrivePosition(double pos){
    for (var module : modules) {
      module.setDriveMotorPosition(pos);;
    }
  }

  public void setModuleDrivePosition(double pos, int index){
    modules[index].setDriveMotorPosition(pos);
  }

  public double[] getModulesDriveVelocity(){
    double[] Velocities = new double[modules.length];
    for (int i = 0; i < modules.length; i++) {
      Velocities[i] = modules[i].getDriveMotorVelocity();
    }
    return Velocities;
  }

  public double getModuleDriveVelocity(int index){
    return modules[index].getDriveMotorVelocity();
  }

  public double[] getModulesDrivePosition(){
    double[] Positions = new double[modules.length];
    for (int i = 0; i < modules.length; i++) {
      Positions[i] = modules[i].getDrivePosition();
    }
    return Positions;
  }

  public double getModuleDrivePosition(int index){
    return modules[index].getDrivePosition();
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
      module.setSteerMotorVelocity(vel);
    }
  }
  
  public void setModuleSteerVelocity(int vel, int index){
    modules[index].setSteerMotorVelocity(vel);
  }

  public void setModulesSteerPosition(double pos) {
    for (var module : modules) {
      module.setSteerMotorPosition(pos);
    }
  }
  
  public void setModuleSteerPosition(int pos, int index){
    modules[index].setSteerMotorPosition(pos);
  }

  public void setModulesSteeMotionMagic(double position) {
    for (var module : modules) {
      module.setSteerMotorMotionMagic(position);
    }
  }

  public void setModuleSteerMotionMagic(int position, int index){
    modules[index].setSteerMotorMotionMagic(position);
  }

  public double[] getModulesSteerVelocity(){
    double[] Velocities = new double[modules.length];
    for (int i = 0; i < modules.length; i++) {
      Velocities[i] = modules[i].getSteerVelocity();
    }
    return Velocities;
  }

  public double getModuleSteerVelocity(int index){
    return modules[index].getSteerVelocity();
  }

  public double[] getModulesSteerPosition(){
    double[] Positions = new double[modules.length];
    for (int i = 0; i < modules.length; i++) {
      Positions[i] = modules[i].getSteerPosition();
    }
    return Positions;
  }

  public double getModuleSteerPosition(int index){
    return modules[index].getSteerPosition();
  }

  public void setVelocities(ChassisSpeeds speeds){
    
  }
}
