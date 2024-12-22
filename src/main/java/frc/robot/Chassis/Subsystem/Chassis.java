// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Chassis.Subsystem;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;

import frc.robot.Chassis.ChassisConstants.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Chassis extends SubsystemBase {
  /** Creates a new Chassis. */
  private final Module[] modules;
  public final Pigeon2 gyro;
  private final Field2d field;
  private SwerveDriveKinematics kinematics;
  private SwerveDrivePoseEstimator poseEstimator;
  public Chassis() {
    modules = new Module[]{
      new Module(MODULES.LEFT_FRONT),
      new Module(MODULES.RIGHT_FRONT),
      new Module(MODULES.LEFT_BACK),
      new Module(MODULES.RIGHT_BACK)
    };
    gyro = new Pigeon2(GYRO.GYRO_ID,GYRO.GYRO_CANBUS);
    kinematics = new SwerveDriveKinematics
    (
      MODULES.FRONT_LEFT_LOCATION, MODULES.FRONT_RIGHT_LOCATION,
      MODULES.BACK_LEFT_LOCATION, MODULES.BACK_RIGHT_LOCATION
    );
    poseEstimator = new SwerveDrivePoseEstimator(kinematics, getGyroAngle(), getSwerveModulesPositions(), new Pose2d());
    field = new Field2d();
  }

  @Override
  public void periodic() {
    poseEstimator.update(getGyroAngle(), getSwerveModulesPositions());
    field.setRobotPose(getPose2d().plus(new Transform2d(0, 0, new Rotation2d())));
  }

  public void setPose2d(Pose2d pose2d){
    poseEstimator.resetPosition(getGyroAngle(), getSwerveModulesPositions(), pose2d);
  }

  public void setPose2d(Double angle){
    Pose2d newPose = new Pose2d(getPose2d().getTranslation(), Rotation2d.fromRadians(angle));
    poseEstimator.resetPosition(getGyroAngle(), getSwerveModulesPositions(), newPose);
  }

  public Pose2d getPose2d(){
    return poseEstimator.getEstimatedPosition();
  }

  public Rotation2d getGyroAngle(){
    return Rotation2d.fromDegrees(gyro.getAngle());
  }

  public SwerveModulePosition[] getSwerveModulesPositions(){
    SwerveModulePosition[] swerveModulesPositions = new SwerveModulePosition[modules.length];
    for (int i = 0; i < modules.length; i++) {
      swerveModulesPositions[i] = modules[i].getSwerveModulePosition();
    }
    return swerveModulesPositions;
  }

  public SwerveModulePosition getSwerveModulePosition(int index){
    return modules[index].getSwerveModulePosition();
  }

  public void setBrake(boolean isDriveBrake, boolean isSteerBrake, int index){
    modules[index].setDriveMotorBrake(isDriveBrake);
    modules[index].setSteerMotorBrake(isSteerBrake);
  }

  public void setDriveBrake(boolean isDriveBrake, int index){
    modules[index].setDriveMotorBrake(isDriveBrake);
  }

  public void setSteerBrake(boolean isSteerBrake, int index){
    modules[index].setSteerMotorBrake(isSteerBrake);
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

  public void setModuelsStates(SwerveModuleState[] states){
    for(int i = 0; i < states.length; i++){
      modules[i].setState(states[i]);
    }
  }

  public void setModuelState(SwerveModuleState state, int index){
    modules[index].setState(state);
  }

  public void setVelocities(ChassisSpeeds speeds){
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
    setModuelsStates(states);
  }

  public SwerveDrivePoseEstimator getSwerveDrivePoseEstimator(){
    return poseEstimator;
  }
}
