// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Chassis.ChassisConstants.MODULES;
import frc.robot.Chassis.Subsystem.Chassis;
import frc.robot.Chassis.Subsystem.Module;
import frc.robot.utils.LogManager;


public class RobotContainer {
  
  LogManager logManager;
  Module module;
  Chassis chassis;
  RunCommand testSteerPow;
  RunCommand testSteerVel;
  RunCommand testSteerMagicMotion;
  RunCommand testDrivePow;
  RunCommand testDriveVel;
  RunCommand testDrivePos;
  RunCommand testSteerPos;
  RunCommand testModuleAngle;
  RunCommand testModuleState;
  
  public RobotContainer() {
    logManager = new LogManager();
    module = new Module(MODULES.LEFT_FRONT);
    chassis = new Chassis();
    testSteerPow = new RunCommand(() -> module.setSteerMotorPower(0.1));
    testSteerVel = new RunCommand(() -> module.setSteerMotorVelocity(2 * Math.PI));
    testSteerMagicMotion = new RunCommand(() -> module.setSteerMotorVelocity(0));
    testDrivePow = new RunCommand(() -> module.setDriveMotorPower(0.1));
    testDriveVel = new RunCommand(() -> module.setDriveMotorVelocity(2 * Math.PI));
    testDrivePos = new RunCommand(() -> module.setDriveMotorPosition(0));
    testSteerPos = new RunCommand(() -> module.setSteerMotorPosition(0));
    testModuleAngle = new RunCommand(() -> chassis.setPose2d(Math.PI/4));
    testModuleState = new RunCommand(() -> chassis.setVelocities(new ChassisSpeeds(1, 1, Math.PI/4)));
    configureBindings();
    SmartDashboard.putData("test steer power",testSteerPow);
    SmartDashboard.putData("test steer velocity",testSteerVel);
    SmartDashboard.putData("test steer magic motion",testSteerMagicMotion);
    SmartDashboard.putData("test drive power",testDrivePow);
    SmartDashboard.putData("test drive velocity",testDriveVel);
    SmartDashboard.putData("test drive position",testDrivePos);
    SmartDashboard.putData("test steer position",testSteerPos);
    SmartDashboard.putData("test module angle",testModuleAngle);
  }


  private void configureBindings() {

  }

  public Command getAutonomousCommand() {
    return null;
  }
}