// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Chassis.ChassisConstants.MODULES;
import frc.robot.Chassis.Subsystem.Module;
import frc.robot.utils.LogManager;


public class RobotContainer {
  
  LogManager logManager;
  Module module;
  RunCommand testSteer;
  
  public RobotContainer() {
    logManager = new LogManager();
    module = new Module(MODULES.LEFT_FRONT);
    testSteer = new RunCommand(() -> module.setDriveMotorVelocity(4 * Math.PI));
    configureBindings();
    SmartDashboard.putData("test steer",testSteer);
  }


  private void configureBindings() {

  }

  public Command getAutonomousCommand() {
    return testSteer;
    //return null;
  }
}