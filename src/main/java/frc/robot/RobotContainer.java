// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.chassis.ChassisConstants;
import frc.robot.subsystems.chassis.SwerveModule;
import frc.robot.utils.LogManager;


public class RobotContainer {
  
  LogManager logManager;

  SwerveModule module;

  public RobotContainer() {
    logManager = new LogManager();
    module = new SwerveModule(ChassisConstants.BACK_RIGHT_STEER, frc.robot.subsystems.chassis.ChassisConstants.FRONT_RIGHT_DRIVE);

    configureBindings();
  }


  private void configureBindings() {

  }

  public Command getAutonomousCommand() {
    return new RunCommand(() -> module.setSteerPosition(10));
  }
}
