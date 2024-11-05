// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.test.TempSubSystem;
import frc.robot.utils.LogManager;


public class RobotContainer {
  
  LogManager logManager;
  TempSubSystem tempSubSystem;
  
  public RobotContainer() {
    logManager = new LogManager();
    tempSubSystem = new TempSubSystem();

    configureBindings();
  }


  private void configureBindings() {

  }

  public Command getAutonomousCommand() {
    return null;
  }
}
