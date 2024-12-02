// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.utils.LogManager;
import frc.robot.utils.Recording;


public class RobotContainer {
  
  LogManager logManager;
  
  public RobotContainer() {
    logManager = new LogManager();

    SmartDashboard.putData("Recording", Recording.getInstance());
    configureBindings();
  }


  private void configureBindings() {

  }

  public Command getAutonomousCommand() {
    return null;
  }
}
