// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.test.TempSubSystem;
import frc.robot.utils.LogManager;


public class RobotContainer {
  
  LogManager logManager;
  TempSubSystem tempSubSystem;
  
  public static Boolean isRed = false;
  public static RobotContainer robotContainer;

  public RobotContainer() {
    logManager = new LogManager();
    tempSubSystem = new TempSubSystem();

    SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
    // SmartDashboard.putData("PDH", new PowerDistribution(1, ModuleType.kRev));
    configureBindings();
  }


  private void configureBindings() {

  }

  public void isRed(boolean isRed) {
    RobotContainer.isRed = isRed;
  }
  
  public boolean isRed() {
    return isRed;
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
