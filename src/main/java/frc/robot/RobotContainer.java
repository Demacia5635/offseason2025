// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.utils.LogManager;


public class RobotContainer {
  
  LogManager logManager;
  // Consumer<Double> setPow = power -> SmartDashboard.getNumber("DutyCycle/setPow", 0.1);
  // Supplier<Double> getVelocity = ()-> (tempSubSystem.getCurrentVel());
  // double minPower = 0.1;
  // double maxPower = 0.8;
  // double duration = 0.02;
  // double delay = 0.02;
  // Sysid id = new Sysid(setPow, getVelocity, minPower, maxPower, tempSubSystem);
  
  
  public RobotContainer() {
    logManager = new LogManager();

    SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
    // SmartDashboard.putData("PDH", new PowerDistribution(1, ModuleType.kRev));
    configureBindings();
  }


  private void configureBindings() {

  }

  public Command getAutonomousCommand() {
    return null;
  }
}
