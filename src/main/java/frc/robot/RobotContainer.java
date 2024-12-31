// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.utils.LogManager;
import frc.robot.PathFollow.Util.ExceedTheSpeed;
import frc.robot.PathFollow.Util.TrigShell;
import frc.robot.PathFollow.Util.TriggerHandler;
import frc.robot.PathFollow.Util.Triggertest;
import frc.robot.commands.*;

public class RobotContainer {
  
  LogManager logManager;
  Triggertest test = new Triggertest();
  ExceedTheSpeed com;
  
  public RobotContainer() {
    logManager = new LogManager();

    configureBindings();
    TrigShell shell = new TrigShell(() -> test.exceedsSpeed());
    TriggerHandler.set("exceedsSpeed",shell);
    this.com = new ExceedTheSpeed(test);
    InstantCommand print = new InstantCommand(() -> LogManager.log("EXCEEDS"),test);
    TriggerHandler.get("exceedsSpeed").onTrue(print);
  }


  private void configureBindings() {

  }

  public Command getAutonomousCommand() {
    return this.com;
  }
}
