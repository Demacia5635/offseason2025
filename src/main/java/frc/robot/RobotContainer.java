// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.chassis.commands.DriveCommand;
import frc.robot.chassis.subsystem.Chassis;
import frc.robot.utils.LogManager;


public class RobotContainer {
  
  LogManager logManager;
  public static Boolean isRed = true;
  CommandXboxController commandController;

  public Chassis chassis;

  public RobotContainer() {
    logManager = new LogManager();
    chassis = new Chassis();
    commandController = new CommandXboxController(0);
    chassis.setDefaultCommand(new DriveCommand(chassis, commandController));
  }

  public void isRed(boolean isRed) {
    this.isRed = isRed;
  }

  public static boolean isRed() {
    return isRed;
  }

  public Command getAutonomousCommand() {
    return new RunCommand(()->chassis.setSteerAngle(0), chassis);
  }

}
