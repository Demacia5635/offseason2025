// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.Drive;
import frc.robot.subsystems.Chassis;
import frc.robot.utils.LogManager;


public class RobotContainer {
  
  LogManager logManager;

  Chassis chassis;
  Joystick joystick;
  Drive drive;
  
  public RobotContainer() {
    chassis = new Chassis(15);
    joystick = new Joystick(0);
    drive = new Drive(chassis, joystick);

    chassis.setDefaultCommand(drive);

    logManager = new LogManager();

    configureBindings();
  }


  private void configureBindings() {

  }

  public Command getAutonomousCommand() {
    return null;
  }
}
