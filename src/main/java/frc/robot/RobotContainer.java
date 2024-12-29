// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.chassis.Drive;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.utils.LogManager;


public class RobotContainer {
  
  LogManager logManager;

  Chassis chassis;
  Drive drive;

  public RobotContainer() {
    logManager = new LogManager();
    chassis = new Chassis();
    drive = new Drive(chassis, new CommandXboxController(0));
    chassis.setDefaultCommand(drive);
    
    configureBindings();
  }


  private void configureBindings() {

  }

  public Command getAutonomousCommand() {
    return new RunCommand(() -> chassis.setDriveVelocities(1), chassis);
      // return new RunCommand(() -> chassis.setSteerPositions(0.5 * Math.PI), chassis);
  }
}
