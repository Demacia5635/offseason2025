// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.chassis.commands.Drive;
import frc.robot.chassis.subsystems.Chassis;
import frc.robot.utils.LogManager;
import frc.robot.vision.subsystem.Tag;


public class RobotContainer{
  
  LogManager logManager;
  public static Boolean isRed = false;
  CommandXboxController commandController;

  public Chassis chassis;
  public Tag tag;

  public RobotContainer() {
    logManager = new LogManager();
    chassis = new Chassis();
    commandController = new CommandXboxController(0);
    chassis.setDefaultCommand(new Drive(chassis, commandController));
    tag = new Tag(()->chassis.getGyroRotation());

  }

  public void isRed(boolean isRed) {
    this.isRed = isRed;
  }

  public static boolean isRed() {
    return isRed;
  }

  public Command getAutonomousCommand() {
    //return new RunCommand(()->chassis.setSteerAngle(0), chassis);
    return null;
  }
  private Rotation2d tagTestAngle(double angle){
    return Rotation2d.fromDegrees(angle);
  }

}
