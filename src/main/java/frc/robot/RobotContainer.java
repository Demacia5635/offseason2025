// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.chassis.commands.DriveCommand;
import frc.robot.chassis.subsystems.Chassis;
import frc.robot.utils.LogManager;
import frc.robot.vision.subsystem.Tag;

import static frc.robot.Constants.*;

public class RobotContainer implements Sendable{
  public static RobotContainer robotContainer;
  public static Boolean isRed = false;
  public static Chassis chassis;
  public DriveCommand driveCommand;
  CommandXboxController driverController;
  LogManager log = new LogManager();

  Tag tag;
  public RobotContainer() {
    robotContainer = this;
    chassis = new Chassis();
    SmartDashboard.putData("RobotContainer", this);
    driverController = new CommandXboxController(DRIVER_CONTROLLER_PORT);
    driveCommand = new DriveCommand(chassis, driverController);
    chassis.setDefaultCommand(driveCommand);
    tag = new Tag(()->chassis.getAngle());
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
  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addBooleanProperty("is Red",this::isRed, this::isRed);
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
