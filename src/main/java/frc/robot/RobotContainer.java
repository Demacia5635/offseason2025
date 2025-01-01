// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
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
import frc.robot.commands.chassis.Drive;
import frc.robot.subsystems.chassis.Chassis;

public class RobotContainer implements Sendable{
  public static Boolean isRed = false;
  Chassis chassis;
  Drive drive;
  double num = 0;

  LogManager logManager = new LogManager();
  Triggertest test = new Triggertest();
  ExceedTheSpeed com;
  
  public RobotContainer() {

    configureBindings();
    TrigShell shell = new TrigShell(() -> test.exceedsSpeed());
    TriggerHandler.set("exceedsSpeed",shell);
    this.com = new ExceedTheSpeed(test);
    InstantCommand print = new InstantCommand(() -> LogManager.log("EXCEEDS"),test);
    TriggerHandler.get("exceedsSpeed").onTrue(print);
  }
  public double getNum(){ return num;}
  public void setNum(double num){this.num = num;}


  private void configureBindings() {

  }
  public void isRed(boolean isRed) {
    this.isRed = isRed;
  }

  public static boolean isRed() {
    return isRed;
  }
  @Override
  public void initSendable(SendableBuilder builder) {
      builder.addDoubleProperty("NUM", ()->getNum(), (double num)->setNum(num));
  }

  public Command getAutonomousCommand() {
    return this.com;
  }
}
