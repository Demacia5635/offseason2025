// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Shooter.Subsystems.AngleChanger;
import frc.robot.Shooter.Subsystems.Intake;
import frc.robot.Shooter.Subsystems.Shooter;
import frc.robot.utils.LogManager;


public class RobotContainer {
  
  LogManager logManager;
  Shooter shooter;
  public static AngleChanger angleChanger;
  Intake intake;

  public static boolean isDriverOverwriteShooter = false;
  
  
  public RobotContainer() {
    logManager = new LogManager();
    shooter = new Shooter();
    angleChanger = new AngleChanger();
    intake = new Intake();


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
