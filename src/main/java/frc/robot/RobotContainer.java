// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.utils.LogManager;
import frc.robot.PathFollow.Util.ExceedTheSpeed;
import frc.robot.PathFollow.Util.PathFollow;
import frc.robot.PathFollow.Util.PathsConstants;
import frc.robot.PathFollow.Util.StationNav;
import frc.robot.PathFollow.Util.TrigShell;
import frc.robot.PathFollow.Util.TriggerHandler;
import frc.robot.PathFollow.Util.Triggertest;
import frc.robot.PathFollow.Util.pathPoint;
import frc.robot.commands.*;
import frc.robot.commands.chassis.Drive;
import frc.robot.subsystems.chassis.Chassis;



public class RobotContainer implements Sendable{
  public static Boolean isRed = false;
  public static RobotContainer rc;
  public Chassis chassis = new Chassis(); // im sorry
  Drive drive;
  double num = 0;


  LogManager logManager = new LogManager();
  Triggertest test = new Triggertest();
  ExceedTheSpeed com;
  
  public RobotContainer() {
    this.logManager = new LogManager();
    rc = this;

    Translation2d init = new Translation2d(0,0);
    Pose2d fin = new Pose2d(new Translation2d(5,-30), new Rotation2d(0));
    
    PathFollow path = StationNav.genLineByDis(init, fin, 2);

    pathPoint[] points = path.getPoints();

    System.out.println("incoming");
    for(int i = 0; i < points.length; i++)
    {
      System.out.println(points[i]);
    }
    System.out.println("incoming");

    
    // configureBindings();
    // TrigShell shell = new TrigShell(() -> test.exceedsSpeed());
    // TriggerHandler.set("exceedsSpeed",shell);
    // this.com = new ExceedTheSpeed(test);
    // InstantCommand print = new InstantCommand(() -> LogManager.log("EXCEEDS"),test);
    // TriggerHandler.get("exceedsSpeed").onTrue(print);
    // Translation2d a = new Translation2d(4,8);
    // Translation2d b = new Translation2d(0,4);
    // Translation2d c_pos = new Translation2d(4,4);
    // double r = 4;

    // System.out.println("Count : " + PathsConstants.cSegCircleInter(a, b, c_pos, r));
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
