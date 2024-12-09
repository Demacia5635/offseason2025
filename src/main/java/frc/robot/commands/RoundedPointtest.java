// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.math.geometry.Rotation2d;


import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.PathFollow.Util.RoundedPoint;
import frc.robot.PathFollow.Util.pathPoint;

public class RoundedPointtest extends Command {
  /** Creates a new RoundedPointtest. */
  public RoundedPointtest() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Rotation2d zero = new Rotation2d(0);
    pathPoint a = new pathPoint(new Translation2d(0,0),zero,0);
    pathPoint b = new pathPoint(new Translation2d(3,3),zero,0.1);
    pathPoint c = new pathPoint(new Translation2d(0,6),zero,0);
    RoundedPoint point = new RoundedPoint(a,b,c, false);
    double velocity = 3;
    point.setMinRadius(velocity);
    System.out.println(point.getAtoCurveLeg());
    System.out.println(point.getArc());
    System.out.println(point.getCtoCurveLeg());

    double time = point.getArc().getLength()/velocity;

    System.out.println("Time to do arc : " + time);
    System.out.println("Angular velocity : " + ((point.getArc().getLength()/time)/point.getRadius()));

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
