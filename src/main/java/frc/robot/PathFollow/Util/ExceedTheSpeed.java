// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.PathFollow.Util;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.TalonMotor;

public class ExceedTheSpeed extends Command {
  /** Creates a new ExceedTheSpeed. */
  Triggertest left_back;
  public ExceedTheSpeed(Triggertest left_back) {
    this.left_back = left_back;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.left_back.exceedSpeed();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.left_back.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
