// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Shooter.Subsystems.Intake;
import frc.robot.Shooter.Subsystems.Shooter;

public class Get_Note_Shoot extends Command {

  Intake intake;
  Shooter shooter;
  boolean isNote;

  /** Creates a new Get_Note_Shoot. */
  public Get_Note_Shoot() {
    intake = new Intake();
    shooter = new Shooter();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isNote = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!isNote){
      
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
