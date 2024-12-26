// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Chassis.Command;

import frc.robot.Chassis.ChassisConstants;
import frc.robot.Chassis.ChassisConstants.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.Chassis.Subsystem.Chassis;

public class CassisDrive extends Command {
  private final Chassis chassis;
  private final CommandXboxController commandXboxController;
  public CassisDrive(Chassis chassis, CommandXboxController commandXboxController) {
    this.chassis = chassis;
    this.commandXboxController = commandXboxController;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double joyX = commandXboxController.getLeftY();
    double joyY = commandXboxController.getLeftX();
    double rot = commandXboxController.getLeftTriggerAxis()
        - commandXboxController.getRightTriggerAxis();
    double velX = joyX * ChassisConstants.MAX_DRIVE_VELOCITY;
    double velY = joyY * ChassisConstants.MAX_DRIVE_VELOCITY;
    double velRot = rot * ChassisConstants.MAX_OMEGA_VELOCITY;
    ChassisSpeeds speeds = new ChassisSpeeds(velX, velY, velRot);
    chassis.setVelocities(speeds);
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
