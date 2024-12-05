// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Intake.IntakeConstants;
import frc.robot.Intake.Commands.IntakeToNote;
import frc.robot.Intake.Subsystem.IntakeSubsystem;
import frc.robot.utils.LogManager;


public class RobotContainer {
  
  LogManager logManager;
  IntakeToNote intakeToNote;
  IntakeSubsystem intake;
  Command spitNote;
  Command senterCommand;
  CommandXboxController controller;
  public RobotContainer() {
    logManager = new LogManager();
    controller = new CommandXboxController(0);
    intake = new IntakeSubsystem();





    senterCommand = new RunCommand(()->intake.setPowerMotors(IntakeConstants.POWER_TO_CENTER), intake).withTimeout(0.5)
          .andThen(new RunCommand(()->intake.setPowerMotors(-IntakeConstants.POWER_TO_CENTER), intake).withTimeout(0.5))
          .andThen(new RunCommand(()->intake.setPowerMotors(IntakeConstants.POWER_TO_CENTER), intake).withTimeout(0.5))
          .andThen(new RunCommand(()->intake.setPowerMotors(-IntakeConstants.POWER_TO_CENTER), intake).withTimeout(0.25))
          .andThen(new RunCommand(()->intake.setPowerMotors(0), intake));
    spitNote = new RunCommand(()->intake.setPowerMotors(-0.35), intake).withTimeout(2)
              .andThen(new RunCommand(()->intake.setPowerMotors(0), intake));

    intakeToNote = new IntakeToNote(intake);
    configureBindings();
  }


  private void configureBindings() {
    controller.a().onTrue(intakeToNote);
    controller.povDown().onTrue(spitNote);
    controller.b().onTrue(senterCommand);
    controller.leftBumper().onTrue(stopAll());
  }
  public Command stopAll() {
    return new InstantCommand(()-> {
      intake.setPowerMotors(0);
      
      // shooter.shooterState = STATE.IDLE;
      // angleChanging.angleState = STATE.IDLE;
    }, intake);
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
