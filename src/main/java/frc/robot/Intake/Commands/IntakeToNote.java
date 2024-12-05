// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Intake.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Intake.IntakeConstants;
import frc.robot.Intake.IntakeConstants.NotePosition;
import frc.robot.Intake.Subsystem.IntakeSubsystem;

public class IntakeToNote extends Command {
  private IntakeSubsystem intakeSubsystem;
  /**timer for the command */
  Timer timerCommand;
  /**timer for intake motors to move the note */
  Timer timerIntake;

  boolean hasTaken;
  boolean touchedDownWheels = false;

  /**
   * Takes the intake subsystem
   * @param intake
   */
  public IntakeToNote(IntakeSubsystem intake) {
    this.intakeSubsystem = intake;
    timerCommand = new Timer();
    timerIntake = new Timer();
    addRequirements(intakeSubsystem);
  }

  @Override
  public void initialize() {
    intakeSubsystem.currentPosition = NotePosition.NO_NOTE;
    timerCommand.start();
    timerIntake.reset();
    timerIntake.stop();
    touchedDownWheels = false;
  }


    /**
   * Case for no note in intake, set motors to almost max
   * <br></br> case for note touching first motor pic up , Set motors to FIRST_TOUCH powers
   */
  @Override
  public void execute() {
    System.out.println("--------------------------------------------------------");
    if (intakeSubsystem.AmperHighMotorPickUp() && !touchedDownWheels) {
      System.out.println("+++++++++++++++++++++++++++++++++++++++++++++++++++++++");
      intakeSubsystem.currentPosition = NotePosition.FIRST_TOUCH;
      touchedDownWheels = true;
    }
    if(touchedDownWheels && intakeSubsystem.AmperHighMotorPickUp2()) {
      System.out.println("***************************************************");
      intakeSubsystem.isNoteInIntake = true;
      timerIntake.reset();
      timerIntake.start();
      touchedDownWheels = false;
    } 
    intakeSubsystem.setPowerMotorMove(intakeSubsystem.currentPosition.movePow);
    intakeSubsystem.setPowerMotorFeed(intakeSubsystem.currentPosition.pickUpPow);
  }


    /**
   * When the funcation end we updated the boolean vlaue 
   * <br></br> of is note in intake , turn the motors to 0 , reset the timer and center the note if needed
   */
  @Override
  public void end(boolean interrupted) {
    System.out.println("ended");
    intakeSubsystem.setPowerMotors(0);
    timerCommand.stop();
    timerCommand.reset();
    timerIntake.stop();
    timerIntake.reset();
    //intakeSubsystem.centerNote();
  }


    /**
   * Case when the note is in the intake
   */
  @Override
  public boolean isFinished() {
    
    return timerIntake.get()*1000 > IntakeConstants.STOP_AFTER_NOTE || timerCommand.get()*1000 > IntakeConstants.STOP_COMMAND_TIME || !intakeSubsystem.AmperHighMotorMove();
  }
}
