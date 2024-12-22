// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter.Commands;

import static frc.robot.RobotContainer.*;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Shooter.ShooterConstants.AMP_VAR;
import frc.robot.Shooter.ShooterConstants.DELIVERY_VAR;
import frc.robot.Shooter.ShooterConstants.SHOOTER_ATRIBUTES;
import frc.robot.Shooter.ShooterConstants.SHOOTER_POW;
import frc.robot.Shooter.ShooterConstants.STAGE_VAR;
import frc.robot.Shooter.ShooterConstants.STATE;
import frc.robot.Shooter.ShooterConstants.SUBWOFFER_VAR;
import frc.robot.Shooter.Subsystems.AngleChanger;
import frc.robot.Shooter.Subsystems.Intake;
import frc.robot.Shooter.Subsystems.Shooter;
import frc.robot.Shooter.Utils.LookUpTable;
import edu.wpi.first.wpilibj.Timer;

public class Shoot extends Command {


  private double upMotorVelocity;
  private double downMotorVelocity;
  private double testingUpMotorVelocity;
  private double testingDownMotorVelocity;
  private Intake intake;

  public STATE state;
  private double distance;
  private double distenceX;

  public boolean isReady;
  public boolean isfinished;

  private LookUpTable lookupTable;

  private Timer shooterTimer;
  private boolean isTimerRunning;

  //private Translation2d speaker;

  private Shooter shooter;

  /** Creates a new Shoot. */
  public Shoot() {
    shooter = new Shooter();
    state = shooter.shooterState;

    shooterTimer = new Timer();
    isTimerRunning = false;
    isReady = false;
    isfinished = false;

    upMotorVelocity = 0;
    downMotorVelocity = 0;
    testingUpMotorVelocity = 0;
    testingDownMotorVelocity = 0;
    distance = 0;
    distenceX = 0;

    intake = new Intake();

    SmartDashboard.putData(this);
    addRequirements(shooter, intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isTimerRunning = false;
    shooterTimer.stop();
    shooterTimer.reset();
    state = shooter.shooterState;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (state) {
      case AMP:
        upMotorVelocity = AMP_VAR.MOTOR_UP_AMP_VELOCITY;
        downMotorVelocity = AMP_VAR.MOTOR_DOWN_AMP_VELOCITY;
        break;

      case STAGE:
        upMotorVelocity = STAGE_VAR.MOTOR_UP_STAGE_VELOCITY;
        downMotorVelocity = STAGE_VAR.MOTOR_DOWN_STAGE_VELOCITY;
        break;

      case SUBWOFFER:
        upMotorVelocity = SUBWOFFER_VAR.MOTOR_UP_SUBWOFFER_VELOCITY;
        downMotorVelocity = SUBWOFFER_VAR.MOTOR_DOWN_SUBWOFFER_VELOCITY;
        break;

      case SPEAKER:
        //distance = speaker.minus(Chassis)
        double[] lookUpTableData = lookupTable.get(distance);
        upMotorVelocity = lookUpTableData[1];
        downMotorVelocity = lookUpTableData[2];
        break;

      case TESTING:
        upMotorVelocity = testingUpMotorVelocity;
        downMotorVelocity = testingDownMotorVelocity;
        break;

      case DELIVERY:
        upMotorVelocity = DELIVERY_VAR.MOTOR_UP_DELIVERY_VEL;
        downMotorVelocity = DELIVERY_VAR.MOTOR_DOWN_DELIVERY_VEL;
        break;

      case IDLE:
        upMotorVelocity = 0;
        downMotorVelocity = 0;
        break;
    }

    isReady = isDriverOverwriteShooter;
    if (isReady) {
      shooter.setFeedingPower(SHOOTER_POW.FEEDING_MOTOR_POWER);
      intake.setPowerMotors(SHOOTER_POW.INTAKE_MOTOR_POWER);


      if (!isTimerRunning) {
        shooterTimer.start();
        isTimerRunning = true;
      }

      if (shooterTimer.get()*1000 >= SHOOTER_ATRIBUTES.MIL_SEC_TO_SHOOT) {
        isfinished = true;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
    if (state != STATE.TESTING && state != STATE.IDLE && state != STATE.DELIVERY) {
      angleChanger.angleState = STATE.SPEAKER;
      shooter.shooterState = STATE.SPEAKER;
    }

    shooter.isShooterReady = false;
    shooterTimer.stop();
    shooterTimer.reset();
    isTimerRunning = false;
    shooter.setVoltage(0);
    shooter.setFeedingPower(0);
    intake.isNoteInIntake = false;

    

    //shooter.isShooterReady = Ready.isUpMotorReady(upMotorVelocity) && Ready.isDownMotorReady(downMotorVelocity);
    //isReady = isDriverOverwriteShooter;

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isfinished || state == STATE.IDLE;
  }
}
