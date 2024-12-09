// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter.Commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Shooter.ShooterConstants.AMP_VAR;
import frc.robot.Shooter.ShooterConstants.DELIVERY_VAR;
import frc.robot.Shooter.ShooterConstants.STAGE_VAR;
import frc.robot.Shooter.ShooterConstants.STATE;
import frc.robot.Shooter.ShooterConstants.SUBWOFFER_VAR;
import frc.robot.Shooter.Subsystems.AngleChanger;
import frc.robot.Shooter.Utils.LookUpTable;

public class GoToAngle extends Command {

  AngleChanger angleChanger;
  LookUpTable lookUp;  

  private LookUpTable lookupTable;
  private double wantedAngle;
  private double testingAngle;
  private double distance;
  private double distanceX;
  public STATE state;
  public static boolean isAngleReady = false;

  /** Creates a new GoToAngle. */
  public GoToAngle() {
    angleChanger = new AngleChanger();
    lookUp = angleChanger.lookUp;


    wantedAngle = angleChanger.getAngle();
    state = angleChanger.angleState;

    SmartDashboard.putData(this);
    addRequirements(angleChanger);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    state = angleChanger.angleState;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    state = angleChanger.angleState;

    switch (state) {
      case AMP:
        wantedAngle = AMP_VAR.AMP_ANGLE;
        break;

      case STAGE:
        wantedAngle = STAGE_VAR.STAGE_ANGLE;

        break;

      case SUBWOFFER:
        wantedAngle = SUBWOFFER_VAR.SUBWOFFER_ANGLE;
        break;

      case SPEAKER:
        //distance = speaker.minus(chassis.getPose().getTranslation()).getNorm()+0.35;
        double[] speakerLookUpTableData = lookupTable.get(distance);
        wantedAngle = speakerLookUpTableData[0];
        break;

      case DELIVERY:
        wantedAngle = DELIVERY_VAR.DELIVERY_ANGLE;
      
      case TESTING:
        wantedAngle = testingAngle;
        break;

      case IDLE:
        wantedAngle = angleChanger.getAngle();
        break;
    }

    //isAngleReady = Ready.isAngleReady(wantedAngle);

    // angleChanger.goToAngle(wantedAngle);
    angleChanger.goToAnglePositionVol(wantedAngle);
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
