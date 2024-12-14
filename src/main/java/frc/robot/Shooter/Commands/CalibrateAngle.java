package frc.robot.Shooter.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Shooter.ShooterConstants.ANGLE_CHANGING_CALIBRATION;
import frc.robot.Shooter.ShooterConstants.STATE;
import frc.robot.Shooter.Subsystems.AngleChanger;

import static frc.robot.Shooter.ShooterConstants.ANGLE_CHANGING_VAR.*;

public class CalibrateAngle extends Command {
  
  private AngleChanger angleChanger;
  private int finishedState = 0;

  /** Creates a new Calibrition. */
  public CalibrateAngle(AngleChanger angleChanger) {
    this.angleChanger = angleChanger;

    addRequirements(angleChanger);
  }

  
  @Override
  public void initialize() {
    finishedState = 0;
    angleChanger.gotToAngle(TOP_ANGLE);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(angleChanger.isMaxAngle()){
      angleChanger.setVoltage(ANGLE_CHANGING_CALIBRATION.UP_SPEED_CALIBRATION);
      finishedState = 1;
    }
    if(!angleChanger.isMaxAngle() && finishedState == 1){
      finishedState = 2;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    angleChanger.setVoltage(0);
    angleChanger.angleState = STATE.SUBWOFFER;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (finishedState == 2);
  }
}