// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter.Subsystems;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Shooter.ShooterConstants.ANGLE_CHANGING_VAR;
import frc.robot.Shooter.ShooterConstants.LOOKUP_TABLE_DATA;
import frc.robot.Shooter.ShooterConstants.STATE;
import frc.robot.Shooter.Utils.LookUpTable;
import frc.robot.utils.TalonConfig;
import frc.robot.utils.TalonMotor;
import static frc.robot.Shooter.ShooterConstants.MOTOR_IDS.*;

public class AngleChanger extends SubsystemBase {

  TalonMotor angleChanging;

  private boolean isCalibrated;
  public LookUpTable lookUp;
  public STATE angleState;
  /** Creates a new AngleChanger. */
  public AngleChanger() {
    angleChanging = new TalonMotor(
      new TalonConfig(ANGLE_CHANGING_ID, CANBUS, "Angle Changing")
    );

    angleState = STATE.STAGE.IDLE;
    lookUp = new LookUpTable(LOOKUP_TABLE_DATA.DATA);
    
    isCalibrated = false;

    SmartDashboard.putData("Angle Changing", this);

  }

  public double getAngle(){
    return -1;
  }

  public void gotToAngle(double angle){
    angleChanging.setMotionMagic(angle);
  }


  public void goToAnglePositionVol(double wantedAngle) {
    if (wantedAngle < ANGLE_CHANGING_VAR.MIN_ANGLE) {
      return ;
    }
    if (wantedAngle > ANGLE_CHANGING_VAR.TOP_ANGLE) {
      return ;
    }

    if (!isCalibrated) {
      return ;
    }

    //double distance = ShooterUtils.angleToDistance(wantedAngle);
    gotToAngle(wantedAngle);;
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
