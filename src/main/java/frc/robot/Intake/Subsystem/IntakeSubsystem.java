// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Intake.Subsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Intake.IntakeConstants.idMotors;
import frc.robot.utils.TalonConfig;
import frc.robot.utils.TalonMotor;

public class IntakeSubsystem extends SubsystemBase {
  /** motor floor to intake */
  TalonMotor motorFeed;
  /**motor intake to shooter */
  TalonMotor motorMove;
  /** Config for motor floor to intake */
  TalonConfig configFeed;
  /** Config for intake to shooter */
  TalonConfig configMove;



  /**Configs the motors and the Configs of the motors */
  public IntakeSubsystem() {

    configFeed = new TalonConfig(idMotors.idMotorfeed, idMotors.CANBUS, "motorFeed");
    configFeed.withBrake(true);
    configFeed.withInvert(false);
    motorFeed = new TalonMotor(configFeed);

    configMove = new TalonConfig(idMotors.idMotorfeed, idMotors.CANBUS, "motorMove");
    configMove.withBrake(true);
    configMove.withInvert(false);
    motorMove = new TalonMotor(configFeed);
  }
  /** Gives power to both motors */
  public void setPowerMotors(double power){
    motorFeed.setDuty(power);
    motorMove.setDuty(power);
  }

  /** Gives power to  motorMove */
  public void setPowerMotorMove(double power){
    motorMove.setDuty(power);
  }

  /** Gives power to motorFeed */
  public void setPowerMotorFeed(double power){
    motorFeed.setDuty(power);
  }




  @Override
  public void periodic() {//TODO:Make a button to do the command
    
  }
}
