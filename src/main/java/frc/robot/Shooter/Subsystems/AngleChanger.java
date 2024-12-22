// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter.Subsystems;


import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Shooter.ShooterConstants.ANGLE_CHANGING_VAR;
import frc.robot.Shooter.ShooterConstants.LOOKUP_TABLE_DATA;
import frc.robot.Shooter.ShooterConstants.STATE;
import frc.robot.Shooter.Utils.LookUpTable;
import frc.robot.utils.TalonConfig;
import frc.robot.utils.TalonMotor;
import static frc.robot.Shooter.ShooterConstants.MOTOR_IDS.*;
import static frc.robot.Shooter.ShooterConstants.ANGLE_CHANGING_PID_FF.*;
import static frc.robot.Shooter.ShooterConstants.ANGLE_CHANGING_CONFIGS.*;

public class AngleChanger extends SubsystemBase {

  TalonMotor angleChanging;

  
  private boolean isCalibrated;
  public DigitalInput limitSwitch;
  public LookUpTable lookUp;
  public STATE angleState;

  double dutyTest = 0;
  double velTest = 0;


  /** Creates a new AngleChanger. */
  public AngleChanger() {
    angleChanging = new TalonMotor(
      new TalonConfig(ANGLE_CHANGING_ID, CANBUS, "Angle Changing")
      .withPID(0.0, KI, KD, KS, KV, KA, 0)   // Reminder values were made without Radians (false values)
      .withMotorRatio(ANGLE_CHANGING_GEAR_RATIO).withRadiansMotor()
    );

    angleState = STATE.IDLE;
    lookUp = new LookUpTable(LOOKUP_TABLE_DATA.DATA);
    
    isCalibrated = false;

    SmartDashboard.putData("Change Angle" , new RunCommand(()->{
      angleChanging.setDuty(dutyTest);
    }, this));
    SmartDashboard.putData("angle changing Vel", new RunCommand(()->{
      angleChanging.setVelocity(velTest);
    }, this));

    SmartDashboard.putData("motor stop", new InstantCommand(()-> angleChanging.setDuty(0), this));

    SmartDashboard.putData("Angle Changing", this);

  }

  public boolean isMaxAngle() {
    return !limitSwitch.get();
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


  public void setVoltage(double voltage){
    angleChanging.setVoltage(voltage);
  }

  public void setDuty(double duty){
    angleChanging.setDuty(duty);
  }

  /**Need to add ShooterUtils so for not default is -1
   * 
   * @return current angle
   */
  public double getAngle() {
    //return ShooterUtils.distanceToAngle(angleChangingMotor.getPosition().getValueAsDouble());
    return -1;
  }


  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("test pow", ()-> dutyTest, (double pow)-> dutyTest = pow);
    builder.addDoubleProperty("test Vel", ()-> velTest, (double vel) -> velTest = vel);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
