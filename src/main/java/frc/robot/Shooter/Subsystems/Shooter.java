// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter.Subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Shooter.ShooterConstants.STATE;
import frc.robot.utils.TalonConfig;
import frc.robot.utils.TalonMotor;
import static frc.robot.Shooter.ShooterConstants.MOTOR_IDS.*;
import static frc.robot.Shooter.ShooterConstants.SHOOTER_PID_FF.*;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Shooter extends SubsystemBase {
  
  public boolean isShotoerReady;
  public STATE shooterState;

  private TalonMotor motorUp;
  private TalonMotor motorDown;
  private TalonSRX feedingMotor;

  private double disposeTest = 0.5;
  

  /** Creates a new Shooter. */
  public Shooter() {
    motorDown = new TalonMotor(
      new TalonConfig(MOTOR_DOWN_ID, CANBUS, "down motor")
      .withPID(UP_MOTOR_KP, UP_MOTOR_KI, UP_MOTOR_KD, SHOOTER_KS, SHOOTER_KV, SHOOTER_KA, 0)
    );

    motorUp = new TalonMotor(
      new TalonConfig(MOTOR_UP_ID, CANBUS, "Up motor")
      .withPID(UP_MOTOR_KP, UP_MOTOR_KI, UP_MOTOR_KD, SHOOTER_KS, SHOOTER_KV, SHOOTER_KA, 0)
    );

    feedingMotor = new TalonSRX(MOTOR_FEEDING_ID);

    shooterState = STATE.SPEAKER;

    isShotoerReady = false;
    motorDown.hotReloadPidFf(0);
    motorUp.hotReloadPidFf(0);

    SmartDashboard.putData("just shoot", new InstantCommand(()->{
      motorDown.setDuty(0.4);
      motorUp.setDuty(0.4);
    }));

    //SmartDashboard.putData("motor disposal", new InstantCommand(()->feedingMotor.set(ControlMode.PercentOutput, disposeTest)));

    SmartDashboard.putData("Shooter", this);
  }

  public void setFeedingPower(double power){
    feedingMotor.set(ControlMode.PercentOutput, power);
  }

  public void pidRunMotor(double velocity){
    motorDown.setVelocity(velocity);
    motorUp.setVelocity(velocity);
  }

  public void pidUpMotor(double velocity){
    motorUp.setVelocity(velocity);
  }

  public void pidDownMotor(double velocity){
    motorDown.setVelocity(velocity);
  }

  public void setVoltage(double voltage){
    motorDown.set(voltage);
    motorUp.set(voltage);
  }
  
  @Override
  public void initSendable(SendableBuilder builder) {
      builder.addDoubleProperty("test disposal", ()-> disposeTest, null);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
