// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter.Subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Shooter.ShooterConstants.STATE;
import frc.robot.utils.TalonConfig;
import frc.robot.utils.TalonMotor;
import static frc.robot.Shooter.ShooterConstants.MOTOR_IDS.*;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Shooter extends SubsystemBase {
  
  public boolean isReady;
  public STATE shooterState;

  TalonMotor motorUp;
  TalonMotor motorDown;
  TalonSRX feedingMotor;

  /** Creates a new Shooter. */
  public Shooter() {
    motorDown = new TalonMotor(
      new TalonConfig(MOTOR_DOWN_ID, CANBUS, "down motor")
      .withPID(ANGLE_CHANGING_ID, ANGLE_CHANGING_ID, MOTOR_UP_ID, MOTOR_FEEDING_ID, MOTOR_DOWN_ID, LIMIT_SWITCH_ID, ANGLE_CHANGING_ID)
    );

    motorUp = new TalonMotor(
      new TalonConfig(MOTOR_UP_ID, CANBUS, "Up motor")
    );

    feedingMotor = new TalonSRX(MOTOR_FEEDING_ID);

    shooterState = STATE.SPEAKER;

    isReady = false;

    SmartDashboard.putData("Shooter", this);
  }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
