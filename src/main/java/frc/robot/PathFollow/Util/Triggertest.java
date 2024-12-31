// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.PathFollow.Util;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.chassis.ChassisConstants;
import frc.robot.utils.TalonMotor;

public class Triggertest extends SubsystemBase {
  /** Creates a new triggertest. */
  TalonMotor motor = new TalonMotor(ChassisConstants.BACK_LEFT.DRIVE_CONFIG);
  public Triggertest() {

  }

  public boolean exceedsSpeed()
  {
    return motor.getVelocity().getValueAsDouble() > 1;
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
