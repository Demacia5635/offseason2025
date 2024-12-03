// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter.Subsystems;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.TalonConfig;
import frc.robot.utils.TalonMotor;
import static frc.robot.Shooter.ShooterConstants.MOTOR_IDS.*;

public class AngleChanger extends SubsystemBase {

  TalonMotor AngleChanging;

  /** Creates a new AngleChanger. */
  public AngleChanger() {
    AngleChanging = new TalonMotor(
      new TalonConfig(ANGLE_CHANGING_ID, CANBUS, "Angle Changing")
    );

    SmartDashboard.putData("Angle Changing", this);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
