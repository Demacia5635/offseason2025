// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.test;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.LogManager;
import frc.robot.utils.TalonConfig;
import frc.robot.utils.TalonMotor;

public class TempSubSystem extends SubsystemBase {
  
  TalonMotor motor;

  double dutyTest = 0;
  double velTest = 0;
  double motionMagicTest = 0;

  /** Creates a new tempSubSystem. */
  public TempSubSystem() {
    motor = new TalonMotor(
      new TalonConfig(8, "rio", "test talon motor")
      .withPID(0, 0, 0, 0, 0, 0, 0)
  
    );

    LogManager.addEntry("motor test pow", ()-> dutyTest).setConsumer((Double pow, Long time)-> dutyTest = pow);
    LogManager.addEntry("motor test vel", ()-> velTest).setConsumer((Double vel, Long time)-> velTest = vel);
    LogManager.addEntry("motor motion magic", ()-> motionMagicTest).setConsumer((Double position, Long time)-> motionMagicTest = position);

    SmartDashboard.putData("motor set pow", new InstantCommand(()-> motor.setDuty(dutyTest), this));
    SmartDashboard.putData("motor set vel", new InstantCommand(()-> motor.setVelocity(velTest), this));
    SmartDashboard.putData("motor set motion magic", new InstantCommand(()-> motor.setMotionMagic(motionMagicTest), this));
    SmartDashboard.putData("motor stop", new InstantCommand(()-> motor.setDuty(0), this));
  }

  @Override
  public void periodic() {
  }
}
