// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.test;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Cancoder;
import frc.robot.utils.LogManager;
import frc.robot.utils.TalonConfig;
import frc.robot.utils.TalonMotor;

public class TempSubSystem extends SubsystemBase {
  
  TalonMotor motor;
  Cancoder cancoder;

  double dutyTest = 0;
  double velTest = 0;
  double motionMagicTest = 0;

  /** Creates a new tempSubSystem. */
  //kp = 0.1, ki = 0.7, kd = 0.07
  public TempSubSystem() {
    motor = new TalonMotor(
      new TalonConfig(8, "rio", "test talon motor")
      .withPID(0.17, 0.004, 0.0, 0.3295543024, 0.2385745774, -0.003105620266, 0)
      .withMotionMagic(30, 20, 0)
      .withBrake(false)
      .withMotorRatio(12.8).withRadiansMotor()
    );

    cancoder = new Cancoder(9, "rio");

    SmartDashboard.putData("test subsystem", this);

    SmartDashboard.putData("motor set pow", new RunCommand(()-> motor.setDuty(dutyTest), this));
    SmartDashboard.putData("motor set vel", new RunCommand(()-> motor.setVelocity(velTest), this));
    SmartDashboard.putData("motor set motion magic", new RunCommand(()-> motor.setMotionMagic(motionMagicTest), this));
    SmartDashboard.putData("motor stop", new InstantCommand(()-> motor.setDuty(0), this));
  } 

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    
    builder.addDoubleProperty("test pow", ()-> dutyTest, (double pow)-> dutyTest = pow);
    builder.addDoubleProperty("test vel", ()-> velTest, (double vel)-> velTest = vel);
    builder.addDoubleProperty("test motion magic pos", ()-> motionMagicTest, (double position)-> motionMagicTest = position);

    LogManager.addEntry("cancoder angle", cancoder::getAbsRotation2d);
    LogManager.addEntry("cancoder vel", cancoder::getVelocityRotation2dPerSec);
  }

  @Override
  public void periodic() {
  }
}
