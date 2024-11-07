// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.test;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.TalonConfig;
import frc.robot.utils.TalonMotor;

public class TempSubSystem extends SubsystemBase {
  
  TalonMotor motor;

  double dutyTest = 0;
  double velTest = 0;
  double motionMagicTest = 0;

  /** Creates a new tempSubSystem. */
  //kp = 0.1, ki = 0.7, kd = 0.07
  public TempSubSystem() {
    motor = new TalonMotor(
      new TalonConfig(8, "rio", "test talon motor")
      .withPID(0, 0, 0, 0.0353203957, 0.0151666609/12, 0.0095467237, 0)
      .withMotionMagic(5, 20, 0)
      .withBrake(false)
      .withMotorRatio(12.8).withRadiansMotor()
    );

    SmartDashboard.putData("test subsystem", this);

    SmartDashboard.putData("motor set pow", new InstantCommand(()-> motor.setDuty(dutyTest), this));
    SmartDashboard.putData("motor set vel", new InstantCommand(()-> motor.setVelocity(velTest), this));
    SmartDashboard.putData("motor set motion magic", new InstantCommand(()-> motor.setMotionMagic(motionMagicTest), this));
    SmartDashboard.putData("motor stop", new InstantCommand(()-> motor.setDuty(0), this));
  } 

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    
    builder.addDoubleProperty("test pow", ()-> dutyTest, (double pow)-> dutyTest = pow);
    builder.addDoubleProperty("test vel", ()-> velTest, (double vel)-> velTest = vel);
    builder.addDoubleProperty("test motion magic pos", ()-> motionMagicTest, (double position)-> motionMagicTest = position);
  }

  @Override
  public void periodic() {
  }
}
