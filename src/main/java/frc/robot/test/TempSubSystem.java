// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.test;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Cancoder;
import frc.robot.utils.CancoderConfig;
import frc.robot.utils.LogManager;
import frc.robot.utils.TalonConfig;
import frc.robot.utils.TalonMotor;

public class TempSubSystem extends SubsystemBase {
  
  TalonMotor steerMotor;
  TalonMotor driveMotor;
  Cancoder cancoder;

  double dutyTest = 0;
  double velTest = 0;
  double motionMagicTest = 0;

  /** Creates a new tempSubSystem. */
  //kp = 0.1, ki = 0.7, kd = 0.07
  public TempSubSystem() {
    steerMotor = new TalonMotor(
      new TalonConfig(11, "rio", "steer motor")
      .withPID(0.8, 10.0, 0.0, 0.3295543024, 0.2385745774, -0.003105620266, 0)
      .withMotionMagic(3*2*Math.PI, 15*2*Math.PI, 6*2*Math.PI)
      .withBrake(true).withInvert(false)
      .withMotorRatio(12.8).withRadiansMotor()
    );

    driveMotor = new TalonMotor(
      new TalonConfig(10, "rio", "drive motor")
      .withPID(0, 0, 0, 0, 0, 0, 0)
    );
    cancoder = new Cancoder(
      new CancoderConfig(12, "rio", "cancoder")
      .withInvert(false).withOffset(0)
      );

    steerMotor.setPosition(cancoder.getAbsPositionRadians());
    //steerMotor.hotReloadPidFf(0);
    SmartDashboard.putData("steer motor", steerMotor);
    
    SmartDashboard.putData("test subsystem", this);

    SmartDashboard.putData("motor set pow", new RunCommand(()-> steerMotor.setDuty(dutyTest), this));
    SmartDashboard.putData("motor set vel", new RunCommand(()-> steerMotor.setVelocity(velTest), this));
    SmartDashboard.putData("motor set motion magic", new RunCommand(()-> steerMotor.setMotionMagic(motionMagicTest), this));
    SmartDashboard.putData("motor stop", new InstantCommand(()-> steerMotor.setDuty(0), this));
    SmartDashboard.putData("cancoder", cancoder);
    SmartDashboard.putData("drive pow", new RunCommand(()-> driveMotor.setDuty(dutyTest), this));
    SmartDashboard.putData("swerve drive", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SwerveDrive");

        builder.addDoubleProperty("Back Right Angle", ()-> cancoder.getAbsPositionRadians(), null);
        builder.addDoubleProperty("Back Right Velocity", ()-> driveMotor.getCurrentVelocity(), null);
      }
    });
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
