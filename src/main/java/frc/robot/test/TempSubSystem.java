// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.test;

import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import javax.xml.crypto.Data;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.units.Power;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Sysid.Sysid;
import frc.robot.utils.Cancoder;
import frc.robot.utils.CancoderConfig;
import frc.robot.utils.LogManager;
import frc.robot.utils.TalonConfig;
import frc.robot.utils.TalonMotor;

public class TempSubSystem extends SubsystemBase {
  
  TalonMotor steerMotor;
  TalonMotor driveMotor;
  Cancoder cancoder;

  Sysid id;
  double dutyTest = 0;
  double velTest = 0;
  double motionMagicTest = 0;

//radian 1 min 3 max radian
  double minPow = 0.0;
  double maxPow = 5;
  double duration = 2;
  double delay = 0.02;
  public DoubleSupplier getV;
  public DoubleSupplier getAccel;

  public DoubleConsumer setSteerPow;


  public double getDutyTest(){
    return this.dutyTest;
  }
  /** Creates a new tempSubSystem. */
  //kp = 0.1, ki = 0.7, kd = 0.07
  public TempSubSystem() { 
    steerMotor = new TalonMotor(
      new TalonConfig(8, "canivore", "steer motor") //KS:0.2608668730650151, KV:0.020429816749889147, KA: 0.027562309396102925
      .withPID(0.0, 0.0, 0, 0.20278460278460256, 0.2620802620802621, 0.0, 0)//KS: 0.06007968127490036, KV: 0.02357481130738511, KA: 0.054110711834295666
      .withMotionMagic(3*2*Math.PI, 5*2*Math.PI, 50*2*Math.PI)
      .withBrake(true).withInvert(true)
      .withMotorRatio(12.8).withRadiansMotor()
    );

    driveMotor = new TalonMotor(
      new TalonConfig(7, "canivore", "drive motor")
      .withPID(0, 0, 0, 0, 0, 0, 0)
    );
    cancoder = new Cancoder(
      new CancoderConfig(9, "canivore", "cancoder")
      .withInvert(false).withOffset(0)
      );

    steerMotor.setPosition(cancoder.getAbsPositionRadians() / (2*Math.PI));

    getV = ()->(steerMotor.getCurrentVelocity());
    getAccel = ()->(steerMotor.getAcceleration().getValueAsDouble());
    
    steerMotor.hotReloadPidFf(0);
    SmartDashboard.putData("steer motor", steerMotor);
    
    setSteerPow = Power -> steerMotor.setDuty(Power);

    SmartDashboard.putData("set sysid" , id.runNormalSysId());

    SmartDashboard.putData("motor set pow", new RunCommand(()-> {
      steerMotor.setDuty(dutyTest);
    }, this));
    SmartDashboard.putData("motor set vel", new RunCommand(()-> steerMotor.setVelocity(velTest), this));
    SmartDashboard.putData("motor set motion magic", new RunCommand(()-> steerMotor.setMotionMagic(motionMagicTest), this));
    SmartDashboard.putData("motor stop", new InstantCommand(()-> steerMotor.setDuty(0), this));
    SmartDashboard.putData("cancoder", cancoder);
    SmartDashboard.putData("drive po w", new RunCommand(()-> driveMotor.setDuty(dutyTest), this));
    SmartDashboard.putData("swerve drive", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SwerveDrive");

        builder.addDoubleProperty("Back Right Angle", ()-> cancoder.getAbsPositionRadians(), null);
        builder.addDoubleProperty("Back Right Velocity", ()-> driveMotor.getCurrentVelocity(), null);
      }
    });
    SmartDashboard.putData("test subsystem", this);
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
