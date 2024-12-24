// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.test;

import java.util.ArrayList;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class moveMotorLog extends Command {
  TempSubSystem subsystem;
  Timer timer;
  double power;
  int count;
  ArrayList<Double> vel;
  ArrayList<Double> accel;
  Supplier<Double> getVel;
  Supplier<Double> getAccel;
  public moveMotorLog(ArrayList<Double> vel, ArrayList<Double> accel, TempSubSystem subsystem, double power, Supplier<Double> getVel, Supplier<Double> getAccel) {
    this.subsystem = subsystem;
    this.power = power;
    this.getVel = getVel;
    this.getAccel = getAccel;

    
    
    addRequirements(subsystem);
  }

  
  @Override
  public void initialize() {
    timer.start();
    
    
  } 

  @Override
  public void execute() {
    subsystem.steerMotor.set(power);
    vel.add(getVel.get());
    vel.add(getAccel.get());
  }

  @Override
  public void end(boolean interrupted) {
    subsystem.steerMotor.set(0);
  }

  @Override
  public boolean isFinished() {
    return timer.get() >= 1.5;
  }
}
