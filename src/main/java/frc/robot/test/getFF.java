// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.test;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class getFF extends Command {
  TempSubSystem subsystem;
  ArrayList<Double> vel;
  ArrayList<Double> accel;
  double minPow;
  double maxPow;
  double curPow;
  double deltaV = 0.1;
  double[] powers;
  boolean isFinished;
  public getFF(TempSubSystem subsystem, double minPow, double maxPow) {
    this.subsystem = subsystem;
    this.minPow = minPow;
    this.maxPow = maxPow;
    this.curPow = minPow;
    this.powers = new double[(int)((maxPow - minPow) / deltaV)];

  }

  
  @Override
  public void initialize() {
    isFinished = false;
    vel = new ArrayList<Double>();
    accel = new ArrayList<Double>();

    for(int i = 0; i < powers.length; i++){
      powers[i] = minPow + (deltaV * i);
    }

    for(int i = 0; i < (maxPow - minPow) / deltaV; i++){
      new moveMotorLog(vel, accel, subsystem, minPow + (i * deltaV), subsystem.getV, subsystem.getAccel).andThen(new WaitCommand(1)).schedule();
      new moveMotorLog(vel, accel, subsystem, -(minPow + (i * deltaV)), subsystem.getV, subsystem.getAccel).andThen(new WaitCommand(1)).schedule();

    }
    FeedForward.GetFF(powers, (Double[])vel.toArray(), (Double[])accel.toArray());
    isFinished = true;
  }

  @Override
  public void end(boolean interrupted) {
    subsystem.steerMotor.set(0);
  }


  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
