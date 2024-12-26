// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.test;

import java.util.ArrayList;
import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.utils.LogManager;

public class getFFAccel extends Command {
  Consumer<Double> setPower;
  Supplier<Double> getVel;
  Supplier<Double> getAccel;

  ArrayList<Double> vel;
  ArrayList<Double> accel;
  ArrayList<Double> powers;
  double minPow;
  double maxPow;
  double curPow;
  double diff;
  double[] ffValues;
  double delta;
  boolean withNegative;

  double time;
  

  public getFFAccel(Consumer<Double> setPower, Supplier<Double> getVel, Supplier<Double> getAccel, double minPow, double maxPow, double time, boolean withNegative) {

    this.setPower = setPower;
    this.withNegative = withNegative;
    this.getVel = getVel;
    this.getAccel = getAccel;
    this.minPow = minPow;
    this.maxPow = maxPow;
    this.curPow = minPow;
    this.time = time;
    this.diff = maxPow-minPow;
    this.delta = diff / (time * 50);

    ffValues = new double[3];
    for(int i = 0; i < ffValues.length;i++){
      ffValues[0] = 0;
    }

    SmartDashboard.putData(this);
  }

  
  @Override
  public void initialize() {
    vel = new ArrayList<Double>();
    accel = new ArrayList<Double>();
    powers = new ArrayList<Double>();


    
  
  }

  @Override
  public void execute() {
    if(curPow >= maxPow && withNegative) delta = -Math.abs(delta);
    curPow+= delta;

    setPower.accept(curPow);
    vel.add(getVel.get());
    accel.add(getAccel.get());
    powers.add(curPow);
  }
  

  @Override
  public void end(boolean interrupted) {
    setPower.accept(0.0);
    ffValues = FeedForward.GetFF(powers, vel, accel);
   
  }
  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleArrayProperty("FF VALUES", ()->ffValues, null);
    builder.addDoubleProperty("CUR POW: ", ()-> curPow, null);

  }


  @Override
  public boolean isFinished() {
    return (withNegative) ? curPow < minPow : curPow >= maxPow;
  }
}
