// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.test;

import java.util.ArrayList;
import java.util.function.Consumer;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.utils.LogManager;

public class getFFDrive extends Command {
  Consumer<Double> setPower;
  Supplier<Double> getVel;
  Supplier<Double> getAccel;
  ArrayList<Double> vel;
  ArrayList<Double> accel;
  ArrayList<Double> powers;
  double minPow;
  double maxPow;
  double curPow;
  double deltaP = 0.1;
  double[] ffValues;
  int direction;

  double currentCycleCount = 0;
  double maxCycleCount = 75;
  boolean withNegative
  

  public getFFDrive(Consumer<Double> setPower, Supplier<Double> getVel, Supplier<Double> getAccel, double minPow, double maxPow, boolean withNegative) {
    this.direction = 1;
    this.setPower = setPower;
    this.getVel = getVel;
    this.getAccel = getAccel;
    this.withNegative = withNegative;
    this.minPow = minPow;
    this.maxPow = maxPow;
    this.curPow = minPow;
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
    if(curPow > maxPow && withNegative) deltaP = -Math.abs(deltaP);
    if(currentCycleCount >= maxCycleCount){
      currentCycleCount = 0;
      curPow+= deltaP;
    }
    setPower.accept(curPow);
    vel.add(getVel.get());
    accel.add(getAccel.get());
    powers.add(curPow);
    currentCycleCount++;
  }
  

  @Override
  public void end(boolean interrupted) {
    setPower.accept(0);
    ffValues = FeedForward.GetFF(powers, vel, accel);
   
  }
  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleArrayProperty("FF VALUES", ()->ffValues, null);
    builder.addDoubleProperty("CUR POW: ", ()-> curPow, null);

  }


  @Override
  public boolean isFinished() {
    return (withNegative) ?  curPow < minPow : curPow > maxPow;
  }
}
