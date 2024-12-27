// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.test;

import java.util.ArrayList;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class getElevatorFF extends Command {

  DoubleConsumer setPower;
  DoubleSupplier getVel;
  DoubleSupplier getAccel;
  ArrayList<Double> vel;
  ArrayList<Double> accel;
  ArrayList<Double> power;
  double minPow;
  double maxPow;
  double curPow;
  double deltaP;
  double mass;
  double[] ffValues;

  double currentCycleCount = 0;
  double maxCycleCount;
  boolean withNegative;

  /** Creates a new getElevatorFF. */
  public getElevatorFF(DoubleConsumer setPower, DoubleSupplier getVelocity, DoubleSupplier getAccel, double maxPow, double minPow, double mass, double cycles) {
    this.setPower = setPower;
    this.getVel = getVelocity;
    this.getAccel = getAccel;
    this.maxPow = maxPow;
    this.minPow = minPow;
    this.mass = mass;
    this.maxCycleCount = cycles;
    this.deltaP = (this.maxPow - this.minPow)/this.maxCycleCount;
    ffValues = new double[4];

    SmartDashboard.putData(this);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    vel = new ArrayList<>();
    accel = new ArrayList<>();
    power = new ArrayList<>();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(currentCycleCount <= maxCycleCount){
      if (curPow >= maxPow && withNegative)
      deltaP = -Math.abs(deltaP);
      curPow += deltaP;

      setPower.accept(curPow);
      vel.add(getVel.getAsDouble());
      accel.add(getAccel.getAsDouble());
      power.add(curPow);
      currentCycleCount++;
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    setPower.accept(0);
    ffValues = FeedForward.ElevatorFF(power, vel, accel, mass);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleArrayProperty("FF VALUES", ()->ffValues, null);
    builder.addDoubleProperty("CUR POW: ", ()-> curPow, null);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (currentCycleCount > maxCycleCount);
  }
}
