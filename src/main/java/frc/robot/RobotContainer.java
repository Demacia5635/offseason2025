// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.chassis.subsystems.Chassis;


public class RobotContainer {

  public Chassis chassis;
  public RobotContainer() {
    chassis = new Chassis();
    
    configureBindings();
  }


  private void configureBindings() {

  }

 
  public Command getAutonomousCommand() {
    return new RunCommand(()->chassis.setVelocities(new ChassisSpeeds(0,0,0)), chassis);
  }
}
