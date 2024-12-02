
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.chassis.commands.DriveCommand;
import frc.robot.chassis.subsystems.Chassis;
import frc.robot.utils.LogManager;
import frc.robot.utils.Utils;

import static frc.robot.Constants.*;

/** The container for the robot. Contains subsystems, OI devices, and commands. */
public class RobotContainer implements Sendable{
  public static RobotContainer robotContainer;
  public static Boolean isRed = false;
  CommandXboxController driverController;
  public LogManager logManager = new LogManager();

  public static Chassis chassis;
  
  public Command resetOdometry;
  public DriveCommand driveCommand;
  
  Pigeon2 gyro;
  Field2d field;



  public RobotContainer() {

    robotContainer = this;
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
    driverController = new CommandXboxController(DRIVER_CONTROLLER_PORT);
  
    chassis = new Chassis();
    resetOdometry = new InstantCommand(()-> chassis.setOdometryToForward())
                        .ignoringDisable(true);
    driveCommand = new DriveCommand(chassis, driverController);
    chassis.setDefaultCommand(driveCommand);
  
    SmartDashboard.putData("RobotContainer", this);
    SmartDashboard.putData("reset gyro", new InstantCommand(()->gyro.setYaw(0)));


    configureBindings();
  }


  private void configureBindings() {
    driverController.povRight().onTrue(new InstantCommand(()->driveCommand.setPrecision()));
    }
    
   
  public void isRed(boolean isRed) {
    RobotContainer.isRed = isRed;
  }
  public boolean isRed() {
    return isRed;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addBooleanProperty("is Red",this::isRed, this::isRed);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new RunCommand(()-> chassis.setVelocities(new ChassisSpeeds(0,0,1)), chassis);
  }

}