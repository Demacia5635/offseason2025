package frc.robot.chassis.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.chassis.subsystems.Chassis;
import frc.robot.utils.LogManager;

import static frc.robot.chassis.ChassisConstants.*;
import static frc.robot.utils.Utils.deadband;

public class DriveCommand extends Command  implements Sendable{
  private final Chassis chassis;
  private final CommandXboxController commandXboxController;

  private double direction;

  private boolean isRed;
  private boolean precisionDrive = false;


  public DriveCommand(Chassis chassis, CommandXboxController commandXboxController) {
    this.chassis = chassis;
    
    this.commandXboxController = commandXboxController;
    
    addRequirements(chassis);
    SmartDashboard.putData(this);
  }

  public void setPrecision(){
    this.precisionDrive = !precisionDrive;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    isRed = chassis.isRed();
    direction = isRed ? 1 : -1;

    double joyX = deadband(commandXboxController.getLeftY(), 0.13) * direction;
    double joyY = deadband(commandXboxController.getLeftX(), 0.13) * direction;
    double rot = (deadband(commandXboxController.getRightTriggerAxis(), 0.003)
        - deadband(commandXboxController.getLeftTriggerAxis(), 0.003));

    double velX = Math.pow(joyX, 2) * MAX_DRIVE_VELOCITY * Math.signum(joyX);
    double velY = Math.pow(joyY, 2) * MAX_DRIVE_VELOCITY * Math.signum(joyY);
    double velRot = rot * MAX_OMEGA_VELOCITY;


    if (precisionDrive) {
      velX /= 4;
      velY /= 4;
      velRot /= 4;
    }
    ChassisSpeeds speeds = new ChassisSpeeds(velX, velY, velRot);

    chassis.setVelocities(speeds);

  }

  @Override
  public void initSendable(SendableBuilder builder) {
  }

  @Override
  public void end(boolean interrupted) {

    chassis.stop();
  }
}
