package frc.robot.chassis.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.chassis.subsystem.Chassis;
import static frc.robot.chassis.SwerveConstants.*;
import static frc.robot.utils.Utils.*;

/**
 * Command responsible for controlling the swerve drive chassis using an Xbox controller
 */
public class DriveCommand extends Command {
  // Subsystem and controller references
  private final Chassis chassis;
  private final CommandXboxController commandXboxController;

  // Alliance color direction multiplier
  private double direction;

  // Flags for drive mode
  private boolean isRed;
  private boolean precisionDrive = false;

  /**
   * Constructor for DriveCommand
   * @param chassis The swerve drive chassis subsystem
   * @param commandXboxController The Xbox controller used for driving
   */
  public DriveCommand(Chassis chassis, CommandXboxController commandXboxController) {
    this.chassis = chassis;
    this.commandXboxController = commandXboxController;
    
    // Require the chassis subsystem
    addRequirements(chassis);
    
    // Toggle precision drive mode when B button is pressed
    commandXboxController.b().onTrue(new InstantCommand(() -> precisionDrive = !precisionDrive));
  }

  @Override
  public void initialize() {
    // Optional initialization logic if needed
  }

  @Override
  public void execute() {
    // Determine alliance color and direction
    isRed = chassis.isRed();
    direction = isRed ? 1 : -1;
    
    // normal drive
    // Apply deadband to controller inputs and multiply by direction
    double joyX = deadband(commandXboxController.getLeftY(), 0.1) * direction;
    double joyY = deadband(commandXboxController.getLeftX(), 0.1) * direction;
    
    // Calculate rotation from trigger axes
    double rot = (deadband(commandXboxController.getRightTriggerAxis(), 0.1)
        - deadband(commandXboxController.getLeftTriggerAxis(), 0.1));

    //turn with right joy stick
    Translation2d stickRight = new Translation2d(commandXboxController.getRightX(), commandXboxController.getRightY());
    Rotation2d angle = stickRight.getAngle();
    

    // Calculate velocities with squared inputs for more precise control
    double velX = Math.pow(joyX, 2) * MAX_DRIVE_VELOCITY * Math.signum(joyX);
    double velY = Math.pow(joyY, 2) * MAX_DRIVE_VELOCITY * Math.signum(joyY);
    double velRot = Math.pow(rot, 2) * MAX_OMEGA_VELOCITY * Math.signum(rot);

    // Reduce velocities in precision drive mode
    if (precisionDrive) {
      velX /= 4;
      velY /= 4;
      velRot /= 4;
    }
    

    // Create chassis speeds and set velocities
    ChassisSpeeds speeds = new ChassisSpeeds(velX, velY, velRot);
    if (commandXboxController.getRightX() != 0 && commandXboxController.getRightY() != 0){
      chassis.setVelocitiesTurnTo(speeds, angle, 2);
    }
    else{
      chassis.setVelocities(speeds);
    }
    
    
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the chassis when command ends
    chassis.setVelocities(new ChassisSpeeds(0, 0, 0));
  }

  @Override
  public boolean isFinished() {
    // This command runs continuously while the robot is being driven
    return false;
  }
}