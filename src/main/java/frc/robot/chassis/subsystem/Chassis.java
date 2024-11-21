package frc.robot.chassis.subsystem;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.chassis.SwerveConstants;

/**
 * Represents the swerve drive chassis subsystem for robot movement
 */
public class Chassis extends SubsystemBase {
    // Swerve Modules
    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;

    private SwerveModule[] modules;
    // Kinematics
    private final SwerveDriveKinematics kinematics;

    // Sensors
    private final Pigeon2 gyro;

    /**
     * Constructor for Chassis subsystem
     */
    public Chassis() {
        // Initialize Gyro
        gyro = new Pigeon2(SwerveConstants.GYRO_ID);

        // Create Swerve Modules
        frontLeft = new SwerveModule(SwerveConstants.FRONT_LEFT, this);
        frontRight = new SwerveModule(SwerveConstants.FRONT_RIGHT, this);
        backLeft = new SwerveModule(SwerveConstants.BACK_LEFT, this);
        backRight = new SwerveModule(SwerveConstants.BACK_RIGHT, this);

        modules = new SwerveModule[4];
        modules[0] = frontLeft;
        modules[1] = frontRight;
        modules[2] = backLeft;
        modules[3] = backRight;



        // Initialize Kinematics
        kinematics = SwerveConstants.KINEMATICS;


        SmartDashboard.putData("Swerve Drive", new Sendable() {
          @Override
          public void initSendable(SendableBuilder builder) {
            builder.setSmartDashboardType("SwerveDrive");
        
            builder.addDoubleProperty("Front Left Angle", () -> frontLeft.getAbsoluteAngle(), null);
            builder.addDoubleProperty("Front Left Velocity", () -> frontLeft.getVelocity(), null);
        
            builder.addDoubleProperty("Front Right Angle", () -> frontRight.getAbsoluteAngle(), null);
            builder.addDoubleProperty("Front Right Velocity", () -> frontRight.getVelocity(), null);
        
            builder.addDoubleProperty("Back Left Angle", () -> backLeft.getAbsoluteAngle(), null);
            builder.addDoubleProperty("Back Left Velocity", () -> backLeft.getVelocity(), null);
        
            builder.addDoubleProperty("Back Right Angle", () -> backRight.getAbsoluteAngle(), null);
            builder.addDoubleProperty("Back Right Velocity", () -> backRight.getVelocity(), null);
        
            builder.addDoubleProperty("Robot Angle", () -> getGyroRotation().getRadians(), null);
          }
        });


    }

    @Override
    public void periodic() {

        // Update SmartDashboard with telemetry
        updateSmartDashboard();
    }

    /**
     * Set chassis velocities based on ChassisSpeeds
     * @param chassisSpeeds Desired chassis speeds
     */
    public void setVelocities(ChassisSpeeds chassisSpeeds) {
        // Convert chassis speeds to module states
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, getGyroRotation()));
        
        // Normalize wheel speeds
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, SwerveConstants.MAX_DRIVE_VELOCITY);

        // Set individual module states
        frontLeft.setState(moduleStates[0]);
        frontRight.setState(moduleStates[1]);
        backLeft.setState(moduleStates[2]);
        backRight.setState(moduleStates[3]);
    }


    /**
     * Get gyro rotation
     * @return Current gyro rotation
     */
    public Rotation2d getGyroRotation() {
        return gyro.getRotation2d();
    }

    public void setSteerAngle(double angle){
      for (SwerveModule m : modules) {
        m.setSteerAngle(Rotation2d.fromDegrees(angle));
      }
    }
    /**
     * Determine alliance color
     * @return True if on red alliance, false if blue
     */
    public boolean isRed() {
        return RobotContainer.isRed();
    }

    /**
     * Update SmartDashboard with chassis telemetry
     */
    private void updateSmartDashboard() {
        SmartDashboard.putNumber("Gyro Angle", getGyroRotation().getDegrees());
    }


}