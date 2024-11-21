package frc.robot.chassis.subsystem;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.chassis.SwerveConstants.SwerveModuleConstants;
import frc.robot.utils.Cancoder;
import frc.robot.utils.TalonMotor;

/**
 * Represents a single swerve drive module with drive and steer motors
 */
public class SwerveModule extends SubsystemBase {
    // Motor and sensor components
    private final TalonMotor driveMotor;
    private final TalonMotor steerMotor;
    private final Cancoder canCoder;
    
    // Angle offset for precise positioning
    private final double angleOffset;
    
    // Parent chassis reference
    private final Chassis chassis;
    
    // Module name for identification
    private final String moduleName;

    /**
     * Constructor for SwerveModule
     * @param constants Module-specific configuration constants
     * @param chassis Parent chassis subsystem
     */
    public SwerveModule(SwerveModuleConstants constants, Chassis chassis) {
        this.chassis = chassis;
        this.moduleName = constants.name;

        // Initialize motors and sensors
        driveMotor = new TalonMotor(constants.driveConfig);
        steerMotor = new TalonMotor(constants.steerConfig);
        canCoder = new Cancoder(constants.cancoderConfig);

        // Set angle offset
        angleOffset = constants.steerOffset;
        
        // Initialize motor positions
        initializeMotorPositions();

        // Add to SmartDashboard for debugging
        SmartDashboard.putData(moduleName, this);
    }

    /**
     * Initialize motor positions based on absolute encoder
     */
    private void initializeMotorPositions() {
        driveMotor.setPosition(0);
        steerMotor.setPosition(getAbsoluteAngle() + angleOffset);
    }

    @Override
    public void periodic() {
        // Optional periodic update logic
        updateSmartDashboard();
    }

    /**
     * Get absolute angle from CANCoder
     * @return Absolute angle in radians
     */
    public double getAbsoluteAngle() {
        return canCoder.getAbsPositionRadians();
    }

    /**
     * Get current steer motor angle
     * @return Steer angle in radians
     */
    public double getSteerAngle() {
        return steerMotor.getCurrentPosition();
    }

    /**
     * Get steer angle as Rotation2d
     * @return Steer angle as Rotation2d
     */
    public Rotation2d getSteerAngleRotation2D() {
        return Rotation2d.fromRadians(getSteerAngle());
    }

    /**
     * Get current drive motor velocity
     * @return Velocity in meters per second
     */
    public double getVelocity() {
        return driveMotor.getCurrentVelocity();
    }

    /**
     * Set steer motor angle
     * @param pos Desired angle as Rotation2d
     */
    public void setSteerAngle(Rotation2d pos) {
        steerMotor.setMotionMagic(pos.getRadians());
    }

    /**
     * Set drive motor velocity
     * @param velocity Desired velocity in meters per second
     */
    public void setVelocity(double velocity) {
        driveMotor.setVelocity(velocity);
    }

    /**
     * Get current module state
     * @return SwerveModuleState with velocity and angle
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocity(), getSteerAngleRotation2D());
    }

    /**
     * Set module state with angle optimization
     * @param state Desired module state
     */
    public void setState(SwerveModuleState state) {
        // Optimize module state to minimize rotation
        SwerveModuleState optimized = SwerveModuleState.optimize(state, getSteerAngleRotation2D());
        
        // Set steer angle and velocity
        setSteerAngle(optimized.angle);
        setVelocity(optimized.speedMetersPerSecond);
    }

    /**
     * Get current module position
     * @return SwerveModulePosition with distance and angle
     */
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(
            driveMotor.getCurrentPosition(), 
            getSteerAngleRotation2D()
        );
    }

    /**
     * Update SmartDashboard with module telemetry
     */
    private void updateSmartDashboard() {
        SmartDashboard.putNumber(moduleName + " Absolute Angle", getAbsoluteAngle());
        SmartDashboard.putNumber(moduleName + " Steer Angle", getSteerAngle());
        SmartDashboard.putNumber(moduleName + " Velocity", getVelocity());
    }
}