package frc.robot.subsystems.chassis;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.subsystems.chassis.ChassisConstants.SwerveModuleConfigs;
import frc.robot.utils.Cancoder;
import frc.robot.utils.TalonMotor;

public class SwerveModule {
    private TalonMotor steerMotor;
    private TalonMotor driveMotor;
    private Cancoder cancoder;

    public SwerveModule(SwerveModuleConfigs configs) {
        steerMotor = new TalonMotor(configs.STEER_CONFIG);
        driveMotor = new TalonMotor(configs.DRIVE_CONFIG);
        cancoder = new Cancoder(configs.CANCODER_CONFIG);

        steerMotor.setPosition(getAbsoluteAngle() - configs.STEER_OFFSET);
    }


    public void setSteerPower(double power) {
        steerMotor.set(power);
    }

    public double getAbsoluteAngle() {
        return cancoder.getAbsPositionRadians();
    }

    public void setDrivePower(double power) {
        driveMotor.set(power);
    }

    public void setSteerVelocity(double velocityRadsPerSecond) {
        steerMotor.setVelocity(velocityRadsPerSecond);
    }

    public void setDriveVelocity(double velocityMetersPerSecond) {
        driveMotor.setVelocity(velocityMetersPerSecond);
    }

    public void setSteerPosition(double positionRadians) {
        steerMotor.setMotionMagic(positionRadians);
    }

    public Rotation2d getSteerAngle() {
        return Rotation2d.fromRadians(steerMotor.getCurrentPosition());
    }

    public void setState(SwerveModuleState state) {
        state = SwerveModuleState.optimize(state, getSteerAngle());
        setDriveVelocity(state.speedMetersPerSecond);
        setSteerPosition(state.angle.getRadians());
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(driveMotor.getCurrentPosition(), Rotation2d.fromRadians(steerMotor.getCurrentPosition()));
    }
}
