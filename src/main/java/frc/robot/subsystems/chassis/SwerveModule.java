package frc.robot.subsystems.chassis;

import frc.robot.utils.TalonConfig;
import frc.robot.utils.TalonMotor;

public class SwerveModule {
    private TalonMotor steerMotor;
    private TalonMotor driveMotor;

    public SwerveModule(TalonConfig steerConfig, TalonConfig driveConfig) {
        steerMotor = new TalonMotor(steerConfig);
        driveMotor = new TalonMotor(driveConfig);
    }


    public void setSteerPower(double power) {
        steerMotor.set(power);
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
}
