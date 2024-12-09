package frc.robot.subsystems;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.simulation.SimMotor;

public class Chassis extends SubsystemBase {
    private SimMotor mLeft;
    private SimMotor mRight;
    private double mass;
    private double gyro;
    private double lastGyro;

    private DifferentialDriveKinematics kinematics;
    private DifferentialDrivePoseEstimator poseEstimator;

    private Field2d field;

    public Chassis(float mass) {
        mLeft = new SimMotor();
        mRight = new SimMotor();
        this.mass = mass;

        mLeft.setP(0.07);
        mRight.setP(0.07);

        kinematics = new DifferentialDriveKinematics(1);
        poseEstimator = new DifferentialDrivePoseEstimator(kinematics, new Rotation2d(), 0, 0, new Pose2d());

        field = new Field2d();
        SmartDashboard.putData(this);
        SmartDashboard.putData(field);
    }

    public void setVelocity(ChassisSpeeds speeds) {
        var wSpeeds = kinematics.toWheelSpeeds(speeds);
        mLeft.setVelocity(wSpeeds.leftMetersPerSecond);
        mRight.setVelocity(wSpeeds.rightMetersPerSecond);
    }

    @Override
    public void periodic() {
        lastGyro = gyro;
        applyTorque();

        poseEstimator.update(Rotation2d.fromRadians(gyro), new DifferentialDriveWheelPositions(mLeft.getDistancePassed(), mRight.getDistancePassed()));
        field.setRobotPose(poseEstimator.getEstimatedPosition());
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("left velocity", mLeft::getVelocity, null);
        builder.addDoubleProperty("right velocity", mRight::getVelocity, null);
        builder.addDoubleProperty("gyro", () -> gyro, null);
        builder.addDoubleProperty("gyro velocity", () -> (gyro - lastGyro) * 0.02, null);
    }

    private void applyTorque() {
        Translation2d leftForce = new Translation2d(-0.5, mass * mLeft.getVelocity());
        Translation2d rightForce = new Translation2d(0.5, mass * mRight.getVelocity());
        Translation2d netForce = leftForce.minus(rightForce);
        double torque = 0.5 * netForce.getNorm() * netForce.getAngle().getSin();
        double accel = torque / mass;
        gyro += accel * 0.02;
    }
}