package frc.robot.subsystems.chassis;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
    private double gyro;
    private double lastGyro;

    private DifferentialDriveKinematics kinematics;
    private DifferentialDrivePoseEstimator poseEstimator;

    private Field2d field;

    public Chassis() {
        mLeft = new SimMotor();
        mRight = new SimMotor();

        mLeft.setP(ChassisConstants.MOTOR_KP);
        mRight.setP(ChassisConstants.MOTOR_KP);
        mLeft.setI(ChassisConstants.MOTOR_KI);
        mRight.setI(ChassisConstants.MOTOR_KI);
        mLeft.setD(ChassisConstants.MOTOR_KD);
        mRight.setD(ChassisConstants.MOTOR_KD);

        kinematics = new DifferentialDriveKinematics(ChassisConstants.TRACK_WIDTH);
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
        applyRotation();

        poseEstimator.update(Rotation2d.fromRadians(gyro), new DifferentialDriveWheelPositions(mLeft.getDistancePassed(), mRight.getDistancePassed()));
        field.setRobotPose(poseEstimator.getEstimatedPosition());
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("left velocity", mLeft::getVelocity, null);
        builder.addDoubleProperty("right velocity", mRight::getVelocity, null);
        builder.addDoubleProperty("gyro", () -> gyro, null);
        builder.addDoubleProperty("gyro velocity", () -> (gyro - lastGyro) / 0.02, null);
    }

    private void applyRotation() {
        double leftVelocity = mLeft.getVelocity();
        double rightVelocity = mRight.getVelocity();
        double netVelocity = leftVelocity - rightVelocity;
        gyro += netVelocity * 0.02;
    }
}