// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Odometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.WheelPositions;

/** Add your docs here. */
public class DemaciaOdometry {
    Pose2d pose;
    Rotation2d gyroOffset;
    private Rotation2d prevAngle;
    private WheelPositions prevWheelPositions;

    public DemaciaOdometry(Rotation2d gyroAngle, WheelPositions wheelPositions, Pose2d initPose){
        this.pose = initPose;
        this.gyroOffset = pose.getRotation().minus(gyroAngle);
        this.prevAngle = gyroAngle;
        this.prevWheelPositions = wheelPositions;
    }
    
    public void resetPosition(Rotation2d gyroAngle, WheelPositions wheelPos, Pose2d poseToReset){
        pose = poseToReset;
        prevAngle = pose.getRotation();
        gyroOffset = pose.getRotation().minus(prevAngle);
        prevWheelPositions = wheelPos;
    }

    public Pose2d getPose(){
        return pose;
    }

    public Pose2d updatePose(Rotation2d gyroAngle, WheelPositions currentWheelPositions){
        
    }

     
    



    private Translation2d[] calcDiff(SwerveModulePosition[] prevWheelPos, SwerveModulePosition[] curWheelPos){
        Translation2d[] diff = new Translation2d[prevWheelPos.length];
        for(int i = 0; i < prevWheelPos.length; i++){
            SwerveModulePosition startModule = prevWheelPos[i];
            SwerveModulePosition endModule = curWheelPos[i];

            double distance = (startModule.distanceMeters - endModule.distanceMeters);
            double angleRad = endModule.angle.plus(startModule.angle).div(2).getRadians();

            diff[i] = new Translation2d(distance - (distance*Math.cos(angleRad)) , distance * Math.sin(angleRad));

        }
        return diff;
    }
}
