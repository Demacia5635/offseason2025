// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision.subsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LatencyCompensation extends SubsystemBase {
  
  private NetworkTable table;
  private Pose2d robotPose;
  
  //Latency (ms)
  private double pipelineLatency; // Pipeline's latency contribution
  private double imageCaptureLatency; // Capture pipeline latency (default 11 ms)
  private double totalLatency; // Total latency
  
  public LatencyCompensation(Pose2d robotPose) {
    this.robotPose = robotPose;
    this.table = NetworkTableInstance.getDefault().getTable("limelight-tag");
  }

  @Override
  public void periodic() {
    // Fetch latency
    pipelineLatency = table.getEntry("tl").getDouble(0);
    imageCaptureLatency = table.getEntry("cl").getDouble(11);
    totalLatency = pipelineLatency + imageCaptureLatency;
  }

  public double getTotalLatency() {
    // Convert to seconds
    return totalLatency / 1000;
  }

  public double getTotalLatencyMS() {
    // Keep in milliseconds
    return totalLatency;
  }

  public double getPipelineLatency(){
    // Returns in milliseconds
    return pipelineLatency;
  }

  public double getImageCaptureLatency(){
    // Returns in milliseconds
    return imageCaptureLatency;
  }

  public Pose2d predictPosition(ChassisSpeeds chassisSpeeds) {
    double latencySeconds = getTotalLatency();

    // Convert robot-relative velocities to field-relative velocities
    Rotation2d robotRotation = robotPose.getRotation();
    double fieldRelativeVx = chassisSpeeds.vxMetersPerSecond * robotRotation.getCos() - chassisSpeeds.vyMetersPerSecond * robotRotation.getSin();
    double fieldRelativeVy = chassisSpeeds.vxMetersPerSecond * robotRotation.getSin() + chassisSpeeds.vyMetersPerSecond * robotRotation.getCos();

    // Calculate predicted X and Y displacements
    double predictedX = robotPose.getX() + fieldRelativeVx * latencySeconds;
    double predictedY = robotPose.getY() + fieldRelativeVy * latencySeconds;

    return new Pose2d(predictedX, predictedY, robotPose.getRotation());
  }
  
  public double predictRotation(ChassisSpeeds chassisSpeeds) {
    double latency = getTotalLatency();
    Rotation2d currentAngle = robotPose.getRotation();

    double predictedRotation = currentAngle.getRadians() + chassisSpeeds.omegaRadiansPerSecond * latency;
    return predictedRotation;
  }
}
