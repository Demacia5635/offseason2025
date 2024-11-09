// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision.subsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.vision.VisionConstants.*;

import com.ctre.phoenix6.hardware.Pigeon2;

/**
 * Subsystem for processing AprilTag vision data and calculating robot position.
 * Uses Limelight camera data and a Pigeon2 gyro to determine robot position on field.
 */
public class Tag extends SubsystemBase {

  // NetworkTables communication
  private NetworkTable table;
  private NetworkTableEntry tvEntry;  // Whether target is visible (0 or 1)
  private NetworkTableEntry txEntry;  // Horizontal offset from crosshair to target (-31.65 to 31.65 degrees)
  private NetworkTableEntry tyEntry;  // Vertical offset from crosshair to target (-24.85 to 24.85 degrees)
  private NetworkTableEntry tidEntry; // ID of currently tracked AprilTag

  // Vision processing variables
  private double camToTagYaw;   // Horizontal angle to tag
  private double camToTagPitch; // Vertical angle to tag
  private double id;           // Current tag ID
  private double height;       // Height of current tag
  
  private Pigeon2 gyro;       // Gyroscope for robot orientation
  private Field2d field;      // Field visualization for debugging

  /**
   * Creates a new Tag subsystem
   * @param gyro Pigeon2 gyroscope for determining robot orientation
   */
  public Tag(Pigeon2 gyro) {
    this.gyro = gyro;

        // Initialize NetworkTables connections
        table = NetworkTableInstance.getDefault().getTable(TAG_TABLE);
        tvEntry = table.getEntry("tv");
        txEntry = table.getEntry("tx");
        tyEntry = table.getEntry("ty");
        tidEntry = table.getEntry("tid");

        field = new Field2d();

  }

  @Override
    public void periodic() {
        // Process vision data if a target is visible
        if(tvEntry.getDouble(0) != 0) {
            // Get latest measurements from Limelight
            camToTagYaw = txEntry.getDouble(0);
            camToTagPitch = tyEntry.getDouble(0);
            id = tidEntry.getDouble(0);

            // Calculate robot position if valid tag ID is detected
            if(id > 0 && id < TAG_ANGLE.length) {
                Pose2d pose = new Pose2d(getOriginToRobot(), getGyroRotation2d());
                field.setRobotPose(pose);
            }
            // set smart crop to tag 
        }
        else{
          //set smart crop to full screen
        }
    }
  

  /**
     * Calculates straight-line distance from camera to AprilTag
     * Uses trigonometry with known tag height and camera angle
     * @return Distance in meters
     */
    public double GetDistFromCamera() {
      double alpha = camToTagPitch + TAG_CAM_ANGLE;
      double dist = (Math.abs(height - TAG_CAM_HIGHT)) / (Math.tan(Math.toRadians(alpha)));
      return dist;
  }

  /**
     * Calculates vector from robot center to detected AprilTag
     * Accounts for camera offset from robot center
     * @return Translation2d representing vector to tag
     */
    public Translation2d getRobotToTag() {
      // Convert camera measurements to vector
      Translation2d cameraToTag = new Translation2d(GetDistFromCamera(), 
          Rotation2d.fromDegrees(camToTagYaw));
      // Add camera offset to get robot center to tag vector
      Translation2d robotToTag = ROBOT_TO_CAM.plus(cameraToTag);
      return robotToTag;
  }

  /**
     * Calculates robot position relative to field origin
     * Uses known AprilTag position and measured vector to tag
     * @return Translation2d representing robot position on field
     */
    public Translation2d getOriginToRobot() {
      Translation2d originToRobot;
      Translation2d origintoTag = O_TO_TAG[(int)this.id];
      Rotation2d tagAngle = TAG_ANGLE[(int)this.id];

      if(origintoTag != null) {
          // Get vector from robot to tag
          Translation2d robotToTag = getRobotToTag();
          Rotation2d robotToTagYaw = robotToTag.getAngle();
          
          // Convert to field coordinates using gyro
          Rotation2d robotToTagYawFC = robotToTagYaw.plus(getGyroRotation2d());
          
          // Calculate angle from tag to robot in field coordinates
          Rotation2d tagToRobotYawFC = tagAngle.minus(robotToTagYawFC)
              .rotateBy(Rotation2d.fromDegrees(180)).unaryMinus();
          
          // Calculate final robot position using tag position and vector
          double robotToTagDist = robotToTag.getNorm();
          originToRobot = origintoTag.plus(
              new Translation2d(robotToTagDist, tagToRobotYawFC));
          return originToRobot;
      }
      return new Translation2d();
  }

  /**
     * Gets current robot rotation from gyroscope
     * @return Rotation2d representing robot orientation
     */
    public Rotation2d getGyroRotation2d() {
      return Rotation2d.fromDegrees(gyro.getAngle());
  }

}
