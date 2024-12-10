// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision.subsystem;

import java.lang.reflect.Field;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.vision.VisionConstants.*;



/**
 * Subsystem for processing note vision data and and calculating note position to origin.
 * Uses Limelight camera data ,Pigeon2 gyro and the robots posetion to determine note position on field.
 */
public class Note extends SubsystemBase {
  
  // NetworkTables communication
  private NetworkTable table;
  private NetworkTableEntry tx; // Horizontal offset from crosshair to target (-31.65 to 31.65 degrees)
  private NetworkTableEntry tv; // Whether target is visible (0 or 1)
  private NetworkTableEntry ty; // Vertical offset from crosshair to target (-24.85 to 24.85 degrees)

  private Pose2d robotPose2d;
  private double noteYaw;
  private double notePitch;
  private Pigeon2 gyro;
  Field2d field;


    /**
   * Creates a new Tag subsystem
   * @param gyro Pigeon2 gyroscope for determining robot orientation
   * 
   * @param robotPose2d gets the pose of robot to the origin point
   */
  public Note(Pigeon2 gyro, Pose2d robotPose2d) {
    
    this.gyro = gyro;
    this.robotPose2d = robotPose2d;
    // Initialize NetworkTables connections
    table = NetworkTableInstance.getDefault().getTable(TAG_TABLE);
    tv = table.getEntry("tv");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    field = new Field2d();
  }

  @Override
  public void periodic() {
    // Process vision data if a target is visible
    if(tv.getDouble(0) != 0){
      // Get latest measurements from Limelight
      noteYaw = ty.getDouble(0);
      notePitch = tx.getDouble(0);

      Pose2d notePose2d = new Pose2d(getNoteToOrigin(),Rotation2d.fromDegrees(0));//i dont need the degrees of the note
      field.setRobotPose(notePose2d);//sets the note pose on field
    }
  }


  /**
   * Calculates straight-line distance from camera to note
   * Uses trigonometry with known cemra height and camera angle
   * @return Distance in meters
  */
  public double GetDistFromCameraToNote(){
    double angle = notePitch;
    double noteDist = (NOTE_CAM_HIGHT) / (Math.tan(Math.toRadians(angle)));
    return noteDist;
  }

/**
 * Calculates vector from robot center to detected note
 * Accounts for camera offset from robot center
 * @return Translation2d representing vector to note
 */
  public Translation2d getRobotToNote(){
    // Convert camera measurements to vector
    Translation2d cameraToNote = new Translation2d(GetDistFromCameraToNote(),
      Rotation2d.fromDegrees(noteYaw));

    // Add camera offset to get robot center to note vector
    Translation2d robotToNote = ROBOT_TO_CAM_NOTE.plus(cameraToNote);
    return robotToNote;
  }

/**
 * Calculates note position relative to field origin
 * Uses known robot position and measured vector to note
 * @return Translation2d representing note position on field
 */
  public Translation2d getNoteToOrigin(){
    // Get vector from robot to note
    Translation2d robotToNote = getRobotToNote();

    if(robotToNote != null){

      Translation2d originToNote;
      Translation2d originToRobot = robotPose2d.getTranslation();


      // Convert to field coordinates using gyro
      Rotation2d angleRobotToNoteFinal = robotToNote.getAngle().plus(getGyroRotation2d());
      robotToNote = new Translation2d(robotToNote.getNorm(),angleRobotToNoteFinal);

      // Calculate final note position using a vector of robot position and a vector of note to robot position Converted to field coordinates
      originToNote = originToRobot.plus(robotToNote);
      return originToNote;
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
