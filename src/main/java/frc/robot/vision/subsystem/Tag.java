// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision.subsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.vision.VisionConstants.*;

import java.util.List;
import java.util.function.Supplier;


/**
 * Subsystem for processing AprilTag vision data and calculating robot position.
 * Uses Limelight camera data and a Pigeon2 gyro to determine robot position on field.
 */
public class Tag extends SubsystemBase{

  // NetworkTables communication
  private NetworkTable table;
  private NetworkTableEntry tvEntry;  // Whether target is visible (0 or 1)
  private NetworkTableEntry txEntry;  // Horizontal offset from crosshair to target (-31.25 to 31.25 degrees)
  private NetworkTableEntry tyEntry;  // Vertical offset from crosshair to target (-24.45 to 24.45 degrees)
  private NetworkTableEntry tidEntry; // ID of currently tracked AprilTag
  private NetworkTableEntry cropEntry; // crop the image
  private NetworkTableEntry tlEntry;
  private NetworkTableEntry clEntry;

  // Vision processing variables
  private double camToTagYaw;   // Horizontal angle to tag
  private double camToTagPitch; // Vertical angle to tag
  private double id;           // Current tag ID
  private double height;       // Height of current tag
  private double pipelineLatency; // Pipeline's latency contribution
  private double imageCaptureLatency; // Capture pipeline latency (default 11 ms)
  private double totalLatency; // Total latency
  private double dist;
  
  private Supplier<Rotation2d> getRobotAngle;       // Gyroscope for robot orientation
  private Field2d field;      // Field visualization for debugging

  private Translation2d origin = new Translation2d(0,0);

  /**
   * Creates a new Tag subsystem
   * @param robot_angle_from_pose Pigeon2 gyroscope for determining robot orientation
   */
  public Tag(Supplier<Rotation2d> robot_angle_from_pose) {
    this.getRobotAngle = robot_angle_from_pose;

    // Initialize NetworkTables connections
    table = NetworkTableInstance.getDefault().getTable(TAG_TABLE);
    tvEntry = table.getEntry("tv");
    txEntry = table.getEntry("tx");
    tyEntry = table.getEntry("ty");
    tidEntry = table.getEntry("tid");
    cropEntry = table.getEntry("crop");
    tlEntry = table.getEntry("tl");
    clEntry = table.getEntry("cl");

    field = new Field2d();
    SmartDashboard.putData("Tag", this);
    SmartDashboard.putData("field-tag",field);
  }
    /**
   * Creates a new Tag subsystem for tests
   * @param robot_angle_from_pose Pigeon2 gyroscope for determining robot orientation
   * @parm dist the dist from tag
   * @parm camToTagYaw Yaw to tag
   * @Parm id the id of the thag
   */
  public Tag (Supplier<Rotation2d> robot_angle_from_pose, double dist, double camToTagYaw, double id){
    this.getRobotAngle = robot_angle_from_pose;
    this.dist = dist;
    this.camToTagYaw = -camToTagYaw;
    this.id = id;
    
    // Initialize NetworkTables connections
    table = NetworkTableInstance.getDefault().getTable(TAG_TABLE);
    tvEntry = table.getEntry("tv");
    txEntry = table.getEntry("tx");
    tyEntry = table.getEntry("ty");
    tidEntry = table.getEntry("tid");
    cropEntry = table.getEntry("crop");
    tlEntry = table.getEntry("tl");
    clEntry = table.getEntry("cl");

    field = new Field2d();
    SmartDashboard.putData("Tag", this);
    SmartDashboard.putData("field-tag",field);
  }


  @Override
    public void periodic() {
        // Process vision data if a target is visible
        if(tvEntry.getDouble(0) != 0) {
            // Get latest measurements from Limelight
            camToTagYaw = -txEntry.getDouble(0);
            camToTagPitch = tyEntry.getDouble(0);
            id = tidEntry.getDouble(0);
            // Calculate robot position if valid tag ID is detected
            if(id > 0 && id < TAG_ANGLE.length) {
                crop(camToTagYaw, camToTagPitch);
                Pose2d pose = new Pose2d(getOriginToRobot(), getRobotAngle.get());
                field.setRobotPose(pose);




            }
        }
        else{
          cropStop();
        }
    }
  

  /**
     * Calculates straight-line distance from camera to AprilTag
     * Uses trigonometry with known tag height and camera angle
     * @return Distance in meters
     */
    public double GetDistFromCamera() {
      double alpha = camToTagPitch + TAG_CAM_ANGLE;
      dist = (Math.abs(height - TAG_CAM_HIGHT)) / (Math.tan(Math.toRadians(alpha)));
      dist = dist/Math.cos(Math.toRadians(camToTagYaw));
      return dist;
  }

  /**
     * Calculates vector from robot center to detected AprilTag
     * Accounts for camera offset from robot center
     * @return Translation2d representing vector to tag
     */
    public Translation2d getRobotToTagRR() {
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
      field.getObject("Field").setTrajectory(vector(origin, origintoTag));

      height = TAG_HIGHT[(int)this.id];
      if(origintoTag != null) {
          // Get vector from robot to tag
          Translation2d robotToTagRR = getRobotToTagRR();//
          // Rotation2d robotToTagYaw = robotToTagRobotRelativ.getAngle();
          // // Convert to field coordinates using gyro
          // Rotation2d robotToTagYawFC = robotToTagYaw.plus(getRobotAngle.get());
          // // Calculate angle from tag to robot in field coordinates
          // Rotation2d tagToRobotYawFC = Rotation2d.fromDegrees(180)
          //   .minus(robotToTagYawFC);
          
          // // Calculate final robot position using tag position and vector
          // double robotToTagDist = robotToTagRobotRelativ.getNorm();
          Translation2d robotToTagFC = robotToTagRR.rotateBy(getRobotAngle.get());
          originToRobot = origintoTag.plus(robotToTagFC.rotateBy(Rotation2d.fromDegrees(180)));
          //field.getObject("Robot").setTrajectory(vector(origin, originToRobot));
          // //-----------------------------
          // System.out.println("Tag Angle: " + tagAngle.getDegrees());
          // System.out.println("Robot Angle: " + getRobotAngle.get().getDegrees());
          // System.out.println("Robot to Tag Yaw: " + robotToTagYaw.getDegrees());
          // System.out.println("Robot to Tag Yaw FC: " + robotToTagYawFC.getDegrees());
          // System.out.println("Tag to Robot Yaw FC: " + tagToRobotYawFC.getDegrees());
          // //-----------------------------
          return originToRobot;
      }
      return new Translation2d();
      
  }

    public Trajectory vector(Translation2d start, Translation2d end){
      // double x = end.getDistance(start);
      // Rotation2d alpha = Rotation2d.fromDegrees((Math.asin(start.getNorm()*Math.sin(Math.toRadians(start.getAngle().getDegrees()-end.getAngle().getDegrees()))))/x);
      // Rotation2d beta = Rotation2d.fromDegrees(90-alpha.getDegrees());
      return TrajectoryGenerator.generateTrajectory(
            List.of(
              new Pose2d(start, end.getAngle().minus(start.getAngle())),
              new Pose2d(end, end.getAngle().minus(start.getAngle()))),
            new TrajectoryConfig(4.0, 4.0));
    }

    public void crop(double camToTagYaw, double camToTagPitch){
      double YawCrop = camToTagYaw/31.25;
      double PitchCrop = camToTagPitch/24.45;
      double[] crop = {YawCrop-CROP_OFSET,YawCrop+CROP_OFSET,PitchCrop-CROP_OFSET,PitchCrop+CROP_OFSET};
      cropEntry.setDoubleArray(crop);
    }
    public void cropStop(){
      double[] crop = {-1,1,-1, 1};
      cropEntry.setDoubleArray(crop);
    }
    public double getLatency(){
      pipelineLatency = tlEntry.getDouble(0);
      imageCaptureLatency = clEntry.getDouble(0);
      totalLatency = pipelineLatency + imageCaptureLatency;
      return totalLatency;
    }


}

/*https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api
more:
Increasing detector downscale will always increase pipeline framerate. It will decrease effective range, but in some cases this may be negligible.
Reducing exposure will always improve motion-blur resilience. This is actually really easy to observe. This may reduce range.
Reducing the brightness and contrast of the image will generally improve pipeline framerate and reduce range.
Increasing Sensor gain allows you to increase brightness without increasing exposure. 



To track AprilTags:

Change "Pipeline Type" to "Fiducial Markers"
Set "Black Level" to zero
At this point, it is a matter of balancing sensor gain and exposure time. 
You want to be able to see the tags with the smallest exposure possible to minimize motion blur.
This usually calls for a high sensor gain setting.
For simple 2D tracking, it is often advisable to max-out your sensor gain, 
and then increase your exposure from zero until targets are sufficiently tracked.
Make sure the correct family is selected in the "Standard" tab if tracking isn't working.

Cropping removes content from the image for huge performance boosts. Use the NT "crop" key to crop dynamically during matches


latency:

tl  double  The pipeline's latency contribution (ms). Add to "cl" to get total latency.

cl  double  Capture pipeline latency (ms). Time between the end of the exposure of the middle row of the sensor to the beginning of the tracking pipeline.
 */
