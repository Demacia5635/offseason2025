// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision.subsystem;

import java.lang.reflect.Field;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.vision.VisionConstants.*;

public class Note extends SubsystemBase {
  /** */
  private NetworkTable table;
  private NetworkTableEntry tx;
  private NetworkTableEntry tv;
  private NetworkTableEntry ty;
  private NetworkTableEntry tid;


  private double noteYaw;
  private double notePitch;
  private boolean seeNote = false;
  private Pigeon2 gyro;
  Field2d field;

  public Note(Pigeon2 gyro) {
    this.gyro = gyro;
    table = NetworkTableInstance.getDefault().getTable(TAG_TABLE);
    tv = table.getEntry("tv");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    tid = table.getEntry("tid");
    field = new Field2d();
  }

  @Override
  public void periodic() {
    if(tv.getDouble(0) != 0){
      noteYaw = ty.getDouble(0);
      notePitch = tx.getDouble(0);
    }
  }
  public double GetDistFromCameraToNote(){
    double angle = notePitch;
    double noteDist = (NOTE_CAM_HIGHT) / (Math.tan(Math.toRadians(angle)));
    return noteDist;
  }
}
