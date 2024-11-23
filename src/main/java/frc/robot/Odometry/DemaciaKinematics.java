// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Odometry;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/** Add your docs here. */
public class DemaciaKinematics extends SwerveDriveKinematics {

    Translation2d[] moduleTranslation;
    double maxVelocity = 3.8;

    double maxForwardAccel = -1;

    public DemaciaKinematics(Translation2d... moduleTranslationsMeters){
        super(moduleTranslationsMeters);
        this.moduleTranslation = moduleTranslationsMeters;
    }
    public Translation2d[] getModulesTranslation()
    {
        return this.moduleTranslation;
    }
    
   @Override
  public Twist2d toTwist2d(SwerveDriveWheelPositions start, SwerveDriveWheelPositions end) {
    
    SwerveModulePosition[] newPositions = new SwerveModulePosition[start.positions.length];
    for (int i = 0; i < start.positions.length; i++) {
      SwerveModulePosition startModule = start.positions[i];
      SwerveModulePosition endModule = end.positions[i];
      newPositions[i] =
          new SwerveModulePosition(
            endModule.distanceMeters - startModule.distanceMeters,
            (endModule.angle.plus(startModule.angle)).div(2));
    }
    return super.toTwist2d(newPositions);
  }


  @Override
  public SwerveModuleState[] toSwerveModuleStates(ChassisSpeeds chassisSpeeds, Translation2d currentVelocity){

    
    SwerveModuleState[] wantedModuleStates = new SwerveModuleState[moduleTranslation.length];

    double factor = 1;
    Translation2d finalWantedVel = new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
    
    
    Translation2d velocityVector = new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
    for(int i = 0; i< wantedModuleStates.length; i++){

      Translation2d rotationVelocity = getRotationVelocity(chassisSpeeds.omegaRadiansPerSecond, moduleTranslation[i]);
      Translation2d moduleVel = velocityVector.plus(rotationVelocity);

      if((moduleVel.getNorm() / maxVelocity) < factor) factor = moduleVel.getNorm() / maxVelocity;
        
      wantedModuleStates[i] = new SwerveModuleState(moduleVel.getNorm(), moduleVel.getAngle());
    }
    factorVelocities(wantedModuleStates, factor);
    return wantedModuleStates;
  }
  private Translation2d getRotationVelocity(double omegaRadians, Translation2d moduleRadius){

    return new Translation2d(omegaRadians * moduleRadius.getNorm(),
      moduleRadius.rotateBy(Rotation2d.fromDegrees(90*Math.signum(omegaRadians))).getAngle());

  }
  
  private void factorVelocities(SwerveModuleState[] arr, double factor){
    for(int i = 0; i < arr.length; i++){
      arr[i].speedMetersPerSecond = arr[i].speedMetersPerSecond * factor;
    }
  }

  private double getWantedAccel(Translation2d finalWantedVel, Translation2d currentVel){
    double wantedAccel = (finalWantedVel.minus(currentVel).getNorm()) / 0.02;


    return wantedAccel;
    
  }
  private void limitForwardAccel(double accel, double currentVel){
    accel = maxForwardAccel * (1-(currentVel / maxVelocity));
  }

}

