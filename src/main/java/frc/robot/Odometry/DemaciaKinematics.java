// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Odometry;

import edu.wpi.first.math.geometry.Pose2d;
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
    double MAX_VELOCITY = 3.8;
    double radius = 0.3;
    double MIN_ANGLE = -1;


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

  public SwerveModuleState[] toSwerveModuleStatesWithAccel(ChassisSpeeds chassisSpeeds, Pose2d curPose, Translation2d currentVelocity, Rotation2d robotAngle, SwerveModuleState[] curModuleStates){
    Translation2d nextVel = getNextWantedVel(currentVelocity,
     new Translation2d(chassisSpeeds.vxMetersPerSecond,chassisSpeeds.vyMetersPerSecond));
    
    return toSwerveModuleStates(new ChassisSpeeds(nextVel.getX(),nextVel.getY(),
    chassisSpeeds.omegaRadiansPerSecond), curPose, curModuleStates);
  }
  
  private Translation2d getNextWantedVel(Translation2d currentVel,Translation2d finalWantedVel){
    Translation2d accel = currentVel.times(-1).plus(finalWantedVel);
    if(accel.getNorm() <= radius) return finalWantedVel;
    Rotation2d alpha = accel.getAngle();
    double nextWantedNorm = Math.sqrt(square(radius) + square(currentVel.getNorm())
     - (currentVel.getNorm() * radius * 2)
      * alpha.getCos()); 
    Rotation2d nextWantedAngle = Rotation2d.fromRadians(Math.asin((nextWantedNorm*alpha.getSin())/radius));
    return new Translation2d(nextWantedNorm,nextWantedAngle);
    
  }



  private double square(double n){
    return Math.pow(n, 2);
  }


  public SwerveModuleState[] toSwerveModuleStates(ChassisSpeeds chassisSpeeds, Pose2d curPose, SwerveModuleState[] curModuleStates){

    SwerveModuleState[] wantedModuleStates = new SwerveModuleState[moduleTranslation.length];

    double factor = 1;  
    
    boolean isRobotRotating = Math.abs(chassisSpeeds.vxMetersPerSecond) >= 0.1;
    boolean isRobotDriving = Math.abs(chassisSpeeds.vyMetersPerSecond) >= 0.1 || Math.abs(chassisSpeeds.omegaRadiansPerSecond) >= 0.1;
    Translation2d driveVector = new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
    

    for(int i = 0; i < wantedModuleStates.length; i++){
      
      if(!isRobotDriving && !isRobotRotating) wantedModuleStates[i] = new SwerveModuleState(0, curModuleStates[i].angle);
      else if(isRobotDriving && !isRobotRotating) wantedModuleStates[i] = new SwerveModuleState(driveVector.getNorm(), driveVector.getAngle());
      else if(!isRobotDriving && isRobotRotating){
        Translation2d rotateVector = moduleTranslation[i].rotateBy(curPose.getRotation().plus(Rotation2d.fromDegrees(90 * Math.signum(chassisSpeeds.omegaRadiansPerSecond))));
        wantedModuleStates[i] = new SwerveModuleState(rotateVector.getNorm(), rotateVector.getAngle());
      }
      

      else{
        Pose2d estimatedPose = getEstimatedNextPose(curPose, chassisSpeeds);
        Translation2d curModulePos = curPose.getTranslation().plus(moduleTranslation[i].rotateBy(curPose.getRotation()));
        Translation2d nextModulePos = moduleTranslation[i].plus(estimatedPose.getTranslation()).rotateBy(estimatedPose.getRotation());
        Translation2d moduleDiffPos = nextModulePos.minus(curModulePos);
        Rotation2d alpha = curModuleStates[i].angle.minus(moduleDiffPos.getAngle());

        double radius = moduleDiffPos.getNorm() / (2 * Math.sin(alpha.getRadians())); // using that alpha cant be 0 based on the prev checks
        double wantedVelocity = (alpha.getRadians() * radius) / 0.02;
        Rotation2d wantedAngle = alpha.times(2).minus(estimatedPose.getRotation().minus(curPose.getRotation()));

        wantedModuleStates[i] = new SwerveModuleState(wantedVelocity, wantedAngle);

      }
      
    }
    factorVelocities(wantedModuleStates, factor);
    return wantedModuleStates;
  }
  private Pose2d getEstimatedNextPose(Pose2d curPose, ChassisSpeeds wantedSpeeds){
    return new Pose2d(curPose.getX() + (wantedSpeeds.vxMetersPerSecond * 0.02),
     curPose.getY() + (wantedSpeeds.vyMetersPerSecond * 0.02),
      curPose.getRotation().plus(Rotation2d.fromRadians(wantedSpeeds.omegaRadiansPerSecond * 0.02)));

  }


  private void factorVelocities(SwerveModuleState[] arr, double factor){
    for(int i = 0; i < arr.length; i++){
      arr[i].speedMetersPerSecond = arr[i].speedMetersPerSecond * factor;
    }
  }
}

