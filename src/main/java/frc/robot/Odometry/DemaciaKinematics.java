// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Odometry;

import java.lang.module.ModuleDescriptor;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.LogManager;

/** Add your docs here. */
public class DemaciaKinematics extends SwerveDriveKinematics implements Sendable {

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

  /*public SwerveModuleState[] toSwerveModuleStatesWithAccel(ChassisSpeeds chassisSpeeds, Pose2d curPose, Translation2d currentVelocity, Rotation2d robotAngle, SwerveModuleState[] curModuleStates){
    Translation2d nextVel = getNextWantedVel(currentVelocity,
     new Translation2d(chassisSpeeds.vxMetersPerSecond,chassisSpeeds.vyMetersPerSecond));
    
    return toSwerveModuleStates(new ChassisSpeeds(nextVel.getX(),nextVel.getY(),
    chassisSpeeds.omegaRadiansPerSecond), curPose, curModuleStates);
  }*/
  
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


  public SwerveModuleState[] toSwerveModuleStates(ChassisSpeeds chassisSpeeds, Pose2d curPose, Translation2d currentVelocity, Rotation2d robotAngle, SwerveModuleState[] curModuleStates){

    SwerveModuleState[] wantedModuleStates = new SwerveModuleState[moduleTranslation.length];

    double factor = 1;  
    Translation2d velocityVector = new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);

    for(int i = 0; i < wantedModuleStates.length; i++){
      Translation2d rotationVelocity = new Translation2d(chassisSpeeds.omegaRadiansPerSecond 
        * moduleTranslation[i].getNorm(),
        moduleTranslation[i].rotateBy(Rotation2d.fromDegrees(
        90)).getAngle());

      if(Math.abs(velocityVector.getNorm()) <= 0.1 && Math.abs(rotationVelocity.getNorm()) <= 0.1) wantedModuleStates[i] = new SwerveModuleState(0, curModuleStates[i].angle);
      else if(Math.abs(chassisSpeeds.omegaRadiansPerSecond) <= 0.1) wantedModuleStates[i] = new SwerveModuleState(velocityVector.getNorm(), velocityVector.getAngle());
      else if(velocityVector.getNorm() <= 0.1) wantedModuleStates[i] = new SwerveModuleState(rotationVelocity.getNorm(), rotationVelocity.getAngle());
      


      else{
        
        
        wantedModuleStates[i] = calcModuleState(curPose,chassisSpeeds, curModuleStates, i);
      } 
      /*if(Math.abs(delta.getDegrees())<=4
      && velocityVector.getNorm() > 0.1 && rotationVelocity.getNorm() > 0.1){
        wantedModuleStates[i].angle = wantedModuleStates[i].angle.minus(delta);
      }*/
        
      
    }
    factorVelocities(wantedModuleStates, factor);
    return wantedModuleStates;
  }
  private Pose2d getEstimatedNextPose(Pose2d curPose, ChassisSpeeds wantedSpeeds){
    return new Pose2d(curPose.getX() + (wantedSpeeds.vxMetersPerSecond * 0.02),
     curPose.getY() + (wantedSpeeds.vyMetersPerSecond * 0.02),
      curPose.getRotation().plus(Rotation2d.fromRadians(wantedSpeeds.omegaRadiansPerSecond * 0.02)));

  }

  private SwerveModuleState calcModuleState(Pose2d curPose, ChassisSpeeds wantedSpeeds, SwerveModuleState[] curModuleStates, int i){
    Pose2d estimatedPose = new Pose2d(wantedSpeeds.vxMetersPerSecond * 0.02, wantedSpeeds.vyMetersPerSecond * 0.02, Rotation2d.fromRadians(wantedSpeeds.omegaRadiansPerSecond * 0.02));
    LogManager.log("Estiamted Pose: " + estimatedPose);
    LogManager.log("VELOCITY: " + wantedSpeeds.vxMetersPerSecond * 0.02);

    Translation2d estimatedModulePos = estimatedPose.getTranslation().plus(moduleTranslation[i].rotateBy(estimatedPose.getRotation()));
    Translation2d moduleDiff = estimatedModulePos.minus(moduleTranslation[i]);
    Rotation2d alpha = curModuleStates[i].angle.minus(moduleDiff.getAngle());
    LogManager.log("alpha: " + alpha);
    Rotation2d wantedAngle = curModuleStates[i].angle.plus(alpha.times(2));
  
    double radius = moduleDiff.getNorm() / (2 * Math.sin(alpha.getRadians())); // using that alpha cant be 0 based on the prev checks
    double wantedVelocity = (2*alpha.getRadians() * radius) / 0.02;
 
    return new SwerveModuleState(wantedVelocity, wantedAngle);
    // Pose2d estimatedWantedPose = getEstimatedNextPose(curPose, wantedSpeeds);
    // Translation2d curModulePos = curPose.getTranslation().plus(moduleTranslation[i].rotateBy(curPose.getRotation()));
    // Translation2d nextModulePos = estimatedWantedPose.getTranslation().plus(moduleTranslation[i].rotateBy(estimatedWantedPose.getRotation()));
    // Translation2d moduleDiffPos = nextModulePos.minus(curModulePos);
    // Rotation2d alpha = curModuleStates[i].angle.minus(moduleDiffPos.getAngle());

    // double radius = moduleDiffPos.getNorm() / (2 * Math.sin(alpha.getRadians())); // using that alpha cant be 0 based on the prev checks
    // double wantedVelocity = (alpha.getRadians() * radius) / 0.02;
    // Rotation2d wantedAngle = curModuleStates[i].angle.plus(alpha.times(2));

    // return new SwerveModuleState(wantedVelocity, wantedAngle);


    //Ron tries to fix above code
    /*Pose2d estimatedWantedPose = getEstimatedNextPose(curPose, wantedSpeeds);
    Translation2d curModulePos = curPose.getTranslation().plus(moduleTranslation[i].rotateBy(curPose.getRotation()));
    Translation2d nextModulePos = estimatedWantedPose.getTranslation().plus(moduleTranslation[i].rotateBy(estimatedWantedPose.getRotation()));
    LogManager.log("cur pose: " + curPose);
    LogManager.log("next pose: " + estimatedWantedPose);

    Translation2d moduleDiffPos = nextModulePos.minus(curModulePos);
    //Added rotation of the module angle so it is field relative and not self relative
    Rotation2d alpha = curModuleStates[i].angle.plus(curPose.getRotation()).minus(moduleDiffPos.getAngle());
    
    double radius = moduleDiffPos.getNorm() / (2 * Math.sin(alpha.getRadians())); // using that alpha cant be 0 based on the prev checks
    // double circle_circumfrence = 2 * Math.PI * radius;
    // double partial_arch = (2 * alpha.getDegrees() / 360) * circle_circumfrence;
    double wantedVelocity = (2*alpha.getRadians() * radius) / 0.02;
    LogManager.log(i + " vel " + wantedVelocity + " alpha Radians " + alpha.getRadians() + " radius " + radius);
    Rotation2d wantedAngle = curModuleStates[i].angle.plus(alpha.times(2));
    LogManager.log(i + " angle" + wantedAngle);
    LogManager.log("in the function");

    return new SwerveModuleState(wantedVelocity, wantedAngle);
     */ 

  
   
  }

  private void factorVelocities(SwerveModuleState[] arr, double factor){
    for(int i = 0; i < arr.length; i++){
      arr[i].speedMetersPerSecond = arr[i].speedMetersPerSecond * factor;
    }
  }

  public void initSendable(SendableBuilder builder) {

  }


  
}

