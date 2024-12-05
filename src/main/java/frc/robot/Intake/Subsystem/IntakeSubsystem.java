// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Intake.Subsystem;

import javax.print.FlavorException;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Intake.IntakeConstants;
import frc.robot.Intake.IntakeConstants.NotePosition;
import frc.robot.Intake.IntakeConstants.idMotors;
import frc.robot.utils.TalonConfig;
import frc.robot.utils.TalonMotor;

public class IntakeSubsystem extends SubsystemBase {
  /** motor floor to intake */
  TalonMotor motorFeed;
  /**motor intake to shooter */
  TalonMotor motorMove;
  /** Config for motor floor to intake */
  TalonConfig configFeed;
  /** Config for intake to shooter */
  TalonConfig configMove;

  /** IR Sensor */
  //public AnalogInput analogIRSenor;

  /** The position of the note */
  public NotePosition currentPosition;

    /** If the note is the intake */
    public boolean isNoteInIntake = false;

  /**Configs the motors, ir sensor and the Configs of the motors */
  public IntakeSubsystem() {
    //analogIRSenor = new AnalogInput(10);
    configFeed = new TalonConfig(idMotors.idMotorfeed, idMotors.CANBUS, "motorFeed");
    configFeed.withBrake(true);
    configFeed.withInvert(false);
    motorFeed = new TalonMotor(configFeed);

    configMove = new TalonConfig(idMotors.idMotorIntakeToShooter, idMotors.CANBUS, "motorMove");
    configMove.withBrake(true);
    configMove.withInvert(false);
    motorMove = new TalonMotor(configMove);

    currentPosition = NotePosition.NO_NOTE;
    SmartDashboard.putData("intake", this);
  }
    /**
   * Checks if the note is in the motorFeed
   * @return is the amper is high for the motor motorFeed 
   */
  public boolean AmperHighMotorPickUp(){
    return motorFeed.getSupplyCurrent().getValue() >= IntakeConstants.NOTE_AMPER;
  }
    public boolean AmperHighMotorPickUp2(){
    return motorFeed.getSupplyCurrent().getValue() >= IntakeConstants.NOTE_AMPER2;
  }

  public void swichMotorDirection(){
    motorMove.setInverted(!motorMove.getInverted() ? true : false);
  }

  
      /**
   * Checks if the note is in the motorMove
   * @return is the amper is high for the motorMov1
   */
  public boolean AmperHighMotorMove(){
    return motorMove.getSupplyCurrent().getValue() <= IntakeConstants.NOTE_AMPER_MOVE;
  }

  
  /** Gives power to both motors */
  public void setPowerMotors(double power){
    if(power >0.35)
    {
      power = 0.35;
    }
    motorFeed.setDuty(power);
    motorMove.setDuty(power);
  }

  /** Gives power to  motorMove */
  public void setPowerMotorMove(double power){
    if(power >0.35)
    {
      power = 0.35;
    }
    motorMove.setDuty(power);
  }

  /** Gives power to motorFeed */
  public void setPowerMotorFeed(double power){
    if(power >0.35)
    {
      power = 0.35;
    }
    motorFeed.setDuty(power);
  }
  

  /**this func center the note */
  public void centerNote(){

  }



  /**
   * Gives me the amper of the motor move than i use it for the shuffle-board
   * @return the amper of motor move
   */
  public double getCurrentOfMotorMove(){
    return motorMove.getSupplyCurrent().getValueAsDouble();
  }

  public double getCurrentOfPickUp(){
    return motorFeed.getSupplyCurrent().getValueAsDouble();
  }

  /**
   * Gives me the name of the currentPosition and tha i use it in the shuffle-board
   * @return the name of the currentPosition
   */
  public String getcurrentPosition(){
    return currentPosition.toString();
  }

  /**
   * Gives me the voltage of the ir-sensor so i can use it in the shuffle-board
   * @return the voltage of the sensor
   */
  /*public double getVoltageSensor(){
    return analogIRSenor.getVoltage();
  }*/

  /**
   * The func that I send to the shuffle-board 
   */
  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.addDoubleProperty("Amper motor Move",this::getCurrentOfMotorMove,null);
    builder.addDoubleProperty("Amper of motor PickUp",this::getCurrentOfPickUp,null);
    builder.addStringProperty("Current Position",this::getcurrentPosition,null);
    //builder.addDoubleProperty("voltage sensor",this::getVoltageSensor,null);
  }
}