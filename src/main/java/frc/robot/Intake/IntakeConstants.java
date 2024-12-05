// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Intake;

/** Add your docs here. */
public final class IntakeConstants {
    public static class idMotors{
        public static final String CANBUS = "rio";
        public static final int idMotorfeed = 40;
        public static final int idMotorIntakeToShooter = 41;
        public static final int idIRSensor = 3;
    }
    
      /**The voltege of the sensor when he detects the note */
    public static final double NOTE_VOLTEGE = 3.3;

    //The amper of when the not is fed to the intake
    public static final double NOTE_AMPER = 25.0;
    public static final double NOTE_AMPER2 = 25.0;
    public static final double NOTE_AMPER_MOVE = 2.7;

    /**The amount of time the intake command will auto stop (in miliseconds) */
    public static final double STOP_COMMAND_TIME = 20000;

    /**The amount of time the intake command will stop after takinng note */
    public static final double STOP_AFTER_NOTE = 1.5;

    public static final double TIME_CENTER = 1.5;
    public static final double POWER_TO_CENTER = 0.2;

    /**
    * The current position of the note to power fo the motors
    */
    public enum NotePosition{
        NO_NOTE(0.35, 025),
        FIRST_TOUCH(0.35, 0.25),
        AFTER_SEEING_NOTE(0.35, 0.25);
    
        /** The voltage to the motors */
        public final double pickUpPow;
        public final double movePow;
    
        /**
         * Constractor for enum
         * @param movePow The voltage to the motors thst you get
         */
        NotePosition(double pickUpPow, double movePow) {
          this.movePow = movePow;
          this.pickUpPow = pickUpPow;
        }
    }
}


