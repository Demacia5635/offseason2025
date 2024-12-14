package frc.robot.Shooter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class ShooterConstants  {

  public static class MOTOR_IDS{
    
    public static final int MOTOR_UP_ID = 21;
    public static final int MOTOR_DOWN_ID = 22;
    public static final int MOTOR_FEEDING_ID = 20;
    public static final int ANGLE_CHANGING_ID = 30;

    public static final int LIMIT_SWITCH_ID = 0;

    public static final String CANBUS = "rio";
  }

  public static class ANGLE_CHANGING_CONFIGS{

    public static final double ANGLE_CHANGING_MAX_VELOCITY = 25;
    public static final double ANGLE_CHANGING_MAX_Acceleration  = 30;
    public static final double ANGLE_CHANGING_MAX_JERK = 0;

    public static final double ANGLE_VOLTAGE_RAMP = 0.7;

    public static final boolean IS_ANGLE_MOTOR_INVERT = true;
    
    public static final boolean IS_ANGLE_MOTORS_BRAKE = true;

    public static final double ANGLE_CHANGING_FREQHZ = 200;
    
    public static final double ANGLE_CHANGING_GEAR_RATIO = 1/2.0;
  }


  public static class SHOOTER_CONFIGS{
    public static final boolean IS_UP_MOTOR_INVERT = true;
    public static final boolean IS_DOWN_MOTOR_INVERT = false;
    public static final boolean IS_FEEDING_MOTOR_INVERT = false;

    public static final boolean IS_SHOOTING_MOTORS_BRAKE = false;
    public static final boolean IS_FEEDING_MOTOR_BRAKE = true;

    public static final double FREQHZ = 200;

  }

  public static class SHOOTER_POW{
    public static final double FEEDING_MOTOR_POWER = 1;
    public static final double REVERSE_FEEDING_POWER = -1;
    public static final double INTAKE_MOTOR_POWER = 1;

  }

  public static class AMP_VAR{
    
      public static final double AMP_ANGLE = 46;
      public static final double MOTOR_UP_AMP_VELOCITY = 15;
      public static final double MOTOR_DOWN_AMP_VELOCITY = 25;
    
  }

  public static class STAGE_VAR{
    
      public static final double STAGE_ANGLE = 0;
      public static final double MOTOR_UP_STAGE_VELOCITY = 0;
      public static final double MOTOR_DOWN_STAGE_VELOCITY = 0;
    
  }

  public static class SUBWOFFER_VAR{
    
      public static final double SUBWOFFER_ANGLE = 53;
      public static final double MOTOR_UP_SUBWOFFER_VELOCITY = 56;
      public static final double MOTOR_DOWN_SUBWOFFER_VELOCITY = 53; 
  }

  public static class MAX_ERRORS{
    
      public static final double ANGLE_MAX_ERRORS = 1;
      public static final double UP_MOTOR_VEL_MAX_ERRORS = 2;
      public static final double DOWN_MOTOR_VEL_MAX_ERRORS = 2;
    
  }

  public static class ANGLE_CHANGING_CALIBRATION{

    public static final double UP_SPEED_CALIBRATION = 0.3;
    public static final double DOWN_SPEED_CALIBRATION = -0.3;

  }

  public static class DELIVERY_VAR {
    public static final double DELIVERY_ANGLE = 42;
    public static final double MOTOR_UP_DELIVERY_VEL = 50;
    public static final double MOTOR_DOWN_DELIVERY_VEL = 50;
  }

  public static class SHOOTER_ATRIBUTES {
    public static final double RADIOS = 0.36;
    public static final double SHOOTING_MOTORS_ROTATION_TO_METER = 2 * RADIOS * Math.PI;
    public static final double MIL_SEC_TO_SHOOT = 1000;

  }  

  public static class LOOKUP_TABLE_DATA {
    public static final double[][] DATA = {
      {1.3, 53, 56, 53}, {2.5, 42, 58, 58}, {3.48, 38.5, 63, 60}
    };
  }

  public static class ANGLE_CHANGING_VAR{

    public static final double A = 128;
    public static final double B = 128;
    public static final double C = 80;

    public static final double BASE_DIS = 25.12;
    public static final double TOP_ANGLE = 70;
    public static final double MIN_ANGLE = 37;

}


  public enum STATE{
    AMP, STAGE, SUBWOFFER, DELIVERY, SPEAKER, IDLE, TESTING;
  }

}
