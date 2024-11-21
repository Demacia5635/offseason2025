package frc.robot.utils;

import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class TalonMotor extends TalonFX {
	TalonConfig config;
  String name;
  TalonFXConfiguration cfg;
  VelocityVoltage velocityVoltage = new VelocityVoltage(0).withSlot(0);
  VoltageOut voltageOut = new VoltageOut(0);

  DutyCycleOut dutyCycle = new DutyCycleOut(0);
  MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0).withSlot(0);
  LogManager.LogEntry dutyCycleEntry;
  LogManager.LogEntry velocityEntry;
  LogManager.LogEntry positionEntry;


  public TalonMotor(TalonConfig config) {
		super(config.id, config.canbus);
		this.config = config;
		name = config.name;
		configMotor();
		addLog();
		LogManager.log(name + " motor initialized");
  }
  
  /**
   * creates a widget in elastic of the pid and ff hot reload
   * @param slot the slot of the close loop perams (from 0 to 2)
   */
  public void hotReloadPidFf(int slot) {

    Command configPid = new InstantCommand(()-> {
      SlotConfigs cfg = new SlotConfigs();
      cfg.SlotNumber = slot;
      switch (slot) {
        case 0:
          cfg.kP = config.pid.kp;
          cfg.kI = config.pid.ki;
          cfg.kD = config.pid.kd;
          cfg.kS = config.pid.ks;
          cfg.kV = config.pid.kv;
          cfg.kA = config.pid.ka;
          cfg.kG = config.pid.kg;
          break;

        case 1:
          cfg.kP = config.pid1.kp;
          cfg.kI = config.pid1.ki;
          cfg.kD = config.pid1.kd;
          cfg.kS = config.pid1.ks;
          cfg.kV = config.pid1.kv;
          cfg.kA = config.pid1.ka;
          cfg.kG = config.pid1.kg;
          break;

        case 2:
          cfg.kP = config.pid2.kp;
          cfg.kI = config.pid2.ki;
          cfg.kD = config.pid2.kd;
          cfg.kS = config.pid2.ks;
          cfg.kV = config.pid2.kv;
          cfg.kA = config.pid2.ka;
          cfg.kG = config.pid2.kg;
          break;
      
        default:
          cfg.kP = config.pid.kp;
          cfg.kI = config.pid.ki;
          cfg.kD = config.pid.kd;
          cfg.kS = config.pid.ks;
          cfg.kV = config.pid.kv;
          cfg.kA = config.pid.ka;
          cfg.kG = config.pid.kg;
          break;
      }

      getConfigurator().apply(cfg);
    }).ignoringDisable(true);

    SmartDashboard.putData(name + " pid+ff config", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("pid+ff Talon motor");
        
        switch(slot) {
          case 0:
            builder.addDoubleProperty("kP", ()-> config.pid.kp, (double newValue) -> config.pid.kp = newValue);
            builder.addDoubleProperty("kI", ()-> config.pid.ki, (double newValue) -> config.pid.ki = newValue);
            builder.addDoubleProperty("kD", ()-> config.pid.kd, (double newValue) -> config.pid.kd = newValue);
            builder.addDoubleProperty("kS", ()-> config.pid.ks, (double newValue) -> config.pid.ks = newValue);
            builder.addDoubleProperty("kV", ()-> config.pid.kv, (double newValue) -> config.pid.kv = newValue);
            builder.addDoubleProperty("kA", ()-> config.pid.ka, (double newValue) -> config.pid.ka = newValue);
            builder.addDoubleProperty("kG", ()-> config.pid.kg, (double newValue) -> config.pid.kg = newValue);
            break;

          case 1:
            builder.addDoubleProperty("kP", ()-> config.pid1.kp, (double newValue) -> config.pid1.kp = newValue);
            builder.addDoubleProperty("kI", ()-> config.pid1.ki, (double newValue) -> config.pid1.ki = newValue);
            builder.addDoubleProperty("kD", ()-> config.pid1.kd, (double newValue) -> config.pid1.kd = newValue);
            builder.addDoubleProperty("kS", ()-> config.pid1.ks, (double newValue) -> config.pid1.ks = newValue);
            builder.addDoubleProperty("kV", ()-> config.pid1.kv, (double newValue) -> config.pid1.kv = newValue);
            builder.addDoubleProperty("kA", ()-> config.pid1.ka, (double newValue) -> config.pid1.ka = newValue);
            builder.addDoubleProperty("kG", ()-> config.pid1.kg, (double newValue) -> config.pid1.kg = newValue);
            break;

          case 2:
            builder.addDoubleProperty("kP", ()-> config.pid2.kp, (double newValue) -> config.pid2.kp = newValue);
            builder.addDoubleProperty("kI", ()-> config.pid2.ki, (double newValue) -> config.pid2.ki = newValue);
            builder.addDoubleProperty("kD", ()-> config.pid2.kd, (double newValue) -> config.pid2.kd = newValue);
            builder.addDoubleProperty("kS", ()-> config.pid2.ks, (double newValue) -> config.pid2.ks = newValue);
            builder.addDoubleProperty("kV", ()-> config.pid2.kv, (double newValue) -> config.pid2.kv = newValue);
            builder.addDoubleProperty("kA", ()-> config.pid2.ka, (double newValue) -> config.pid2.ka = newValue);
            builder.addDoubleProperty("kG", ()-> config.pid2.kg, (double newValue) -> config.pid2.kg = newValue);
            break;

          default:
            builder.addDoubleProperty("kP", ()-> config.pid.kp, (double newValue) -> config.pid.kp = newValue);
            builder.addDoubleProperty("kI", ()-> config.pid.ki, (double newValue) -> config.pid.ki = newValue);
            builder.addDoubleProperty("kD", ()-> config.pid.kd, (double newValue) -> config.pid.kd = newValue);
            builder.addDoubleProperty("kS", ()-> config.pid.ks, (double newValue) -> config.pid.ks = newValue);
            builder.addDoubleProperty("kV", ()-> config.pid.kv, (double newValue) -> config.pid.kv = newValue);
            builder.addDoubleProperty("kA", ()-> config.pid.ka, (double newValue) -> config.pid.ka = newValue);
            builder.addDoubleProperty("kG", ()-> config.pid.kg, (double newValue) -> config.pid.kg = newValue);
        }
        
        builder.addBooleanProperty("update", ()-> configPid.isScheduled(), 
          value -> {
            if (value) {
              if (!configPid.isScheduled()) {
                configPid.schedule();
              }
            } else {
              if (configPid.isScheduled()) {
                configPid.cancel();
              }
            }
          }
        );
      }
    });
  }

  private void configMotor() {
		cfg = new TalonFXConfiguration();
		cfg.CurrentLimits.SupplyCurrentLimit = config.maxCurrent;
		cfg.CurrentLimits.SupplyCurrentThreshold = config.maxCurrentTriggerTime;
		cfg.CurrentLimits.SupplyTimeThreshold = config.maxCurrentTriggerTime;
		cfg.CurrentLimits.SupplyCurrentLimitEnable = true;

		cfg.ClosedLoopRamps.VoltageClosedLoopRampPeriod = config.rampUpTime;
		cfg.OpenLoopRamps.VoltageOpenLoopRampPeriod = config.rampUpTime;

		cfg.MotorOutput.Inverted = config.inverted ? InvertedValue.CounterClockwise_Positive
			: InvertedValue.Clockwise_Positive;
		cfg.MotorOutput.NeutralMode = config.brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
		cfg.MotorOutput.PeakForwardDutyCycle = config.maxVolt / 12.0;
		cfg.MotorOutput.PeakReverseDutyCycle = config.minVolt / 12.0;

		cfg.Slot0.kP = config.pid.kp;
		cfg.Slot0.kI = config.pid.ki;
		cfg.Slot0.kD = config.pid.kd;
		cfg.Slot0.kS = config.pid.ks; 
		cfg.Slot0.kV = config.pid.kv;
		cfg.Slot0.kA = config.pid.ka;
		cfg.Slot0.kG = config.pid.kg;
		if(config.pid1 != null) {
			cfg.Slot1.kP = config.pid1.kp;
			cfg.Slot1.kI = config.pid1.ki;
			cfg.Slot1.kD = config.pid1.kd;
			cfg.Slot1.kS = config.pid1.ks; 
			cfg.Slot1.kV = config.pid1.kv;
			cfg.Slot1.kA = config.pid1.ka;
			cfg.Slot1.kG = config.pid1.kg;
		}
		if(config.pid2 != null) {
			cfg.Slot2.kP = config.pid2.kp;
			cfg.Slot2.kI = config.pid2.ki;
			cfg.Slot2.kD = config.pid2.kd;
			cfg.Slot2.kS = config.pid2.ks; 
			cfg.Slot2.kV = config.pid2.kv;
			cfg.Slot2.kA = config.pid2.ka;
			cfg.Slot2.kG = config.pid2.kg;
		}

		cfg.Voltage.PeakForwardVoltage = config.maxVolt;
		cfg.Voltage.PeakReverseVoltage = config.minVolt;

		cfg.Feedback.SensorToMechanismRatio = config.motorRatio;
		cfg.MotionMagic.MotionMagicAcceleration = config.motionMagicAccel;
		cfg.MotionMagic.MotionMagicCruiseVelocity = config.motionMagicVelocity;
		cfg.MotionMagic.MotionMagicJerk = config.motionMagicJerk;
		cfg.MotionMagic.MotionMagicExpo_kA = config.pid.ka;
		cfg.MotionMagic.MotionMagicExpo_kV = config.pid.kv;

		velocityVoltage.UpdateFreqHz = 200;
		dutyCycle.UpdateFreqHz = 200;
		motionMagicVoltage.UpdateFreqHz = 200;
		

		getConfigurator().apply(cfg);
		getPosition().setUpdateFrequency(200);
		getVelocity().setUpdateFrequency(200);
		getAcceleration().setUpdateFrequency(200);
		getMotorVoltage().setUpdateFrequency(200);

  }

  /*
   * set motor to brake or coast
   */
  public void setBrake(boolean brake) {
		this.getConfigurator().refresh(cfg.MotorOutput);
		cfg.MotorOutput.NeutralMode = config.brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
		getConfigurator().apply(cfg.MotorOutput);
  }

  private void addLog() {    

  /**
   * override the sendable of the talonFX to our costum widget in elastic
   * <br></br>
   * to activate do <pre> SmartDashboard.putData("talonMotor name", talonMotor);</pre>
   */
  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("TalonMotor");

    builder.addStringProperty("controlMode", ()-> getControlMode().getValue().toString(), null);
    builder.addBooleanProperty("isinvert", this::getInverted, null);
    builder.addDoubleProperty("closeLoopSP", ()-> getClosedLoopReference().getValueAsDouble(), null);
    builder.addDoubleProperty("closeLoopError", ()-> getClosedLoopError().getValueAsDouble(), null);
    builder.addDoubleProperty("position", ()-> getPosition().getValueAsDouble(), null);
    builder.addDoubleProperty("velocity", ()-> getVelocity().getValueAsDouble(), null);
    builder.addDoubleProperty("acceleration", ()-> getAcceleration().getValueAsDouble(), null);
    builder.addDoubleProperty("voltage", ()-> getMotorVoltage().getValueAsDouble(), null);
    LogManager.addEntry(name + "/Position", getPosition());// rotation
    LogManager.addEntry(name + "/Velocity", getVelocity());// rotation per seconds
    LogManager.addEntry(name + "/Acceleration", getAcceleration());// rotation per seconds^2
    LogManager.addEntry(name + "/Voltage", getMotorVoltage());
    LogManager.addEntry(name + "/Current", getStatorCurrent());
    LogManager.addEntry(name + "/CloseLoopError", getClosedLoopError());
    LogManager.addEntry(name + "/CloseLoopOutput", getClosedLoopOutput());
    LogManager.addEntry(name + "/CloseLoopP", getClosedLoopProportionalOutput());
    LogManager.addEntry(name + "/CloseLoopI", getClosedLoopIntegratedOutput());
    LogManager.addEntry(name + "/CloseLoopD", getClosedLoopDerivativeOutput());
    LogManager.addEntry(name + "/CloseLoopFF", getClosedLoopFeedForward());
    LogManager.addEntry(name + "/CloseLoopSP", getClosedLoopReference());

    dutyCycleEntry = LogManager.getEntry(name + "/SetDutyCycle");
    velocityEntry = LogManager.getEntry(name + "/SetVelocity");
    positionEntry = LogManager.getEntry(name + "/SetPosition");
  }

	/**
   * set power from 1 to -1 (v/12) no PID/FF
   */
  public void setDuty(double power) {
    setControl(dutyCycle.withOutput(power));
    dutyCycleEntry.log(power);
  }

	/**
   * set volocity to motor with PID and FF
   */
  public void setVelocity(double velocity, double feedForward) {
    setControl(velocityVoltage.withVelocity(velocity).withFeedForward(feedForward));
    velocityEntry.log(velocity);
  }

	public void setVelocity(double velocity) {
		setVelocity(velocity, 0);
	}

	public void setMotionMagic(double position, double feedForward) {
		setControl(motionMagicVoltage.withPosition(position).withFeedForward(feedForward));
		positionEntry.log(position);
	}

	public void setMotionMagic(double position) {
		setMotionMagic(position, 0);
	}

	public double getCurrentPosition() {
		return getPosition().getValueAsDouble();
	}

	public double getCurrentVelocity() {
		return getVelocity().getValueAsDouble();
	}
	  
	public void setVelocityWithFeedForward(double velocity) {
    	setVelocity(velocity,velocityFeedForward(velocity));
  	}

	public void setMotionMagicWithFeedForward(double velocity) {
		setVelocity(velocity, positionFeedForward(velocity));
	}

  private double velocityFeedForward(double velocity) {
    return velocity * velocity * Math.signum(velocity) * config.kv2;
  }
  private double positionFeedForward(double positin) {
    return Math.sin(positin*config.posToRad)*config.kSin;
  }

  /**
   * change the slot of the pid and feed forward.
   * will not work if the slot is null
   * @param slot the wanted slot between 0 and 2
   */
  public void changeSlot(int slot) {    
    if (slot < 0 || slot > 2) {
      LogManager.log("slot is not between 0 and 2"); 
      return;
    }

    if (slot == 1 && config.pid1 == null) {
      LogManager.log("slot is null, add config for slot 1");
      return;
    }

    if (slot == 2 && config.pid2 == null) {
      LogManager.log("slot is null, add config for slot 2");
      return;
    }

    velocityVoltage.withSlot(slot);
    motionMagicVoltage.withSlot(slot);
  }
}
