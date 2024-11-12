package frc.robot.utils;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.configs.CANcoderConfiguration;

public class Cancoder extends CANcoder {
    CancoderConfig config;
    String name;
    private CANcoderConfiguration canConfig;
    public Cancoder(CancoderConfig config) {
        super(config.id, config.canbus);
        this.config = config;
		name = config.name;
		configCancoder();
        addLog();
		LogManager.log(name + " motor initialized");
    }

    private void configCancoder() {
		canConfig = new CANcoderConfiguration();
		canConfig.MagnetSensor.MagnetOffset = config.offset;
        canConfig.MagnetSensor.SensorDirection = config.inverted ? SensorDirectionValue.Clockwise_Positive: SensorDirectionValue.CounterClockwise_Positive;
        getConfigurator().apply(canConfig);
  }

    /**when the cancoder opens its start at the absolute position
     * @return the none absolute amaunt of rotations the motor did in rotation2d
    */
    public double getPositionRadians() {
        return getPosition().getValueAsDouble() * 2 * Math.PI;
    }
    /**
     * @return the absolute amaunt of rotations the motor did in rotation2d
     */
    public double getAbsPositionRadians() {
        return getAbsolutePosition().getValueAsDouble() * 2 * Math.PI;
    }
    /** 
     * @return the amount of rotations the motor do per second in Rotation2d
     */
    public double getVelocityRadiansPerSec(){
        return getVelocity().getValueAsDouble() * 2 * Math.PI;
    }

    private void addLog() {
      LogManager.addEntry(name + "/Position", this::getPositionRadians);// radians
      LogManager.addEntry(name + "/Absolute position", this::getAbsPositionRadians);// radians
      LogManager.addEntry(name + "/Velocity", this::getVelocityRadiansPerSec);// radians per seconds
    }
}
