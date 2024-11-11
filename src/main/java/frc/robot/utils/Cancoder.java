package frc.robot.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.configs.CANcoderConfiguration;

public class Cancoder extends CANcoder {
    private CANcoderConfiguration canConfig;
    public Cancoder(int ID, String CANBUS) {
        super(ID, CANBUS);
        canConfig = new CANcoderConfiguration();
        getConfigurator().apply(canConfig);
    }

    /**when the cancoder opens its start at the absolute position
     * @return the none absolute amaunt of rotations the motor did in rotation2d
    */
    public double getNonAbsRotation2d() {
        return getPosition().getValueAsDouble();
    }
    /**
     * @return the absolute amaunt of rotations the motor did in rotation2d
     */
    public double getAbsRadians() {
        return getAbsolutePosition().getValueAsDouble() * 2 * Math.PI;
    }
    /** 
     * @return the amount of rotations the motor do per second in Rotation2d
     */
    public double getVelocityRotation2dPerSec(){
        return getVelocity().getValueAsDouble();
    }
    /** set the offset of the cancoder to offset
     * subtracts from the position the offset at all time
     * @param 
     */
    public void setOffset(double offset){
        canConfig.MagnetSensor.MagnetOffset = offset;
        getConfigurator().apply(canConfig); 
    }
    /** set the diraction of the cancoder
     * when changing the diraction the position will moltiply by minus one at all time
     * @param
     */
    public void setCanCoderClockwise(Boolean boolDirection){
        canConfig.MagnetSensor.SensorDirection = boolDirection ? SensorDirectionValue.Clockwise_Positive: SensorDirectionValue.CounterClockwise_Positive;
        getConfigurator().apply(canConfig);
    }
}
