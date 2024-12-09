package frc.robot.utils.simulation;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SimMotor extends SubsystemBase {
    private double distancePassed;
    private double velocity;
    private double acceleration = 99;

    private PIDController pid;

    public SimMotor() {
        pid = new PIDController(0, 0, 0);
    }

    public void setVelocity(double v) {
        pid.setSetpoint(v);
    }

    public void setP(double p) {
        pid.setP(p);
    }

    public void setI(double i) {
        pid.setI(i);
    }

    public void setD(double d) {
        pid.setD(d);
    }

    public double getDistancePassed() { return distancePassed; }
    public double getVelocity() { return velocity; }

    @Override
    public void periodic() {
        velocity += acceleration * pid.calculate(velocity) * 0.02;

        distancePassed += velocity * 0.02;
    }
}
