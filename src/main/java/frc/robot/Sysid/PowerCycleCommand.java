package frc.robot.Sysid;

import java.util.function.Consumer;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.test.TempSubSystem;

/**
 * Class to run a sysid power cycle
 * It just set the power once - in initialize
 * It collect the data each cycle
 * 
 */
public class PowerCycleCommand extends Command {

    double velocity; // the power to use
    DataCollector dataCollector; // data collector
    Consumer<Double> setPower; // set power function
    boolean resetDataCollector; // reset the data collector data for rerun of the same command group
    double minVelocity;
    double maxVelocity;
    boolean isRadian;
    /**
     * default Constructor - does not reset the data collector
     * 
     * @param setPower      power given to the motor
     * @param power         wanted power
     * @param dataCollector collects data
     * @param subSystem     needed subsystem
     */
    public PowerCycleCommand(Consumer<Double> setPower, double velocity, DataCollector dataCollector, double maxVelocity, double minVelocity, boolean isRadian,
            Subsystem... subSystem) {

        this(setPower, velocity, dataCollector, false, maxVelocity, minVelocity, isRadian, subSystem);
        addRequirements(subSystem);
        
    }

    /**
     * Constructor - can reset the data collector data in intialize
     *
     * @param setPower           saves the power given to the motor
     * @param power              wanted power
     * @param dataCollector      collects data
     * @param resetDataCollector param to reset data collected
     * @param subSystem          needed subsystem
     */
    public PowerCycleCommand(Consumer<Double> setPower, double velocity, DataCollector dataCollector,
            boolean resetDataCollector, double maxVelocity, double minVelocity, boolean isRadian, Subsystem... subSystem) {
        this.velocity = velocity;
        this.dataCollector = dataCollector;
        this.setPower = setPower;
        this.resetDataCollector = resetDataCollector;
        this.minVelocity = minVelocity;
        this.maxVelocity = maxVelocity;
        this.isRadian = isRadian;
        addRequirements(subSystem);
    }

    /**
     * starts timer
     * reset data collector if needed
     * adds power to the set power variable
     * reset last velocity
     */

    @Override
    public void initialize() {
        if (resetDataCollector) {
            dataCollector.resetData();
        }
        setPower.accept(velocity);
        dataCollector.resetLastV();
    }

    @Override
    public void execute() {
        dataCollector.collect(velocity, maxVelocity, minVelocity, isRadian);
        
    }

    @Override
    public void end(boolean interrupted) {
        // System.out.println(" sysid-powercycle-end " + power);
        
    }



}