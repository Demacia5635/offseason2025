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

    double power; // given power to the motor
    DataCollector dataCollector; // data collector
    Consumer<Double> setPower; // set power function
    boolean resetDataCollector; // reset the data collector data for rerun of the same command group
    double maxVel; // max velocity given
    /**
     * default Constructor - does not reset the data collector
     * 
     * @param setPower      power given to the motor
     * @param power         wanted power
     * @param dataCollector collects data
     * @param subSystem     needed subsystem
     */
    public PowerCycleCommand(Consumer<Double> setPower, double power, DataCollector dataCollector, double maxVel,
            Subsystem... subSystem) {

        this(setPower, power, dataCollector, false, maxVel, subSystem);
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
    public PowerCycleCommand(Consumer<Double> setPower, double power, DataCollector dataCollector,
            boolean resetDataCollector, double maxVel, Subsystem... subSystem) {
        this.power = power;
        this.dataCollector = dataCollector;
        this.setPower = setPower;
        this.resetDataCollector = resetDataCollector;
        this.maxVel = maxVel;
        addRequirements(subSystem);
    }

    /**
     * reset data collector if needed
     * adds power to the set power variable
     * reset last velocity
     */

    @Override
    public void initialize() {
        if (resetDataCollector) {
            dataCollector.resetData();
        }
        setPower.accept(power);
        dataCollector.resetLastV();
    }

    @Override
    public void execute() {
        dataCollector.collect(maxVel);
        
    }

    @Override
    public void end(boolean interrupted) {
        // System.out.println(" sysid-powercycle-end " + power);
        
    }



}