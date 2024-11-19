package frc.robot.Sysid;

import java.util.function.Consumer;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.test.TempSubSystem;
import frc.robot.utils.TalonMotor;

/**
 * Class to run a sysid power cycle
 * It just set the power once - in initialize
 * It collect the data each cycle
 * 
 */
public class PowerCycleCommand extends Command {

    double power; // the power to use
    DataCollector dataCollector; // data collector
    Consumer<Double> setPower; // set power function
    boolean resetDataCollector; // reset the data collector data for rerun of the same command group
    double sec;
    Timer timer;

    TempSubSystem temp = new TempSubSystem();

    /**
     * default Constructor - does not reset the data collector
     * 
     * @param setPower      power given to the motor
     * @param power         wanted power
     * @param dataCollector collects data
     * @param subSystem     needed subsystem
     */
    public PowerCycleCommand(Consumer<Double> setPower, double power, DataCollector dataCollector,
            Subsystem... subSystem) {

        this(setPower, power, dataCollector, false, subSystem);
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
            boolean resetDataCollector, Subsystem... subSystem) {
        this.power = power;
        this.dataCollector = dataCollector;
        this.setPower = setPower;
        this.resetDataCollector = resetDataCollector;
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
        setPower.accept(power);
        dataCollector.resetLastV();
    }

    @Override
    public void execute() {
        dataCollector.collect(power);
    }

    @Override
    public void end(boolean interrupted) {
        // System.out.println(" sysid-powercycle-end " + power);
        setPower.accept(0.0);
    }



}