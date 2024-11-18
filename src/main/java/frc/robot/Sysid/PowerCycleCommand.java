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
     * @param sec           time given to accomplish command
     */
    public PowerCycleCommand(Consumer<Double> setPower, double power, DataCollector dataCollector, double sec,
            Subsystem... subSystem) {

        this(setPower, power, dataCollector, false, sec, subSystem);
        if(subSystem != null){
            addRequirements(subSystem);
        }
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
            boolean resetDataCollector, double sec, Subsystem... subSystem) {
        this.power = power;
        this.dataCollector = dataCollector;
        this.setPower = setPower;
        this.resetDataCollector = resetDataCollector;
        this.sec = sec;
        timer = new Timer();
        if (subSystem != null)
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
        timer.start();
        if (resetDataCollector) {
            dataCollector.resetData();
        }
        setPower.accept(power);
        dataCollector.resetLastV();
    }

    /*TODO add setPower function to a tempSubsystem */
    @Override
    public void execute() {
        dataCollector.collect(power);
    }

    @Override
    public void end(boolean interrupted) {
        // System.out.println(" sysid-powercycle-end " + power);
        timer.stop();
        setPower.accept(0.0);
        SmartDashboard.putNumber("Time ran in seconds", timer.get());
        System.out.println(timer.get());
        timer.reset();
    }

    @Override
    public boolean isFinished() {//3.1 - 3 = 0.1
        double error = Math.abs(timer.get() - sec);
        return (error > 0 );
    }

}