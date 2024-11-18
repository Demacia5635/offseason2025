package frc.robot.Sysid;

import java.util.function.Consumer;
import java.util.function.Supplier;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/**
 * Class to calculate feed forward gains for velocity/acceleration control
 * there are different gains - the default are
 * KS (times signum(v))
 * KV (times v)
 * KA (times v-lastV)
 * Others:
 * KRad (times angle of mechanism in Radians)
 * KMeter (times position of mechanism in Meters)
 * KV2 (times v squared)
 * KVsqrt (times sqrt of v)
 * KSin (times sin(radians))
 * KCos (times cos(radians))
 * KTan (times tan(radians))
 * 
 * It operates the mechanism/motors at different power setting for a duration
 * It collect the velocity and optionaly radians and position of the mechanism
 * It than calculate the best fit gains
 * 
 */
public class Sysid {

    /**
     * Gains enum - type of gains
     */
    public static enum Gains {
        KS, KV, KA, KRad, KMeter, KV2, KVsqrt, KSin, KCos, KTan;
    }

    Consumer<Double> setPower; // function to set the power
    DataCollector dataCollector; // the data collector class
    double minPower;
    double maxPower;
    double deltaPower; // the change of power between power cycles
    int nPowerCycles; // how many powers to use (the system will run each power in positive and
                      // negative values)
    double powerCycleDuration; // how long each power cycle
    double powerCycleDelay; // delay between power cycles
    Subsystem[] subsystems; // for add requirements
    final static double defaultDuration = 2.5;
    final static double defaultDelay = 10;
    Gains[] gains; // the gains we are looking for
    double[] result = null; // the result, after analyze
    boolean steadyOnly = false; // if we need steady only

    /**
     * Constructor with default values - only required the setPower, setVelocity,
     * min/max power and subsystems
     * 
     * @param setPower    the power wanted
     * @param getVelocity motors velocity
     * @param minPower    the min power given
     * @param maxPower    the max power
     * @param subsystems  subsystem for require
     */
    public Sysid(Consumer<Double> setPower,
            Supplier<Double> getVelocity,
            double minPower,
            double maxPower,
            Subsystem... subsystems) {
        this(new Gains[] { Gains.KS, Gains.KV, Gains.KA, Gains.KV2 },
                setPower,
                getVelocity,
                null,
                minPower,
                maxPower,
                3,
                defaultDuration,
                defaultDelay,
                subsystems);
    }

    /**
     * Constructor with additional values - duration and delay
     * 
     * @param setPower           holds the power given to the motor
     * @param getVelocity        current velocity
     * @param minPower           min power that can be given
     * @param maxPower           max power that can be given
     * @param subsystems         needed subsystem
     * @param powerCycleDuration for how long each power should be activated
     */
    public Sysid(Consumer<Double> setPower,
            Supplier<Double> getVelocity,
            double minPower,
            double maxPower,
            double powerCycleDuration,
            double powerCycleDelay,
            Subsystem... subsystems) {
        this(new Gains[] { Gains.KS, Gains.KV, Gains.KA },
                setPower,
                getVelocity,
                null,
                minPower,
                maxPower,
                3,
                powerCycleDuration,
                powerCycleDelay,
                subsystems);
    }

    /**
     * Constructor with all parameters
     * 
     * @param setPower    holds in the power
     * @param getVelocity gives the motors velocity
     * @param minPower    min power that can be given
     * @param maxPower    max power that can be given
     * @param subsystems  needed subsystem
     */
    public Sysid(Gains[] types,
            Consumer<Double> setPower,
            Supplier<Double> getVelocity,
            Supplier<Double> getRadians,
            double minPower,
            double maxPower,
            int nPowerCycles,
            double powerCycleDuration,
            double powerCycleDelay,
            Subsystem... subsystems) {

        this.setPower = setPower;
        dataCollector = new DataCollector(types, getVelocity, getRadians, nPowerCycles, powerCycleDuration);
        this.minPower = minPower;
        this.maxPower = maxPower;
        this.nPowerCycles = nPowerCycles;
        this.powerCycleDelay = powerCycleDelay;
        this.powerCycleDuration = powerCycleDuration;
        deltaPower = (maxPower - minPower) / (nPowerCycles - 1);
        this.subsystems = subsystems;
        this.gains = types;
    }

    /**
     * 
     * @param setPower    power given to the motor
     * @param getVelocity current velocity
     * @param minPower    min power (you cant go below it)
     * @param maxPower    max power (you cant go above it)
     * @param powerStep   
     * @param minVelocity min velocity (you cant go below it)
     * @param maxVelocity max velocity (you cant go below it)
     * @param isMeter     decides which unints to use
     * @param subsystems  subsystem needed to run on
     * @return
     */
    public static Command getSteadyCommand(Consumer<Double> setPower, Supplier<Double> getVelocity, double minPower,
            double maxPower, double powerStep, double minVelocity, double maxVelocity, boolean isMeter,
            Subsystem... subsystems) {
        Sysid id = new Sysid(new Gains[] { Gains.KS, Gains.KV, Gains.KV2 }, setPower, getVelocity, null, minPower,
                maxPower, 2, 1, 1, subsystems);
        id.steadyOnly = true;
        Command cmd = new NoAccelerationPowerCommand(setPower, minPower, maxPower, powerStep, id.dataCollector, false,
                minVelocity, maxVelocity, maxVelocity, subsystems);
        return cmd.andThen(new InstantCommand(() -> id.analyze()));
    }

    /**
     * run the command
     */
    public void run() {
        getCommand().schedule();
    }

    /**
     * calculate the power for cycle
     * cycles run - minPower, -minPower, (minPower+delta),
     * -(minPower+delts).....(maxPower), -max(Power)
     * 
     * @param cycle
     * @param pow   power for every cycle
     * @return double
     */
    double power(int cycle) {
        int pow = cycle / 2;
        double sign = cycle % 2 == 0 ? 1 : -1;
        return sign * (minPower + pow * deltaPower);
    }

    /**
     * Generate the command to run the system at different powers, collect the data
     * and analyze the result
     * there is a delay between each power cycle
     * 
     * @return Command
     */
    public Command getCommand() {
        boolean resetDataCollector = true;
        Command cmd = new WaitCommand(powerCycleDelay);
        for (int c = 0; c < nPowerCycles; c++) {
            double power = minPower + c * deltaPower;
            cmd = cmd.andThen(getPowerCommand(power, resetDataCollector));
            resetDataCollector = false;
            cmd = cmd.andThen(getPowerCommand(-power, resetDataCollector));
        }
        return cmd.andThen(new InstantCommand(() -> analyze()));
    }

    public Command getCommandOneWay() {
        boolean resetDataCollector = true;
        Command cmd = new WaitCommand(powerCycleDelay);
        for (int c = 0; c < nPowerCycles; c++) {
            double power = minPower + c * deltaPower;
            cmd = cmd.andThen(getPowerCommand(power, resetDataCollector));
            resetDataCollector = false;
            // cmd = cmd.andThen(getPowerCommand(-power, resetDataCollector));
        }
        return cmd.andThen(new InstantCommand(() -> analyze()));
    }

    /**
     * Get the command for a power - with the duration and delay
     */
    Command getPowerCommand(double power, boolean resetDataCollector) {
        return ((new PowerCycleCommand(setPower, power, dataCollector, resetDataCollector, powerCycleDuration,
                subsystems))
                .withTimeout(powerCycleDuration)).andThen(new WaitCommand(powerCycleDelay));
    }

    /**
     * Analyze the result
     * Using data collector solve
     * It also calculate the worst error and the avg error squared
     * 
     * @param feedForwardValues feed forward values for the specific range
     * @param power             the power matrix,
     * @param result            double arr that holds all feed forward values (such
     *                          as kS, kV and kA)
     * @param dataCollector holds in raw data such as current velocity, acceleration
     * 
     */
    void analyze() {
        SimpleMatrix feedForwardValues = dataCollector.solve();
        result = new double[gains.length];
        for (int i = 0; i < gains.length; i++) {
            result[i] = feedForwardValues.get(i, 0);
            SmartDashboard.putNumber("SysID/" + gains[i] + "/0-20 ranges", result[i]);
            // System.out.println("Sysid: " + gains[i] + " = " + result[i]);
        }
        
        SimpleMatrix power = dataCollector.dataRange20().mult(feedForwardValues);
        SimpleMatrix e = dataCollector.power().minus(power);
        SimpleMatrix ee = e.elementMult(e);
        double max = Math.sqrt(ee.elementMax());
        double avg = ee.elementSum() / ee.getNumRows();
        SmartDashboard.putNumber("Sysid/Max Error", max);
        SmartDashboard.putNumber("Sysid/Avg Error Sqr", avg);
        double kp = (valueOf(Gains.KV, gains, result) + valueOf(Gains.KA, gains, result)) / 5.0;
        SmartDashboard.putNumber("Sysid/KP (Roborio)", kp);
    }

    /**
     * Value of a specific Gain type
     * 
     * @param gain
     * @param gains
     * @param values
     * @return
     */
    double valueOf(Gains gain, Gains[] gains, double[] values) {
        for (int i = 0; i < gains.length; i++) {
            if (gains[i] == gain) {
                return values[i];
            }
        }
        return 0;
    }

    /**
     * 
     * @return result array of gain values
     */
    public double[] result() {
        return result;
    }

    /**
     * 
     * @return result array of gain types
     */
    public Gains[] gains() {
        return gains;
    }

}