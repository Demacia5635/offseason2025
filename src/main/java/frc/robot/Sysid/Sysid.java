package frc.robot.Sysid;

import java.util.function.Consumer;
import java.util.function.Supplier;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
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
        KS, KV, KA, KG, KRad, KV2, KVsqrt, KSin, KCos, KTan;
    }

    Consumer<Double> setPower; // function to set the power
    DataCollector dataCollector; // the data collector class
    double minPow;
    double maxVel;
    double deltaPower; // the change of power between power cycles
    int nPowerCycles; // how many powers to use (the system will run each power in positive and
                      // negative values)
    double powerCycleDuration; // how long each power cycle
    double powerCycleDelay; // delay between power cycles
    Subsystem[] subsystems; // for add requirements
    final static double defaultDuration = 2.5;
    final static double defaultDelay = 10;
    Gains[] gains; // the gains we are looking for
    double[] result30 = null; // the result, after analyze
    double[] result50 = null;
    double[] resultFF = null;
    boolean steadyOnly = false; // if we need steady only
    final static double voltageForHorizontal = 4; // voltage for staying at a horizontal state

    /**
     * Constructor with default values - only required the setPower, setVelocity,
     * min/max power and subsystems
     * Constructor for meter unit
     * 
     * @param setPower    the power wanted
     * @param getVelocity motors velocity
     * @param minPow      the min power given
     * @param maxPow      the max power
     * @param subsystems  subsystem required
     */
    public Sysid(Consumer<Double> setPower,
            Supplier<Double> getVelocity,
            double minPow,
            double maxPow,
            Supplier<Double> getAccel,
            Subsystem... subsystems) {
        this(new Gains[] { Gains.KS, Gains.KV, Gains.KA, Gains.KV2 },
                setPower,
                getVelocity,
                minPow,
                maxPow,
                20,
                defaultDuration,
                defaultDelay,
                getAccel,
                subsystems);
    }

    /**
     * Constructor with additional values - duration and delay
     * 
     * @param setPower           holds the power given to the motor
     * @param getVelocity        current velocity
     * @param minPow             min power that can be given
     * @param maxPow             max power that can be given
     * @param subsystems         needed subsystem
     * @param powerCycleDuration for how long each power should be activated
     */
    public Sysid(Consumer<Double> setPower,
            Supplier<Double> getVelocity,
            double minVel,
            double maxVel,
            double powerCycleDuration,
            double powerCycleDelay,
            Supplier<Double> getAccel,
            Subsystem... subsystems) {
        this(new Gains[] { Gains.KS, Gains.KV, Gains.KA },
                setPower,
                getVelocity,
                minVel,
                maxVel,
                20,
                powerCycleDuration,
                powerCycleDelay,
                getAccel,
                subsystems);
    }

    /**
     * Constructor with all parameters
     * 
     * @param setPower    holds in the power
     * @param getVelocity gives the motors velocity
     * @param minPow      min power that can be given
     * @param maxPow      max power that can be given
     * @param subsystems  needed subsystem
     */
    public Sysid(Gains[] types,
            Consumer<Double> setPower,
            Supplier<Double> getVelocity,
            double minVel,
            double maxVel,
            int nPowerCycles,
            double powerCycleDuration,
            double powerCycleDelay,
            Supplier<Double> getAccel,
            Subsystem... subsystems) {

        this.setPower = setPower;
        dataCollector = new DataCollector(types, getVelocity, nPowerCycles, powerCycleDuration, voltageForHorizontal, getAccel);
        this.maxVel = maxVel;
        this.nPowerCycles = nPowerCycles;
        this.powerCycleDelay = powerCycleDelay;
        this.powerCycleDuration = powerCycleDuration;
        deltaPower = (double) (maxVel - minVel) / (nPowerCycles);
        this.subsystems = subsystems;
        this.gains = types;
        SmartDashboard.putNumber("deltaPow", deltaPower);

    }



    /**
     * 
     * @param setPower    power given to the motor
     * @param getVelocity current velocity
     * @param minPow      min power (you cant go below it)
     * @param maxPow      max power (you cant go above it)
     * @param powerStep
     * @param minPow      min velocity (you cant go below it)
     * @param maxPow      max velocity (you cant go below it)
     * @param isMeter     decides which unints to use
     * @param subsystems  subsystem needed to run on
     * @param isRadian    TBD for now false
     * @return commad
     */
    public static Command getSteadyCommand(Consumer<Double> setPower, Supplier<Double> getVelocity, double minPow,
            double maxPow, double powerStep, boolean isMeter, Supplier<Double> getAccel,
            Subsystem... subsystems) {
        Sysid id = new Sysid(new Gains[] { Gains.KS, Gains.KV, Gains.KV2 }, setPower, getVelocity,  minPow,
                maxPow, 20, 1, 1, getAccel, subsystems);
        id.steadyOnly = true;
        Command cmd = new NoAccelerationPowerCommand(setPower, powerStep, id.dataCollector, false,
                minPow, maxPow, maxPow,  subsystems);
        return cmd.andThen(new InstantCommand(() -> id.analyze()));
    }

    /**
     * run the command
     */
    public void run() {
        runNormalSysId().schedule();
    }


    /**
     * calculate the power for cycle
     * cycles run - minPow, -minPow, (minPow+delta),
     * -(minPow+delts).....(maxPow), -max(Power)
     * 
     * @param cycle
     * @param pow   power for every cycle
     * @return double
     */
    double power(int cycle) {
        int pow = cycle / 2;
        double sign = cycle % 2 == 0 ? 1 : -1;
        return sign * (minPow + pow * deltaPower);
    }

    /**
     * Generate the command to run the system at different powers, collect the data
     * and analyze the result
     * there is a delay between each power cycle
     * 
     * @return Command
     */
    public Command runNormalSysId() {
        boolean resetDataCollector = true;
        Command cmd = new WaitCommand(powerCycleDelay);
        for (int cycle = 0; cycle < nPowerCycles; cycle++) {
            double power = minPow + cycle * 0.01;
            cmd = cmd.andThen(getPowerCommand(power, resetDataCollector));
            resetDataCollector = false;
        }
        SmartDashboard.putNumber("deltaPower", deltaPower);
        return cmd.andThen(new InstantCommand(() -> analyze()));
    }


    /**
     * Get the command for a power - with the duration and delay
     */
    Command getPowerCommand(double power, boolean resetDataCollector) {
        return ((new PowerCycleCommand(setPower, power, dataCollector, resetDataCollector, maxVel,
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
     * @param dataCollector     holds in raw data such as current velocity,
     *                          acceleration
     * @param power             calculated output
     * @param error             is the error between the expected output (power())
     *                          and the calculated one power
     * @param errorSquared      is
     * @param max               the largest error, a measure of the worst-case
     *                          deviation
     * @param avg               the average squared error, a measure of overall
     *                          accuracy.
     */
    public void analyze() {
        setPower.accept(0.0);
        // SimpleMatrix feedForwardValues30 = dataCollector.solve();
        // SimpleMatrix feedForwardValues50 = dataCollector.solveRange60();
        // SimpleMatrix feedForwardValues70 = dataCollector.solveRange100();

        // SimpleMatrix testFF = dataCollector.testFF();
        SimpleMatrix test = dataCollector.solveFirstRange();

        result30 = new double[gains.length];
        result50 = new double[gains.length];
        resultFF = new double[gains.length];
        for (int i = 0; i < gains.length; i++) {
            //result30[i] = feedForwardValues30.get(i, 0);
            result50[i] = test.get(i, 0);
            //resultFF[i] = testFF.get(i, 0);
            SmartDashboard.putNumber("SysID-" + gains[i] + "-0-30 ranges", result30[i]);
            SmartDashboard.putNumber("SysId-" + gains[i] + "-testVel", result50[i]);
            SmartDashboard.putNumber("SysId-" + gains[i] + "testFF", resultFF[i]);
            // System.out.println("Sysid: " + gains[i] + " = " + result[i]);
        }
        double avgKS, avgKV, avgKA = 0.0;
        
        avgKS = (result30[0] + result50[0] + resultFF[0])/gains.length;
        avgKV = (result30[1] + result50[1] + resultFF[1])/gains.length;
        avgKA = (result30[2] + result50[2] + resultFF[2])/gains.length;
        SmartDashboard.putNumber("avgKS", avgKS);
        SmartDashboard.putNumber("avgKV", avgKV);
        SmartDashboard.putNumber("avgKA", avgKA);
        // for(int i = 0; i < gains.length; i++){
        // SmartDashboard.putNumber("SysID-" + gains[i] + "-0-50 ranges", result50[i]);
        // }
        SimpleMatrix power = dataCollector.dataRange30().mult(test);
        SimpleMatrix error = dataCollector.power().minus(power);
        SimpleMatrix errorSquared = error.elementMult(error);
        double max = Math.sqrt(errorSquared.elementMax());
        double avg = errorSquared.elementSum() / errorSquared.getNumRows();
        SmartDashboard.putNumber("Sysid-Max Error", max);
        SmartDashboard.putNumber("Sysid-Avg Error Sqr", avg);
        double kp = (valueOf(Gains.KV, gains, result30) + valueOf(Gains.KA, gains, result30)) / 5.0;
        SmartDashboard.putNumber("Sysid-KP (Roborio)", kp);
    }

    /**
     * Value of a specific Gain type
     * 
     * @param gain   the given feedforward variable
     * @param gains  all the given feedforward variables (like KS, KV, KA, etc...)
     * @param values raw data of the ff variables (such as velocity, acceleration)
     * @return the value needed for the ff variable or 0 if found none
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
        return result30;
    }

    /**
     * 
     * @return result array of gain types
     */
    public Gains[] gains() {
        return gains;
    }

}