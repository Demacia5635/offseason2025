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
        KS, KV, KA, KRad, KV2, KVsqrt, KSin, KCos, KTan;
    }

    Consumer<Double> setPower; // function to set the power
    DataCollector dataCollector; // the data collector class
    double minPow;
    double maxPow;
    double deltaPower; // the change of power between power cycles
    int nPowerCycles; // how many powers to use (the system will run each power in positive and
                      // negative values)
    double powerCycleDuration; // how long each power cycle
    double powerCycleDelay; // delay between power cycles
    Subsystem[] subsystems; // for add requirements
    final static double defaultDuration = 2.5;
    final static double defaultDelay = 10;
    Gains[] gains; // the gains we are looking for
    double[] result20 = null; // the result, after analyze
    double[] result50 = null;
    double[] result70 = null;
    boolean steadyOnly = false; // if we need steady only
    boolean isRadian;

    /**
     * Constructor with default values - only required the setPower, setVelocity,
     * min/max power and subsystems
     * Constructor for meter unit
     * 
     * @param setPower    the power wanted
     * @param getVelocity motors velocity
     * @param minPow the min power given
     * @param maxPow the max power
     * @param subsystems  subsystem required
     */
    public Sysid(Consumer<Double> setPower,
            Supplier<Double> getVelocity,
            double minPow,
            double maxPow,
            Subsystem... subsystems) {
        this(new Gains[] { Gains.KS, Gains.KV, Gains.KA, Gains.KV2 },
                setPower,
                getVelocity,
                null,
                minPow,
                maxPow,
                20,
                defaultDuration,
                defaultDelay,
                false,
                subsystems);
    }

    /**
     * Constructor with additional values - duration and delay
     * 
     * @param setPower           holds the power given to the motor
     * @param getVelocity        current velocity
     * @param minPow        min power that can be given
     * @param maxPow        max power that can be given
     * @param subsystems         needed subsystem
     * @param powerCycleDuration for how long each power should be activated
     */
    public Sysid(Consumer<Double> setPower,
            Supplier<Double> getVelocity,
            double minPow,
            double maxPow,
            double powerCycleDuration,
            double powerCycleDelay,
            boolean isRadian,
            Subsystem... subsystems) {
        this(new Gains[] { Gains.KS, Gains.KV, Gains.KA },
                setPower,
                getVelocity,
                null,
                minPow,
                maxPow,
                20,
                powerCycleDuration,
                powerCycleDelay,
                isRadian,
                subsystems);
    }

    /**
     * Constructor with all parameters
     * 
     * @param setPower    holds in the power
     * @param getVelocity gives the motors velocity
     * @param minPow min power that can be given
     * @param maxPow max power that can be given
     * @param subsystems  needed subsystem
     */
    public Sysid(Gains[] types,
            Consumer<Double> setPower,
            Supplier<Double> getVelocity,
            Supplier<Double> getRadians,
            double minPow,
            double maxPow,
            int nPowerCycles,
            double powerCycleDuration,
            double powerCycleDelay,
            boolean isRadian,
            Subsystem... subsystems) {

        this.setPower = setPower;
        dataCollector = new DataCollector(types, getVelocity, nPowerCycles, powerCycleDuration);
        this.minPow = minPow;
        this.maxPow = maxPow;
        this.nPowerCycles = nPowerCycles;
        this.powerCycleDelay = powerCycleDelay;
        this.powerCycleDuration = powerCycleDuration;
        deltaPower = (double) (maxPow - minPow) / (nPowerCycles);
        this.subsystems = subsystems;
        this.gains = types;
        SmartDashboard.putNumber("deltaPow", deltaPower);

    }

    /**
     * 
     * @param setPower    power given to the motor
     * @param getVelocity current velocity
     * @param minPow min power (you cant go below it)
     * @param maxPow max power (you cant go above it)
     * @param powerStep
     * @param minPow min velocity (you cant go below it)
     * @param maxPow max velocity (you cant go below it)
     * @param isMeter     decides which unints to use
     * @param subsystems  subsystem needed to run on
     * @param isRadian    TBD for now false
     * @return commad
     */
    public static Command getSteadyCommand(Consumer<Double> setPower, Supplier<Double> getVelocity, double minPow,
            double maxPow, double powerStep, boolean isMeter,
            Subsystem... subsystems) {
        Sysid id = new Sysid(new Gains[] { Gains.KS, Gains.KV, Gains.KV2 }, setPower, getVelocity, null, minPow,
                maxPow, 20, 1, 1, false, subsystems);
        id.steadyOnly = true;
        Command cmd = new NoAccelerationPowerCommand(setPower, powerStep, id.dataCollector, false,
                minPow, maxPow, maxPow, false, subsystems);
        return cmd.andThen(new InstantCommand(() -> id.analyze()));
    }

    /**
     * run the command
     */
    public void run() {
        getCommand().schedule();
    }

    public Command runSysId() {
        return getCommand();
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
    public Command getCommand() {
        boolean resetDataCollector = true;
        Command cmd = new WaitCommand(powerCycleDelay);
        for (int cycle = 0; cycle < nPowerCycles; cycle++) {
            double power = minPow + cycle * deltaPower;
            cmd = cmd.andThen(getPowerCommand(power, resetDataCollector));
            resetDataCollector = false;
        }
        SmartDashboard.putNumber("deltaPower", deltaPower);
        return cmd.andThen(new InstantCommand(() -> analyze()));
    }

    public Command getCommandOneWay() {
        boolean resetDataCollector = true;
        Command cmd = new WaitCommand(powerCycleDelay);
        for (int c = 0; c < nPowerCycles; c++) {
            double power = minPow + c * deltaPower;
            cmd = cmd.andThen(getPowerCommand(power, resetDataCollector));
            resetDataCollector = false;
            // cmd = cmd.andThen(getPowerCommand(-power, resetDataCollector));
        }
        return cmd.andThen(new InstantCommand(() -> analyze()));
    }

    /**
     * Get the command for a power - with the duration and delay
     */
    Command getPowerCommand(double velocity, boolean resetDataCollector) {
        return ((new PowerCycleCommand(setPower, velocity, dataCollector, resetDataCollector, maxPow, minPow, subsystems))
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
     * 
     */
    public void analyze() {
        setPower.accept(0.0);
        SimpleMatrix feedForwardValues20 = dataCollector.solve();
        SimpleMatrix feedForwardValues50 = dataCollector.solveRange50();
        SimpleMatrix feedForwardValues70 = dataCollector.solveRange70();
        result20 = new double[gains.length];
        result50 = new double[gains.length];
        result70 = new double[gains.length];
        for (int i = 0; i < gains.length; i++) {
            result20[i] = feedForwardValues20.get(i, 0);
            result50[i] = feedForwardValues50.get(i,0);
            result70[i] = feedForwardValues70.get(i,0);
            SmartDashboard.putNumber("SysID-" + gains[i] + "-0-20 ranges", result20[i]);
            // System.out.println("Sysid: " + gains[i] + " = " + result[i]);
        }
        // for(int i = 0; i < gains.length; i++){
        //     SmartDashboard.putNumber("SysID-" + gains[i] + "-0-50 ranges", result50[i]);    
        // }
        SmartDashboard.putNumber("sysid-Check" + gains[0], result20[0]);
        SmartDashboard.putNumber("feed forward 20 columns", feedForwardValues20.getNumCols());
        SmartDashboard.putNumber("data range 20 rows", dataCollector.dataRange20().getNumRows());
        SimpleMatrix power = dataCollector.dataRange20().mult(feedForwardValues20);
        SimpleMatrix e = dataCollector.power().minus(power);
        SimpleMatrix ee = e.elementMult(e);
        double max = Math.sqrt(ee.elementMax());
        double avg = ee.elementSum() / ee.getNumRows();
        SmartDashboard.putNumber("Sysid/Max Error", max);
        SmartDashboard.putNumber("Sysid/Avg Error Sqr", avg);
        double kp = (valueOf(Gains.KV, gains, result20) + valueOf(Gains.KA, gains, result20)) / 5.0;
        SmartDashboard.putNumber("Sysid/KP (Roborio)", kp);
    }
    // R ﻿﻿ 1 ﻿﻿ The startCompetition() method (or methods called by it) should have
    // handled the exception above. ﻿﻿
    // edu.wpi.first.wpilibj.RobotBase.runRobot(RobotBase.java:386) ﻿﻿﻿

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
        return result20;
    }

    /**
     * 
     * @return result array of gain types
     */
    public Gains[] gains() {
        return gains;
    }
/*RROR ﻿﻿ 1 ﻿﻿ The startCompetition() method (or methods called by it) should have handled the exception above. ﻿﻿ edu.wpi.first.wpilibj.RobotBase.runRobot(RobotBase.java:386) ﻿﻿﻿
﻿﻿﻿﻿﻿﻿ Error at edu.wpi.first.wpilibj.RobotBase.runRobot(RobotBase.java:386): The startCompetition() method (or methods called by it) should have handled the exception above. ﻿
 */

 /*
  * t frc.robot.Main.main(Main.java:23) ﻿
﻿﻿﻿﻿﻿﻿  ﻿
﻿﻿﻿﻿﻿﻿﻿﻿Warning ﻿﻿ 1 ﻿﻿ The robot program quit unexpectedly. This is usually due to a code error.
  The above stacktrace can help determine where the error occurred.
  See https://wpilib.org/stacktrace for more information. ﻿﻿ edu.wpi.first.wpilibj.RobotBase.runRobot(RobotBase.java:379) ﻿﻿﻿
﻿﻿﻿﻿﻿﻿ ﻿Warning﻿ at edu.wpi.first.wpilibj.RobotBase.runRobot(RobotBase.java:379): The robot program quit unexpectedly. This is usually due to a code error. ﻿
﻿﻿﻿﻿﻿﻿   The above stacktrace can help determine where the error occurred. ﻿
﻿﻿﻿﻿﻿﻿   See https://wpilib.org/stacktrace for more information. ﻿
﻿﻿﻿﻿﻿﻿﻿﻿ERROR ﻿﻿ 1 ﻿﻿ The startCompetition() method (or methods called by it) should have handled the exception above. ﻿﻿ edu.wpi.first.wpilibj.RobotBase.runRobot(RobotBase.java:386) ﻿﻿﻿
﻿﻿﻿﻿﻿﻿ Error at edu.wpi.first.wpilibj.RobotBase.runRobot(RobotBase.java:386): The startCompetition() method (or methods called by it) should have handled the exception above. ﻿
﻿﻿﻿﻿﻿﻿ ********** Robot program starting ********** ﻿

  */

  /*
   *  	Shuffleboard.update(): 0.000019s ﻿
﻿﻿﻿﻿﻿﻿  ﻿
﻿﻿﻿﻿﻿﻿﻿﻿ERROR ﻿﻿ -1003 ﻿﻿ CAN frame not received/too-stale. Check the CAN bus wiring, CAN bus utilization, and power to the device. ﻿﻿ talon fx 8 ("rio") Refresh Config ﻿﻿﻿
﻿﻿﻿﻿﻿﻿﻿﻿Warning ﻿﻿ 1 ﻿﻿ Loop time of 0.02s overrun
 ﻿﻿ edu.wpi.first.wpilibj.IterativeRobotBase.printLoopOverrunMessage(IterativeRobotBase.java:412) ﻿﻿﻿
﻿﻿﻿﻿﻿﻿ ﻿Warning﻿ at edu.wpi.first.wpilibj.IterativeRobotBase.printLoopOverrunMessage(IterativeRobotBase.java:412): Loop time of 0.02s overrun ﻿
﻿﻿﻿﻿﻿﻿  ﻿
﻿﻿﻿﻿﻿﻿﻿﻿Warning ﻿﻿ 1 ﻿﻿ 	teleopPeriodic(): 0.000445s
	SmartDashboard.updateValues(): 0.046671s
	robotPeriodic(): 0.001800s
	LiveWindow.updateValues(): 0.000000s
	Shuffleboard.update(): 0.000000s
 ﻿﻿ edu.wpi.first.wpilibj.Tracer.lambda$printEpochs$0(Tracer.java:62) ﻿﻿﻿
﻿﻿﻿﻿﻿﻿ ﻿Warning﻿ at edu.wpi.first.wpilibj.Tracer.lambda$printEpochs$0(Tracer.java:62): 	teleopPeriodic(): 0.000445s ﻿
﻿﻿﻿﻿﻿﻿ 	SmartDashboard.updateValues(): 0.046671s ﻿
﻿﻿﻿﻿﻿﻿ 	robotPeriodic(): 0.001800s ﻿
﻿﻿﻿﻿﻿﻿ 	LiveWindow.updateValues(): 0.000000s ﻿
﻿﻿﻿﻿﻿﻿ 	Shuffleboard.update(): 0.000000s ﻿
﻿﻿﻿﻿﻿﻿  ﻿
﻿﻿﻿﻿﻿﻿﻿﻿Warning ﻿﻿ 1 ﻿﻿ Loop time of 0.02s overrun
 ﻿﻿ edu.wpi.first.wpilibj.IterativeRobotBase.printLoopOverrunMessage(IterativeRobotBase.java:412) ﻿﻿﻿
﻿﻿﻿﻿﻿﻿ ﻿Warning﻿ at edu.wpi.first.wpilibj.IterativeRobotBase.printLoopOverrunMessage(IterativeRobotBase.java:412): Loop time of 0.02s overrun ﻿
﻿﻿﻿﻿﻿﻿  ﻿

   */
}