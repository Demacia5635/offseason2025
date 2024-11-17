package frc.robot.Sysid;

import java.util.function.Supplier;
import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.units.Velocity;

import static frc.robot.Sysid.Sysid.Gains;

/**
 * Data collector class
 * 
 * Store the data in simple matrix (n X number of gains)
 * Store the power in a different simple matrx (n X 1)
 * Uses supplied get function to get the velocity and other optional data
 * (radinas and position)
 * calculate the best gains
 */
public class DataCollector {

    SimpleMatrix data;
    SimpleMatrix dataRange50;
    SimpleMatrix dataRange70;
    SimpleMatrix powerRange20;
    SimpleMatrix powerRange50;
    SimpleMatrix powerRange70;
    int nextRow;
    Gains[] gains;
    Supplier<Double> getVelocity;
    Supplier<Double> getRadians;
    Supplier<Double> getMeter;
    int nPowerCycles;
    double powerCycleDuration;
    double lastV = 0;

    /**
     * Constructor with all required data
     * 
     * @param gains              values you want to recieve (for example ks, kv ,
     *                           ka)
     * @param getVelocity        motors velocity
     * @param getRadians         holds the radians can be null if work with meters
     * @param getMeter           holds the meters can be nukk if work with radians
     * @param nPowerCycles
     * @param powerCycleDuration
     * @param isMeter            decides if sysid works on drive or angle
     */

    public DataCollector(Gains[] gains, Supplier<Double> getVelocity, Supplier<Double> getRadians,
            Supplier<Double> getMeter, int nPowerCycles, double powerCycleDuration) {
        this.gains = gains;
        this.getVelocity = getVelocity;
        this.getRadians = getRadians;
        this.getMeter = getMeter;
        this.nPowerCycles = nPowerCycles;
        this.powerCycleDuration = powerCycleDuration;
        int matrixRows = (int) (nPowerCycles * 2 * powerCycleDuration / 0.02) + 100; // the maximum number of rows we
                                                                                     // will need + safety value
        data = new SimpleMatrix(matrixRows, gains.length);
        dataRange50 = new SimpleMatrix(matrixRows, gains.length);
        dataRange70 = new SimpleMatrix(matrixRows, gains.length);
        powerRange20 = new SimpleMatrix(matrixRows, 1);
        powerRange50 = new SimpleMatrix(matrixRows, 1);
        powerRange70 = new SimpleMatrix(matrixRows, 1);
        nextRow = 0;
        lastV = 0;
    }

    /**
     * Function to collect the data
     * Adding the values for each required gain type
     * Adding the power to the right power matrix which is between the ranges of
     * 1-100 (including minus)
     * 
     * @param power        is in -12 to 12 voltage unit
     * @param nextRow      adds a new row each time a data is collected
     * @param powerRange20 adds all the power from range 1-20
     * @param data         add raw data to the matrix (velocity, signum(velocity))
     * @param i            signify the column of the maxtrix and index
     */
    public void collect(double power) {
        if (Math.abs(1 - power) >= 0) {
            double powerInPerecent = Math.abs(power * 100);
            if (powerInPerecent <= 100 && powerInPerecent >= 70) {
                this.powerRange70.set(nextRow, 0, power);
            }

            else if (powerInPerecent < 70 && powerInPerecent >= 20) {
                this.powerRange50.set(nextRow, 0, power);
            }

            else {
                this.powerRange20.set(nextRow, 0, power);
            }

            double velocity = getVelocity.get();
            double rad = getRadians != null ? getRadians.get() : 0;
            double meter = getMeter != null ? getMeter.get() : 0;
            for (int i = 0; i < gains.length; i++) {
                if (velocity > 0 && velocity <= 10) {
                    data.set(nextRow, i, value(gains[i], velocity, rad, meter));
                }

                else if (velocity >= 11 && velocity <= 20) {
                    dataRange50.set(nextRow, i, value(gains[i], velocity, rad, meter));
                }

                else {
                    dataRange70.set(nextRow, i, value(gains[i], velocity, rad, meter));
                }
                // data.set(nextRow, i, value(gains[i], v, rad, meter));
            }
            lastV = velocity;
            nextRow++;
        }

    }

    /**
     * Calculate the applicable value based on the gain type and provided data
     * 
     * @param gain
     * @param velocity
     * @param rad
     * @param meter
     * @return applicable value
     */
    double value(Gains gain, double velocity, double rad, double meter) {
        switch (gain) {
            case KS:
                return Math.signum(velocity);
            case KV:
                return velocity;
            case KA:
                return velocity - lastV;
            case KRad:
                return rad;
            case KMeter:
                return meter;
            case KCos:
                return Math.cos(rad);
            case KSin:
                return Math.sin(rad);
            case KTan:
                return Math.tan(rad);
            case KV2:
                return velocity * velocity;
            case KVsqrt:
                return Math.sqrt(Math.abs(velocity));
            default:
                return 0;
        }
    }

    /**
     * Function to set the lastV
     */
    public void resetLastV() {
        lastV = getVelocity.get();
    }

    /**
     * Function to reset the data
     */
    public void resetData() {
        nextRow = 0;
    }

    /**
     * 
     * @return the colllected data in power ranges of 0 - 20
     */
    public SimpleMatrix dataRange20() {
        return data.rows(0, nextRow);
    }

    public SimpleMatrix dataRange50() {
        return dataRange50.rows(0, nextRow);
    }

    public SimpleMatrix dataRange70() {
        return dataRange70.rows(0, nextRow);
    }

    /**
     * 
     * @return the collected power range 0-20 (power matrix for ranges of 0-20
     *         power)
     */
    public SimpleMatrix power() {
        return powerRange20.rows(0, nextRow);
    }

    /**
     * 
     * @return the collected power range 21-69
     */
    public SimpleMatrix powerRange50() {
        return powerRange50.rows(0, nextRow);
    }

    /**
     * 
     * @return the collected power range 70-100
     */
    public SimpleMatrix powerRange70() {
        return powerRange70.rows(0, nextRow);
    }

    /**
     * 
     * @return the gains (feed forward values) matrix for ranges 0-20 power
     */
    public SimpleMatrix solve() {
        return dataRange20().solve(power());
    }

    /**
     * 
     * @return the gains matrix for ranges 21-69
     */
    public SimpleMatrix solveRange50() {
        return dataRange50().solve(powerRange50());
    }

    /**
     * 
     * @return the gains matrix for ranges 70-100
     */
    public SimpleMatrix solveRange70() {
        return dataRange70().solve(powerRange70());
    }

    /**
     * 
     * @return the array of gains type
     */
    public Gains[] gains() {
        return gains;
    }

}