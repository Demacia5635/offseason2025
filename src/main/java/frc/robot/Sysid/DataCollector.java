package frc.robot.Sysid;

import java.util.function.Supplier;
import org.ejml.simple.SimpleMatrix;
import org.opencv.core.Mat;

import edu.wpi.first.units.Velocity;

import static frc.robot.Sysid.Sysid.Gains;

/**
 * Data collector class
 * 
 * Store the dataRange20 in simple matrix (n X number of gains)
 * Store the power in a different simple matrx (n X 1)
 * Uses supplied get function to get the velocity and other optional dataRange20
 * (radinas and position)
 * calculate the best gains
 */
public class DataCollector {

    SimpleMatrix dataRange20;
    SimpleMatrix dataRange50;
    SimpleMatrix dataRange70;
    SimpleMatrix velocityRange20;
    SimpleMatrix velocityRange50;
    SimpleMatrix velocityRange70;
    int nextRowRange20;
    int nextRowRange50;
    int nextRowRange70;
    Gains[] gains;
    Supplier<Double> getVelocity;
    int nPowerCycles;
    double powerCycleDuration;
    double lastV = 0;
    double Range1;
    double Range20;
    double Range50;
    double rad = 0;


    /**
     * Constructor for getRadians option
     * @param gains feed forward values
     * @param getVelocity current velocity
     * @param getRadians radians
     * @param nPowerCycles 
     * @param powerCycleDuration
     */
    public DataCollector(Gains[] gains, Supplier<Double> getVelocity,
    int nPowerCycles, double powerCycleDuration) {
         this.gains = gains;
        this.getVelocity = getVelocity;
        this.nPowerCycles = nPowerCycles;
        this.powerCycleDuration = powerCycleDuration;
        int matrixRows = (int) (nPowerCycles * 2 * powerCycleDuration / 0.02) + 100; // the maximum number of rows we
                                                                                     // will need + safety value
        dataRange20 = new SimpleMatrix(matrixRows, gains.length);
        dataRange50 = new SimpleMatrix(matrixRows, gains.length);
        dataRange70 = new SimpleMatrix(matrixRows, gains.length);
        velocityRange20 = new SimpleMatrix(matrixRows, 1);
        velocityRange50 = new SimpleMatrix(matrixRows, 1);
        velocityRange70 = new SimpleMatrix(matrixRows, 1);
        nextRowRange20 = 0;
        nextRowRange50 = 0;
        nextRowRange70 = 0;
        lastV = 0;
    }

    /**
     * Constructor with all required dataRange20
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

  

    /**
     * Function to collect the dataRange20
     * Adding the values for each required gain type
     * Adding the power to the right power matrix which is between the ranges of
     * 1-100 (including minus)
     * 
     * @param power        is in -12 to 12 voltage unit
     * @param nextRow      adds a new row each time a dataRange20 is collected
     * @param velocityRange20 adds all the power from range 1-20
     * @param dataRange20         add raw dataRange20 to the matrix (velocity, signum(velocity))
     * @param i            signify the column of the maxtrix and index
     */
    public void collect(double power, double maxVelocity, double minVelocity, boolean isRadian) {
        if (valid(power, minVelocity, isRadian)) {
            Range1 = maxVelocity - minVelocity; 
            Range20 = minVelocity + (Range1*0.2);
            Range50 = minVelocity + (Range1 * 0.5);
            //100 rotation per sec, nextRow for each matrix
            double velocity = getVelocity.get();
            for (int i = 0; i < gains.length; i++) {
                if (velocity >= Range20) {
                    dataRange20.set(nextRowRange20, i, value(gains[i], velocity, rad));
                    velocityRange20.set(i, 0, power);
                    nextRowRange20++;
                }

                else if (velocity >= Range50) {
                    dataRange50.set(nextRowRange50, i, value(gains[i], velocity, rad));
                    velocityRange50.set(i, 0, power);
                    nextRowRange50++;
                }

                else if(velocity <= maxVelocity){
                    dataRange70.set(nextRowRange70, i, value(gains[i], velocity, rad));
                    velocityRange70.set(i, 0, power);
                    nextRowRange70++;
                }
                
                //this.velocityRange70.set(nextRow, 0, power);
                // dataRange20.set(nextRow, i, value(gains[i], v, rad, meter));
            }
            lastV = velocity;
        }

    }

    private boolean valid(double power, double minVelocity, boolean isRadian){
        if(getVelocity == null) return false;
        if(isRadian){
            
            return (power != 0 && Math.abs(getVelocity.get()) > minVelocity);
        }
        return (power != 0 && Math.abs(getVelocity.get()) > minVelocity);
    }

    /**
     * Calculate the applicable value based on the gain type and provided dataRange20
     * 
     * @param gain wanted feed forward param
     * @param velocity current velocity
     * @param rad
     * @param meter
     * @return raw value of each gain (feed forward raw dataRange20)
     */
    double value(Gains gain, double velocity, double rad) {
        switch (gain) {
            case KS:
                return Math.signum(velocity);
            case KV:
                return velocity;
            case KA:
                return velocity - lastV;
            case KRad:
                return rad;
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
     * Function to reset the dataRange20
     */
    public void resetData() {
        nextRowRange20 = 0;
        nextRowRange50 = 0;
        nextRowRange70 = 0;
    }

    /**
     * 
     * @return extracts a sub matrix with the values of the original from the start to nextRow
     */
    public SimpleMatrix dataRange20() {
        return dataRange20.rows(0, nextRowRange20);
    }

    public SimpleMatrix dataRange50() {
        return dataRange50.rows(0, nextRowRange50);
    }

    public SimpleMatrix dataRange70() {
        return dataRange70.rows(0, nextRowRange70);
    }

    /**
     * 
     * @return the collected power range 0-20 (power matrix for ranges of 0-20
     *         power)
     */
    public SimpleMatrix power() {
        return velocityRange20.rows(0, nextRowRange20);
    }

    /**
     * 
     * @return the collected power range 21-69
     */
    public SimpleMatrix velocityRange50() {
        return velocityRange50.rows(0, nextRowRange50);
    }

    /**
     * 
     * @return the collected power range 70-100
     */
    public SimpleMatrix velocityRange70() {
        return velocityRange70.rows(0, nextRowRange70);
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
        return dataRange50().solve(velocityRange50());
    }

    /**
     * 
     * @return the gains matrix for ranges 70-100
     */
    public SimpleMatrix solveRange70() {
        return dataRange70().solve(velocityRange70());
    }

    /**
     * 
     * @return the array of gains type
     */
    public Gains[] gains() {
        return gains;
    }

}