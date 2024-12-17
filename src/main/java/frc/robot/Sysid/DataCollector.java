package frc.robot.Sysid;

import java.util.function.Supplier;
import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.units.Velocity;
import frc.robot.utils.LogManager;
import frc.robot.utils.LogManager.LogEntry;

import static frc.robot.Sysid.Sysid.Gains;

/**
 * Data collector class
 * 
 * Store the dataRange30 in simple matrix (n X number of gains)
 * Store the power in a different simple matrx (n X 1)
 * Uses supplied get function to get the velocity and other optional dataRange30
 * (radinas and position)
 * calculate the best gains
 */
public class DataCollector {

    SimpleMatrix dataRange30;
    SimpleMatrix dataRange60;
    SimpleMatrix dataRange100;
    SimpleMatrix powerRange30;
    SimpleMatrix powerRange60;
    SimpleMatrix powerRange100;
    int nextRowRange30;
    int nextRowRange60;
    int nextRowRange100;
    Gains[] gains;
    Supplier<Double> getVelocity;
    int nPowerCycles;
    double powerCycleDuration;
    double lastV = 0;
    double Range1;
    double Range30;
    double Range50;
    double rad = 0; //current angle in in rad
    double voltageForHorizontal;
    Supplier<Double> getAccel;

    SimpleMatrix testForData;
    SimpleMatrix testForPower;

    DataTable[] datas;
    int row = 0;

    

    /**
     * Constructor for Arm feedforward
     * @param gains feed forward values
     * @param getVelocity current velocity
     * @param nPowerCycles 
     * @param powerCycleDuration
     * @param voltageForHorizontal needed voltage for arm to stay in a horizontal state
     */
    public DataCollector(Gains[] gains, Supplier<Double> getVelocity,
    int nPowerCycles, double powerCycleDuration, double voltageForHorizontal, Supplier<Double> getAccel) {
         this.gains = gains;
        this.getVelocity = getVelocity;
        this.nPowerCycles = nPowerCycles;
        this.powerCycleDuration = powerCycleDuration;
        int matrixRows = (int) (nPowerCycles * 2 * powerCycleDuration / 0.02) + 100; // the maximum number of rows we
                                                                                     // will need + safety value
        dataRange30 = new SimpleMatrix(matrixRows, gains.length);
        dataRange60 = new SimpleMatrix(matrixRows, gains.length);
        dataRange100 = new SimpleMatrix(matrixRows, gains.length);
        powerRange30 = new SimpleMatrix(matrixRows, 1);
        powerRange60 = new SimpleMatrix(matrixRows, 1);
        powerRange100 = new SimpleMatrix(matrixRows, 1);
        // nextRowRange30 = 0;
        // nextRowRange60 = 0;
        // nextRowRange100 = 0;
        lastV = 0;
        this.getAccel = getAccel;
        this.voltageForHorizontal = voltageForHorizontal;
        testForData = new SimpleMatrix(matrixRows, gains.length);
        testForPower = new SimpleMatrix(matrixRows, 1);
        datas = new DataTable[3];
        datas[0] = new DataTable(0, matrixRows, gains.length);
        datas[1] = new DataTable(0, matrixRows, gains.length);
        datas[2] = new DataTable(0, matrixRows, gains.length);
        
    }



    /**
     * Constructor with all required dataRange30
     * 
     * @param gains              values you want to recieve (for example ks, kv ,
     *                           ka)
     * @param getVelocity        motors velocity
     * @param nPowerCycles
     * @param powerCycleDuration
     * @param isMeter            decides if sysid works on drive or angle
     */

  

    /**
     * Function to collect the dataRange30
     * Adding the values for each required gain type
     * Adding the power to the right power matrix which is between the ranges of
     * 1-100 (including minus)
     * 
     * @param power        is in -1 to 1 voltage unit
     * @param nextRow      adds a new row each time a dataRange30 is collected
     * @param powerRange30 adds all the power from range 1-30
     * @param powerRange100 adds all the power from range 31-70
     * @param powerRange100 adds all the power from range 71-100
     * @param dataRange30         add raw dataRange30 to the matrix (velocity, signum(velocity))
     * @param i            signify the column of the maxtrix and index
     */
    public void collect(double maxVel) {
        if (valid(maxVel)) {
            double range = maxVel * 0.3;
            double velocity = getVelocity.get();
            double acceleration = 0;//getAccel.get() !=  null ? getAccel.get() : 0;
            row = getRange(maxVel, velocity);
            for (int i = 0; i < gains.length; i++) {
                datas[row].dataMatrix.set(datas[0].getRow(), i, value(gains[i], velocity, rad, acceleration));
                datas[row].updateRow();
                if(velocity <= range){
                    dataRange30.set(nextRowRange30, i, value(gains[i], velocity, rad, acceleration));
                    nextRowRange30++;
                }
            //     if (velocity <= Range30) {
            //         dataRange30.set(nextRowRange30, i, value(gains[i], velocity, rad, acceleration));
            //         powerRange30.set(nextRowRange30, 0, velocity);
            //         nextRowRange30++;
            //     }
                
            //     else if (velocity <= Range50) {
            //         dataRange60.set(nextRowRange60, i, value(gains[i], velocity, acceleration, rad));
            //         powerRange60.set(nextRowRange60, 0, velocity);
            //         nextRowRange60++;
            //     }

            //     else if(power <= maxPow){
            //         dataRange100.set(nextRowRange100, i, value(gains[i], velocity, acceleration, rad));
            //         powerRange100.set(nextRowRange100, 0, power);
            //         nextRowRange100++;
            //     }

                
            }
            lastV = velocity;
        }

    }

    private int getRange(double maxVel, double velocity){
        double range30 = maxVel * 0.3;
        double range60 = maxVel * 0.6;
        if(velocity <= range30){
            return 0;
        }
        
        else if(velocity <= range60){
            return 1;
        }

        else if(velocity <= maxVel){
            return 2;
        }

        return -1;
    }

    


    private boolean valid(double maxVel){
        if(getVelocity == null) return false;
        if(maxVel < Math.abs(getVelocity.get())) return false;
        return true;
    }

    /**
     * Calculate the applicable value based on the gain type and provided dataRange30
     * 
     * @param gain wanted feed forward param
     * @param velocity current velocity
     * @param rad
     * @param meter
     * @return raw value of each gain (feed forward raw dataRange30)
     */
    double value(Gains gain, double velocity, double acceleration, double rad) {
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
            case KG:
                return voltageForHorizontal;
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
     * Function to reset the dataRange30
     */
    public void resetData() {
        nextRowRange30 = 0;
        nextRowRange60 = 0;
        nextRowRange100 = 0;
    }

    /**
     * 
     * @return extracts a sub matrix with the values of the original from the start to nextRow
     */
    public SimpleMatrix dataRange30() {
        return dataRange30.rows(0, nextRowRange30);
    }

    public SimpleMatrix dataRange60() {
        return dataRange60.rows(0, nextRowRange60);
    }

    public SimpleMatrix dataRange100() {
        return dataRange100.rows(0, nextRowRange100);
    }

    public SimpleMatrix testFF(){
        return dataRange30.solve(powerRange30);
    }

    /**
     * 
     * @return the collected power range 0-20 (power matrix for ranges of 0-20
     *         power)
     */
    public SimpleMatrix power() {
        return powerRange30.rows(0, nextRowRange30);
    }

    /**
     * 
     * @return the collected power range 21-69
     */
    public SimpleMatrix powerRange60() {
        return powerRange60.rows(0, nextRowRange60);
    }

    /**
     * 
     * @return the collected power range 70-100
     */
    public SimpleMatrix powerRange100() {
        return powerRange100.rows(0, nextRowRange100);
    }

    public SimpleMatrix solveFirstRange(){
        return datas[0].solve();
    }

    public SimpleMatrix solveSecRange(){
        return datas[1].solve();
    }

    public SimpleMatrix solveThirdRange(){
        return datas[2].solve();
    }

    /**
     * 
     * @return the gains (feed forward values) matrix for ranges 0-30 power
     */
    public SimpleMatrix solve() {
        return dataRange30().solve(power());
    }

    /**
     * 
     * @return the gains matrix for ranges 31-60
     */
    public SimpleMatrix solveRange60() {
        return dataRange60().solve(powerRange60());
    }

    /**
     * 
     * @return the gains matrix for ranges 61-100
     */
    public SimpleMatrix solveRange100() {
        return dataRange100().solve(powerRange100());
    }

    /**
     * 
     * @return the array of gains type
     */
    public Gains[] gains() {
        return gains;
    }

}