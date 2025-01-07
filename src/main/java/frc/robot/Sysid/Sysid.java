package frc.robot.Sysid;


import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;


public class    Sysid extends Command {

    final static double defaultDuration = 2.5;
    final static double defaultDelay = 10;

    /**
     * Gains enum - type of gains
     */
    public static enum Gains {
        KS, KV, KA, KG, KRad, KV2, KSin, KCos, KTan;
    }
    class Data {
        SimpleMatrix data;
        SimpleMatrix volts;
        int nextRow = 0;
        double maxVelocity;
        Data(double maxVelocity, int nRows, int nCol) {
            data = new SimpleMatrix(nRows, nCol);
            volts = new SimpleMatrix(nRows,1);
            this.maxVelocity = maxVelocity;
        }

        void analyze() {
            if(nextRow > 50) {
                SimpleMatrix d = data.rows(0, nextRow-1);
                SimpleMatrix v = volts.rows(0,nextRow-1);
                SimpleMatrix res = d.solve(v);
                String name = "Sysid-" + (maxVelocity != Double.MAX_VALUE?maxVelocity:"MAX") + "/";
                for(int i = 0; i < gains.length; i++) {
                    SmartDashboard.putNumber(name + gains[i], res.get(i,0));
                }
                SimpleMatrix p = d.mult(res);
                SimpleMatrix dif = v.minus(p);
                double difE = dif.dot(dif);
                SmartDashboard.putNumber(name + "error" , difE / nextRow);
                SmartDashboard.putNumber(name + "num" , nextRow);
                int r = nextRow / 2;
                for(int i = 0; i < gains.length; i++) {
                    SmartDashboard.putNumber(name + "data" + gains[i] , d.get(r, i));
                }
                SmartDashboard.putNumber(name + "volt" , p.get(r, 0));
            }
        }
    }

    DoubleConsumer setVolt; // function to set the volt
    DoubleSupplier getVelocity;
    DoubleSupplier getAcceleration;
    DoubleSupplier getAngle;    
    DoubleSupplier getVolt;

    int nPowerCycles;
    double powerCycleDuration;
    double lastV = 0;
    double currentAngle = 0; //current angle in rad
    double baseVolt = 0;
    double maxVolt = 12;
    double accelerationTime = 2;

    double currentVolt = 0;
    boolean forwardCycle = true;
    boolean accelerationCycle = false;
    boolean ended = false;
    boolean waitTime = false;
    double cycleStartTime = 0;

    Data[] data;
    Gains[] gains;

    double deltaVolt; // the change of power between power cycles
    double powerCycleDelay; // delay between power cycles
    Subsystem[] subsystems; // for add requirements


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
            DoubleConsumer setVolt,
            DoubleSupplier getVolt,
            DoubleSupplier getVelocity,
            DoubleSupplier getAcceleration,
            DoubleSupplier getAngle,
            double[] velocitiesRange,
            int nPowerCycles,
            double powerCycleDuration,
            double powerCycleDelay,
            double baseVolt,
            double maxVolt,
            double accelerationTime,
            Subsystem... subsystems) {

        this.setVolt = setVolt;
        this.getVolt = getVolt;
        this.getVelocity = getVelocity;
        this.getAcceleration = getAcceleration;
        this.getAngle = getAngle;
        this.gains = types;
        this.nPowerCycles = Math.max(1,nPowerCycles);
        this.powerCycleDelay = powerCycleDelay;
        this.powerCycleDuration = powerCycleDuration;
        this.accelerationTime = accelerationTime;
        this.baseVolt = baseVolt;
        this.maxVolt = maxVolt;
        this.deltaVolt = this.maxVolt/nPowerCycles;
        this.subsystems = subsystems;
        int nRange = velocitiesRange.length;
        int nRows = (int)(50*(nPowerCycles * 2 * powerCycleDuration + accelerationTime * 2 + 1));
        data = new Data[nRange + 1];
        for(int i = 0; i < nRange; i++ ) {
            data[i] = new Data(velocitiesRange[i], nRows, gains.length);
        }
        data[nRange] = new Data(Double.MAX_VALUE, nRows, gains.length);
    }

    private Data velocityData(double v) {
        double absV = Math.abs(v);
        for(Data d : data) {
            if(d.maxVelocity > absV) {
                return d;
            }
        }
        return data[data.length-1];
    }

    

   
    public void collect() {
        double v = getVelocity.getAsDouble();
        double a = getAcceleration.getAsDouble();
        double angle = getAngle != null ? getAngle.getAsDouble() : 0;
        double volt = getVolt.getAsDouble();
        if(volt != 0) {
            Data d = velocityData(v);
            int row = d.nextRow;
            for(int i = 0; i < gains.length; i++) {
                d.data.set(row, i, value(gains[i], v, a, angle));
            }
            d.volts.set(row, volt);
            d.nextRow++;
        }
    }

    double value(Gains gain, double velocity, double acceleration, double rad) {
        switch (gain) {
            case KS:
                return Math.signum(velocity);
            case KV:
                return velocity;
            case KA:
                return acceleration;
            case KRad:
                return rad;
            case KCos:
                return Math.cos(rad);
            case KSin:
                return Math.sin(rad);
            case KTan:
                return Math.tan(rad);
            case KV2:
                return velocity * velocity * Math.signum(velocity);
            case KG:
                return 1;
            default:
                return 0;
        }
    }

    private void setSteadyCycle(double volt) {
        cycleStartTime = Timer.getFPGATimestamp();
        currentVolt = volt;
        forwardCycle = volt > 0;
        accelerationCycle = false;
        waitTime = false;
    }
    private void setAcceleratioCycle(boolean forward) {
        cycleStartTime = Timer.getFPGATimestamp();
        currentVolt = 0;
        forwardCycle = forward;
        accelerationCycle = true;
        waitTime = false;
    }

    double getAccelerationVolt(double cycleTime) {
        if(cycleTime > accelerationTime) { // finish accleration cycle
            if(!forwardCycle) { // end deaccleration
                ended = true;
                currentVolt = 0;
                return 0; 
            } else { // switch to deacceleration
                setAcceleratioCycle(false);
                return 0;
            }
        } else if(forwardCycle) { // continue acceleration
            return (cycleTime / accelerationTime) * maxVolt;
        } else { // continue deaccelartion
            return ((accelerationTime - cycleTime) / accelerationTime) * maxVolt;
        }
    }

    private void hadleWaitTime(double cycleTime) {
        if(cycleTime > powerCycleDelay) { // wait completed
            if(forwardCycle) { // was forward - start reverse
                setSteadyCycle(-currentVolt);
            } else { // was forward - next cycle
                currentVolt = -currentVolt + deltaVolt;
                if(currentVolt > maxVolt) { // no more cycles - start accelration
                    if(accelerationTime == 0) {
                        ended = true;
                        currentVolt = 0;
                    } else {
                        setAcceleratioCycle(true);
                    }
                } else {
                    setSteadyCycle(currentVolt);
                }
            }
        }
    }

    double getCurrentVolt() {
        if(ended) {
            return 0;
        }
        if(cycleStartTime == 0) {
            setSteadyCycle(deltaVolt);
            return currentVolt;
        }
        double cycleTime = Timer.getFPGATimestamp() - cycleStartTime;
        if(accelerationCycle) { // in accelaration
            return getAccelerationVolt(cycleTime);
        } else if(waitTime) { // waiting between cycles
            hadleWaitTime(cycleTime);
            return 0;
        } else { // normal cycle
            if(cycleTime > powerCycleDuration) { // cycle ended - start wait
                waitTime = true;
                cycleStartTime = Timer.getFPGATimestamp();
                return 0; 
            } else {
                return currentVolt;
            }
        }
    }

    public void execute() {
        collect();
        double p = getCurrentVolt();
//        LogManager.log("set volt to " + p);
        setVolt.accept(p);
    }

    public boolean isFinished() {
        return ended;
    }

    public void end(boolean interupt) {
        setVolt.accept(0.0);
        analyze();
    }

    public void analyze() {
        setVolt.accept(0.0);
        for(Data d : data) {
            d.analyze();
        }
    }

    public static Command simpleMotorSysidCommand(
            DoubleConsumer setVolt,
            DoubleSupplier getVolt,
            DoubleSupplier getVelocity,
            DoubleSupplier getAcceleration,
            double[] velocitiesRange,
            int nPowerCycles,
            double powerCycleDuration,
            double powerCycleDelay,
            double maxVolt,
            double accelerationTime,
            Subsystem... subsystems) {
        Gains[] gains = {Gains.KS, Gains.KV, Gains.KA};
       return new Sysid(gains,
         setVolt, getVolt, getVelocity, getAcceleration, null,
         velocitiesRange,nPowerCycles,powerCycleDuration,powerCycleDelay,0, maxVolt,accelerationTime,subsystems);
    }

}