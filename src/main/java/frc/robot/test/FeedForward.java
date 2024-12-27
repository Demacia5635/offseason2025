package frc.robot.test;

import java.util.ArrayList;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.math.controller.ArmFeedforward;

public class FeedForward {
    private static int kSindex = 0;
    private static int kVindex = 1;
    private static int kAindex = 2;
    private static int kGindex = 3;

    private static double g = 9.8;

    /**
     * 
     * Ks , Kv, Ka
     */
    public static double[] GetFF(ArrayList<Double> power, ArrayList<Double> velocity, ArrayList<Double> accel) {
        double[] signumV = new double[velocity.size()];
        for (int i = 0; i < signumV.length; i++) {
            signumV[i] = Math.signum(velocity.get(i));
        }
        double[][] matrix = new double[velocity.size()][3];
        for (int i = 0; i < signumV.length; i++) {
            matrix[i][kSindex] = signumV[i];
            matrix[i][kVindex] = velocity.get(i);
            matrix[i][kAindex] = accel.get(i);
        }

        double[] powers = new double[power.size()];
        for (int i = 0; i < powers.length; i++) {
            powers[i] = power.get(i) * 12;
        }
        SimpleMatrix data = new SimpleMatrix(matrix);
        SimpleMatrix powerMatrix = new SimpleMatrix(powers);

        return data.solve(powerMatrix).transpose().toArray2()[0];
    }

    public static double[] flyWheelFF(ArrayList<Double> power, ArrayList<Double> velocity, ArrayList<Double> accel) {

        return new double[] { 1, 2, 3 };
    }

    /**
     * Does Elevator equation: ks * Math.signum(velocity) + kg*W + kv * velocity +
     * ka * acceleration
     * 
     * @param power    list of powers
     * @param velocity list of velocities
     * @param accel    list of accel
     * @param mass     the mass of the subsystem
     * @return Ks, Kv, Ka, Kg
     */
    public static double[] ElevatorFF(ArrayList<Double> power, ArrayList<Double> velocity, ArrayList<Double> accel,
            double mass) {

        double[] signumV = new double[velocity.size()];
        for (int i = 0; i < signumV.length; i++) {
            signumV[i] = Math.signum(velocity.get(i));
        }
        double[][] matrix = new double[velocity.size()][4];
        for (int i = 0; i < signumV.length; i++) {
            matrix[i][kSindex] = signumV[i];
            matrix[i][kVindex] = velocity.get(i);
            matrix[i][kAindex] = accel.get(i);
            matrix[i][kGindex] = mass * g;
        }

        double[] powers = new double[power.size()];
        for (int i = 0; i < powers.length; i++) {
            powers[i] = power.get(i) * 12;
        }
        SimpleMatrix data = new SimpleMatrix(matrix);
        SimpleMatrix powerMatrix = new SimpleMatrix(powers);

        return data.solve(powerMatrix).transpose().toArray2()[0];
    }

    /**
     * Arm Feed Forward
     * 
     * @param power     array list of all the powers
     * @param velocity  array list of all the velocities
     * @param accel     array list of all the accelerations
     * @param angles    array list of all the angles
     * @param armLength length of the arm
     * @param mass
     * @return arm feed forward {Ks, Kv, Ka, Kg}
     */
    public static double[] ArmFeedforward(ArrayList<Double> power, ArrayList<Double> velocity, ArrayList<Double> accel,
            ArrayList<Double> angles, double armLength, double mass) {
        /*
         * ks * Math.signum(velocityRadPerSec)
         * + kg * Math.cos(positionRadians)
         * + kv * velocityRadPerSec
         * + ka * accelRadPerSecSquared;
         */

        double[] signumV = new double[velocity.size()];
        for (int i = 0; i < signumV.length; i++) {
            signumV[i] = Math.signum(velocity.get(i));
        }
        double[][] matrix = new double[velocity.size()][4];
        for (int i = 0; i < signumV.length; i++) {
            matrix[i][kSindex] = signumV[i];
            matrix[i][kVindex] = velocity.get(i);
            matrix[i][kAindex] = accel.get(i);
            matrix[i][kGindex] = mass * g * armLength * Math.cos(angles.get(i));
        }

        double[] powers = new double[power.size()];
        for (int i = 0; i < powers.length; i++) {
            powers[i] = power.get(i) * 12;
        }
        SimpleMatrix data = new SimpleMatrix(matrix);
        SimpleMatrix powerMatrix = new SimpleMatrix(powers);

        return data.solve(powerMatrix).transpose().toArray2()[0];
    }
}
