package frc.robot.test;

import java.util.ArrayList;

import org.ejml.simple.SimpleMatrix;

public class FeedForward {
    private static int kSindex = 0;
    private static int kVindex = 1;
    private static int kAindex = 2;


    /**
     * 
     * Ks , Kv
     */
    public static double[] GetFF(ArrayList<Double> power, ArrayList<Double> velocity, ArrayList<Double> accel){
        double[] signumV = new double[velocity.size()];
        for(int i = 0; i < signumV.length; i++){
            signumV[i] = Math.signum(velocity.get(i));
        }
        double[][] matrix = new double[velocity.size()][3];
        for(int i = 0; i < signumV.length; i++){
            matrix[i][kSindex] = signumV[i];
            matrix[i][kVindex] = velocity.get(i);
            matrix[i][kAindex] = accel.get(i);
            System.out.println("I: " + i + "SIGNUM: " + signumV[i] + " V: " + velocity.get(i) + " Accel: " + accel.get(i));
        }

        double[] powers = new double[power.size()];
        for(int i = 0; i < powers.length; i++){
            powers[i] = power.get(i) * 12;
        }
        SimpleMatrix data = new SimpleMatrix(matrix);
        SimpleMatrix powerMatrix = new SimpleMatrix(powers);


        return data.solve(powerMatrix).transpose().toArray2()[0];
    } 
}
