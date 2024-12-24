package frc.robot.Sysid.Utils;

import org.ejml.simple.SimpleMatrix;

public class FeedForward {
    private static int kSindex = 0;
    private static int kVindex = 1;
    private static int kAindex = 2;

    private static void gatherData(double accel){
        //double[] accel = new double[100];
        double[] vel = new double[100];
        double[] power = new double[100];
        GetFF(null, null, null);
    }


    /**
     * 
     * Ks , Kv
     */
    public static double[] GetFF(double[] power, Double[] velocity, Double[] accel){
        double[] signumV = new double[velocity.length];
        for(int i = 0; i < signumV.length; i++){
            signumV[i] = Math.signum(velocity[i]);
        }
        double[][] matrix = new double[3][velocity.length];
        for(int i = 0; i < matrix[kSindex].length; i++){
            matrix[kSindex][i] = signumV[i];
            matrix[kVindex][i] = velocity[i];
            matrix[kAindex][i] = accel[i];
        }
        SimpleMatrix data = new SimpleMatrix(matrix);
        SimpleMatrix powerMatrix = new SimpleMatrix(power);

        return data.solve(powerMatrix).toArray2()[0];
    } 
}
