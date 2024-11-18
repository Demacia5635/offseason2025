package frc.robot.Sysid;

public class FeedForward_SVA {
    double KS;
    double KV;
    double KA;

    public FeedForward_SVA(double KS, double KV, double KA) {
        this.KA = KA;
        this.KV = KV;
        this.KS = KS;
    }

    public FeedForward_SVA(double[] K) {
        this(K[0], K[1], K[2]);
    }


    /**
     * 
     * @param velocity current velocity
     * @param lastV last velocity
     * @return calculated power 
     */
    public double calculate(double velocity, double lastV) {
        return KS*Math.signum(velocity) + KV*velocity + KA*(velocity-lastV);
    }
}