package pidexample;

/**
 * A set of configuration parameters for a PID controller.
 *
 * @author Adrian Rumpold (a.rumpold@gmail.com)
 */
public class PIDParameters {
    private double kp; // Proportional gain
    private double ki; // Integral gain
    private double kd; // Differential gain

    private double alpha = 1.0f; // Low-pass filter ratio

    private Double minOutput = null;
    private Double maxOutput = null;
    private double maxIntegral = Double.MAX_VALUE;
    private float controllerFrequency; // frequency of the calls to the PID-Controller in HZ
    private int senderID;

    /**
     * TODO Maybe move into the "Plane" class
     * @return An integer value distinguishing the different sources of throttle commands
     */
    public int getSenderID(){
    	return senderID;
    }
    /**
     * TODO Maybe move into the "Plane" class
     * @return The frequency in Hz of how often the controller should be called
     */
    public float getControllerFrequency(){
    	return controllerFrequency;
    }
    /**
     * @return the proportional gain factor
     */
    public double getKp() {
        return kp;
    }

    /**
     * @return the integral gain factor
     */
    public double getKi() {
        return ki;
    }

    /**
     * @return the differential gain factor
     */
    public double getKd() {
        return kd;
    }

    /**
     * @return the output moving average filter coefficient
     */
    public double getAlpha() {
        return alpha;
    }

    /**
     * @return the optional lower output bound
     */
    public Double getMinOutput() {
        return minOutput;
    }

    /**
     * @return the optional upper output bound
     */
    public Double getMaxOutput() {
        return maxOutput;
    }

    /**
     * @return the optional integral clamping threshold
     */
    public double getMaxIntegral() {
        return maxIntegral;
    }

    @Override
    public String toString() {
        return String.format(
                "[PID parameters:%n\t(Kp, Ki, Kd) = (%.4f, %.4f, %.4f)%n\toutputRange = [%s, %s]%n\tmaxIntegral = %.3f, "
                + "alpha = %.3f\n\tsenderID = %d, controllerFrequency = %.1fHz]",
                kp, ki, kd, minOutput, maxOutput, maxIntegral, alpha, senderID, controllerFrequency);
    }
}
