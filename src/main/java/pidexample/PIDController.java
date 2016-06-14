package pidexample;

/**
 * A PID controller with optional output bounds, output moving average filter
 * and integral error clamping.
 * <p>
 * Instances should be created through the Builder pattern using the
 * {@link #withGains(double, double, double)} method or by passing a set of
 * parameters to the {@link #PIDController(PIDParameters)} constructor.
 *
 * @author Adrian Rumpold (a.rumpold@gmail.com)
 * @see PIDParameters
 */
public class PIDController {
    private double kp; // Proportional gain
    private double ki; // Integral gain
    private double kd; // Differential gain

    private double alpha = 1.0f; // Low-pass filter ratio

    private Double minOutput = null;
    private Double maxOutput = null;
    private Double setpoint = null;
    private double maxIntegral = Double.MAX_VALUE;
    private double integral = 0.0f;
    private double lastError;
    private double lastOutput = Double.NaN;

    private boolean debug = false;

    /**
     * Default constructor inaccessible - use Builder methods
     */
    private PIDController() {
    }

    /**
     * Construct a new PID controller with the supplied set of parameters.
     *
     * @param params
     */
    public PIDController(PIDParameters params) {
        this.kp = params.getKp();
        this.kd = params.getKd();
        this.ki = params.getKi();

        this.minOutput = params.getMinOutput();
        this.maxOutput = params.getMaxOutput();
        this.alpha = params.getAlpha();
    }

    /**
     * Create a builder instance for a PID controller with the specified gain
     * factors.
     *
     * @param kp the proportional gain factor
     * @param ki the integral gain factor
     * @param kd the differential gain factor
     * @return a {@link Builder} for a PID controller
     */
    public static Builder withGains(double kp, double ki, double kd) {
        return new Builder(kp, ki, kd);
    }

    /**
     * Helper class for Builder pattern construction of a PID controller
     */
    public static class Builder {
        private PIDController controller = new PIDController();

        private Builder(double kp, double ki, double kd) {
            controller.kp = kp;
            controller.ki = ki;
            controller.kd = kd;
        }

        /**
         * Set the upper output limit
         *
         * @param max the maximum output value
         * @return the current {@link Builder} instance for method chaining
         */
        public Builder maxOutput(double max) {
            controller.maxOutput = max;
            return this;
        }

        /**
         * Set the lower output limit
         *
         * @param min the minimum output value
         * @return the current {@link Builder} instance for method chaining
         */
        public Builder minOutput(double min) {
            controller.minOutput = min;
            return this;
        }

        /**
         * Set the threshold value for integral clamping
         *
         * @param maxIntegral the integral clamping threshold
         * @return the current {@link Builder} instance for method chaining
         */
        public Builder maxIntegral(double maxIntegral) {
            controller.maxIntegral = maxIntegral;
            return this;
        }

        /**
         * Set the filter ratio for average filtering of the output.
         *
         * @param ratio the minimum output value
         * @return the current {@link Builder} instance for method chaining
         */
        public Builder filterRatio(double ratio) {
            controller.alpha = ratio;
            return this;
        }

        /**
         * Set the initial controller setpoint
         *
         * @param setpoint the initial setpoint
         * @return the current {@link Builder} instance for method chaining
         */
        public Builder setpoint(double setpoint) {
            controller.setSetpoint(setpoint);
            return this;
        }

        /**
         * Returns the PID controller initialized with the settings from this
         * Builder instance.
         *
         * @return the PID controller
         */
        public PIDController build() {
            return controller;
        }
    }

    /**
     * Change the output value by applying the moving average filter
     */
    private double filter(double output) {
        if (Double.isNaN(lastOutput)) {
            lastOutput = output;
        }
        final double filtered = alpha * output + (1.0 - alpha) * lastOutput;
        lastOutput = filtered;
        return filtered;
    }

    /**
     * Feed the current input to the controller and obtain updated output.
     *
     * @param current the current input value
     * @return the controller output after applying optional bounds clamping and
     * output filtering
     * @throws IllegalStateException if the controller does not have a setpoint
     * @oaram deltaT elapsed time in seconds since the last update
     */
    public double update(double deltaT, double current) {
        if (deltaT < 0) {
            throw new IllegalArgumentException("Negative elapsed time, deltaT = " + deltaT);
        }

        if (!hasSetpoint()) {
            throw new IllegalStateException(
                    "PID controller must have a setpoint");
        }

        final double error = setpoint - current;
        integral += error;

        // Clamp integral error to preset bounds to prevent integral windup
        if (integral > maxIntegral) {
            integral = maxIntegral;
        } else if (integral < -maxIntegral) {
            integral = -maxIntegral;
        }

        final double deltaErr = error - lastError;
        lastError = error;

        if (debug) {
            System.out.printf("dT = %.4f, SP = %.2f, err = %.3f, int = %.2f, deltaErr = %.3f\n",
                    deltaT, setpoint, error, integral, deltaErr);
        }

        double output = kp * error + ki * deltaT * integral + kd * deltaErr / deltaT;
        output = filter(output);

        if (minOutput != null && output < minOutput) {
            output = minOutput;
        }

        if (maxOutput != null && output > maxOutput) {
            output = maxOutput;
        }
        return output;
    }

    /**
     * @return the output moving average filter coefficient
     */
    public double getFilterRatio() {
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
     * Check if this controller has a setpoint
     *
     * @return the setpoint state
     */
    public boolean hasSetpoint() {
        return setpoint != null;
    }

    /**
     * @return the setpoint value
     */
    public Double getSetpoint() {
        return setpoint;
    }

    /**
     * Update the controller setpoint value
     *
     * @param setpoint the new setpoint
     * @param reset    indicates if the integral error should be reset to 0
     */
    public void setSetpoint(double setpoint, boolean reset) {
        if (reset) {
            integral = 0.0;
        }
        this.setpoint = setpoint;
    }

    /**
     * Update the controller setpoint value
     * <p/>
     * Note, that this operation resets the integral error to 0
     *
     * @param setpoint the new setpoint
     */
    public void setSetpoint(double setpoint) {
        setSetpoint(setpoint, true);
    }

    /**
     * @return the current integral error
     */
    public double getIntegral() {
        return integral;
    }
}
