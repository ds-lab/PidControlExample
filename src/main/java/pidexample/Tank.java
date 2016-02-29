package pidexample;

/**
 * @author Adrian Rumpold (a.rumpold@ds-lab.org)
 */
public final class Tank {
    /** Tank height in meters */
    private final double height;

    /** Tank radius in meters */
    private final double radius;

    /** Maximum fill rate in m^3/s */
    private final double maxFillRate;

    /** Current fill rate in m^3/s */
    private double fillRate = 0;

    /** Current drain rate in m^3/s */
    private double drainRate;

    /** Current fill level in meters */
    private double fillLevel = 0;

    /** Current volume in m^3 */
    private double volume = 0;

    /** Maximum volume in m^3 */
    private final double maxVolume;

    public Tank(double height, double radius, double maxFillRate) {
        this.height = height;
        this.radius = radius;
        this.maxFillRate = maxFillRate;

        this.drainRate = .2 * maxFillRate;
        this.maxVolume = Math.PI * radius * radius * height / 3.0;
    }

    /**
     * Performs a simulation time step and returns the new fill level after the update.
     * @param timestep the simulation update time step in seconds
     * @return the new fill level in meters
     */
    public double update(double timestep) {
        if (timestep <= 0) {
            throw new IllegalArgumentException("Timestep for update must be a positive number, was: " + timestep);
        }

        final double deltaV = (fillRate - drainRate) * timestep;
        volume += deltaV;

        if (volume < 0) {
            volume = 0;
        } if (volume > maxVolume) {
            volume = maxVolume;
        }

        //fillLevel = 3 * volume / (Math.PI * radius * radius);
        fillLevel = Math.sqrt(3 * volume * height / (Math.PI * radius * radius));
        if (fillLevel > height) {
            fillLevel = height;
        } else if (fillLevel < 0) {
            fillLevel = 0;
        }

        return fillLevel;
    }

    public void setFillRate(double fillRate) {
        if (fillRate > maxFillRate) {
            throw new IllegalArgumentException("Fill rate must be less than maximum fill rate");
        } else if (fillRate < 0) {
            throw new IllegalArgumentException("Fill rate must be a positive number, was " + fillRate);
        }

        this.fillRate = fillRate;
    }

    public double getHeight() {
        return height;
    }

    public double getRadius() {
        return radius;
    }

    public double getMaxFillRate() {
        return maxFillRate;
    }

    public double getFillRate() {
        return fillRate;
    }

    public double getFillLevel() {
        return fillLevel;
    }
}
