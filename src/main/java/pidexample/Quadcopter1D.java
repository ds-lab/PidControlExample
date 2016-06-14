package pidexample;

import org.apache.commons.math3.exception.DimensionMismatchException;
import org.apache.commons.math3.exception.MaxCountExceededException;
import org.apache.commons.math3.ode.FirstOrderDifferentialEquations;
import org.apache.commons.math3.ode.FirstOrderIntegrator;
import org.apache.commons.math3.ode.nonstiff.ClassicalRungeKuttaIntegrator;
import org.apache.commons.math3.ode.sampling.StepHandler;
import org.apache.commons.math3.ode.sampling.StepInterpolator;
import org.apache.commons.math3.util.FastMath;
import org.jfree.chart.ChartFactory;
import org.jfree.chart.ChartPanel;
import org.jfree.chart.JFreeChart;
import org.jfree.chart.renderer.xy.XYLineAndShapeRenderer;
import org.jfree.data.xy.XYDataset;
import org.jfree.data.xy.XYSeries;
import org.jfree.data.xy.XYSeriesCollection;
import org.jfree.ui.RefineryUtilities;

import javax.swing.*;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;
import java.awt.*;
import java.io.BufferedWriter;
import java.io.FileWriter;
import java.util.Locale;
import java.util.concurrent.atomic.AtomicBoolean;

/**
 * Analyze data in R with
 * <p>
 * d <- read.csv2('c:/Users/Adriano/Desktop/data.csv', col.names = c('t', 'z', 'v'))
 * ggplot(d, aes(x = t, y = z)) + geom_line() + geom_hline(yintercept = 0, col='red', size=2)
 *
 * @author Adrian Rumpold (a.rumpold@ds-lab.org)
 */
public class Quadcopter1D implements FirstOrderDifferentialEquations, ChangeListener {
    private final AtomicBoolean inSimulation = new AtomicBoolean(false);

    private final JLabel pValue = new JLabel();
    private final JLabel iValue = new JLabel();
    private final JLabel dValue = new JLabel();
    private final JLabel timeValue = new JLabel();

    private class LineChart extends JFrame {
        private static final long serialVersionUID = 1L;
        private final XYSeriesCollection dataset = new XYSeriesCollection();
        private final JFreeChart chart;

        @SuppressWarnings("unchecked")
        public LineChart(String appTitle, String chartTitle) {
            super(appTitle);
            dataset.addSeries(new XYSeries("Altitude"));
            dataset.addSeries(new XYSeries("Setpoint"));

            chart = createChart(dataset, chartTitle);
            chart.setAntiAlias(true);

            final ChartPanel panel = new ChartPanel(chart);
            panel.setPreferredSize(new Dimension(800, 400));
            setContentPane(panel);
        }

        private JFreeChart createChart(XYDataset dataset, String chartTitle) {
            final JFreeChart chart = ChartFactory.createScatterPlot(chartTitle, "Time", "Measurement", dataset);
            chart.getXYPlot().setRenderer(new XYLineAndShapeRenderer(true, false));
            return chart;
        }

        public XYSeriesCollection getDataset() {
            return dataset;
        }

        public JFreeChart getChart() {
            if (chart != null) {
                return chart;
            } else {
                return null;
            }
        }
    }

    private class ControlGui extends JFrame {
        public ControlGui() throws HeadlessException {
            super("PID control parameters");
            final GridBagLayout layout = new GridBagLayout();
            setLayout(layout);

            final GridBagConstraints sliderConstraint = new GridBagConstraints();
            sliderConstraint.gridx = GridBagConstraints.RELATIVE;
            sliderConstraint.gridy = 0;
            sliderConstraint.weightx = 1;
            sliderConstraint.fill = GridBagConstraints.HORIZONTAL;

            final GridBagConstraints labelConstraints = new GridBagConstraints();
            labelConstraints.gridx = GridBagConstraints.RELATIVE;
            labelConstraints.gridy = 0;
            labelConstraints.ipadx = 10;
            labelConstraints.ipady = 4;

            final JLabel pLabel = new JLabel("Kp = ");
            final JSlider pSlider = new JSlider(0, 50, 0);
            add(pLabel, labelConstraints);
            pSlider.setName("kp");
            pSlider.addChangeListener(Quadcopter1D.this);
            add(pSlider, sliderConstraint);
            add(pValue, labelConstraints);

            ++sliderConstraint.gridy;
            ++labelConstraints.gridy;

            final JLabel iLabel = new JLabel("Ki = ");
            final JSlider iSlider = new JSlider(0, 100, 0);
            add(iLabel, labelConstraints);
            iSlider.setName("ki");
            iSlider.addChangeListener(Quadcopter1D.this);
            add(iSlider, sliderConstraint);
            add(iValue, labelConstraints);

            ++sliderConstraint.gridy;
            ++labelConstraints.gridy;

            final JLabel dLabel = new JLabel("Kd = ");
            final JSlider dSlider = new JSlider(0, 100, 0);
            add(dLabel, labelConstraints);
            dSlider.setName("kd");
            dSlider.addChangeListener(Quadcopter1D.this);
            add(dSlider, sliderConstraint);
            add(dValue, labelConstraints);

            ++sliderConstraint.gridy;
            ++labelConstraints.gridy;

            final JLabel timeLabel = new JLabel("Simulation length");
            final JSlider lengthSlider = new JSlider(0, 60, (int) simulationTime);
            add(timeLabel, labelConstraints);
            lengthSlider.setName("time");
            lengthSlider.addChangeListener(Quadcopter1D.this);
            add(lengthSlider, sliderConstraint);
            add(timeValue, labelConstraints);

            stateChanged(new ChangeEvent(lengthSlider));
        }

    }

    private final LineChart chart;
    private final ControlGui controls;

    private static final double GRAVITY_CONST = 9.81;   // m/s^2

    private final double mass = 0.18;   // kg
    private final double rotorLength = 0.086;   // m
    private final double minThrust = 0;
    private final double maxThrust = 1.2 * mass * GRAVITY_CONST;
    // private final double Ixx = 0.00025;

    private PIDController controller;

    private final double z0 = 2;
    private double thrust = 0;

    private double lastTime = 0;

    // User-changeable settings
    private double kp;
    private double ki;
    private double kd;
    private double simulationTime = 10;

    private Quadcopter1D() {
        chart = new LineChart("1D Quadcopter", null);
        controls = new ControlGui();
    }

    @Override
    public void stateChanged(ChangeEvent e) {
        final JSlider source = (JSlider) e.getSource();
        final int value = source.getValue();

        switch (source.getName()) {
            case "time": {
                simulationTime = value;
                break;
            }
            case "kp": {
                kp = value;
                break;
            }
            case "ki": {
                ki = value / 10.0;
                break;
            }
            case "kd": {
                kd = value / 10.0;
                break;
            }
        }

        timeValue.setText(String.format("%.1f", simulationTime));
        pValue.setText(String.format("%.1f", kp));
        iValue.setText(String.format("%.1f", ki));
        dValue.setText(String.format("%.1f", kd));

        performSimulation();
    }

    private void start() {
        chart.pack();
        controls.pack();

        RefineryUtilities.centerFrameOnScreen(chart);
        RefineryUtilities.positionFrameOnScreen(controls, .8, .5);

        chart.setVisible(true);
        controls.setVisible(true);

        chart.setDefaultCloseOperation(WindowConstants.EXIT_ON_CLOSE);
        controls.setDefaultCloseOperation(WindowConstants.EXIT_ON_CLOSE);
    }

    private void control(double deltaT, double z, double vZ) {
        /*final double Kp = 100;
        final double Kd = 10;

        final double errZ = setpoint[0] - z;
        final double errV = setpoint[1] - vZ;

        thrust = mass * (Kp * errZ + Kd * errV + GRAVITY_CONST);*/

        if (deltaT == 0) {
            return;
        }

        double thrustUnclamped = controller.update(deltaT, z);

        // Clamp to physical limits
        thrust = FastMath.min(FastMath.max(thrustUnclamped, minThrust), maxThrust);
        if (thrustUnclamped != thrust) {
            System.out.println("Clamped thrust from " + thrustUnclamped + " to " + thrust);
        }
    }

    @Override
    public int getDimension() {
        return 2;
    }

    @Override
    public void computeDerivatives(double t, double[] y, double[] yDot) throws MaxCountExceededException, DimensionMismatchException {
        // Invoke control loop
        control(t - lastTime, y[0], y[1]);
        lastTime = t;

        yDot[0] = y[1]; // xDot = Velocity
        yDot[1] = thrust / mass - GRAVITY_CONST;    // xDDot = Acceleration
    }

    public void performSimulation() {
        if (inSimulation.compareAndSet(false, true)) {
            return;
        }

        System.out.printf("Performing simulation with: t = %.2f, Kp = %.2f, Ki = %.3f, Kd = %.3f\n",
                simulationTime, kp, ki, kd);

        controller = PIDController
                .withGains(kp, ki, kd)
                .maxOutput(maxThrust)
                .minOutput(minThrust)
                .build();

        chart.getChart().getXYPlot().getDomainAxis().setRange(0, simulationTime);
        final XYSeries series = chart.getDataset().getSeries(0);
        final XYSeries setpointSeries = chart.getDataset().getSeries(1);
        series.clear();
        setpointSeries.clear();

//        final FirstOrderIntegrator integrator = new DormandPrince853Integrator(1.0e-12, 1.0, 1.0e-10, 1.0e-10);
        final FirstOrderIntegrator integrator = new ClassicalRungeKuttaIntegrator(1e-2);
        double[] y = new double[]{z0, 0};

        lastTime = 0;
        thrust = 0;
        controller.setSetpoint(z0);

        try (BufferedWriter output = new BufferedWriter(new FileWriter("c:/Users/Adriano/Desktop/data.csv"))) {
            integrator.addStepHandler(new StepHandler() {
                @Override
                public void init(double t0, double[] y0, double t) {
                    output(t0, y0);
                }

                @Override
                public void handleStep(StepInterpolator interpolator, boolean isLast) throws MaxCountExceededException {
                    double t = interpolator.getCurrentTime();
                    double[] y = interpolator.getInterpolatedState();

                    if (t > 5) {
                        controller.setSetpoint(4, false);
                    }

                    output(t, y);
                }

                private void output(double t, double[] y) {
                    try {
                        series.add(t, y[0]);
                        setpointSeries.add(t, (double) controller.getSetpoint());

                        output.write(String.format("%g;%g;%g\n", t, y[0], y[1]));
                    } catch (Exception e) {
                        e.printStackTrace();
                    }
                }
            });

            integrator.integrate(this, 0, y, simulationTime, y);
            inSimulation.set(false);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public static void main(String[] args) {
        Locale.setDefault(Locale.GERMAN);
        final Quadcopter1D sim = new Quadcopter1D();
        sim.start();
    }
}
