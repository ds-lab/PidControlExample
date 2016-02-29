package pidexample;

import org.jfree.chart.ChartFactory;
import org.jfree.chart.ChartPanel;
import org.jfree.chart.JFreeChart;
import org.jfree.data.time.FixedMillisecond;
import org.jfree.data.time.TimeSeries;
import org.jfree.data.time.TimeSeriesCollection;
import org.jfree.data.xy.XYDataset;
import org.jfree.ui.RefineryUtilities;

import javax.swing.*;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;
import java.awt.*;

/**
 * @author Adrian Rumpold (a.rumpold@ds-lab.org)
 */
public final class PIDTestGui implements ChangeListener {
    private class LineChart extends JFrame {
        private static final long serialVersionUID = 1L;
        private final TimeSeriesCollection dataset = new TimeSeriesCollection();

        @SuppressWarnings("unchecked")
        public LineChart(String appTitle, String chartTitle) {
            super(appTitle);
            dataset.addSeries(new TimeSeries("Process variable"));
            dataset.addSeries(new TimeSeries("Setpoint"));

            JFreeChart chart = createChart(dataset, chartTitle);
            chart.setAntiAlias(true);
            chart.getXYPlot().getRenderer().setBaseToolTipGenerator((ds, series, item) -> {
                final Number x = ds.getX(series, item);
                final Number y = ds.getY(series, item);

                final StringBuilder out = new StringBuilder();
                out.append("<html>");
                out.append(String.format("<p>X = %.2f s</p>", x.doubleValue() / 1000));
                out.append(String.format("<p>Y = %.2f m</p>", y));
                out.append("</html>");

                return out.toString();
            });

            ChartPanel panel = new ChartPanel(chart);
            panel.setPreferredSize(new Dimension(800, 400));
            setContentPane(panel);
        }

        private JFreeChart createChart(XYDataset dataset, String chartTitle) {
            JFreeChart chart = ChartFactory.createTimeSeriesChart(chartTitle,
                    "cat", "val", dataset);
            return chart;
        }

        public TimeSeriesCollection getDataset() {
            return dataset;
        }
    }

    private class ControlGui extends JFrame {
        public ControlGui() throws HeadlessException {
            super("PID control parameters");
            setLayout(new GridLayout(4, 2));

            final JLabel pLabel = new JLabel("Kp = ");
            final JSlider pSlider = new JSlider(0, 50, 0);
            add(pLabel);
            pSlider.setName("kp");
            pSlider.addChangeListener(PIDTestGui.this);
            add(pSlider);

            final JLabel iLabel = new JLabel("Ki = ");
            final JSlider iSlider = new JSlider(0, 100, 0);
            add(iLabel);
            iSlider.setName("ki");
            iSlider.addChangeListener(PIDTestGui.this);
            add(iSlider);

            final JLabel dLabel = new JLabel("Kd = ");
            final JSlider dSlider = new JSlider(0, 100, 0);
            add(dLabel);
            dSlider.setName("kd");
            dSlider.addChangeListener(PIDTestGui.this);
            add(dSlider);

            final JLabel timeLabel = new JLabel("Simulation length");
            final JSlider lengthSlider = new JSlider(0, 240, (int) simulationTime);
            add(timeLabel);
            lengthSlider.setName("time");
            lengthSlider.addChangeListener(PIDTestGui.this);
            add(lengthSlider);
        }

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
                kp = value / 10.0;
                break;
            }
            case "ki": {
                ki = value / 100.0;
                break;
            }
            case "kd": {
                kd = value;
                break;
            }
        }

        performSimulation();
    }

    private final LineChart chart = new LineChart("PID controller test", "Simulation Data");
    private final ControlGui controls = new ControlGui();

    private double kp = 0.0;
    private double ki = 0.0;
    private double kd = 0.0;
    private double simulationTime = 120.0;
    private double timestep = .25;

    final double height = 4;
    final double radius = 1;
    final double maxFillRate = 1;
    final double desiredFillLevel = 0.75;

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

    private void performSimulation() {
        System.out.printf("Performing simulation with: t = %.2f, Kp = %.2f, Ki = %.3f, Kd = %.3f\n",
                simulationTime, kp, ki, kd);

        final TimeSeries series = chart.getDataset().getSeries(0);
        final TimeSeries setpointSeries = chart.getDataset().getSeries(1);
        series.clear();
        setpointSeries.clear();

        final Tank tank = new Tank(height, radius, maxFillRate);
        final PIDController controller = PIDController.withGains(kp, ki, kd)
                .setpoint(desiredFillLevel)
                .minOutput(0.0)
                .maxOutput(tank.getMaxFillRate())
                .build();

        for (double t = 0; t < simulationTime; t += timestep) {
            final double fillLevel = tank.update(timestep);
            final double newFillRate = controller.update(fillLevel);
            tank.setFillRate(newFillRate);

            final FixedMillisecond time = new FixedMillisecond((long) (t * 1000));
            series.addOrUpdate(time, fillLevel);
            setpointSeries.addOrUpdate(time, desiredFillLevel);
        }
    }

    public static void main(String[] args) {
        final PIDTestGui gui = new PIDTestGui();
        gui.start();
    }
}
