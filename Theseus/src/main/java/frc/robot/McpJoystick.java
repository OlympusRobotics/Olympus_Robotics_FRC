package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Reads simulated joystick state from NetworkTables written by the MCP
 * {@code simulate_joystick} tool. Call {@link #poll()} once per robot cycle
 * (in {@code robotPeriodic()}) to count down the duration and auto-release.
 *
 * <p>NT layout: {@code /MCP/Joystick/{port}/axes}, {@code buttons}, {@code pov},
 * {@code seq}, {@code duration}, {@code timestamp}.
 */
public class McpJoystick {
    private final NetworkTable table;
    private long lastSeq = 0;
    private int remainingCycles = 0;

    // Cached state read each cycle
    private double[] axes = new double[6];
    private boolean[] buttons = new boolean[11]; // index 0 unused, 1-10
    private int pov = -1;

    public McpJoystick(int port) {
        table = NetworkTableInstance.getDefault().getTable("MCP/Joystick/" + port);
    }

    /** Call once per robot cycle to read NT and count down auto-release. */
    public void poll() {
        long seq = table.getEntry("seq").getInteger(0);
        if (seq != lastSeq) {
            // Fresh input arrived
            lastSeq = seq;
            remainingCycles = (int) table.getEntry("duration").getInteger(25);
        }

        if (remainingCycles > 0) {
            axes = table.getEntry("axes").getDoubleArray(new double[6]);
            buttons = table.getEntry("buttons").getBooleanArray(new boolean[11]);
            pov = (int) table.getEntry("pov").getInteger(-1);
            remainingCycles--;
        } else {
            // Duration expired — release everything
            axes = new double[6];
            buttons = new boolean[11];
            pov = -1;
        }
    }

    /** @return true if there is active simulated input. */
    public boolean isActive() {
        return remainingCycles > 0;
    }

    public double getAxis(int axis) {
        return (axis >= 0 && axis < axes.length) ? axes[axis] : 0;
    }

    public boolean getButton(int button) {
        return (button >= 1 && button < buttons.length) && buttons[button];
    }

    public int getPOV() {
        return pov;
    }
}
