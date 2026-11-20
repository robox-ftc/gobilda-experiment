package org.firstinspires.ftc.teamcode.devices;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

/**
 * Keeps timed drive moves the same length as the battery runs down.
 *
 * <p>With the drive motors in RUN_WITHOUT_ENCODER (see {@link Drivetrain}),
 * a power is a fraction of the battery voltage, and the robot's speed follows
 * the voltage: about 10% slower at 11.5 V than at 12.7 V. Multiplying every
 * drive power by {@link #factor()} makes it the power it would be at
 * {@link #NOMINAL_VOLTS}, so a move measured once takes the same time on any
 * charge.
 */
public final class BatteryCompensation {

    /** The voltage the measured speeds are quoted at. */
    public static final double NOMINAL_VOLTS = 12.0;

    /** Readings outside this are taken as a bad reading, and not corrected for. */
    static final double MIN_VOLTS = 9.0, MAX_VOLTS = 15.0;

    private final VoltageSensor sensor;

    /** Uses the first voltage sensor in the configuration (the hub's); none means no correction. */
    public BatteryCompensation(HardwareMap hardwareMap) {
        VoltageSensor found = null;
        try {
            for (VoltageSensor s : hardwareMap.voltageSensor) {
                found = s;
                break;
            }
        } catch (RuntimeException e) {
            found = null; // off the robot there may be no voltage sensors at all
        }
        this.sensor = found;
    }

    /** For tests: a given sensor, or null for none. */
    public BatteryCompensation(VoltageSensor sensor) {
        this.sensor = sensor;
    }

    /** The battery voltage, or NaN if there is no usable reading. */
    public double volts() {
        if (sensor == null) return Double.NaN;
        double v = sensor.getVoltage();
        return v >= MIN_VOLTS && v <= MAX_VOLTS ? v : Double.NaN;
    }

    /** What to multiply a drive power by: NOMINAL_VOLTS / battery volts, or 1 without a reading. */
    public double factor() {
        double v = volts();
        return Double.isNaN(v) ? 1.0 : NOMINAL_VOLTS / v;
    }
}
