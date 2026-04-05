package org.firstinspires.ftc.robotarm.data;

/**
 * An inclusive numeric range [min, max].
 * Used for joint position limits, velocity bounds, etc.
 */
public class Range {
    public final double min;
    public final double max;

    public Range(double min, double max) {
        this.min = min;
        this.max = max;
    }

    public boolean contains(double value) {
        return value >= min && value <= max;
    }

    public double clamp(double value) {
        return Math.max(min, Math.min(max, value));
    }

    public double span() {
        return max - min;
    }

    @Override
    public String toString() {
        return String.format("[%.3f, %.3f]", min, max);
    }
}
