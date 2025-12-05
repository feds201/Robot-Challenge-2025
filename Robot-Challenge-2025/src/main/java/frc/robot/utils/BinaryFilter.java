package frc.robot.utils;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUsageId;

import java.util.Arrays;

/**
 * This class implements a digital filter for Boolean signals.
 * Static factory methods are provided to create commonly used types of filters.
 *
 * <p>Unlike linear filters which use gain coefficients, Boolean filters operate using logic
 * windows, thresholds, and state comparisons.
 *
 * <p>What can boolean filters do? They can filter, or diminish, the effects of
 * undesirable signal noise. Rapid toggling, or "bouncing," can be indicative of mechanical
 * switch noise or electrical interference. A "Debouncer" (Low Pass) smooths out the signal,
 * requiring a steady state before changing the output. A "Pulse/Edge" (High Pass) filter detects
 * changes, useful for triggering actions only when a state changes.
 *
 * <p>Example FRC applications of filters:
 * - Debouncing a limit switch that flickers when a mechanism hits it hard.
 * - Detecting the exact moment a beam-break sensor is tripped (Edge Detection).
 * - Ignoring brief "glitches" in communication or vision targets (Voting/Median).
 *
 * <p>Note 1: calculate() should be called by the user on a known, regular period. You can use a
 * Notifier for this or do it "inline" with code in a periodic function.
 */
public class BinaryFilter {
    private final boolean[] m_inputs;
    private final boolean[] m_outputs;
    private final FilterMode m_mode;
    private final int m_param; // Generic parameter used for threshold, tap count, etc.

    private int m_inputIndex = 0;
    private int m_outputIndex = 0;
    private boolean m_lastOutput;

    private static int instances;

    private enum FilterMode {
        VOTING, // Moving Average equivalent
        DEBOUNCE, // Low Pass IIR equivalent
        RISING_EDGE, // Finite Difference equivalent
        FALLING_EDGE,
        TOGGLE // Flip-flop
    }

    /**
     * Private constructor. Use static factories.
     *
     * @param taps  The size of the input history buffer.
     * @param mode  The logic mode of the filter.
     * @param param Auxiliary parameter (threshold, etc) dependent on mode.
     */
    private BinaryFilter(int taps, FilterMode mode, int param) {
        m_inputs = new boolean[taps];
        // Output buffer size 1 is usually sufficient for these boolean modes,
        // but we keep the structure generic.
        m_outputs = new boolean[1];
        m_mode = mode;
        m_param = param;

        instances++;
        MathSharedStore.reportUsage(MathUsageId.kFilter_Linear, instances); // Reporting as Linear family
    }

    /**
     * Creates a Debouncer (Low-Pass Filter) equivalent.
     *
     * <p>The output will only update to match the input if the input has remained constant
     * for the specified duration. This filters out high-frequency noise ("bouncing").
     *
     * @param debounceTime The time the signal must remain stable in seconds.
     * @param period       The period in seconds between samples taken by the user.
     * @return Binary filter.
     */
    public static BinaryFilter debounce(double debounceTime, double period) {
        int taps = (int) Math.ceil(debounceTime / period);
        return new BinaryFilter(Math.max(1, taps), FilterMode.DEBOUNCE, 0);
    }

    /**
     * Creates an Edge Detector (High-Pass / Finite Difference) filter.
     *
     * <p>Returns true for one cycle when the input changes from False to True (Rising)
     * or True to False (Falling).
     *
     * @param type The type of edge to detect (Rising or Falling).
     * @return Binary filter.
     */
    public static BinaryFilter edge(EdgeType type) {
        // We only need 2 samples (current and previous) to detect an edge
        FilterMode mode = (type == EdgeType.Rising) ? FilterMode.RISING_EDGE : FilterMode.FALLING_EDGE;
        return new BinaryFilter(2, mode, 0);
    }

    /**
     * Creates a Voting / Majority filter (Moving Average equivalent).
     *
     * <p>The output is True if the number of True samples in the window exceeds the threshold.
     * This is useful for "glitch" filtering where you want to ignore occasional bad data frames
     * without the strict reset timing of a debouncer.
     *
     * @param taps      The number of samples to check.
     * @param threshold The number of "True" samples required to output "True".
     * @return Binary filter.
     */
    public static BinaryFilter voting(int taps, int threshold) {
        if (taps <= 0) {
            throw new IllegalArgumentException("Number of taps was not at least 1");
        }
        if (threshold > taps) {
            throw new IllegalArgumentException("Threshold cannot be larger than taps");
        }

        return new BinaryFilter(taps, FilterMode.VOTING, threshold);
    }

    /**
     * Reset the filter state.
     */
    public void reset() {
        Arrays.fill(m_inputs, false);
        Arrays.fill(m_outputs, false);
        m_inputIndex = 0;
        m_outputIndex = 0;
        m_lastOutput = false;
    }

    /**
     * Calculates the next value of the filter.
     *
     * @param input Current input value.
     * @return The filtered value at this step
     */
    public boolean calculate(boolean input) {
        // Push to input buffer (Circular Logic)
        m_inputs[m_inputIndex] = input;

        // We increment index after fetching logic, but for calculation we might need
        // to look back.
        // Let's normalize the circular buffer access for clarity in logic below.
        // m_inputIndex is the "Head" (Current).

        boolean retVal = false;

        switch (m_mode) {
            case DEBOUNCE:
                retVal = calculateDebounce(input);
                break;
            case RISING_EDGE:
                retVal = calculateEdge(true);
                break;
            case FALLING_EDGE:
                retVal = calculateEdge(false);
                break;
            case VOTING:
                retVal = calculateVoting();
                break;
            default:
                retVal = input;
                break;
        }

        // Advance Input Head
        m_inputIndex = (m_inputIndex + 1) % m_inputs.length;

        // Store Output
        m_outputs[m_outputIndex] = retVal;
        m_outputIndex = (m_outputIndex + 1) % m_outputs.length;

        m_lastOutput = retVal;
        return retVal;
    }

    /**
     * Logic for Debounce.
     * Requires ALL samples in the buffer to match the input to change state.
     * Otherwise, holds last state.
     */
    private boolean calculateDebounce(boolean input) {
        for (boolean sample : m_inputs) {
            if (sample != input) {
                // If the history is not uniform, return the LAST stable output
                // (Hysteresis behavior)
                return m_lastOutput;
            }
        }
        // If we are here, the entire buffer matches the current input.
        return input;
    }

    /**
     * Logic for Edge Detection.
     *
     * @param rising True for rising edge, False for falling.
     */
    private boolean calculateEdge(boolean rising) {
        // Current is at m_inputIndex
        boolean current = m_inputs[m_inputIndex];

        // Previous is at (Index - 1) wrapping around
        int prevIndex = (m_inputIndex - 1 + m_inputs.length) % m_inputs.length;
        boolean previous = m_inputs[prevIndex];

        if (rising) {
            return current && !previous;
        } else {
            return !current && previous;
        }
    }

    /**
     * Logic for Voting.
     */
    private boolean calculateVoting() {
        int trueCount = 0;
        for (boolean sample : m_inputs) {
            if (sample) {
                trueCount++;
            }
        }
        return trueCount >= m_param;
    }

    /**
     * Returns the last value calculated by the BinaryFilter.
     *
     * @return The last value.
     */
    public boolean lastValue() {
        return m_lastOutput;
    }

    /**
     * Enum for Edge Detection types.
     */
    public enum EdgeType {
        Rising,
        Falling
    }
}