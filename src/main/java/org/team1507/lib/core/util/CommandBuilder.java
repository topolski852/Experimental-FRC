//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.lib.core.util;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.ConcurrentModificationException;
import java.util.function.BiConsumer;
import java.util.function.BooleanSupplier;

import org.team1507.lib.core.logging.Telemetry;

/**
 * A lightweight, allocation‑free command builder for IO‑focused subsystems.
 *
 * <p>CommandBuilder provides a simple, declarative way to construct commands
 * with predictable runtime behavior. All callbacks are stored up front and
 * executed without creating new objects during scheduler cycles.
 *
 * <p>Each command supports:
 * <ul>
 *   <li>Initialization and execution callbacks</li>
 *   <li>Normal finish conditions</li>
 *   <li>Timeout‑based termination</li>
 *   <li>Stall‑based termination</li>
 *   <li>End handlers with full termination‑cause information</li>
 * </ul>
 *
 * <p>Command lifecycle telemetry is emitted automatically, including:
 * <ul>
 *   <li>Start and end timestamps</li>
 *   <li>Active state</li>
 *   <li>Duration</li>
 *   <li>Interruption, timeout, and stall flags</li>
 * </ul>
 *
 * <p>This builder is designed for subsystems that perform direct IO and benefit
 * from deterministic, low‑overhead command execution.
 */
public class CommandBuilder extends Command {

    // -----------------------------
    // Lifecycle callbacks
    // -----------------------------
    private Runnable onInit = () -> {};
    private Runnable onExecute = () -> {};

    /**
     * Handler invoked when the command ends, providing full termination context.
     */
    @FunctionalInterface
    public interface EndHandler {
        /**
         * @param interrupted true if the command was interrupted
         * @param timedOut    true if the command ended due to timeout
         * @param stalled     true if the command ended due to stall detection
         */
        void accept(boolean interrupted, boolean timedOut, boolean stalled);
    }

    private EndHandler onEnd = (i, t, s) -> {};

    // -----------------------------
    // Finish conditions
    // -----------------------------
    private BooleanSupplier finishCondition = () -> false;
    private BooleanSupplier stallCondition = () -> false;

    // -----------------------------
    // Timeout tracking
    // -----------------------------
    private double timeoutSeconds = -1.0;
    private double startTime = 0.0;
    private boolean timedOut = false;

    // -----------------------------
    // Stall tracking
    // -----------------------------
    private boolean stalled = false;

    /**
     * Creates a new CommandBuilder and declares subsystem requirements.
     *
     * @param requirements subsystems required by this command
     */
    public CommandBuilder(Subsystem... requirements) {
        addRequirements(requirements);
    }

    /**
     * Assigns a name to the command for debugging and telemetry.
     *
     * @param name command name
     * @return this builder
     */
    public CommandBuilder named(String name) {
        setName(name);
        return this;
    }

    /**
     * Sets the initialization routine, executed once when the command starts.
     *
     * @param r initialization action
     * @return this builder
     */
    public CommandBuilder onInitialize(Runnable r) {
        ensureNotRunning();
        this.onInit = r;
        return this;
    }

    /**
     * Sets the execution routine, executed every scheduler cycle.
     *
     * @param r execution action
     * @return this builder
     */
    public CommandBuilder onExecute(Runnable r) {
        ensureNotRunning();
        this.onExecute = r;
        return this;
    }

    /**
     * Sets the end handler, receiving full termination‑cause information.
     *
     * @param r end handler
     * @return this builder
     */
    public CommandBuilder onEnd(EndHandler r) {
        ensureNotRunning();
        this.onEnd = r;
        return this;
    }

    /**
     * Convenience overload for end handlers that do not use termination causes.
     *
     * @param r action to run when the command ends
     * @return this builder
     */
    public CommandBuilder onEnd(Runnable r) {
        return onEnd((i, t, s) -> r.run());
    }

    /**
     * Convenience overload for handlers that use interruption and timeout causes.
     *
     * @param r handler receiving (interrupted, timedOut)
     * @return this builder
     */
    public CommandBuilder onEnd(BiConsumer<Boolean, Boolean> r) {
        return onEnd((i, t, s) -> r.accept(i, t));
    }

    /**
     * Sets the normal finish condition for the command.
     *
     * @param condition supplier returning true when the command should end
     * @return this builder
     */
    public CommandBuilder isFinished(BooleanSupplier condition) {
        ensureNotRunning();
        this.finishCondition = condition;
        return this;
    }

    /**
     * Convenience overload for constant finish conditions.
     *
     * @param finished whether the command should finish immediately
     * @return this builder
     */
    public CommandBuilder isFinished(boolean finished) {
        return isFinished(() -> finished);
    }

    /**
     * Sets a timeout for the command. When the timeout expires, the command ends
     * and {@code timedOut} is set to true.
     *
     * @param seconds timeout duration
     * @return this builder
     */
    public CommandBuilder timeout(double seconds) {
        ensureNotRunning();
        this.timeoutSeconds = seconds;
        return this;
    }

    /**
     * Runs the command until the timeout expires, ignoring normal finish conditions.
     *
     * @param seconds timeout duration
     * @return this builder
     */
    public CommandBuilder timeoutOnly(double seconds) {
        ensureNotRunning();
        this.timeoutSeconds = seconds;
        this.finishCondition = () -> false;
        return this;
    }

    /**
     * Sets a stall finish condition. When the supplier returns true, the command
     * ends and {@code stalled} is set to true.
     *
     * @param stallCondition supplier indicating stall state
     * @return this builder
     */
    public CommandBuilder stallFinish(BooleanSupplier stallCondition) {
        ensureNotRunning();
        this.stallCondition = stallCondition;
        return this;
    }

    // -----------------------------
    // WPILib lifecycle
    // -----------------------------

    @Override
    public void initialize() {
        timedOut = false;
        stalled = false;
        startTime = Timer.getFPGATimestamp();

        // Telemetry: command start
        String base = "Command/" + getName();
        Telemetry.set(base + "/Active", true);
        Telemetry.set(base + "/StartTime", startTime);

        onInit.run();
    }

    @Override
    public void execute() {
        onExecute.run();
    }

    @Override
    public void end(boolean interrupted) {
        double endTime = Timer.getFPGATimestamp();
        String base = "Command/" + getName();

        // Telemetry: command end
        Telemetry.set(base + "/Active", false);
        Telemetry.set(base + "/EndTime", endTime);
        Telemetry.set(base + "/Duration", endTime - startTime);

        // Telemetry: termination causes
        Telemetry.set(base + "/Interrupted", interrupted);
        Telemetry.set(base + "/TimedOut", timedOut);
        Telemetry.set(base + "/Stalled", stalled);

        onEnd.accept(interrupted, timedOut, stalled);
    }

    @Override
    public boolean isFinished() {

        // Timeout check
        if (timeoutSeconds >= 0) {
            double elapsed = Timer.getFPGATimestamp() - startTime;
            if (elapsed >= timeoutSeconds) {
                timedOut = true;
                return true;
            }
        }

        // Stall check
        if (stallCondition.getAsBoolean()) {
            stalled = true;
            return true;
        }

        // Normal finish
        return finishCondition.getAsBoolean();
    }

    // -----------------------------
    // Internal safety
    // -----------------------------
    private void ensureNotRunning() {
        if (isScheduled()) {
            throw new ConcurrentModificationException(
                "Cannot modify a running CommandBuilder"
            );
        }
    }
}
