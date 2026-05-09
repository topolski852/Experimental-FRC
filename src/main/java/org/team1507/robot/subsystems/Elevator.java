//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;

import edu.wpi.first.wpilibj2.command.Command;

import org.team1507.lib.core.framework.Subsystem1507;
import org.team1507.lib.core.impl.ctre.Motor1507;
import org.team1507.lib.core.util.CommandBuilder;

import static org.team1507.robot.Constants.kElevator.*;

// ─────────────────────────────────────────────────────────────────────────────
// Elevator
//
// Single-motor linear elevator with a state machine at its core.
//
// ── State Machine ────────────────────────────────────────────────────────────
//
//   IDLE          Motor holds its last position via brake mode.
//                 Manual commands can override with runDuty() directly.
//
//   MOVING_UP     Motor actively seeks the target setpoint.
//   MOVING_DOWN   Transitions to AT_SETPOINT when within TOLERANCE_INCHES.
//                 Transitions to IDLE if a stall is detected.
//
//   AT_SETPOINT   Motor holds the target position with a gravity feedforward.
//
//   FAULT         Motor is stopped. Transitions to IDLE only via clearFault().
//                 Use when a hard mechanical limit or sensor failure occurs.
//
// ── Why a state machine? ─────────────────────────────────────────────────────
//   Every motor call goes through the state machine in updateStateMachine().
//   This means:
//     - Position holding happens automatically (no polling in commands)
//     - Stalls are caught and handled consistently
//     - Faults are isolated — nothing moves until explicitly cleared
//     - NT entries like Elevator/State and Elevator/AtSetpoint can be wired
//       directly to Elastic dashboard indicators for match-day diagnostics
//
// ── Wiring in Robot.java ─────────────────────────────────────────────────────
//   operator.povUp()      → elevator.goToCommand(Setpoint.HIGH)
//   operator.povDown()    → elevator.goToCommand(Setpoint.STOW)
//   operator.leftBumper() → elevator.manualUpCommand()
//   operator.leftTrigger()→ elevator.manualDownCommand()
// ─────────────────────────────────────────────────────────────────────────────
public final class Elevator extends Subsystem1507 {

    // =========================================================================
    // State Machine Types
    // =========================================================================

    /** All possible operating states of the elevator. */
    public enum State {
        /** Motor idle — brake mode holds last position. */
        IDLE,
        /** Motor actively driving upward toward a setpoint. */
        MOVING_UP,
        /** Motor actively driving downward toward a setpoint. */
        MOVING_DOWN,
        /** Motor at the target setpoint — actively holding position. */
        AT_SETPOINT,
        /** Motor stopped due to a fault. Call clearFaultCommand() to recover. */
        FAULT
    }

    /** Named height presets. Heights are defined in {@code kElevator}. */
    public enum Setpoint { STOW, LOW, MID, HIGH }

    // =========================================================================
    // Hardware
    // =========================================================================

    private final Motor1507 motor;
    private final BaseStatusSignal[] motorSignals;

    // =========================================================================
    // State
    // =========================================================================

    private State state = State.IDLE;

    /** Target height in inches. Starts at STOW so the elevator stays down on boot. */
    private double targetInches = STOW_INCHES;

    // =========================================================================
    // Constructor
    // =========================================================================

    public Elevator() {
        super("Elevator");

        motor = new Motor1507(
            key("Motor"),
            Motor1507.Type.FX,
            MOTOR_CAN_ID,
            CONFIG
        );

        motorSignals = motor.getSignals();
    }

    // =========================================================================
    // Periodic — runs every robot loop
    // =========================================================================

    @Override
    public void periodic() {
        // Refresh CAN signals so all reads below reflect the current cycle
        BaseStatusSignal.refreshAll(motorSignals);

        // Run the state machine every loop — this is where all motor control lives
        updateStateMachine();

        // Publish state for Elastic dashboard and AdvantageScope
        log("State",          state.name());
        log("PositionInches", getPositionInches());
        log("TargetInches",   targetInches);
        log("AtSetpoint",     isAtSetpoint());
        log("Stalled",        motor.isStalled());
    }

    // =========================================================================
    // State Machine
    // =========================================================================

    /**
     * Core state machine — called every loop from {@link #periodic()}.
     *
     * <p>This is the only place motor control outputs are written for
     * automatic modes. Manual commands bypass this by calling runDuty() while
     * the elevator is in IDLE (periodic() does nothing in IDLE).
     */
    private void updateStateMachine() {
        switch (state) {

            case MOVING_UP:
            case MOVING_DOWN:
                // Drive toward the target with a constant gravity feedforward
                motor.setPositionVoltage(inchesToRotations(targetInches), GRAVITY_FF_VOLTS);

                if (isAtSetpoint()) {
                    state = State.AT_SETPOINT;

                } else if (motor.isStalled()) {
                    // Stall while moving — back off to IDLE so we don't burn the motor
                    warn("Stall detected while moving — transitioning to IDLE.");
                    motor.stop();
                    state = State.IDLE;
                }
                break;

            case AT_SETPOINT:
                // Hold position actively with gravity compensation
                motor.setPositionVoltage(inchesToRotations(targetInches), GRAVITY_FF_VOLTS);
                break;

            case FAULT:
                // Motor is stopped — nothing moves until clearFaultCommand() is called
                motor.stop();
                break;

            case IDLE:
                // Do not drive the motor — brake mode holds the last position.
                // Manual commands (runDuty) will override freely in this state.
                break;
        }
    }

    // =========================================================================
    // Control Methods
    // =========================================================================

    /**
     * Commands the elevator to a named setpoint.
     *
     * <p>Determines the direction (MOVING_UP or MOVING_DOWN) from the current
     * position and updates the state machine target. If already within tolerance
     * of the target, transitions directly to AT_SETPOINT.
     *
     * @param setpoint the named target position
     */
    public void setSetpoint(Setpoint setpoint) {
        double newTargetInches = switch (setpoint) {
            case STOW -> STOW_INCHES;
            case LOW  -> LOW_INCHES;
            case MID  -> MID_INCHES;
            case HIGH -> HIGH_INCHES;
        };

        if (state == State.FAULT) return;  // Do not accept commands while faulted

        targetInches = newTargetInches;

        if (isAtSetpoint()) {
            state = State.AT_SETPOINT;
        } else if (getPositionInches() < targetInches) {
            state = State.MOVING_UP;
        } else {
            state = State.MOVING_DOWN;
        }
    }

    /**
     * Triggers a fault state, stopping the elevator.
     *
     * <p>Use when a hard mechanical limit or sensor failure is detected.
     * Call {@link #clearFaultCommand()} to recover.
     *
     * @param reason description of the fault (logged to Driver Station)
     */
    public void triggerFault(String reason) {
        fault("FAULT: " + reason);
        state = State.FAULT;
    }

    // =========================================================================
    // State Queries
    // =========================================================================

    /** Returns the current elevator height in inches, from the motor's rotor position. */
    public double getPositionInches() {
        return motor.getRotorPosition() * INCHES_PER_ROTATION;
    }

    /** Returns the target height in inches from the last setSetpoint() call. */
    public double getTargetInches() {
        return targetInches;
    }

    /** Returns true when the elevator is within {@code TOLERANCE_INCHES} of the target. */
    public boolean isAtSetpoint() {
        return Math.abs(getPositionInches() - targetInches) < TOLERANCE_INCHES;
    }

    /** Returns the current state machine state. */
    public State getState() {
        return state;
    }

    // =========================================================================
    // Commands
    // =========================================================================

    /**
     * Drives the elevator to a named setpoint and waits until it arrives.
     *
     * <p>Finishes when the elevator reaches {@code AT_SETPOINT} or enters
     * {@code FAULT}. Motor continues holding position after the command
     * finishes — no onEnd stop needed.
     *
     * @param setpoint the named target position
     */
    public Command goToCommand(Setpoint setpoint) {
        return new CommandBuilder(this)
            .named("Elevator.goTo")
            .onInitialize(() -> setSetpoint(setpoint))
            .isFinished(() -> state == State.AT_SETPOINT || state == State.FAULT);
    }

    /**
     * Drives the elevator upward manually while held.
     *
     * <p>Bypasses the state machine (elevator stays in IDLE). Stops the
     * motor on release. Intended for fine adjustment after initial zeroing.
     */
    public Command manualUpCommand() {
        return new CommandBuilder(this)
            .named("Elevator.manualUp")
            .onExecute(() -> motor.runDuty(MANUAL_DUTY))
            .onEnd((interrupted, timedOut, stalled) -> motor.stop())
            .runsUntilInterrupted();
    }

    /**
     * Drives the elevator downward manually while held.
     *
     * <p>Bypasses the state machine (elevator stays in IDLE). Stops the
     * motor on release.
     */
    public Command manualDownCommand() {
        return new CommandBuilder(this)
            .named("Elevator.manualDown")
            .onExecute(() -> motor.runDuty(-MANUAL_DUTY))
            .onEnd((interrupted, timedOut, stalled) -> motor.stop())
            .runsUntilInterrupted();
    }

    /**
     * Clears a FAULT state and returns the elevator to IDLE.
     *
     * <p>Safe to run while disabled so operators can recover from faults
     * before enabling the robot.
     */
    public Command clearFaultCommand() {
        return new CommandBuilder(this)
            .named("Elevator.clearFault")
            .onInitialize(() -> {
                state = State.IDLE;
                System.out.println("[" + getName() + "] Fault cleared — elevator returned to IDLE.");
            })
            .isFinished(true)
            .runsWhenDisabled(true);
    }

    // =========================================================================
    // Internal Helpers
    // =========================================================================

    /** Converts a height in inches to motor rotations using the gear ratio. */
    private static double inchesToRotations(double inches) {
        return inches / INCHES_PER_ROTATION;
    }
}
