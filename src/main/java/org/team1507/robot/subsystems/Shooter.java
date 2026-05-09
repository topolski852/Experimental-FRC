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

import static org.team1507.robot.Constants.kShooter.*;

// ─────────────────────────────────────────────────────────────────────────────
// Shooter
//
// Single-motor flywheel shooter, velocity-controlled.
//
// ── Design note ──────────────────────────────────────────────────────────────
//   setRPM() is the clean seam for a future ML model upgrade. When a model
//   is available, replace the fixed RPM with:
//
//     motor.setVelocityRPS(model.getRPM(shotRecord) / 60.0)
//
//   The rest of the subsystem stays exactly the same.
//
// ── Wiring in RobotBehaviors.java ────────────────────────────────────────────
//   The Shooter does NOT know about the Feeder. Cross-subsystem coordination
//   lives in RobotBehaviors.shoot(), which:
//     1. Spins up the shooter (via spinUpCommand)
//     2. Waits until isAtRPM()
//     3. Enables the feeder
//   This lets the Shooter stay simple and independently testable.
// ─────────────────────────────────────────────────────────────────────────────
public final class Shooter extends Subsystem1507 {

    private final Motor1507 motor;
    private final BaseStatusSignal[] motorSignals;

    /** Last commanded target in RPM. */
    private double targetRPM = 0.0;

    public Shooter() {
        super("Shooter");

        motor = new Motor1507(
            key("Motor"),
            Motor1507.Type.FX,
            MOTOR_CAN_ID,
            CONFIG
        );

        motorSignals = motor.getSignals();
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(motorSignals);
        log("RPM",       getRPM());
        log("TargetRPM", targetRPM);
        log("AtRPM",     isAtRPM());
    }

    // =========================================================================
    // Control Methods
    // =========================================================================

    /**
     * Commands the shooter to a target speed.
     *
     * <p>This is the primary control entry point. In a future ML-enabled build,
     * replace the fixed {@code rpm} argument with a model output:
     * {@code setRPM(model.getRPM(shotRecord))}.
     *
     * @param rpm target speed in rotations per minute
     */
    public void setRPM(double rpm) {
        targetRPM = rpm;
        motor.setVelocityRPS(rpm / 60.0);
    }

    /** Stops the shooter motor. */
    public void stop() {
        targetRPM = 0.0;
        motor.stop();
    }

    /** Returns the current flywheel speed in RPM. */
    public double getRPM() {
        return motor.getRotorVelocity() * 60.0;
    }

    /** Returns the last commanded target speed in RPM. */
    public double getTargetRPM() {
        return targetRPM;
    }

    /**
     * Returns true when the flywheel is within {@code RPM_TOLERANCE} of the target.
     *
     * <p>Use this in {@link org.team1507.robot.RobotBehaviors} to gate the
     * feeder: don't feed until the shooter confirms it's ready.
     */
    public boolean isAtRPM() {
        return Math.abs(getRPM() - targetRPM) < RPM_TOLERANCE;
    }

    // =========================================================================
    // Commands
    // =========================================================================

    /**
     * Spins up the shooter to a target RPM and runs until interrupted.
     *
     * <p>The command does not stop the motor on completion — the motor stays
     * at speed so the next command in a sequence (e.g. the Feeder) can start
     * immediately without a gap. The calling behavior in
     * {@link org.team1507.robot.RobotBehaviors} is responsible for stopping.
     *
     * @param rpm target speed in rotations per minute
     */
    public Command spinUpCommand(double rpm) {
        return new CommandBuilder(this)
            .named("Shooter.spinUp")
            .onInitialize(() -> setRPM(rpm))
            .onEnd((interrupted, timedOut, stalled) -> stop());
    }

    /**
     * One-shot stop command.
     */
    public Command stopCommand() {
        return new CommandBuilder(this)
            .named("Shooter.stop")
            .onInitialize(this::stop)
            .isFinished(true);
    }
}
