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

import static org.team1507.robot.Constants.kFeeder.*;

// ─────────────────────────────────────────────────────────────────────────────
// Feeder
//
// Single-motor ball feeder, velocity-controlled.
//
// The Feeder subsystem moves game pieces from the intake into the shooter.
// It does NOT know about the Shooter — coordination (wait for shooter RPM,
// then feed) lives in RobotBehaviors.java. This keeps the subsystem simple
// and reusable regardless of how the shooter is controlled.
//
// ── Why velocity control? ────────────────────────────────────────────────────
//   Velocity control at a fixed RPM feeds pieces at a consistent rate
//   regardless of battery voltage or mechanical load. This prevents jams
//   and double-feeds more reliably than open-loop duty cycle.
//
// ── Wiring in RobotBehaviors.java ────────────────────────────────────────────
//   See RobotBehaviors.shoot() for the full Shooter + Feeder coordination.
// ─────────────────────────────────────────────────────────────────────────────
public final class Feeder extends Subsystem1507 {

    private final Motor1507 motor;
    private final BaseStatusSignal[] motorSignals;

    public Feeder() {
        super("Feeder");

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
        log("RPM",     getRPM());
        log("Feeding", motor.getMotorVoltage() > 0);
    }

    // =========================================================================
    // Control Methods
    // =========================================================================

    /** Runs the feeder at the configured feed RPM. */
    public void feed() {
        motor.setVelocityRPS(FEED_RPM / 60.0);
    }

    /** Runs the feeder in reverse at the configured vomit RPM. */
    public void vomit() {
        motor.setVelocityRPS(VOMIT_RPM / 60.0);
    }

    /** Stops the feeder motor. */
    public void stop() {
        motor.stop();
    }

    /** Returns the current feeder RPM (positive = feeding direction). */
    public double getRPM() {
        return motor.getRotorVelocity() * 60.0;
    }

    // =========================================================================
    // Commands
    // =========================================================================

    /**
     * Runs the feeder at feed speed while held.
     *
     * <p>In most cases, call this from {@link org.team1507.robot.RobotBehaviors#shoot()}
     * rather than binding it directly, so the feeder only runs after the
     * shooter has reached its target RPM.
     */
    public Command feedCommand() {
        return new CommandBuilder(this)
            .named("Feeder.feed")
            .onInitialize(this::feed)
            .onEnd(this::stop)
            .runsUntilInterrupted();
    }

    /**
     * Runs the feeder in reverse while held.
     * Use to eject a jammed game piece.
     */
    public Command vomitCommand() {
        return new CommandBuilder(this)
            .named("Feeder.vomit")
            .onInitialize(this::vomit)
            .onEnd(this::stop)
            .runsUntilInterrupted();
    }

    /**
     * One-shot stop command.
     */
    public Command stopCommand() {
        return new CommandBuilder(this)
            .named("Feeder.stop")
            .onInitialize(this::stop)
            .isFinished(true);
    }
}
