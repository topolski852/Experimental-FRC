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

import static org.team1507.robot.Constants.kIntake.*;

// ─────────────────────────────────────────────────────────────────────────────
// Intake
//
// Single-motor duty-cycle roller intake.
//
// This is the simplest possible mechanism example — one motor, three commands.
// Use it as a reference for any mechanism that just runs forward/reverse:
// conveyor belts, ejectors, agitators, etc.
//
// ── Wiring in Robot.java ─────────────────────────────────────────────────────
//   driver.rightBumper() → intake.runCommand()
//   driver.rightTrigger()→ intake.reverseCommand()
// ─────────────────────────────────────────────────────────────────────────────
public final class Intake extends Subsystem1507 {

    private final Motor1507 motor;
    private final BaseStatusSignal[] motorSignals;

    public Intake() {
        super("Intake");

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
        log("Running", Math.abs(motor.getMotorVoltage()) > 0.1);
    }

    // =========================================================================
    // Control Methods
    // =========================================================================

    /** Runs the intake roller forward at the configured duty cycle. */
    public void run() {
        motor.runDuty(DUTY_INTAKE);
    }

    /** Runs the intake roller in reverse at the configured duty cycle. */
    public void reverse() {
        motor.runDuty(DUTY_REVERSE);
    }

    /** Stops the intake motor. */
    public void stop() {
        motor.stop();
    }

    // =========================================================================
    // Commands
    // =========================================================================

    /**
     * Runs the intake forward while held.
     * Use with {@code whileTrue} on a driver button.
     */
    public Command runCommand() {
        return new CommandBuilder(this)
            .named("Intake.run")
            .onExecute(this::run)
            .onEnd(this::stop)
            .runsUntilInterrupted();
    }

    /**
     * Runs the intake in reverse while held.
     * Use with {@code whileTrue} to eject a stuck piece.
     */
    public Command reverseCommand() {
        return new CommandBuilder(this)
            .named("Intake.reverse")
            .onExecute(this::reverse)
            .onEnd(this::stop)
            .runsUntilInterrupted();
    }

    /**
     * One-shot stop command.
     * Use with {@code onTrue} to explicitly stop the intake.
     */
    public Command stopCommand() {
        return new CommandBuilder(this)
            .named("Intake.stop")
            .onInitialize(this::stop)
            .isFinished(true);
    }
}
