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
// Single-motor torque-controlled roller intake (TorqueCurrentFOC).
//
// Uses TorqueCurrentFOC instead of duty cycle to solve the brownout problem
// caused by increasing back-pressure as the hopper fills with balls. In duty
// cycle mode, current draw rises with load — in torque mode, the motor delivers
// a fixed torque regardless of load, keeping current predictable and bounded.
//
// Tune TORQUE_AMPS_INTAKE in Constants.kIntake:
//   - Too low → roller struggles against a full hopper
//   - Too high → brownouts persist under load
//
// ── Wiring in Robot.java ─────────────────────────────────────────────────────
//   operator.rightBumper() → intake.runCommand()
//   operator.rightTrigger()→ intake.reverseCommand()
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
        log("Running",          Math.abs(motor.getMotorVoltage()) > 0.1);
        log("StatorCurrentAmps", motor.getStatorCurrent());
    }

    // =========================================================================
    // Control Methods
    // =========================================================================

    /** Runs the intake roller forward at the configured torque current. */
    public void run() {
        motor.runTorqueCurrent(TORQUE_AMPS_INTAKE);
    }

    /** Runs the intake roller in reverse at the configured torque current. */
    public void reverse() {
        motor.runTorqueCurrent(TORQUE_AMPS_REVERSE);
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
