//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╔╝╚██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

import org.team1507.robot.auto.AutoBuilder;
import org.team1507.robot.subsystems.Elevator.Setpoint;

import static org.team1507.robot.Constants.kShooter.*;

// ─────────────────────────────────────────────────────────────────────────────
// RobotBehaviors
//
// A single shared library of coordinated, multi-subsystem robot behaviors.
// These are complete robot actions that require two or more subsystems working
// together in sequence or in parallel.
//
// KEY PRINCIPLE:
//   A behavior defined here is the ONE definition used everywhere.
//   Whether triggered by a driver button in teleop or a step in an auto
//   routine, it calls the same method. You never write the same behavior twice.
//
// HOW TELEOP USES THIS:
//   In Robot.java:
//     operator.rightTrigger().whileTrue(RobotBehaviors.shoot());
//     driver.a().onTrue(RobotBehaviors.scoreHigh());
//
// HOW AUTO USES THIS:
//   In AutoSequence.java (add a one-line wrapper):
//     public AutoSequence shoot() {
//         steps.add(RobotBehaviors.shoot());
//         return this;
//     }
//   Then in a routine:
//     new AutoSequence().shoot().driveFieldRelative(...).build();
//
// HOW TO ADD A NEW BEHAVIOR:
//   1. Identify which subsystems are involved.
//   2. Write a static method here that composes their individual commands.
//   3. Use Commands.sequence() for ordered steps.
//      Use Commands.parallel() for simultaneous actions.
//      Use Commands.waitUntil() to gate one action on another subsystem's state.
//   4. Add a wrapper in AutoSequence.java if it's needed in auto routines.
//   5. Bind it in Robot.java if it's a driver control.
//
// NAMING CONVENTION:
//   Name behaviors by what the robot DOES, not what the mechanism is.
//   GOOD:  shoot(), scoreHigh(), intakePiece(), ejectPiece()
//   AVOID: armHighAndIntakeForward(), deployArmRunRoller()
// ─────────────────────────────────────────────────────────────────────────────
public final class RobotBehaviors {

    // Prevent instantiation — this is a static utility class.
    private RobotBehaviors() {}

    // ─────────────────────────────────────────────────────────────────
    // SHOOTING BEHAVIORS
    // ─────────────────────────────────────────────────────────────────

    /**
     * Spins up the shooter and feeds a game piece once at speed.
     *
     * <p>How it works:
     * <ol>
     *   <li>The shooter motor spins up to {@code DEFAULT_RPM} immediately.</li>
     *   <li>The feeder waits (via {@link Commands#waitUntil}) until
     *       {@code shooter.isAtRPM()} returns true.</li>
     *   <li>Once the shooter is at speed, the feeder starts and feeds
     *       the piece into the flywheel.</li>
     *   <li>When the driver releases the trigger, both motors stop.</li>
     * </ol>
     *
     * <p>This shows the core multi-subsystem coordination pattern: neither
     * the Shooter nor the Feeder knows about each other — the behavior here
     * connects them using {@link Commands#parallel} and {@link Commands#waitUntil}.
     *
     * <p>Binding: {@code operator.rightTrigger().whileTrue(RobotBehaviors.shoot());}
     */
    public static Command shoot() {
        return Commands.parallel(
            // Shooter: spin up and hold speed for the full duration of the button hold
            AutoBuilder.shooter.spinUpCommand(DEFAULT_RPM),

            // Feeder: wait until shooter is ready, then feed
            Commands.sequence(
                Commands.waitUntil(AutoBuilder.shooter::isAtRPM),
                AutoBuilder.feeder.feedCommand()
            )
        ).withName("RobotBehaviors.shoot");
    }

    // ─────────────────────────────────────────────────────────────────
    // FAILSAFE
    // ─────────────────────────────────────────────────────────────────

    /**
     * Cancels every running command immediately.
     *
     * <p>Use when a mechanism appears stuck or unresponsive. All commands are
     * cancelled on the spot; subsystem default commands (e.g. the swerve drive)
     * are automatically rescheduled by the scheduler on the next cycle, so
     * driving is restored within one loop.
     *
     * <p>Works while disabled ({@code ignoringDisable(true)}) so operators can
     * clear a stuck state before enabling.
     *
     * <p>Binding: {@code driver.back().onTrue(RobotBehaviors.failsafe());}
     */
    public static Command failsafe() {
        return Commands.runOnce(CommandScheduler.getInstance()::cancelAll)
            .withName("Failsafe.cancelAll")
            .ignoringDisable(true);
    }

    // ─────────────────────────────────────────────────────────────────
    // UTILITY BEHAVIORS
    // ─────────────────────────────────────────────────────────────────

    /**
     * Runs the intake in reverse to eject a game piece.
     *
     * <p>This runs continuously while the button is held (use whileTrue):
     * {@code driver.x().whileTrue(RobotBehaviors.ejectPiece());}
     */
    public static Command ejectPiece() {
        return AutoBuilder.intake.reverseCommand()
            .withName("RobotBehaviors.ejectPiece");
    }

    /**
     * Stows all mechanisms to a safe travel position.
     *
     * <p>Call this at the end of a scoring sequence or when transitioning
     * across the field. Safe to bind on a single button press:
     * {@code driver.y().onTrue(RobotBehaviors.stow());}
     */
    public static Command stow() {
        return Commands.sequence(
            AutoBuilder.intake.stopCommand(),
            Commands.parallel(
                AutoBuilder.arm.retractCommand(),
                AutoBuilder.elevator.goToCommand(Setpoint.STOW)
            )
        ).withName("RobotBehaviors.stow");
    }
}
