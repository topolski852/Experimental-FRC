//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import org.team1507.robot.auto.AutoBuilder;

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
//     driver.a().whileTrue(RobotBehaviors.scoreHigh());
//     driver.b().onTrue(RobotBehaviors.intakePiece());
//
// HOW AUTO USES THIS:
//   In AutoSequence.java (add a one-line wrapper):
//     public AutoSequence scoreHigh() {
//         steps.add(RobotBehaviors.scoreHigh());
//         return this;
//     }
//   Then in a routine:
//     new AutoSequence().scoreHigh().driveFieldRelative(...).build();
//
// HOW TO ADD A NEW BEHAVIOR:
//   1. Identify which subsystems are involved.
//   2. Write a static method here that composes their individual commands.
//   3. Use Commands.sequence() for ordered steps.
//      Use Commands.parallel() for simultaneous actions.
//      Use Commands.deadline() when one action sets the duration.
//   4. Add a wrapper in AutoSequence.java if it's needed in auto routines.
//   5. Bind it in Robot.java if it's a driver control.
//
// NAMING CONVENTION:
//   Name behaviors by what the robot DOES, not what the mechanism is.
//   GOOD:  scoreHigh(), intakePiece(), ejectPiece()
//   AVOID: armHighAndMotorForward(), deployArmRunMotor()
// ─────────────────────────────────────────────────────────────────────────────
public final class RobotBehaviors {

    // Prevent instantiation — this is a static utility class.
    private RobotBehaviors() {}

    // ─────────────────────────────────────────────────────────────────
    // EXAMPLE BEHAVIORS — replace these with your game-specific actions.
    //
    // These examples use the generic arm and motor from Robot.java.
    // Once you build real subsystems (intake, shooter, climber, etc.),
    // delete these examples and write new behaviors for your game.
    // ─────────────────────────────────────────────────────────────────

    /**
     * Scores a game piece at the high goal.
     *
     * <p>Step-by-step:
     * <ol>
     *   <li>Deploy the arm to the high scoring position.</li>
     *   <li>Once the arm is ready, run the motor forward to release the piece.</li>
     *   <li>After 0.5 seconds the motor stops and the arm stays up.</li>
     * </ol>
     *
     * <p>Use on a driver button: {@code driver.a().onTrue(RobotBehaviors.scoreHigh());}
     */
    public static Command scoreHigh() {
        return Commands.sequence(
            AutoBuilder.arm.deployCommand(),
            Commands.waitSeconds(0.1),          // brief pause for arm to settle
            AutoBuilder.basicMotor.runForwardCommand().withTimeout(0.5)
        ).withName("RobotBehaviors.scoreHigh");
    }

    /**
     * Scores a game piece at the mid goal (arm at retracted position).
     *
     * <p>Same as scoreHigh but with the arm in the retracted/mid position.
     *
     * <p>Use on a driver button: {@code driver.b().onTrue(RobotBehaviors.scoreMid());}
     */
    public static Command scoreMid() {
        return Commands.sequence(
            AutoBuilder.arm.retractCommand(),
            Commands.waitSeconds(0.1),
            AutoBuilder.basicMotor.runForwardCommand().withTimeout(0.5)
        ).withName("RobotBehaviors.scoreMid");
    }

    /**
     * Runs the intake motor in reverse to eject a game piece.
     *
     * <p>This runs continuously while the button is held (use whileTrue):
     * {@code driver.x().whileTrue(RobotBehaviors.ejectPiece());}
     */
    public static Command ejectPiece() {
        return AutoBuilder.basicMotor.runReverseCommand()
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
            AutoBuilder.basicMotor.stopCommand(),
            AutoBuilder.arm.retractCommand()
        ).withName("RobotBehaviors.stow");
    }
}
