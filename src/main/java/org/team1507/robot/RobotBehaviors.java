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

    // RobotBehaviors accesses subsystems through AutoBuilder's static registry.
    // AutoBuilder.init() must be called in Robot.java before any behavior runs.


    // =========================================================================
    // EXAMPLE BEHAVIORS
    //
    // These demonstrate the patterns students should follow.
    // Replace or expand these each season based on the actual game.
    // =========================================================================

    /**
     * Scores at the HIGH position.
     *
     * Sequence:
     *   1. Move arm to HIGH position (waits until arm arrives).
     *   2. Run the motor forward to score.
     *   3. Wait 0.5 seconds for game piece to eject.
     *   4. Stop the motor.
     *   5. Return arm to STOW.
     *
     * Used in: teleop driver button + auto routines via AutoSequence.scoreHigh()
     */
    public static Command scoreHigh() {
        return Commands.sequence(
            AutoBuilder.armHigh(),
            AutoBuilder.motorForward(),
            Commands.waitSeconds(0.5),
            AutoBuilder.motorStop(),
            AutoBuilder.armStow()
        );
    }

    /**
     * Scores at the MID position.
     *
     * Sequence:
     *   1. Move arm to MID position.
     *   2. Run motor forward to score.
     *   3. Wait for ejection.
     *   4. Stop motor, return arm to STOW.
     */
    public static Command scoreMid() {
        return Commands.sequence(
            AutoBuilder.armMid(),
            AutoBuilder.motorForward(),
            Commands.waitSeconds(0.5),
            AutoBuilder.motorStop(),
            AutoBuilder.armStow()
        );
    }

    /**
     * Ejects a game piece quickly (e.g. to clear a jam or barf a piece).
     *
     * Runs motor in reverse while arm stays in current position.
     * Intended to be bound as a whileTrue() button — motor stops when button is released.
     */
    public static Command ejectPiece() {
        return AutoBuilder.motorReverse();
    }


    // =========================================================================
    // ADD NEW BEHAVIORS BELOW THIS LINE (each year)
    //
    // Pattern — sequential behavior (one thing after another):
    //   public static Command myBehavior() {
    //       return Commands.sequence(
    //           AutoBuilder.step1(),
    //           AutoBuilder.step2(),
    //           Commands.waitSeconds(0.3),
    //           AutoBuilder.step3()
    //       );
    //   }
    //
    // Pattern — parallel behavior (things happening at the same time):
    //   public static Command myBehavior() {
    //       return Commands.parallel(
    //           AutoBuilder.subsystem1Action(),
    //           AutoBuilder.subsystem2Action()
    //       );
    //   }
    //
    // Pattern — deadline behavior (one action sets the duration):
    //   public static Command myBehavior() {
    //       return Commands.deadline(
    //           AutoBuilder.primaryAction(),     // this one determines duration
    //           AutoBuilder.backgroundAction()   // this runs until primary ends
    //       );
    //   }
    //
    // Pattern — conditional behavior (different actions based on sensor state):
    //   public static Command myBehavior() {
    //       return Commands.either(
    //           AutoBuilder.actionIfTrue(),
    //           AutoBuilder.actionIfFalse(),
    //           () -> SomeSensor.isTriggered()
    //       );
    //   }
    //
    // Real examples from past seasons:
    //
    //   Intake + hopper coordination (2025 style):
    //     public static Command intakePiece() {
    //         return Commands.sequence(
    //             AutoBuilder.hopperExtend(),
    //             Commands.waitUntil(() -> hopper.isSafeForIntake()),
    //             Commands.parallel(
    //                 AutoBuilder.intakeArmDown(),
    //                 AutoBuilder.intakeRollerRun()
    //             )
    //         );
    //     }
    //
    //   Shooter ramp-up then feed (controller pattern):
    //     public static Command shoot() {
    //         return Commands.sequence(
    //             AutoBuilder.shooterSpin(),
    //             Commands.waitUntil(() -> shooter.isAtSpeed()),
    //             AutoBuilder.feederRun(),
    //             Commands.waitUntil(() -> shooter.hasFired()),
    //             AutoBuilder.feederStop(),
    //             AutoBuilder.shooterStop()
    //         );
    //     }
    // =========================================================================
}