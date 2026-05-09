//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.robot.auto;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import org.team1507.robot.subsystems.ArmSystem.Position;

// ─────────────────────────────────────────────────────────────────────────────
// AutoSequence
//
// Fluent builder for constructing autonomous routines. Each method appends a
// Command to an internal list, and build() assembles them into a single
// sequential Command that runs step by step.
//
// BASIC USAGE:
//   new AutoSequence()
//       .resetPose(new Pose2d())
//       .driveFieldRelative(2.0, 0.0, 0.0, 1.5)
//       .stop()
//       .build();
//
// SPEED MODIFIERS:
//   Speed modifiers must appear immediately before a motion command.
//   They apply to that one step only and then reset automatically.
//
//   .withSpeed(2.0).driveFieldRelative(2.0, 0.0, 0.0, 1.5)
//   .slow().driveFieldRelative(1.0, 0.0, 0.0, 1.0)
//   .creep().driveFieldRelative(0.5, 0.0, 0.0, 0.5)
//
// GROUPS (parallel / race / deadline):
//   Each branch inside a group is its own mini-sequence, written as a lambda:
//       seq -> seq.step1().step2()
//
//   The "seq" is a fresh AutoSequence for that branch. You can chain as many
//   steps as needed inside one branch:
//       seq -> seq.slow().driveFieldRelative(1.0, 0.0, 0.0, 1.0).stop()
//
//   Speed modifiers work normally inside branches — they only affect the step
//   immediately after them inside that branch.
//
//   Examples:
//     .parallel(
//         seq -> seq.armHigh(),
//         seq -> seq.motorForward()
//     )
//     .race(
//         seq -> seq.driveFieldRelative(2.0, 0.0, 0.0, 3.0),
//         seq -> seq.waitSeconds(2.0)
//     )
//     .deadline(
//         seq -> seq.driveFieldRelative(2.0, 0.0, 0.0, 3.0),  // <-- deadline branch
//         seq -> seq.motorForward()                             // <-- runs until deadline ends
//     )
//
// HOW TO ADD NEW AUTO STEPS:
//   1. Add your command to AutoBuilder.java (or RobotBehaviors.java if multi-subsystem).
//   2. Add a one-line wrapper method here following the pattern of existing methods.
//   3. Use it in your routine file.
// ─────────────────────────────────────────────────────────────────────────────
public final class AutoSequence {

    // -------------------------------------------------------------------------
    // Core Fields
    // -------------------------------------------------------------------------

    private final List<Command> steps = new ArrayList<>();

    // Speed state — consumed by the next motion command, then reset.
    // Always set with .withSpeed(), .slow(), or .creep() immediately before
    // the motion step you want it to apply to.
    private Double nextSpeedOverride   = null;
    private Double nextAngularOverride = null;

    // Autonomous match timer — started with .startTimer(), read by .shootUntil() etc.
    private final Timer autoTimer = new Timer();


    // -------------------------------------------------------------------------
    // Constructor
    // -------------------------------------------------------------------------

    /**
     * Creates a new AutoSequence builder.
     * No arguments needed — all subsystems are accessed through AutoBuilder.
     */
    public AutoSequence() {}


    // =========================================================================
    // SPEED MODIFIERS
    //
    // These do NOT add a step. They set a temporary override that the NEXT
    // motion command will consume. After that command runs, the override resets.
    //
    // Rule: always place a speed modifier immediately before a motion command.
    //   CORRECT:   .slow().driveFieldRelative(1.0, 0.0, 0.0, 1.5)
    //   INCORRECT: .slow().armHigh().driveFieldRelative(...)  ← slow is wasted on armHigh
    // =========================================================================

    /**
     * Applies a custom translational speed to the next motion step (m/s).
     * Angular rate uses the default.
     */
    public AutoSequence withSpeed(double speedMetersPerSec) {
        this.nextSpeedOverride   = speedMetersPerSec;
        this.nextAngularOverride = AutoBuilder.swerve.getMaxAngular();
        return this;
    }

    /**
     * Applies a custom translational AND angular speed to the next motion step.
     */
    public AutoSequence withSpeed(double speedMetersPerSec, double angularRadPerSec) {
        this.nextSpeedOverride   = speedMetersPerSec;
        this.nextAngularOverride = angularRadPerSec;
        return this;
    }

    /**
     * Moderately slow movement — 50% speed, 75% angular rate.
     * Good for approach paths where precision matters.
     */
    public AutoSequence slow() {
        this.nextSpeedOverride   = AutoBuilder.swerve.getMaxSpeed() * 0.5;
        this.nextAngularOverride = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
        return this;
    }

    /**
     * Very slow, precise movement — 30% speed, 50% angular rate.
     * Good for final alignment steps or tight corridor navigation.
     */
    public AutoSequence creep() {
        this.nextSpeedOverride   = AutoBuilder.swerve.getMaxSpeed() * 0.3;
        this.nextAngularOverride = RotationsPerSecond.of(0.50).in(RadiansPerSecond);
        return this;
    }

    // Internal helper — reads and clears the speed override for one step.
    private double consumeSpeed() {
        double speed = (nextSpeedOverride != null) ? nextSpeedOverride : AutoBuilder.swerve.getMaxSpeed();
        nextSpeedOverride   = null;
        nextAngularOverride = null;
        return speed;
    }

    private double consumeAngular() {
        double angular = (nextAngularOverride != null) ? nextAngularOverride : AutoBuilder.swerve.getMaxAngular();
        // Note: nextAngularOverride is already cleared by consumeSpeed().
        // If using consumeAngular() independently, clear here too:
        nextAngularOverride = null;
        return angular;
    }


    // =========================================================================
    // DRIVE COMMANDS
    // =========================================================================

    /**
     * Resets the robot's field pose to the given Pose2d.
     * Always call this as the first step in an auto routine.
     */
    public AutoSequence resetPose(Pose2d pose) {
        steps.add(AutoBuilder.swerve.resetPoseCommand(pose));
        return this;
    }

    public AutoSequence driveForwardMeters(double distanceMeters, double velocity, boolean stopAtEnd) {
        steps.add(AutoBuilder.swerve.driveForwardMeters(distanceMeters, velocity, stopAtEnd));
        return this;
    }

    public AutoSequence driveToPoint(Pose2d target, double velocity, boolean stopAtEnd) {
        steps.add(AutoBuilder.swerve.driveToPoint(target, velocity, stopAtEnd));
        return this;
    }

    public AutoSequence changeHeading(double headingDeg) {
        steps.add(AutoBuilder.swerve.changeHeading(headingDeg));
        return this;
    }

    /** Stops all swerve modules. */
    public AutoSequence stop() {
        steps.add(AutoBuilder.swerve.stopCommand());
        return this;
    }


    // =========================================================================
    // ARM COMMANDS
    //
    // Positional mechanism — deploy / retract / goTo vocabulary.
    // =========================================================================

    /** Moves the arm to the HIGH position and waits until it arrives. */
    public AutoSequence armHigh() {
        steps.add(AutoBuilder.arm.goToCommand(Position.HIGH));
        return this;
    }

    /** Moves the arm to the MID position and waits until it arrives. */
    public AutoSequence armMid() {
        steps.add(AutoBuilder.arm.goToCommand(Position.MID));
        return this;
    }

    /** Returns the arm to STOW and waits until it arrives. */
    public AutoSequence armStow() {
        steps.add(AutoBuilder.arm.goToCommand(Position.STOW));
        return this;
    }


    // =========================================================================
    // INTAKE COMMANDS
    //
    // Roller intake — run / reverse / stop vocabulary.
    // =========================================================================

    /** Runs the intake roller forward. */
    public AutoSequence intakeRun() {
        steps.add(AutoBuilder.intake.runCommand());
        return this;
    }

    /** Runs the intake roller in reverse. */
    public AutoSequence intakeReverse() {
        steps.add(AutoBuilder.intake.reverseCommand());
        return this;
    }

    /** Stops the intake. */
    public AutoSequence intakeStop() {
        steps.add(AutoBuilder.intake.stopCommand());
        return this;
    }


    // =========================================================================
    // ROBOT BEHAVIORS
    //
    // Multi-subsystem coordinated actions defined in RobotBehaviors.java.
    // These are the same commands used for teleop button bindings.
    // Add a one-line wrapper here for each behavior you want available in auto.
    //
    // Example (once RobotBehaviors has the method):
    //   public AutoSequence scoreHigh() {
    //       steps.add(RobotBehaviors.scoreHigh());
    //       return this;
    //   }
    // =========================================================================


    // =========================================================================
    // ADD NEW SUBSYSTEM STEPS BELOW THIS LINE (each year)
    //
    // Pattern for a free-spinner subsystem (e.g. Shooter):
    //   public AutoSequence shooterSpin() {
    //       steps.add(AutoBuilder.shooterSpin());
    //       return this;
    //   }
    //   public AutoSequence shooterStop() {
    //       steps.add(AutoBuilder.shooterStop());
    //       return this;
    //   }
    //
    // Pattern for a positional subsystem (e.g. Elevator):
    //   public AutoSequence elevatorHigh() {
    //       steps.add(AutoBuilder.elevatorHigh());
    //       return this;
    //   }
    //   public AutoSequence elevatorStow() {
    //       steps.add(AutoBuilder.elevatorStow());
    //       return this;
    //   }
    // =========================================================================


    // =========================================================================
    // GROUPS: PARALLEL / RACE / DEADLINE
    //
    // Each group takes one or more Branch lambdas. A Branch is a lambda that
    // receives a fresh AutoSequence and adds steps to it:
    //
    //     seq -> seq.step1().step2()
    //
    // "seq" is a new AutoSequence for that branch — it is NOT the outer sequence.
    // You can chain as many steps as you want inside a branch.
    // Speed modifiers work normally inside branches.
    //
    // PARALLEL  — all branches run at the same time, ends when ALL are done.
    // RACE      — all branches run at the same time, ends when the FIRST finishes.
    // DEADLINE  — all branches run at the same time, ends when the FIRST ARGUMENT
    //             (the deadline branch) finishes. All others are cancelled.
    // =========================================================================

    /**
     * Runs all branches simultaneously. Ends when ALL branches finish.
     *
     * Example:
     *   .parallel(
     *       seq -> seq.armHigh(),
     *       seq -> seq.motorForward()
     *   )
     */
    public AutoSequence parallel(Branch... branches) {
        List<Command> commands = new ArrayList<>();
        for (Branch branch : branches) {
            AutoSequence sub = new AutoSequence();
            branch.build(sub);
            commands.add(sub.build());
        }
        steps.add(Commands.parallel(commands.toArray(Command[]::new)));
        return this;
    }

    /**
     * Runs all branches simultaneously. Ends when the FIRST branch finishes,
     * cancelling all others.
     *
     * Example:
     *   .race(
     *       seq -> seq.driveFieldRelative(2.0, 0.0, 0.0, 5.0),
     *       seq -> seq.waitSeconds(2.0)
     *   )
     *   // Robot drives for at most 5 seconds, but stops after 2 seconds
     *   // because the waitSeconds branch finishes first.
     */
    public AutoSequence race(Branch... branches) {
        List<Command> commands = new ArrayList<>();
        for (Branch branch : branches) {
            AutoSequence sub = new AutoSequence();
            branch.build(sub);
            commands.add(sub.build());
        }
        steps.add(Commands.race(commands.toArray(Command[]::new)));
        return this;
    }

    /**
     * Runs all branches simultaneously. The FIRST branch is the deadline —
     * when it finishes, all other branches are cancelled.
     *
     * Use this when you want one action to set the duration and everything
     * else runs alongside it.
     *
     * Example:
     *   .deadline(
     *       seq -> seq.driveFieldRelative(2.0, 0.0, 0.0, 3.0),  // drives for 3 sec (deadline)
     *       seq -> seq.motorForward()                             // motor runs while driving
     *   )
     */
    public AutoSequence deadline(Branch deadlineBranch, Branch... others) {
        AutoSequence deadlineSeq = new AutoSequence();
        deadlineBranch.build(deadlineSeq);

        List<Command> otherCommands = new ArrayList<>();
        for (Branch branch : others) {
            AutoSequence sub = new AutoSequence();
            branch.build(sub);
            otherCommands.add(sub.build());
        }

        steps.add(Commands.deadline(
            deadlineSeq.build(),
            otherCommands.toArray(Command[]::new)
        ));
        return this;
    }


    // =========================================================================
    // TIMER UTILITIES
    //
    // The auto timer lets you gate actions on match time rather than duration.
    // Always call .startTimer() as your first step if you plan to use
    // .shootUntil(), .waitUntilTime(), or .endAtTime().
    // =========================================================================

    /**
     * Starts the autonomous match timer.
     * Call this as the very first step in any routine that uses timer-gated steps.
     */
    public AutoSequence startTimer() {
        steps.add(Commands.runOnce(() -> {
            autoTimer.reset();
            autoTimer.start();
        }));
        return this;
    }

    /**
     * Waits until the auto timer reaches a specific match time (in seconds).
     * Useful for precisely aligning actions to the match clock.
     */
    public AutoSequence waitUntilTime(double matchTimeSeconds) {
        steps.add(Commands.waitUntil(() -> autoTimer.get() >= matchTimeSeconds));
        return this;
    }

    /**
     * Returns a Command that completes when the timer reaches a given time.
     * Use this as the deadline branch in a .race() or .deadline() group.
     *
     * Example:
     *   .race(
     *       seq -> seq.addCommand(endAtTime(13.5)),
     *       seq -> seq.motorForward()
     *   )
     */
    public Command endAtTime(double matchTimeSeconds) {
        return Commands.waitUntil(() -> autoTimer.get() >= matchTimeSeconds);
    }


    // =========================================================================
    // UTILITY
    // =========================================================================

    /** Waits a fixed number of seconds before proceeding to the next step. */
    public AutoSequence waitSeconds(double seconds) {
        steps.add(Commands.waitSeconds(seconds));
        return this;
    }

    /**
     * Waits until a condition becomes true before proceeding.
     * Use with method references: .waitUntil(arm::isAtTarget)
     */
    public AutoSequence waitUntil(java.util.function.BooleanSupplier condition) {
        steps.add(Commands.waitUntil(condition));
        return this;
    }

    /**
     * Adds any Command directly into the sequence.
     * Use this to insert commands from RobotBehaviors or subsystems
     * that don't yet have a named wrapper method here.
     *
     * Example:
     *   .addCommand(RobotBehaviors.scoreHigh())
     *   .addCommand(AutoBuilder.waitSeconds(0.5))
     */
    public AutoSequence addCommand(Command command) {
        steps.add(command);
        return this;
    }


    // =========================================================================
    // BUILD
    // =========================================================================

    /**
     * Builds the final autonomous Command.
     * Call this at the END of every routine's build() method.
     */
    public Command build() {
        return Commands.sequence(steps.toArray(Command[]::new));
    }


    // =========================================================================
    // BRANCH INTERFACE
    //
    // A Branch is a lambda that configures a sub-AutoSequence for use inside
    // .parallel(), .race(), or .deadline().
    //
    // Usage:  seq -> seq.step1().step2()
    //
    // "seq" is automatically created — you just chain steps on it.
    // The result is compiled into a sequential Command for that branch.
    // =========================================================================

    /**
     * Functional interface for defining a branch inside a group.
     *
     * Written as a lambda:  seq -> seq.step1().step2()
     *
     * This is the same as Java's standard functional interface pattern —
     * the lambda receives one argument (seq) and calls methods on it.
     */
    @FunctionalInterface
    public interface Branch {
        void build(AutoSequence seq);
    }
}