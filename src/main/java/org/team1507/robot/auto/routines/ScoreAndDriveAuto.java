//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╔╝██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.robot.auto.routines;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

import org.team1507.robot.auto.AutoSequence;
import org.team1507.robot.RobotBehaviors;

// ─────────────────────────────────────────────────────────────────────────────
// ScoreAndDriveAuto
//
// A more complete example routine demonstrating:
//   - Speed modifiers (.slow(), .creep(), .withSpeed())
//   - .parallel() with multi-step branches
//   - .race() for time-limited actions
//   - .deadline() for background actions
//   - RobotBehaviors for multi-subsystem actions
//   - The auto timer for match-time-gated steps
//
// Read through this file to learn every feature of the AutoSequence builder.
// ─────────────────────────────────────────────────────────────────────────────
public final class ScoreAndDriveAuto {

    private ScoreAndDriveAuto() {}

    /**
     * Builds the ScoreAndDrive autonomous routine.
     *
     * Steps:
     *   1. Start the match timer (so we can use timer-gated steps later).
     *   2. Reset pose to starting position.
     *   3. Score a game piece at HIGH using RobotBehaviors.
     *   4. Back away slowly while stowing the arm at the same time (parallel).
     *   5. Drive to a second position at full speed.
     *   6. Run the motor while driving to the next waypoint (deadline group).
     *   7. Stop and wait for the match clock to hit 5 seconds.
     */
    public static Command build() {
        return new AutoSequence()

            // ── Start timer so timer-gated steps work ─────────────────────
            .startTimer()

            // ── Reset pose to starting position on the field ───────────────
            // Change this Pose2d to match where the robot actually starts.
            .resetPose(new Pose2d(1.5, 5.5, Rotation2d.fromDegrees(180)))

            // ── Score high using a coordinated multi-subsystem behavior ─────
            // This calls RobotBehaviors.scoreHigh(), which:
            //   moves the arm to HIGH, runs the motor, waits, stops, stows arm.
            // The exact sequence lives in RobotBehaviors.java — not here.
            .addCommand(RobotBehaviors.scoreHigh())

            // ── Back away slowly while stowing the arm at the same time ─────
            // PARALLEL: both branches run at the same time.
            //           ends when BOTH are done.
            // Note how .slow() is placed inside the branch, right before the
            // drive command. This is the correct pattern — speed modifiers
            // always go immediately before the motion step they apply to.
            .parallel(
                seq -> seq.slow().driveFieldRelative(-1.0, 0.0, 0.0, 1.0),  // back away
                seq -> seq.armStow()                                           // stow arm
            )

            // ── Drive forward at full speed ────────────────────────────────
            .driveFieldRelative(3.0, 0.0, 0.0, 2.0)

            // ── Run motor while driving to next position ───────────────────
            // DEADLINE: the first branch sets the duration.
            //           the drive command ends after 2 seconds,
            //           and the motor stops when the drive ends.
            // Use deadline when you want one action to time-box everything else.
            .deadline(
                seq -> seq.driveFieldRelative(2.0, 0.0, 0.0, 2.0),  // deadline: 2 sec drive
                seq -> seq.motorForward()                              // runs while driving
            )

            // ── Stop the motor after the deadline group ends ───────────────
            .motorStop()

            // ── Wait until the match timer reaches 5 seconds ──────────────
            // Useful when your routine ends early and you want to delay
            // before moving to the next phase — or to gate shooting windows.
            .waitUntilTime(5.0)

            // ── Example of .race(): drive OR wait, whichever ends first ───
            // RACE: all branches run, ends when the FIRST one finishes.
            // Here, the robot drives for up to 3 seconds but will stop
            // early if the 1.5 second wait finishes first.
            .race(
                seq -> seq.driveFieldRelative(2.0, 0.0, 0.0, 3.0),
                seq -> seq.waitSeconds(1.5)
            )

            // ── Stop and hold position ─────────────────────────────────────
            .stop()

            // ── IMPORTANT: always call .build() last ──────────────────────
            .build();
    }
}