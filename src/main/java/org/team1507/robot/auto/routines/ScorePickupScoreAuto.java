//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.robot.auto.routines;

import edu.wpi.first.wpilibj2.command.Command;

import org.team1507.robot.auto.AutoSequence;
import org.team1507.robot.auto.nodes.Nodes;

// ─────────────────────────────────────────────────────────────────────────────
// ScorePickupScoreAuto
//
// Routine overview:
//   1. Start at the right side and drive to the scoring position.
//   2. Shoot until the auto timer reaches 5 seconds.
//   3. Drive to the fuel pickup station (intake runs the whole way).
//   4. Return to the scoring position.
//   5. Shoot until 14.99 seconds — ends just before auto expires so the
//      command doesn't carry over into teleop.
//
// Leg distances and speed rationale:
//   Start.RIGHT → Score.RIGHT          : ~0.36 m — slow (short repositioning)
//   Score.RIGHT → Pickup.APPROACH_RIGHT: ~3.7 m  — full speed
//   Pickup.APPROACH_RIGHT → STATION    : ~1.0 m  — slow (final approach to wall)
//   Pickup.STATION_RIGHT  → Score.RIGHT: ~4.3 m  — full speed
// ─────────────────────────────────────────────────────────────────────────────
public final class ScorePickupScoreAuto {

    private ScorePickupScoreAuto() {}

    public static Command build() {
        return new AutoSequence()
            .startTimer()
            .resetPose(Nodes.Robot.Start.RIGHT)
            .slow().driveToPoint(Nodes.Robot.Score.RIGHT, true)

            // Phase 1 — shoot until the match clock hits 5 seconds
            .shootUntil(5.0)

            // Pickup leg — intake runs while driving to the station
            .deadline(
                seq -> seq.driveToPoint(Nodes.Robot.Pickup.APPROACH_RIGHT, true),
                seq -> seq.intakeRun()
            )
            .deadline(
                seq -> seq.creep().driveToPoint(Nodes.Robot.Pickup.STATION_RIGHT, true),
                seq -> seq.intakeRun()
            )

            // Return to score position
            .driveToPoint(Nodes.Robot.Score.RIGHT, true)

            // Phase 2 — shoot until 14.99 s so the command ends before auto expires
            .shootUntil(14.99)

            .build();
    }
}
