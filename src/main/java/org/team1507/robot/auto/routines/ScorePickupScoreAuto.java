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
// Five-node routine: start right → score → pickup → score.
//
// Leg distances and speed rationale:
//   Start.RIGHT → Score.RIGHT          : ~0.36 m — slow (short repositioning)
//   Score.RIGHT → Pickup.APPROACH_RIGHT: ~3.7 m — full speed
//   Pickup.APPROACH_RIGHT → STATION    : ~1.0 m — slow (final approach to wall)
//   Pickup.STATION_RIGHT  → Score.RIGHT: ~4.3 m — full speed
// ─────────────────────────────────────────────────────────────────────────────
public final class ScorePickupScoreAuto {

    private ScorePickupScoreAuto() {}

    public static Command build() {
        return new AutoSequence()
            .resetPose(Nodes.Robot.Start.RIGHT)
            .driveToPoint(Nodes.Robot.Score.RIGHT,           1.5, true)
            .driveToPoint(Nodes.Robot.Pickup.APPROACH_RIGHT, 5.0, true)
            .driveToPoint(Nodes.Robot.Pickup.STATION_RIGHT,  1.0, true)
            .driveToPoint(Nodes.Robot.Score.RIGHT,           5.0, true)
            .stop()
            .build();
    }
}
