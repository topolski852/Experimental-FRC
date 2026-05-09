//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╔╝██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.robot.auto.nodes;

import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

import org.junit.jupiter.api.Test;

// ─────────────────────────────────────────────────────────────────────────────
// NodeBoundsTest
//
// Validates that every node in Nodes.java is within the active field boundary.
// This test runs as part of `./gradlew build` — if any node is out of bounds,
// the build fails and the code cannot be deployed to the robot.
//
// This is the equivalent of a compile error for coordinates: you cannot run
// the robot with a node that falls outside the field (or field room).
//
// SWITCHING ENVIRONMENTS:
//   Set Nodes.Field.PRACTICE_MODE = true before building at the field room.
//   The test will then validate against PRACTICE_LENGTH / PRACTICE_WIDTH.
//   Set it back to false before a competition build.
//
// ADDING NEW NODES:
//   When you add a constant to Nodes.java, add a matching check() call here.
//   The test name ("Robot.Score.RIGHT") is what appears in the failure message,
//   so copy the full path exactly — it makes tracking down the bad node fast.
// ─────────────────────────────────────────────────────────────────────────────
class NodeBoundsTest {

    private static final double MAX_X = Nodes.Field.PRACTICE_MODE
        ? Nodes.Field.PRACTICE_LENGTH
        : Nodes.Field.LENGTH;

    private static final double MAX_Y = Nodes.Field.PRACTICE_MODE
        ? Nodes.Field.PRACTICE_WIDTH
        : Nodes.Field.WIDTH;

    @Test
    void allNodesWithinBounds() {

        // ── Robot.Start ───────────────────────────────────────────────────
        check("Robot.Start.RIGHT",  Nodes.Robot.Start.RIGHT);
        check("Robot.Start.CENTER", Nodes.Robot.Start.CENTER);
        check("Robot.Start.LEFT",   Nodes.Robot.Start.LEFT);

        // ── Robot.Score ───────────────────────────────────────────────────
        check("Robot.Score.RIGHT", Nodes.Robot.Score.RIGHT);
        check("Robot.Score.LEFT",  Nodes.Robot.Score.LEFT);

        // ── Robot.Pickup ──────────────────────────────────────────────────
        check("Robot.Pickup.APPROACH_RIGHT", Nodes.Robot.Pickup.APPROACH_RIGHT);
        check("Robot.Pickup.STATION_RIGHT",  Nodes.Robot.Pickup.STATION_RIGHT);
        check("Robot.Pickup.APPROACH_LEFT",  Nodes.Robot.Pickup.APPROACH_LEFT);
        check("Robot.Pickup.STATION_LEFT",   Nodes.Robot.Pickup.STATION_LEFT);

        // ── Robot.Waypoint ────────────────────────────────────────────────
        check("Robot.Waypoint.MIDFIELD_RIGHT",  Nodes.Robot.Waypoint.MIDFIELD_RIGHT);
        check("Robot.Waypoint.MIDFIELD_CENTER", Nodes.Robot.Waypoint.MIDFIELD_CENTER);
        check("Robot.Waypoint.MIDFIELD_LEFT",   Nodes.Robot.Waypoint.MIDFIELD_LEFT);

        // ── FieldElements ─────────────────────────────────────────────────
        // Add checks here when FieldElements is populated after game reveal.
    }

    // ─────────────────────────────────────────────────────────────────────
    // Helpers
    // ─────────────────────────────────────────────────────────────────────

    private void check(String name, Pose2d pose) {
        check(name, pose.getX(), pose.getY());
    }

    private void check(String name, Translation2d point) {
        check(name, point.getX(), point.getY());
    }

    private void check(String name, double x, double y) {
        String bounds = String.format("%.2f m × %.2f m", MAX_X, MAX_Y);
        assertTrue(x >= 0,
            name + " — x=" + x + " is negative (outside field)");
        assertTrue(x <= MAX_X,
            name + " — x=" + x + " exceeds field length " + MAX_X + " (" + bounds + ")");
        assertTrue(y >= 0,
            name + " — y=" + y + " is negative (outside field)");
        assertTrue(y <= MAX_Y,
            name + " — y=" + y + " exceeds field width " + MAX_Y + " (" + bounds + ")");
    }
}
