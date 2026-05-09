//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.robot.auto.routines;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

import org.team1507.robot.auto.AutoSequence;

// ─────────────────────────────────────────────────────────────────────────────
// DriveForwardAuto
//
// The simplest possible auto routine — resets the pose and drives forward.
// Use this as a reference for how all routine files should look.
//
// To create a new routine:
//   1. Copy this file into the routines/ folder.
//   2. Rename the class and the build() method's steps.
//   3. Register it in Robot.java: autoChooser.addOption("My Auto", MyAuto.build());
// ─────────────────────────────────────────────────────────────────────────────
public final class DriveForwardAuto {

    // Prevent instantiation — call build() directly.
    private DriveForwardAuto() {}

    /**
     * Builds the DriveForward autonomous routine.
     *
     * Steps:
     *   1. Reset pose to field origin (0, 0, 0°).
     *   2. Face forward (0°), drive 2 m at full speed — tests APF deceleration.
     *   3. Turn left to 90°, drive 1 m at 3 m/s — tests decel at lower cruise speed.
     *   4. Turn to face back (180°), drive 2 m — returns toward start.
     *   5. Stop.
     */
    public static Command build() {
        return new AutoSequence()
            .resetPose(new Pose2d())
            .driveForwardMeters(5.0, 5.0, true)
            .stop()
            .build();
    }
}