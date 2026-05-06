//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.robot.auto;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import org.team1507.robot.subsystems.Swerve;

public final class AutoSequence {

    private final Swerve swerve;
    private final List<Command> steps = new ArrayList<>();

    public AutoSequence(Swerve swerve) {
        this.swerve = swerve;
    }

    // ------------------------------------------------------------
    // Swerve motion primitives
    // ------------------------------------------------------------

    public AutoSequence resetPose(Pose2d pose) {
        steps.add(swerve.resetPoseCommand(pose));
        return this;
    }

    public AutoSequence driveForTime(
        ChassisSpeeds speeds,
        double seconds
    ) {
        steps.add(swerve.driveForTime(speeds, seconds));
        return this;
    }

    public AutoSequence stop() {
        steps.add(swerve.stopCommand());
        return this;
    }

    // ------------------------------------------------------------
    // Utility
    // ------------------------------------------------------------

    public AutoSequence waitSeconds(double seconds) {
        steps.add(Commands.waitSeconds(seconds));
        return this;
    }

    public Command build() {
        return Commands.sequence(steps.toArray(Command[]::new));
    }
}
