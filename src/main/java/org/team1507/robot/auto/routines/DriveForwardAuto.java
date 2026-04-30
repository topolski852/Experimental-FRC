package org.team1507.robot.auto.routines;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import org.team1507.robot.auto.AutoSequence;
import org.team1507.robot.subsystems.Swerve;

public final class DriveForwardAuto {

    public static Command build(Swerve swerve) {
        return new AutoSequence(swerve)
            .resetPose(new Pose2d())
            .driveForTime(new ChassisSpeeds(1.0, 0.0, 0.0), 1.0)
            .stop()
            .build();
    }
}
