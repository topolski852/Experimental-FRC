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
}