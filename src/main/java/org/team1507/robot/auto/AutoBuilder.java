//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import org.team1507.robot.subsystems.Swerve;
import org.team1507.robot.subsystems.ArmSystem;
import org.team1507.robot.subsystems.BasicMotor;

// ─────────────────────────────────────────────────────────────────────────────
// AutoBuilder
//
// Static registry that holds all subsystem references and exposes them as
// zero-argument command factories. Initialized ONCE from Robot.java after
// all subsystems are created.
//
// HOW TO ADD A NEW SUBSYSTEM EACH YEAR:
//   1. Import your subsystem at the top of this file.
//   2. Add a private static field for it below the existing fields.
//   3. Add it as a parameter to init() and assign it.
//   4. Add your command factory methods in the appropriate section below.
//   5. Call AutoBuilder.init(...) in Robot.java with the new subsystem.
//
// Students should NEVER instantiate this class. Always call AutoBuilder.method().
// ─────────────────────────────────────────────────────────────────────────────
public final class AutoBuilder {

    // -------------------------------------------------------------------------
    // Subsystem Registry
    // -------------------------------------------------------------------------

    private static Swerve swerve;
    private static ArmSystem arm;
    private static BasicMotor basicMotor;

    // ADD NEW SUBSYSTEMS HERE each year:
    // private static Shooter shooter;
    // private static Intake intake;
    // private static Elevator elevator;

    // -------------------------------------------------------------------------
    // Speed Constants
    //
    // These are the default speeds used by motion commands. Override them
    // per-step in AutoSequence using .withSpeed(), .slow(), or .creep().
    // -------------------------------------------------------------------------

    static double MAX_SPEED          = 4.0;   // meters per second
    static double MAX_ANGULAR_RATE   = Math.PI; // radians per second

    // -------------------------------------------------------------------------
    // Initialization
    //
    // Called ONCE from Robot.java constructor, after all subsystems are built.
    // Add new subsystem parameters here as the robot grows each year.
    // -------------------------------------------------------------------------

    public static void init(
        Swerve swerve,
        ArmSystem arm,
        BasicMotor basicMotor
        // Add new subsystems here each year:
        // Shooter shooter,
        // Intake intake,
    ) {
        AutoBuilder.swerve      = swerve;
        AutoBuilder.arm         = arm;
        AutoBuilder.basicMotor  = basicMotor;
        // AutoBuilder.shooter  = shooter;
        // AutoBuilder.intake   = intake;
    }

    // Prevent instantiation — this is a static utility class.
    private AutoBuilder() {}


    // =========================================================================
    // DRIVE COMMANDS
    //
    // Core swerve motion primitives. Speed is controlled by the AutoSequence
    // speed modifiers (.withSpeed(), .slow(), .creep()) — these methods accept
    // speed as a parameter so AutoSequence can inject the override.
    // =========================================================================

    /** Drives with the given ChassisSpeeds for a fixed number of seconds. */
    public static Command driveForTime(ChassisSpeeds speeds, double seconds) {
        return swerve.driveForTime(speeds, seconds);
    }

    /**
     * Drives field-relative at a given x/y/rotation speed for a fixed time.
     *
     * @param xMetersPerSec  Forward speed (positive = away from driver wall).
     * @param yMetersPerSec  Strafe speed (positive = left).
     * @param rotRadPerSec   Rotation speed in radians/sec.
     * @param seconds        Duration.
     */
    public static Command driveFieldRelative(
        double xMetersPerSec,
        double yMetersPerSec,
        double rotRadPerSec,
        double seconds
    ) {
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            xMetersPerSec, yMetersPerSec, rotRadPerSec, swerve.getHeading()
        );
        return swerve.driveForTime(speeds, seconds);
    }

    /** Resets the robot's pose estimate to the given field position. */
    public static Command resetPose(Pose2d pose) {
        return swerve.resetPoseCommand(pose);
    }

    /** Stops all swerve modules immediately. */
    public static Command stop() {
        return swerve.stopCommand();
    }


    // =========================================================================
    // ARM COMMANDS
    //
    // Positional mechanism — uses deploy / retract / goTo vocabulary.
    // =========================================================================

    /** Deploys the arm to the HIGH position. */
    public static Command armHigh() {
        return arm.goToCommand(ArmSystem.Position.HIGH);
    }

    /** Deploys the arm to the MID position. */
    public static Command armMid() {
        return arm.goToCommand(ArmSystem.Position.MID);
    }

    /** Returns the arm to the STOW position. */
    public static Command armStow() {
        return arm.goToCommand(ArmSystem.Position.STOW);
    }


    // =========================================================================
    // BASIC MOTOR COMMANDS
    //
    // Free-spinning mechanism — uses runForward / runReverse / stop vocabulary.
    // =========================================================================

    /** Runs the basic motor forward. */
    public static Command motorForward() {
        return basicMotor.runForwardCommand();
    }

    /** Runs the basic motor in reverse. */
    public static Command motorReverse() {
        return basicMotor.runReverseCommand();
    }

    /** Stops the basic motor. */
    public static Command motorStop() {
        return basicMotor.stopCommand();
    }


    // =========================================================================
    // ADD NEW SUBSYSTEM COMMANDS BELOW THIS LINE (each year)
    //
    // Follow the pattern above: one section per subsystem, clearly labeled.
    // Use the naming convention:
    //   Free-spinners  → runForward(), runReverse(), stop()
    //   Positional     → goTo(Position), deploy(), retract()
    //   Behaviors      → use RobotBehaviors.java for multi-subsystem actions
    //
    // Example (Shooter):
    //   public static Command shooterSpin() { return shooter.runForwardCommand(); }
    //   public static Command shooterStop()  { return shooter.stopCommand(); }
    //
    // Example (Elevator):
    //   public static Command elevatorHigh() { return elevator.goToCommand(Elevator.Position.HIGH); }
    //   public static Command elevatorStow() { return elevator.goToCommand(Elevator.Position.STOW); }
    // =========================================================================


    // =========================================================================
    // UTILITY
    // =========================================================================

    /** Waits a fixed number of seconds. */
    public static Command waitSeconds(double seconds) {
        return Commands.waitSeconds(seconds);
    }

    /**
     * Waits until a condition becomes true.
     * Useful for sensor-gated steps: AutoBuilder.waitUntil(arm::isAtTarget)
     */
    public static Command waitUntil(java.util.function.BooleanSupplier condition) {
        return Commands.waitUntil(condition);
    }

    /**
     * Runs a one-shot action with no subsystem requirement.
     * Useful for logging, resetting state, or triggering flags mid-auto.
     */
    public static Command runOnce(Runnable action) {
        return Commands.runOnce(action);
    }
}