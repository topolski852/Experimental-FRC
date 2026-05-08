//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.robot.auto;

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

    public static Swerve swerve;
    public static ArmSystem arm;
    public static BasicMotor basicMotor;

    // ADD NEW SUBSYSTEMS HERE each year:
    // private static Shooter shooter;
    // private static Intake intake;
    // private static Elevator elevator;

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
}