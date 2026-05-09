//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import org.team1507.lib.core.framework.LoggedRobot;
import org.team1507.lib.core.vision.QuestNavSubsystem;
import org.team1507.robot.auto.AutoBuilder;
import org.team1507.robot.auto.nodes.Nodes;
import org.team1507.robot.auto.routines.*;
import org.team1507.robot.Constants.RobotMap;
import org.team1507.robot.Constants.kQuest;
import org.team1507.robot.Constants.kSwerve;
import org.team1507.robot.subsystems.*;
import org.team1507.robot.subsystems.Elevator.Setpoint;

public final class Robot extends LoggedRobot {

    // -------------------------------------------------------------------------
    // Subsystems
    // -------------------------------------------------------------------------

    public final Swerve            swerve;
    public final QuestNavSubsystem questNav;
    public final ArmSystem         arm;
    public final Elevator          elevator;
    public final Shooter           shooter;
    public final Feeder            feeder;
    public final Intake            intake;

    // -------------------------------------------------------------------------
    // Controllers
    // -------------------------------------------------------------------------

    private final CommandXboxController driver;
    private final CommandXboxController operator;

    // -------------------------------------------------------------------------
    // Autonomous
    // -------------------------------------------------------------------------

    private Command m_autoCommand = null;
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    // =========================================================================
    // Constructor
    // =========================================================================

    public Robot() {

        // Subsystems — swerve first; questNav takes method references from it.
        swerve    = new Swerve();
        questNav  = new QuestNavSubsystem(
            swerve::addVisionMeasurement,
            swerve::resetPose,
            kQuest.ROBOT_TO_QUEST
        );
        arm      = new ArmSystem();
        elevator = new Elevator();
        shooter  = new Shooter();
        feeder   = new Feeder();
        intake   = new Intake();

        // Pre-match pose preset buttons (visible in Elastic while disabled).
        // Place the robot at the known starting position and press the matching
        // button. The command waits for Quest to confirm before snapping odometry.
        questNav.setKnownPoseCommand(Nodes.Robot.Start.LEFT)
            .named("Set Pose Left")
            .publishToDashboard();
        questNav.setKnownPoseCommand(Nodes.Robot.Start.CENTER)
            .named("Set Pose Start")
            .publishToDashboard();
        questNav.setKnownPoseCommand(Nodes.Robot.Start.RIGHT)
            .named("Set Pose Right")
            .publishToDashboard();

        // Autonomous chooser
        AutoBuilder.init(swerve, arm, elevator, shooter, feeder, intake);
        autoChooser.setDefaultOption("Drive Forward", DriveForwardAuto.build());
        autoChooser.addOption("Score Pickup Score", ScorePickupScoreAuto.build());
        SmartDashboard.putData("Auto Mode", autoChooser);

        // Controllers and bindings
        driver   = new CommandXboxController(RobotMap.DRIVER_CONTROLLER);
        operator = new CommandXboxController(RobotMap.OPERATOR_CONTROLLER);
        configureBindings();
        configureDefaultBindings();
    }

    // =========================================================================
    // Bindings
    // =========================================================================

    private void configureBindings() {

        // ── Driver — swerve utilities + shooting ───────────────────────────

        // Swerve brake — hold Start to lock wheels in X pattern, release to resume driving
        driver.start().whileTrue(swerve.brakeCommand());

        // Failsafe — cancels all running commands (see RobotBehaviors for details)
        driver.back().onTrue(RobotBehaviors.failsafe());

        // Point the robot toward the opposing alliance wall, then press A
        // to zero the gyro. Do this after any hot code deploy without a power cycle.
        driver.a().onTrue(swerve.zeroHeadingCommand());

        // Shoot — driver holds right trigger to aim and fire.
        // Spinner ramps up immediately; feeder engages once shooter is at RPM.
        driver.rightTrigger().whileTrue(RobotBehaviors.shoot());

        // Feeder vomit — driver holds right bumper to unjam
        driver.rightBumper().whileTrue(feeder.vomitCommand());

        // ── Operator — mechanisms ──────────────────────────────────────────

        // Arm
        operator.x().onTrue(arm.deployCommand());
        operator.y().onTrue(arm.retractCommand());

        // Intake — right bumper runs; right trigger reverses/ejects
        operator.rightBumper() .whileTrue(intake.runCommand());
        operator.rightTrigger().whileTrue(intake.reverseCommand());

        // Elevator — D-pad presets, bumpers for manual jogging
        operator.povUp()      .onTrue(elevator.goToCommand(Setpoint.HIGH));
        operator.povDown()    .onTrue(elevator.goToCommand(Setpoint.STOW));
        operator.povLeft()    .onTrue(elevator.goToCommand(Setpoint.MID));
        operator.povRight()   .onTrue(elevator.goToCommand(Setpoint.LOW));
        operator.leftBumper() .whileTrue(elevator.manualUpCommand());
        operator.leftTrigger().whileTrue(elevator.manualDownCommand());

        // Stow all mechanisms to a safe travel position
        operator.back().onTrue(RobotBehaviors.stow());
    }

    private void configureDefaultBindings() {

        swerve.setDefaultCommand(
            swerve.driveCommand(() -> {
                double x   = MathUtil.applyDeadband(-driver.getLeftY(),  0.12);
                double y   = MathUtil.applyDeadband(-driver.getLeftX(),  0.12);
                double rot = MathUtil.applyDeadband(-driver.getRightX(), 0.12);

                return ChassisSpeeds.fromFieldRelativeSpeeds(
                    x   * kSwerve.MAX_SPEED,
                    y   * kSwerve.MAX_SPEED,
                    rot * Math.PI,
                    swerve.getHeading()
                );
            })
        );
    }

    // =========================================================================
    // Mode callbacks
    // =========================================================================

    @Override
    public void autonomousInit() {
        m_autoCommand = autoChooser.getSelected();
        if (m_autoCommand != null) {
            CommandScheduler.getInstance().schedule(m_autoCommand);
        }
    }

    @Override
    public void teleopInit() {
        if (m_autoCommand != null) {
            m_autoCommand.cancel();
        }
    }
}
