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
import org.team1507.robot.subsystems.*;
import org.team1507.robot.auto.AutoBuilder;
import org.team1507.robot.auto.routines.DriveForwardAuto;
import org.team1507.robot.auto.routines.ScorePickupScoreAuto;
import org.team1507.robot.Constants.RobotMap;
import org.team1507.robot.Constants.kSwerve;

public final class Robot extends LoggedRobot {

    // ------------------------------------------------------------
    // Subsystems
    // ------------------------------------------------------------

    public final ArmSystem arm;
    public final BasicMotor basicMotor;
    public final Swerve swerve;

    // ------------------------------------------------------------
    // Controllers
    // ------------------------------------------------------------

    private final CommandXboxController driver;

    // ------------------------------------------------------------
    // Auto
    // ------------------------------------------------------------

    private Command m_autoCommand = null;
    private final SendableChooser<Command> autoChooser =
        new SendableChooser<>();

    public Robot() {

        // Initialize subsystems
        arm         = new ArmSystem();
        basicMotor  = new BasicMotor();
        swerve      = new Swerve();

        // Initialize Autos

        AutoBuilder.init(swerve, arm, basicMotor);

        autoChooser.setDefaultOption(
            "Drive Forward",
            DriveForwardAuto.build()
        );

        autoChooser.addOption(
            "Score Pickup Score",
            ScorePickupScoreAuto.build()
        );

        SmartDashboard.putData("Auto Mode", autoChooser);

        // Initialize controllers
        driver = new CommandXboxController(RobotMap.DRIVER_CONTROLLER);

        // Bind controls
        configureBindings();
        configureDefaultBindings();
    }

    // ------------------------------------------------------------
    // Bindings
    // ------------------------------------------------------------

    private void configureBindings() {

        // Basic motor examples
        driver.a().whileTrue(basicMotor.runForwardCommand());
        driver.b().whileTrue(basicMotor.runReverseCommand());

        // Arm position examples
        driver.x().onTrue(arm.deployCommand());
        driver.y().onTrue(arm.retractCommand());

        driver.start().onTrue(swerve.lockCommand());
        driver.back().onTrue(swerve.stopCommand());

        // Point the robot toward the opposing alliance wall, then press left bumper
        // to zero the gyro. Required after any hot code deploy without a power cycle.
        driver.leftBumper().onTrue(swerve.zeroHeadingCommand());
    }

    private void configureDefaultBindings() {

        // Swerve default command
        swerve.setDefaultCommand(
            swerve.driveCommand(() -> {
                double x = MathUtil.applyDeadband(-driver.getLeftY(), 0.12);
                double y = MathUtil.applyDeadband(-driver.getLeftX(), 0.12);
                double rot = MathUtil.applyDeadband(-driver.getRightX(), 0.12);

                return ChassisSpeeds.fromFieldRelativeSpeeds(
                    x * kSwerve.MAX_SPEED,
                    y * kSwerve.MAX_SPEED,
                    rot * Math.PI,
                    swerve.getHeading()
                );
            })
        );
    }

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
