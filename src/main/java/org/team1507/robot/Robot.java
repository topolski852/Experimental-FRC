//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;

import static edu.wpi.first.units.Units.Rotations;

import org.team1507.lib.core.framework.LoggedRobot;
import org.team1507.lib.core.impl.ctre.Motor1507;
import org.team1507.lib.core.swerve.SwerveModule1507;
import org.team1507.lib.core.swerve.SwerveModule1507.MathConfig;
import org.team1507.robot.subsystems.*;
import org.team1507.robot.auto.routines.DriveForwardAuto;

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
        arm = new ArmSystem();
        basicMotor = new BasicMotor();

        MathConfig math = new MathConfig(kSwerve.DRIVE_GEAR_RATIO, kSwerve.STEER_GEAR_RATIO, kSwerve.COUPLE_RATIO, kSwerve.WHEEL_RADIUS_METERS);

        swerve = new Swerve(
            new SwerveModule1507("FrontLeft", 
                new Motor1507("FL_Drive", Motor1507.Type.FX, 7,  kSwerve.FRONT_LEFT_DRIVE_CONFIG),
                new Motor1507("FL_Steer", Motor1507.Type.FX, 8,  kSwerve.FRONT_LEFT_STEER_CONFIG), 
                new CANcoder(9),  new Rotation2d(Rotations.of(0.129150390625)), math, kSwerve.DRIVE_METERS_SCALE),

            new SwerveModule1507("FrontRight", 
                new Motor1507("FR_Drive", Motor1507.Type.FX, 1,  kSwerve.FRONT_RIGHT_DRIVE_CONFIG),
                new Motor1507("FR_Steer", Motor1507.Type.FX, 2,  kSwerve.FRONT_RIGHT_STEER_CONFIG), 
                new CANcoder(3),  new Rotation2d(Rotations.of(-0.28515625)), math, kSwerve.DRIVE_METERS_SCALE),

            new SwerveModule1507("BackLeft", 
                new Motor1507("BL_Drive", Motor1507.Type.FX, 4,  kSwerve.BACK_LEFT_DRIVE_CONFIG),
                new Motor1507("BL_Steer", Motor1507.Type.FX, 5,  kSwerve.BACK_LEFT_STEER_CONFIG), 
                new CANcoder(6),  new Rotation2d(Rotations.of(-0.436767578125)), math, kSwerve.DRIVE_METERS_SCALE),

            new SwerveModule1507("BackRight", 
                new Motor1507("BR_Drive", Motor1507.Type.FX, 10, kSwerve.BACK_RIGHT_DRIVE_CONFIG),
                new Motor1507("BR_Steer", Motor1507.Type.FX, 11, kSwerve.BACK_RIGHT_STEER_CONFIG), 
                new CANcoder(12), new Rotation2d(Rotations.of(0.138671875)), math, kSwerve.DRIVE_METERS_SCALE),
            kSwerve.FRONT_LEFT_LOCATION,
            kSwerve.FRONT_RIGHT_LOCATION,
            kSwerve.BACK_LEFT_LOCATION,
            kSwerve.BACK_RIGHT_LOCATION,
            new Pigeon2(30),
            kSwerve.SPEED_AT_12_VOLTS.magnitude(),
            kSwerve.ODOMETRY_STD_DEV,
            kSwerve.VISION_STD_DEV);

        // Initialize Autos
        autoChooser.setDefaultOption(
            "Drive Forward",
            DriveForwardAuto.build(swerve)
        );
        
        SmartDashboard.putData("Auto Mode", autoChooser);

        // Initialize controllers
        driver = new CommandXboxController(0);

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
    }

    private void configureDefaultBindings() {

        // Swerve default command
        swerve.setDefaultCommand(
            swerve.driveCommand(() -> {
                double x = MathUtil.applyDeadband(-driver.getLeftY(), 0.12);
                double y = MathUtil.applyDeadband(-driver.getLeftX(), 0.12);
                double rot = MathUtil.applyDeadband(-driver.getRightX(), 0.12);

                return ChassisSpeeds.fromFieldRelativeSpeeds(
                    x * kSwerve.SPEED_AT_12_VOLTS.magnitude(),
                    y * kSwerve.SPEED_AT_12_VOLTS.magnitude(),
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
            m_autoCommand.schedule();
        }
    }

    @Override
    public void teleopInit() {
        if (m_autoCommand != null) {
            m_autoCommand.cancel();
        }
    }
}
