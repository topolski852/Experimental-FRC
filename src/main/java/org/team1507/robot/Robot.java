//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.robot;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import org.team1507.lib.core.framework.LoggedRobot;
import org.team1507.lib.core.logging.Telemetry;
import org.team1507.robot.subsystems.ArmSystem;
import org.team1507.robot.subsystems.BasicMotor;

public final class Robot extends LoggedRobot {

    private final CommandScheduler scheduler = CommandScheduler.getInstance();

    // ------------------------------------------------------------
    // Subsystems
    // ------------------------------------------------------------

    public final ArmSystem arm;
    public final BasicMotor basicMotor;

    // ------------------------------------------------------------
    // Controllers
    // ------------------------------------------------------------

    private final CommandXboxController driver;

    public Robot() {

        // Initialize subsystems
        arm = new ArmSystem();
        basicMotor = new BasicMotor();

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

        // Arm manual control
        arm.setDefaultCommand(
            arm.manualJoystickCommand(() -> -driver.getLeftY())
        );
    }

    // ------------------------------------------------------------
    // Periodic
    // ------------------------------------------------------------

    @Override
    public void robotPeriodic() {
        scheduler.run();
        Telemetry.update();
    }
}
