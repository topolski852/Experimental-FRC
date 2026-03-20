//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.DoubleSupplier;

import org.team1507.lib.core.framework.Subsystem1507;
import org.team1507.lib.core.impl.ctre.Motor1507;
import org.team1507.lib.core.util.CommandBuilder;
import org.team1507.robot.Constants;

/**
 * Subsystem controlling the robot's arm mechanism.
 *
 * <p>This subsystem owns the arm motor and provides high-level
 * position and manual control in mechanism units (degrees).
 */
public final class ArmSystem extends Subsystem1507 {

    private final Motor1507 motor;

    /** Desired arm angle in degrees. */
    private double targetAngleDeg = Constants.kArmMotor.RETRACTED_ANGLE_DEGREES;

    public ArmSystem() {
        super("Arm");

        motor = new Motor1507(
            "Arm",
            Motor1507.Type.FX,
            21, // CAN ID lives here
            Constants.kArmMotor.CONFIG
        );
    }

    // ============================================================
    // POSITION CONTROL
    // ============================================================

    /**
     * Commands the arm to a target angle in degrees.
     *
     * <p>The requested angle is clamped to the configured
     * mechanical limits.
     *
     * @param angleDeg desired arm angle in degrees
     */
    public void setAngle(double angleDeg) {
        targetAngleDeg = clampAngle(angleDeg);
        motor.setPositionVoltage(degToRotations(targetAngleDeg), 0.0);
    }

    /** Moves the arm to the deployed position. */
    public void deploy() {
        setAngle(Constants.kArmMotor.DEPLOYED_ANGLE_DEGREES);
    }

    /** Moves the arm to the retracted position. */
    public void retract() {
        setAngle(Constants.kArmMotor.RETRACTED_ANGLE_DEGREES);
    }

    // ============================================================
    // MANUAL CONTROL
    // ============================================================

    /**
     * Runs the arm motor manually using a fixed duty cycle.
     *
     * <p>Manual motion is automatically limited to prevent
     * driving past mechanical bounds.
     *
     * @param duty requested motor duty cycle
     */
    public void runManual(double duty) {
        double positionDeg = getCurrentAngle();

        if (duty > 0 && positionDeg >= Constants.kArmMotor.MAX_ANGLE_DEGREES) {
            motor.stop();
            return;
        }

        if (duty < 0 && positionDeg <= Constants.kArmMotor.MIN_ANGLE_DEGREES) {
            motor.stop();
            return;
        }

        if (duty > 0.5) {
            motor.runDuty(Constants.kArmMotor.MANUAL_POSITIVE_POWER);
        } else if (duty < -0.5) {
            motor.runDuty(Constants.kArmMotor.MANUAL_NEGATIVE_POWER);
        } else {
            motor.stop();
        }
    }

    /** Stops the arm motor immediately. */
    public void stop() {
        motor.stop();
    }

    // ============================================================
    // STATE
    // ============================================================

    /**
     * Returns the current arm angle in degrees.
     */
    public double getCurrentAngle() {
        return rotationsToDeg(motor.getRotorPosition());
    }

    /**
     * Returns the last commanded target angle in degrees.
     */
    public double getTargetAngle() {
        return targetAngleDeg;
    }

    // ============================================================
    // UTIL
    // ============================================================

    private static double clampAngle(double angleDeg) {
        return MathUtil.clamp(
            angleDeg,
            Constants.kArmMotor.MIN_ANGLE_DEGREES,
            Constants.kArmMotor.MAX_ANGLE_DEGREES
        );
    }

    private static double degToRotations(double degrees) {
        return degrees / 360.0;
    }

    private static double rotationsToDeg(double rotations) {
        return rotations * 360.0;
    }

    // ============================================================
    // Commands
    // ============================================================

    public Command deployCommand() {
        return new CommandBuilder(this)
            .named("Arm.deploy")
            .onInitialize(this::deploy)
            .onEnd(this::stop)
            .isFinished(false); // runs until interrupted
    }

    public Command retractCommand() {
        return new CommandBuilder(this)
            .named("Arm.retract")
            .onInitialize(this::retract)
            .onEnd(this::stop)
            .isFinished(false);
    }

    public Command manualUpCommand() {
        return new CommandBuilder(this)
            .named("Arm.manualUp")
            .onExecute(() ->
                motor.runDuty(Constants.kArmMotor.MANUAL_POSITIVE_POWER)
            )
            .onEnd(this::stop);
    }

    public Command manualDownCommand() {
        return new CommandBuilder(this)
            .named("Arm.manualDown")
            .onExecute(() ->
                motor.runDuty(Constants.kArmMotor.MANUAL_NEGATIVE_POWER)
            )
            .onEnd(this::stop);
    }

    public Command manualJoystickCommand(DoubleSupplier inputSupplier) {
        return new CommandBuilder(this)
            .named("Arm.manualJoystick")
            .onExecute(() -> runManual(inputSupplier.getAsDouble()))
            .onEnd(this::stop);
    }
}
