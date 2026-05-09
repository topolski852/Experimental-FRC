//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.DoubleSupplier;

import org.team1507.lib.core.framework.Subsystem1507;
import org.team1507.lib.core.impl.ctre.Motor1507;
import org.team1507.lib.core.logging.Telemetry;
import org.team1507.lib.core.util.CommandBuilder;

import static org.team1507.robot.Constants.kArmMotor.*;

/**
 * Subsystem controlling a dual‑motor intake arm mechanism.
 *
 * <p>This subsystem owns two {@link Motor1507} instances that operate in
 * synchronized position or manual control. Each motor uses its own
 * {@link org.team1507.lib.core.util.MotorConfig}, allowing one to be inverted
 * mechanically while both receive identical control commands.
 *
 * <p>The subsystem exposes high‑level arm actions such as deploying,
 * retracting, and manual joystick control, while internally managing
 * soft‑limit enforcement, stall detection, and unit conversions.
 */
public final class ArmSystem extends Subsystem1507 {

    /** Primary arm motor (non‑inverted). */
    private final Motor1507 motorA;

    /** Secondary arm motor (inverted). */
    private final Motor1507 motorB;

    private final BaseStatusSignal[] armSignals;

    /** Last commanded arm angle in degrees. */
    private double targetAngleDeg = RETRACTED_ANGLE_DEGREES;

    /** */
    public static enum Position { STOW, LOW, MID, HIGH }

    /**
     * Creates a new dual‑motor arm subsystem.
     *
     * <p>Both motors are configured independently using {@code CONFIG_A} and
     * {@code CONFIG_B}, allowing one motor to be inverted while maintaining
     * identical control behavior.
     */
    public ArmSystem() {
        super("Arm");

        motorA = new Motor1507(
            "ArmA",
            Motor1507.Type.FX,
            21,
            CONFIG_A
        );

        motorB = new Motor1507(
            "ArmB",
            Motor1507.Type.FX,
            22,
            CONFIG_B
        );

        BaseStatusSignal[] a = motorA.getSignals(); // 6
        BaseStatusSignal[] b = motorB.getSignals(); // 6
        armSignals = new BaseStatusSignal[12];
        System.arraycopy(a, 0, armSignals, 0, 6);
        System.arraycopy(b, 0, armSignals, 6, 6);
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(armSignals);

        Telemetry.set("Arm/AngleDegrees", getCurrentAngle());
        Telemetry.set("Arm/TargetDegrees", getTargetAngle());
        Telemetry.set("Arm/Stalled", isStalled());
    }

    // ============================================================
    // POSITION CONTROL
    // ============================================================

    /**
     * Commands the arm to a target angle in degrees.
     *
     * <p>The requested angle is clamped to the subsystem's mechanical limits
     * before being converted to motor rotations and applied to both motors.
     *
     * @param angleDeg desired arm angle in degrees
     */
    public void setAngle(double angleDeg) {
        targetAngleDeg = clampAngle(angleDeg);
        double rotations = degToRotations(targetAngleDeg);

        motorA.setPositionVoltage(rotations, 0.0);
        motorB.setPositionVoltage(rotations, 0.0);
    }

    /** Moves the arm to its deployed (extended) position. */
    public void deploy() {
        setAngle(DEPLOYED_ANGLE_DEGREES);
    }

    /** Moves the arm to its retracted (stowed) position. */
    public void retract() {
        setAngle(RETRACTED_ANGLE_DEGREES);
    }

    // ============================================================
    // MANUAL CONTROL
    // ============================================================

    /**
     * Runs the arm manually using a duty‑cycle input.
     *
     * <p>Motion is automatically restricted when the arm reaches its
     * configured mechanical limits. When the input exceeds a threshold,
     * fixed manual power values are applied to both motors.
     *
     * @param duty joystick or operator‑requested duty cycle
     */
    public void runManual(double duty) {
        double positionDeg = getCurrentAngle();

        if (duty > 0 && positionDeg >= MAX_ANGLE_DEGREES) {
            stop();
            return;
        }

        if (duty < 0 && positionDeg <= MIN_ANGLE_DEGREES) {
            stop();
            return;
        }

        if (duty > 0.5) {
            motorA.runDuty(MANUAL_POSITIVE_POWER);
            motorB.runDuty(MANUAL_POSITIVE_POWER);
        } else if (duty < -0.5) {
            motorA.runDuty(MANUAL_NEGATIVE_POWER);
            motorB.runDuty(MANUAL_NEGATIVE_POWER);
        } else {
            stop();
        }
    }

    /** Stops both arm motors immediately. */
    public void stop() {
        motorA.stop();
        motorB.stop();
    }

    // ============================================================
    // STATE
    // ============================================================

    /**
     * Returns the current arm angle in degrees, based on the primary motor's
     * rotor position.
     *
     * @return current arm angle in degrees
     */
    public double getCurrentAngle() {
        return rotationsToDeg(motorA.getRotorPosition());
    }

    /**
     * Returns the last commanded target angle in degrees.
     *
     * @return target arm angle in degrees
     */
    public double getTargetAngle() {
        return targetAngleDeg;
    }

    /**
     * Returns whether the arm is physically stalled.
     *
     * <p>The arm is considered stalled if either motor reports a stall
     * condition based on its configured current, velocity, and time thresholds.
     *
     * @return true if the arm is stalled
     */
    public boolean isStalled() {
        return motorA.isStalled() || motorB.isStalled();
    }

    // ============================================================
    // UTIL
    // ============================================================

    private static double clampAngle(double angleDeg) {
        return MathUtil.clamp(angleDeg, MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES);
    }

    private static double degToRotations(double degrees) {
        return degrees / 360.0;
    }

    private static double rotationsToDeg(double rotations) {
        return rotations * 360.0;
    }

    // ============================================================
    // COMMAND FACTORIES
    // ============================================================

    /**
     * Creates a command that deploys the arm and finishes if stalled.
     */
    public Command deployCommand() {
        return new CommandBuilder(this)
            .named("Arm.deploy")
            .onInitialize(this::deploy)
            .stallFinish(this::isStalled)
            .onEnd((interrupted, timedOut, stalled) -> stop())
            .runsUntilInterrupted();
    }

    /**
     * Creates a command that retracts the arm and finishes if stalled.
     */
    public Command retractCommand() {
        return new CommandBuilder(this)
            .named("Arm.retract")
            .onInitialize(this::retract)
            .stallFinish(this::isStalled)
            .onEnd((interrupted, timedOut, stalled) -> stop())
            .runsUntilInterrupted();
    }

    /**
     * Creates a command that drives the arm manually using a joystick input.
     */
    public Command goToCommand(Position position) {
        return new CommandBuilder(this)
            .named("Arm.goTo")
            .onExecute(() -> {
                if(position == Position.LOW) setAngle(RETRACTED_ANGLE_DEGREES);
                if(position == Position.MID) setAngle(DEPLOYED_ANGLE_DEGREES);
                if(position == Position.HIGH) setAngle(DEPLOYED_ANGLE_DEGREES);
                if(position == Position.STOW) setAngle(RETRACTED_ANGLE_DEGREES);
            })
            .stallFinish(this::isStalled)
            .onEnd((interrupted, timedOut, stalled) -> stop())
            .runsUntilInterrupted();
    }

    /**
     * Creates a command that manually drives the arm upward.
     */
    public Command manualUpCommand() {
        return new CommandBuilder(this)
            .named("Arm.manualUp")
            .onExecute(() -> runManual(MANUAL_POSITIVE_POWER))
            .stallFinish(this::isStalled)
            .onEnd((interrupted, timedOut, stalled) -> stop())
            .runsUntilInterrupted();
    }

    /**
     * Creates a command that manually drives the arm downward.
     */
    public Command manualDownCommand() {
        return new CommandBuilder(this)
            .named("Arm.manualDown")
            .onExecute(() -> runManual(MANUAL_NEGATIVE_POWER))
            .stallFinish(this::isStalled)
            .onEnd((interrupted, timedOut, stalled) -> stop())
            .runsUntilInterrupted();
    }

    /**
     * Creates a command that drives the arm manually using a joystick input.
     */
    public Command manualJoystickCommand(DoubleSupplier inputSupplier) {
        return new CommandBuilder(this)
            .named("Arm.manualJoystick")
            .onExecute(() -> runManual(inputSupplier.getAsDouble()))
            .stallFinish(this::isStalled)
            .onEnd((interrupted, timedOut, stalled) -> stop())
            .runsUntilInterrupted();
    }
}
