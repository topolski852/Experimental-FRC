//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.lib.core.impl.ctre;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;

import org.team1507.lib.core.logging.InputField;
import org.team1507.lib.core.logging.Telemetry;
import org.team1507.lib.core.logging.TelemetryRate;
import org.team1507.lib.core.util.MotorConfig;

/**
 * Unified CTRE motor abstraction for Team 1507.
 *
 * <p>{@code Motor1507} owns motor hardware, control, and telemetry declaration.
 * Configuration is applied declaratively via {@link MotorConfig}.
 */
public final class Motor1507 {

    public enum Type { FX, FXS }

    private final Object motor;
    private final CtreMotorSignals signals;
    private final String name;

    // ------------------------------------------------------------
    // Telemetry fields
    // ------------------------------------------------------------

    public final InputField<Double> rotorPosition;
    public final InputField<Double> rotorVelocity;
    public final InputField<Double> supplyCurrent;
    public final InputField<Double> statorCurrent;
    public final InputField<Double> motorVoltage;
    public final InputField<Double> deviceTemp;

    /**
     * Creates a new motor instance and applies the provided configuration.
     *
     * @param name    human-readable motor name (e.g. "Feeder", "IntakeArm")
     * @param type    motor hardware type
     * @param canId   CAN device ID
     * @param configs motor configuration specifications
     */
    public Motor1507(String name, Type type, int canId, MotorConfig... configs) {
        this.name = name;
        this.motor = createMotor(type, canId);
        CtreMotorConfigurator.apply(motor, configs);

        this.signals = CtreMotorSignals.fromMotor(motor);

        // --------------------------------------------------------
        // Declare telemetry
        // --------------------------------------------------------

        rotorPosition = new InputField<>(
            key("Input", "Position"),
            signals::getRotorPosition,
            TelemetryRate.NORMAL
        );

        rotorVelocity = new InputField<>(
            key("Input", "Velocity"),
            signals::getRotorVelocity,
            TelemetryRate.FAST
        );

        supplyCurrent = new InputField<>(
            key("Input", "SupplyCurrent"),
            signals::getSupplyCurrent,
            TelemetryRate.NORMAL
        );

        statorCurrent = new InputField<>(
            key("Input", "StatorCurrent"),
            signals::getStatorCurrent,
            TelemetryRate.NORMAL
        );

        motorVoltage = new InputField<>(
            key("Input", "Voltage"),
            signals::getMotorVoltage,
            TelemetryRate.SLOW
        );

        deviceTemp = new InputField<>(
            key("Input", "Temperature"),
            signals::getDeviceTemp,
            TelemetryRate.SLOW
        );


        Telemetry.register(
            rotorPosition,
            rotorVelocity,
            supplyCurrent,
            statorCurrent,
            motorVoltage,
            deviceTemp
        );
    }

    // ============================================================
    // CONTROL
    // ============================================================

    private void setControl(ControlRequest request) {
        if (motor instanceof TalonFX fx) {
            fx.setControl(request);
        } else if (motor instanceof TalonFXS fxs) {
            fxs.setControl(request);
        }
    }

    public void runDuty(double dutyCycle) {
        setControl(new DutyCycleOut(dutyCycle));
    }

    public void setPositionDuty(double rotations) {
        setControl(new PositionDutyCycle(rotations));
    }

    public void setPositionVoltage(double rotations, double ffVolts) {
        setControl(
            new PositionVoltage(rotations)
                .withFeedForward(ffVolts)
        );
    }

    public void setVelocityRPS(double motorRPS) {
        setControl(new VelocityVoltage(motorRPS));
    }

    public void setVelocityRPS(double motorRPS, double ffVolts) {
        setControl(
            new VelocityVoltage(motorRPS)
                .withFeedForward(ffVolts)
        );
    }

    public void stop() {
        if (motor instanceof TalonFX fx) {
            fx.stopMotor();
        } else if (motor instanceof TalonFXS fxs) {
            fxs.stopMotor();
        }
    }

    // ============================================================
    // OBSERVATION (mechanism-agnostic motor units)
    // ============================================================

    public double getRotorPosition() {
        return signals.getRotorPosition();
    }

    public double getRotorVelocity() {
        return signals.getRotorVelocity();
    }

    public double getSupplyCurrent() {
        return signals.getSupplyCurrent();
    }

    public double getStatorCurrent() {
        return signals.getStatorCurrent();
    }

    public double getMotorVoltage() {
        return signals.getMotorVoltage();
    }

    public double getDeviceTemp() {
        return signals.getDeviceTemp();
    }

    // ============================================================
    // INTERNAL
    // ============================================================

    private String key(String category, String field) {
        return name + "/" + category + "/" + field;
    }

    private static Object createMotor(Type type, int canId) {
        return switch (type) {
            case FX  -> new TalonFX(canId);
            case FXS -> new TalonFXS(canId);
        };
    }
}
