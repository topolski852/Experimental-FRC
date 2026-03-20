//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.lib.core.impl.ctre;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

/**
 * CTRE motor signal wrapper.
 *
 * <p>Owns and manages CTRE {@link StatusSignal} instances for a single motor.
 * This class is responsible only for observation and signal refresh.
 */
public final class CtreMotorSignals {

    private static final double DEFAULT_UPDATE_HZ = 100.0;

    private final StatusSignal<Angle> rotorPosition;
    private final StatusSignal<AngularVelocity> rotorVelocity;
    private final StatusSignal<Current> supplyCurrent;
    private final StatusSignal<Current> statorCurrent;
    private final StatusSignal<Voltage> motorVoltage;
    private final StatusSignal<Temperature> deviceTemp;

    private CtreMotorSignals(
        StatusSignal<Angle> rotorPosition,
        StatusSignal<AngularVelocity> rotorVelocity,
        StatusSignal<Current> supplyCurrent,
        StatusSignal<Current> statorCurrent,
        StatusSignal<Voltage> motorVoltage,
        StatusSignal<Temperature> deviceTemp
    ) {
        this.rotorPosition = rotorPosition;
        this.rotorVelocity = rotorVelocity;
        this.supplyCurrent = supplyCurrent;
        this.statorCurrent = statorCurrent;
        this.motorVoltage  = motorVoltage;
        this.deviceTemp    = deviceTemp;

        BaseStatusSignal.setUpdateFrequencyForAll(
            DEFAULT_UPDATE_HZ,
            rotorPosition,
            rotorVelocity,
            supplyCurrent,
            statorCurrent,
            motorVoltage,
            deviceTemp
        );
    }

    // ============================================================
    // FACTORY
    // ============================================================

    public static CtreMotorSignals fromMotor(Object motor) {
        if (motor instanceof TalonFX fx) {
            return new CtreMotorSignals(
                fx.getPosition(),
                fx.getVelocity(),
                fx.getSupplyCurrent(),
                fx.getStatorCurrent(),
                fx.getMotorVoltage(),
                fx.getDeviceTemp()
            );
        }

        if (motor instanceof TalonFXS fxs) {
            return new CtreMotorSignals(
                fxs.getPosition(),
                fxs.getVelocity(),
                fxs.getSupplyCurrent(),
                fxs.getStatorCurrent(),
                fxs.getMotorVoltage(),
                fxs.getDeviceTemp()
            );
        }

        throw new IllegalArgumentException("Unsupported motor type");
    }

    // ============================================================
    // REFRESH
    // ============================================================

    private void refresh() {
        BaseStatusSignal.refreshAll(
            rotorPosition,
            rotorVelocity,
            supplyCurrent,
            statorCurrent,
            motorVoltage,
            deviceTemp
        );
    }

    // ============================================================
    // ACCESSORS (motor-native units)
    // ============================================================

    public double getRotorPosition() {
        refresh();
        return rotorPosition.getValueAsDouble();
    }

    public double getRotorVelocity() {
        refresh();
        return rotorVelocity.getValueAsDouble();
    }

    public double getSupplyCurrent() {
        refresh();
        return supplyCurrent.getValueAsDouble();
    }

    public double getStatorCurrent() {
        refresh();
        return statorCurrent.getValueAsDouble();
    }

    public double getMotorVoltage() {
        refresh();
        return motorVoltage.getValueAsDouble();
    }

    public double getDeviceTemp() {
        refresh();
        return deviceTemp.getValueAsDouble();
    }
}
