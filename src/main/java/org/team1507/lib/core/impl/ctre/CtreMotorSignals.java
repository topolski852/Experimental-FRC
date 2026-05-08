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
 *
 * <p>Signal update rates are tiered by importance:
 * <ul>
 *   <li>Position and velocity at 100 Hz — needed for odometry and closed-loop control.</li>
 *   <li>Current signals at 50 Hz — fast enough for stall detection without flooding the bus.</li>
 *   <li>Voltage and temperature at 10 Hz — monitoring only, no need for high frequency.</li>
 * </ul>
 */
public final class CtreMotorSignals {

    /** Signals used for closed-loop control and odometry. */
    private static final double ODOMETRY_HZ = 100.0;

    /** Signals used for stall detection. */
    private static final double STALL_HZ = 50.0;

    /** Signals used only for driver station / dashboard monitoring. */
    private static final double MONITOR_HZ = 10.0;

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

        BaseStatusSignal.setUpdateFrequencyForAll(ODOMETRY_HZ, rotorPosition, rotorVelocity);
        BaseStatusSignal.setUpdateFrequencyForAll(STALL_HZ,    supplyCurrent, statorCurrent);
        BaseStatusSignal.setUpdateFrequencyForAll(MONITOR_HZ,  motorVoltage,  deviceTemp);
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

    public void refresh() {
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
        return rotorPosition.getValueAsDouble();
    }

    public double getRotorVelocity() {
        return rotorVelocity.getValueAsDouble();
    }

    public double getSupplyCurrent() {
        return supplyCurrent.getValueAsDouble();
    }

    public double getStatorCurrent() {
        return statorCurrent.getValueAsDouble();
    }

    public double getMotorVoltage() {
        return motorVoltage.getValueAsDouble();
    }

    public double getDeviceTemp() {
        return deviceTemp.getValueAsDouble();
    }
}
