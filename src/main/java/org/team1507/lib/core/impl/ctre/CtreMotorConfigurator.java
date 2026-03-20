//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.lib.core.impl.ctre;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.*;

import org.team1507.lib.core.util.MotorConfig;
import org.team1507.lib.core.util.MotorConfig.ControlMode;
import org.team1507.lib.core.util.MotorConfig.GravityType;

/**
 * Applies {@link MotorConfig} specifications to CTRE motor controllers.
 *
 * <p>This class translates declarative motor configuration into CTRE Phoenix 6
 * configuration objects. It supports both {@link TalonFX} and {@link TalonFXS}
 * motors and applies all configuration in a single deterministic pass.
 */
public final class CtreMotorConfigurator {

    private CtreMotorConfigurator() {}

    /**
     * Applies the provided motor configurations to the given CTRE motor.
     *
     * @param motor   CTRE motor instance (TalonFX or TalonFXS)
     * @param configs motor configuration specifications
     */
    public static void apply(Object motor, MotorConfig... configs) {
        if (configs.length == 0 || configs[0].slotNumber() != 0) {
            throw new IllegalArgumentException("First MotorConfig must be slot 0");
        }

        if (motor instanceof TalonFX fx) {
            applyFX(fx, configs);
        } else if (motor instanceof TalonFXS fxs) {
            applyFXS(fxs, configs);
        } else {
            throw new IllegalArgumentException("Unsupported motor type");
        }
    }

    // ============================================================
    // TALON FX
    // ============================================================

    private static void applyFX(TalonFX motor, MotorConfig[] configs) {
        TalonFXConfiguration cfg = new TalonFXConfiguration();

        for (MotorConfig config : configs) {
            applySlot(cfg, config);
        }

        applyBase(cfg, configs[0]);
        safeApply(motor, cfg);
    }

    // ============================================================
    // TALON FXS (MINION)
    // ============================================================

    private static void applyFXS(TalonFXS motor, MotorConfig[] configs) {
        TalonFXSConfiguration cfg = new TalonFXSConfiguration();

        cfg.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;

        for (MotorConfig config : configs) {
            applySlot(cfg, config);
        }

        applyBase(cfg, configs[0]);
        safeApply(motor, cfg);
    }

    // ============================================================
    // SLOT ROUTING
    // ============================================================

    private static void applySlot(TalonFXConfiguration cfg, MotorConfig config) {
        switch (config.slotNumber()) {
            case 1 -> applyToSlot(cfg.Slot1, config);
            case 2 -> applyToSlot(cfg.Slot2, config);
            default -> applyToSlot(cfg.Slot0, config);
        }
    }

    private static void applySlot(TalonFXSConfiguration cfg, MotorConfig config) {
        switch (config.slotNumber()) {
            case 1 -> applyToSlot(cfg.Slot1, config);
            case 2 -> applyToSlot(cfg.Slot2, config);
            default -> applyToSlot(cfg.Slot0, config);
        }
    }

    // ============================================================
    // SLOT CONFIGURATION
    // ============================================================

    private static void applyToSlot(Slot0Configs slot, MotorConfig config) {
        applyGains(slot, config);
    }

    private static void applyToSlot(Slot1Configs slot, MotorConfig config) {
        applyGains(slot, config);
    }

    private static void applyToSlot(Slot2Configs slot, MotorConfig config) {
        applyGains(slot, config);
    }

    private static void applyGains(Slot0Configs slot, MotorConfig config) {
        if (config.mode() != ControlMode.DUTY_CYCLE) {
            slot.kP = config.kP();
            slot.kI = config.kI();
            slot.kD = config.kD();
            slot.kV = config.kV();
            slot.kS = config.kS();
            slot.kA = config.kA();
        }

        if (config.gravityType() != GravityType.NONE) {
            slot.kG = config.kG();
            slot.GravityType = switch (config.gravityType()) {
                case COSINE   -> GravityTypeValue.Arm_Cosine;
                case CONSTANT -> GravityTypeValue.Elevator_Static;
                default       -> slot.GravityType;
            };
        }
    }

    private static void applyGains(Slot1Configs slot, MotorConfig config) {
        if (config.mode() != ControlMode.DUTY_CYCLE) {
            slot.kP = config.kP();
            slot.kI = config.kI();
            slot.kD = config.kD();
            slot.kV = config.kV();
            slot.kS = config.kS();
            slot.kA = config.kA();
        }

        if (config.gravityType() != GravityType.NONE) {
            slot.kG = config.kG();
            slot.GravityType = switch (config.gravityType()) {
                case COSINE   -> GravityTypeValue.Arm_Cosine;
                case CONSTANT -> GravityTypeValue.Elevator_Static;
                default       -> slot.GravityType;
            };
        }
    }

    private static void applyGains(Slot2Configs slot, MotorConfig config) {
        if (config.mode() != ControlMode.DUTY_CYCLE) {
            slot.kP = config.kP();
            slot.kI = config.kI();
            slot.kD = config.kD();
            slot.kV = config.kV();
            slot.kS = config.kS();
            slot.kA = config.kA();
        }

        if (config.gravityType() != GravityType.NONE) {
            slot.kG = config.kG();
            slot.GravityType = switch (config.gravityType()) {
                case COSINE   -> GravityTypeValue.Arm_Cosine;
                case CONSTANT -> GravityTypeValue.Elevator_Static;
                default       -> slot.GravityType;
            };
        }
    }

    // ============================================================
    // BASE CONFIGURATION (SLOT 0 AUTHORITY)
    // ============================================================

    private static void applyBase(TalonFXConfiguration cfg, MotorConfig base) {
        applyMotorOutput(cfg.MotorOutput, base);
        applyVoltage(cfg.Voltage, base);
        applyLimitSwitches(cfg.HardwareLimitSwitch, base);
        applyCurrentLimits(cfg.CurrentLimits, base);
    }

    private static void applyBase(TalonFXSConfiguration cfg, MotorConfig base) {
        applyMotorOutput(cfg.MotorOutput, base);
        applyVoltage(cfg.Voltage, base);
        applyLimitSwitches(cfg.HardwareLimitSwitch, base);
        applyCurrentLimits(cfg.CurrentLimits, base);
    }

    private static void applyMotorOutput(MotorOutputConfigs mo, MotorConfig base) {
        mo.Inverted = base.motorInverted()
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;

        mo.NeutralMode = base.brakeMode()
                ? NeutralModeValue.Brake
                : NeutralModeValue.Coast;
    }

    private static void applyVoltage(VoltageConfigs v, MotorConfig base) {
        v.withPeakForwardVoltage(Volts.of(base.peakForwardVoltage()))
         .withPeakReverseVoltage(Volts.of(base.peakReverseVoltage()));
    }

    private static void applyLimitSwitches(HardwareLimitSwitchConfigs hw, MotorConfig base) {
        hw.ForwardLimitEnable = base.forwardLimitEnable();
        hw.ForwardLimitAutosetPositionEnable = base.forwardLimitAutosetEnable();
        hw.ForwardLimitAutosetPositionValue = base.forwardLimitAutosetValue();
        hw.ForwardLimitType = base.forwardLimitType();

        hw.ReverseLimitEnable = base.reverseLimitEnable();
        hw.ReverseLimitAutosetPositionEnable = base.reverseLimitAutosetEnable();
        hw.ReverseLimitAutosetPositionValue = base.reverseLimitAutosetValue();
        hw.ReverseLimitType = base.reverseLimitType();
    }

    private static void applyCurrentLimits(CurrentLimitsConfigs cl, MotorConfig base) {
        cl.StatorCurrentLimit = base.statorCurrentLimit().in(Amps);
        cl.StatorCurrentLimitEnable = true;

        cl.SupplyCurrentLimit = base.supplyCurrentLimit().in(Amps);
        cl.SupplyCurrentLimitEnable = true;
    }

    // ============================================================
    // SAFE APPLY
    // ============================================================

    private static void safeApply(TalonFX motor, TalonFXConfiguration cfg) {
        var status = motor.getConfigurator().apply(cfg);
        if (!status.isOK()) {
            System.out.println("[WARN] Failed to apply config to FX "
                    + motor.getDeviceID() + ": " + status);
        }
    }

    private static void safeApply(TalonFXS motor, TalonFXSConfiguration cfg) {
        var status = motor.getConfigurator().apply(cfg);
        if (!status.isOK()) {
            System.out.println("[WARN] Failed to apply config to FXS "
                    + motor.getDeviceID() + ": " + status);
        }
    }
}
