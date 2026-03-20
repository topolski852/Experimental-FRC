//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.robot;

import static edu.wpi.first.units.Units.Amps;

import org.team1507.lib.core.util.MotorConfig;
import org.team1507.lib.core.util.MotorConfig.ControlMode;
import org.team1507.lib.core.util.MotorConfig.GravityType;

import com.ctre.phoenix6.signals.ReverseLimitTypeValue;

public class Constants {

    public static final class kBasicMotor {
        /**
         * MotorConfig now contains ONLY tuning values.
         */
        public static final MotorConfig CONFIG =
            MotorConfig.builder()
                .inverted(true)
                .withVoltageLimits(3.5, -3.5)
                .withStatorCurrentLimit(Amps.of(80))
                .build();


        /** Duty cycles for motor behavior. */
        public static final double DUTY_FORWARD = 0.7;
        public static final double DUTY_REVERSE = -0.3;
    }

    public static final class kArmMotor {

        public static final double MAX_ANGLE_DEGREES = 140.0;
        public static final double MIN_ANGLE_DEGREES = 0.0;
        public static final double DEPLOYED_ANGLE_DEGREES = 138.0;
        public static final double RETRACTED_ANGLE_DEGREES = 82.0;

        public static final double MANUAL_POSITIVE_POWER = 0.4;
        public static final double MANUAL_NEGATIVE_POWER = -0.4;

        public static final MotorConfig CONFIG_A =
            MotorConfig.builder(ControlMode.POSITION)
                .inverted(false)
                .withPID(0.5, 0.0, 0.0)
                .withGravity(0.1, GravityType.COSINE)
                .withReverseLimit(true, true, 0.0) // enable, autoset, reset to 0.0 
                .reverseLimitType(ReverseLimitTypeValue.NormallyOpen)
                .withVoltageLimits(8, -8)
                .withStatorCurrentLimit(Amps.of(100.0))
                .withBrake()
                .build();

        public static final MotorConfig CONFIG_B =
            MotorConfig.builder(ControlMode.POSITION)
                .inverted(true)
                .withPID(0.5, 0.0, 0.0)
                .withGravity(0.1, GravityType.COSINE)
                .withReverseLimit(true, true, 0.0) // enable, autoset, reset to 0.0 
                .reverseLimitType(ReverseLimitTypeValue.NormallyOpen)
                .withVoltageLimits(8, -8)
                .withStatorCurrentLimit(Amps.of(100.0))
                .build();

        public static final class kStall {
            /**
             * Stall detection tuning parameters for the Intake Arm.
             *
             * These values determine how the subsystem decides that the arm has
             * mechanically stalled (i.e., the motors are applying effort but the
             * mechanism is not moving). Adjusting these values changes how sensitive
             * stall detection is.
             *
             * VELOCITY_THRESHOLD:
             *   - Minimum mechanism velocity (in deg/sec) considered "moving".
             *   - If the arm's measured velocity stays BELOW this value while effort
             *     is being applied, it is considered "not moving".
             *   - Increase this value → stall is detected MORE easily (more sensitive).
             *   - Decrease this value → stall is detected LESS easily (less sensitive).
             *
             * EFFORT_THRESHOLD:
             *   - Minimum applied motor voltage (in volts) considered "trying".
             *   - If the motors are applying MORE than this voltage but velocity is low,
             *     the subsystem considers the arm to be pushing against resistance.
             *   - Increase this value → stall requires MORE effort to trigger (less sensitive).
             *   - Decrease this value → stall triggers with LESS effort (more sensitive).
             *
             * TIME_SEC:
             *   - Minimum duration (in seconds) that the stall condition must persist
             *     before being considered a true stall.
             *   - This prevents false positives from brief slowdowns, backlash, or noise.
             *   - Increase this value → stall must last LONGER to trigger (less sensitive).
             *   - Decrease this value → stall triggers FASTER (more sensitive).
             *
             * Together, these three values define the mechanical "signature" of a stall.
             * Tune them based on real‑world behavior: heavier arms, higher friction, or
             * slower gearboxes may require different thresholds.
             */
            public static final double VELOCITY_THRESHOLD = 1.0;   // deg/sec
            public static final double EFFORT_THRESHOLD   = 2.0;   // volts
            public static final double TIME_SEC           = 0.015;  // seconds
        }
    }
}
