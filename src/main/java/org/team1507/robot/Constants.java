//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;

import org.team1507.lib.core.util.MotorConfig;
import org.team1507.lib.core.util.MotorConfig.ControlMode;
import org.team1507.lib.core.util.MotorConfig.GravityType;

import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearVelocity;

public class Constants {

    public static final class kSwerve {

        // Theoretical free speed (m/s) at 12 V applied output;
        // This needs to be tuned to your individual robot
        public static final LinearVelocity SPEED_AT_12_VOLTS = MetersPerSecond.of(5.04);

        // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
        // This may need to be tuned to your individual robot
        public static final double COUPLE_RATIO = 3.5714285714285716;

        public static final double DRIVE_GEAR_RATIO = 6.122448979591837;
        public static final double STEER_GEAR_RATIO = 21.428571428571427;
        public static final double WHEEL_RADIUS_METERS = 0.049581;

        // Swerve steer azimuth does not require much torque output, so we can set a relatively low
        // stator current limit to help avoid brownouts without impacting performance.
        private static final Current STEER_STATOR_CURRENT = Amps.of(60);

        // The standard deviation for odometry calculation
        // in the form [x, y, theta]ᵀ, with units in meters and radians
        public static final Matrix<N3, N1> ODOMETRY_STD_DEV = VecBuilder.fill(0.02, 0.02, 0.05);

        // The standard deviation for vision calculation
        // in the form [x, y, theta]ᵀ, with units in meters and radians
        public static final Matrix<N3, N1> VISION_STD_DEV = VecBuilder.fill(0.02, 0.02, 0.05);

        // This variable is used to modify the offset of the drivetrain in meters. *DO NOT SET TO ZERO*
        public static final double DRIVE_METERS_SCALE = 1.0;
        
        public static final MotorConfig FRONT_LEFT_DRIVE_CONFIG =
            MotorConfig.builder(ControlMode.VELOCITY)
                .inverted(false)
                .withPID(2.0, 0.0, 0.0)
                .withFeedforward(0.12, 2.75, 0.32)
                .build();
        public static final MotorConfig FRONT_LEFT_STEER_CONFIG =
            MotorConfig.builder(ControlMode.POSITION)
                .inverted(true)
                .withPID(70, 0, 0.2)
                .withFeedforward(0.08, 2.2, 0.0)
                .withStatorCurrentLimit(STEER_STATOR_CURRENT)
                .withFeedbackSensor(FeedbackSensorSourceValue.RemoteCANcoder)
                .withRemoteSensorId(9)
                .withSensorToMechanismRatio(1.0)
                .withContinuousWrap()
                .build();
        public static final Translation2d FRONT_LEFT_LOCATION = new Translation2d(Inches.of(10.7375), Inches.of(10.7375));

        public static final MotorConfig FRONT_RIGHT_DRIVE_CONFIG =
            MotorConfig.builder(ControlMode.VELOCITY)
                .inverted(true)
                .withPID(2.0, 0.0, 0.0)
                .withFeedforward(0.12, 2.75, 0.32)
                .build();
        public static final MotorConfig FRONT_RIGHT_STEER_CONFIG =
            MotorConfig.builder(ControlMode.POSITION)
                .inverted(true)
                .withPID(70, 0, 0.2)
                .withFeedforward(0.08, 2.2, 0.0)
                .withStatorCurrentLimit(STEER_STATOR_CURRENT)
                .withFeedbackSensor(FeedbackSensorSourceValue.RemoteCANcoder)
                .withRemoteSensorId(3)
                .withSensorToMechanismRatio(1.0)
                .withContinuousWrap()
                .build();
        public static final Translation2d FRONT_RIGHT_LOCATION = new Translation2d(Inches.of(10.7375), Inches.of(-10.7375));

        public static final MotorConfig BACK_LEFT_DRIVE_CONFIG =
            MotorConfig.builder(ControlMode.VELOCITY)
                .inverted(false)
                .withPID(2.0, 0.0, 0.0)
                .withFeedforward(0.12, 2.75, 0.32)
                .build();
        public static final MotorConfig BACK_LEFT_STEER_CONFIG =
            MotorConfig.builder(ControlMode.POSITION)
                .inverted(true)
                .withPID(70, 0, 0.2)
                .withFeedforward(0.08, 2.2, 0.0)
                .withStatorCurrentLimit(STEER_STATOR_CURRENT)
                .withFeedbackSensor(FeedbackSensorSourceValue.RemoteCANcoder)
                .withRemoteSensorId(6)
                .withSensorToMechanismRatio(1.0)
                .withContinuousWrap()
                .build();
        public static final Translation2d BACK_LEFT_LOCATION = new Translation2d(Inches.of(-10.7375), Inches.of(10.7375));

        public static final MotorConfig BACK_RIGHT_DRIVE_CONFIG =
            MotorConfig.builder(ControlMode.VELOCITY)
                .inverted(true)
                .withPID(2.0, 0.0, 0.0)
                .withFeedforward(0.12, 2.75, 0.32)
                .build();
        public static final MotorConfig BACK_RIGHT_STEER_CONFIG =
            MotorConfig.builder(ControlMode.POSITION)
                .inverted(true)
                .withPID(70, 0, 0.2)
                .withFeedforward(0.08, 2.2, 0.0)
                .withStatorCurrentLimit(STEER_STATOR_CURRENT)
                .withFeedbackSensor(FeedbackSensorSourceValue.RemoteCANcoder)
                .withRemoteSensorId(12)
                .withSensorToMechanismRatio(1.0)
                .withContinuousWrap()
                .build();
        public static final Translation2d BACK_RIGHT_LOCATION = new Translation2d(Inches.of(-10.7375), Inches.of(-10.7375));
    }

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
                .withReverseLimit(true, true, 0.0)
                .reverseLimitType(ReverseLimitTypeValue.NormallyOpen)
                .withVoltageLimits(8, -8)
                .withStatorCurrentLimit(Amps.of(60.0))
                .withSupplyCurrentLimit(Amps.of(10.0))

                .withStallCurrentThreshold(100.0) // amps
                .withStallVelocityThreshold(kStall.VELOCITY_THRESHOLD / 360.0) // deg/sec → rps
                .withStallTime(kStall.TIME_SEC)

                .withBrake()
                .build();

        public static final MotorConfig CONFIG_B =
            MotorConfig.builder(ControlMode.POSITION)
                .inverted(true)
                .withPID(0.5, 0.0, 0.0)
                .withGravity(0.1, GravityType.COSINE)
                .withReverseLimit(true, true, 0.0)
                .reverseLimitType(ReverseLimitTypeValue.NormallyOpen)
                .withVoltageLimits(8, -8)
                .withStatorCurrentLimit(Amps.of(60.0))
                .withSupplyCurrentLimit(Amps.of(10.0))

                .withStallCurrentThreshold(100.0)
                .withStallVelocityThreshold(kStall.VELOCITY_THRESHOLD / 360.0)
                .withStallTime(kStall.TIME_SEC)

                .withBrake()
                .build();
    }

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
