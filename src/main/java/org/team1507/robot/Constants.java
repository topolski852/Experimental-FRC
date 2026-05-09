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

import com.ctre.phoenix6.CANBus;
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

    /**
     * CAN bus used by all CTRE devices on this robot.
     *
     * <p>Change to "canivore" (your CANivore bus name) if a CANivore is added.
     * All TalonFX, TalonFXS, CANcoder, and Pigeon2 devices must be on the same bus.
     */
    public static final CANBus CAN_BUS = new CANBus("rio");

    // ============================================================
    // Hardware Map — all CAN device IDs and sensor offsets in one place.
    // Update these whenever a motor or encoder is replaced or renumbered.
    // ============================================================
    public static final class RobotMap {

        // --- Swerve motors and encoders ---
        // Front Left module
        public static final int FL_DRIVE   = 7;
        public static final int FL_STEER   = 8;
        public static final int FL_ENCODER = 9;
        /** Offset in rotations to align the front-left wheel to straight forward. */
        public static final double FL_ENCODER_OFFSET = 0.129150390625;

        // Front Right module
        public static final int FR_DRIVE   = 1;
        public static final int FR_STEER   = 2;
        public static final int FR_ENCODER = 3;
        /** Offset in rotations to align the front-right wheel to straight forward. */
        public static final double FR_ENCODER_OFFSET = -0.28515625;

        // Back Left module
        public static final int BL_DRIVE   = 4;
        public static final int BL_STEER   = 5;
        public static final int BL_ENCODER = 6;
        /** Offset in rotations to align the back-left wheel to straight forward. */
        public static final double BL_ENCODER_OFFSET = -0.436767578125;

        // Back Right module
        public static final int BR_DRIVE   = 10;
        public static final int BR_STEER   = 11;
        public static final int BR_ENCODER = 12;
        /** Offset in rotations to align the back-right wheel to straight forward. */
        public static final double BR_ENCODER_OFFSET = 0.138671875;

        // --- Gyro ---
        public static final int PIGEON2 = 30;

        // --- Driver / operator controllers (USB port on Driver Station) ---
        public static final int DRIVER_CONTROLLER   = 0;
        public static final int OPERATOR_CONTROLLER = 1;
    }

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
                .withRemoteSensorId(RobotMap.FL_ENCODER)
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
                .withRemoteSensorId(RobotMap.FR_ENCODER)
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
                .withRemoteSensorId(RobotMap.BL_ENCODER)
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
                .withRemoteSensorId(RobotMap.BR_ENCODER)
                .withSensorToMechanismRatio(1.0)
                .withContinuousWrap()
                .build();
        public static final Translation2d BACK_RIGHT_LOCATION = new Translation2d(Inches.of(-10.7375), Inches.of(-10.7375));

        public static final class kTunning {
            // ============================================================
            // Tuning Constants
            // ============================================================

            /** Proportional gain for simple heading control (rad/s per radian of error). */
            public static final double HEADING_KP = 5.5;

            /** PID gains for the rotation controller in moveThroughPose. */
            public static final double THETA_KP = 4.0;
            public static final double THETA_KI = 0.0;
            public static final double THETA_KD = 0.1;

            /**
             * Default pass radius for moveThroughPose (meters).
             * How close the robot must get before the waypoint is considered "passed".
             * Larger = passes through sooner. Smaller = more precise.
             */
            public static final double MOVE_THROUGH_DEFAULT_RADIUS = 0.3;

            /**
             * Stall detection thresholds for moveThroughPose.
             * If the robot moves less than STALL_THRESHOLD meters over STALL_TIMEOUT seconds,
             * the command finishes anyway to prevent the robot from getting permanently stuck.
             */
            public static final double STALL_THRESHOLD = 0.02; // meters
            public static final double STALL_TIMEOUT   = 1.5;  // seconds

            /** Finish distance for driveToPoint / driveForwardMeters (meters). */
            public static final double ARRIVE_THRESHOLD = 0.05; // 5 cm

            /** Finish angle tolerance for pointToTarget / changeHeading (degrees). */
            public static final double HEADING_TOLERANCE_DEG = 3.0;

            /**
             * Lead time for motion compensation in maintainHeadingToTarget (seconds).
             * Increase if shots lag while moving. Decrease if shots lead too far.
             */
            public static final double AIM_LEAD_TIME = 0.25;
        }
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
                .withSimVelocityRps(50.0) // ~50 rps at 100% duty; tune to match real motor
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

                .withSimVelocityRps(0.0833) // ~30 deg/sec in sim; tune as needed

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

                .withSimVelocityRps(0.0833)

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
