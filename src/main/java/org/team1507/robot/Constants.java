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
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearVelocity;

public class Constants {

    // ============================================================
    // CAN Bus — change "canivore" to your CANivore's name if used.
    // All TalonFX, CANcoder, and Pigeon2 devices must share a bus.
    // ============================================================

    public static final CANBus CAN_BUS    = new CANBus("rio");
    public static final CANBus SWERVE_BUS = new CANBus("canivore");

    // ============================================================
    // Hardware Map — all CAN IDs and sensor offsets in one place.
    // Update whenever a motor or encoder is replaced or renumbered.
    // ============================================================

    public static final class RobotMap {

        // Front Left module
        public static final int FL_DRIVE   = 7;
        public static final int FL_STEER   = 8;
        public static final int FL_ENCODER = 9;
        /** Rotations to align the front-left wheel to straight forward. */
        public static final double FL_ENCODER_OFFSET = 0.129150390625;

        // Front Right module
        public static final int FR_DRIVE   = 1;
        public static final int FR_STEER   = 2;
        public static final int FR_ENCODER = 3;
        /** Rotations to align the front-right wheel to straight forward. */
        public static final double FR_ENCODER_OFFSET = -0.28515625;

        // Back Left module
        public static final int BL_DRIVE   = 4;
        public static final int BL_STEER   = 5;
        public static final int BL_ENCODER = 6;
        /** Rotations to align the back-left wheel to straight forward. */
        public static final double BL_ENCODER_OFFSET = -0.436767578125;

        // Back Right module
        public static final int BR_DRIVE   = 10;
        public static final int BR_STEER   = 11;
        public static final int BR_ENCODER = 12;
        /** Rotations to align the back-right wheel to straight forward. */
        public static final double BR_ENCODER_OFFSET = 0.138671875;

        // Gyro
        public static final int PIGEON2 = 30;

        // Example mechanisms — reassign per robot each year
        public static final int ARM_A    = 21;
        public static final int ARM_B    = 22;
        public static final int ELEVATOR = 40;
        public static final int SHOOTER  = 41;
        public static final int FEEDER   = 42;
        public static final int INTAKE   = 43;

        // Driver Station USB ports
        public static final int DRIVER_CONTROLLER   = 0;
        public static final int OPERATOR_CONTROLLER = 1;
    }

    // ============================================================
    // Swerve Drive — kinematics, motor configs, and tuning.
    // ============================================================

    public static final class kSwerve {

        // Theoretical free speed at 12 V — tune to your actual robot.
        public static final LinearVelocity SPEED_AT_12_VOLTS = MetersPerSecond.of(5.04);

        /** Maximum translational speed (m/s). */
        public static final double MAX_SPEED = SPEED_AT_12_VOLTS.magnitude();

        // Gear ratios and wheel geometry
        public static final double COUPLE_RATIO        = 3.5714285714285716;
        public static final double DRIVE_GEAR_RATIO    = 6.122448979591837;
        public static final double STEER_GEAR_RATIO    = 21.428571428571427;
        public static final double WHEEL_RADIUS_METERS = 0.049581;

        // Scale factor applied to drive motor output (in meters). Do not set to zero.
        public static final double DRIVE_METERS_SCALE = 1.0;

        // Steer stator limit — low torque requirement lets us protect against brownouts.
        private static final Current STEER_STATOR_CURRENT = Amps.of(60);

        // Pose estimator standard deviations [x (m), y (m), heading (rad)].
        // Lower = trust that source more.
        public static final Matrix<N3, N1> ODOMETRY_STD_DEV = VecBuilder.fill(0.02, 0.02, 0.05);
        public static final Matrix<N3, N1> VISION_STD_DEV   = VecBuilder.fill(0.02, 0.02, 0.05);

        // Module positions relative to robot center (WPILib: +X forward, +Y left).
        public static final Translation2d FRONT_LEFT_LOCATION  = new Translation2d(Inches.of( 10.7375), Inches.of( 10.7375));
        public static final Translation2d FRONT_RIGHT_LOCATION = new Translation2d(Inches.of( 10.7375), Inches.of(-10.7375));
        public static final Translation2d BACK_LEFT_LOCATION   = new Translation2d(Inches.of(-10.7375), Inches.of( 10.7375));
        public static final Translation2d BACK_RIGHT_LOCATION  = new Translation2d(Inches.of(-10.7375), Inches.of(-10.7375));

        /**
         * Maximum chassis angular rate (rad/s) = v_max / drive_radius.
         * At 5.04 m/s with modules at ±10.7375" on both axes: ~13.1 rad/s.
         */
        public static final double MAX_ANGULAR_RATE =
            MAX_SPEED / FRONT_LEFT_LOCATION.getNorm();

        // -- Motor Configs -----------------------------------------------

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

        // -- Command Tuning ----------------------------------------------

        public static final class kTuning {

            /** Proportional gain for simple heading control (rad/s per radian of error). */
            public static final double HEADING_KP = 5.5;

            /** PID gains for the rotation controller in moveThroughPose. */
            public static final double THETA_KP = 4.0;
            public static final double THETA_KI = 0.0;
            public static final double THETA_KD = 0.1;

            /**
             * Default pass radius for moveThroughPose (meters).
             * How close the robot must get before the waypoint is considered passed.
             */
            public static final double MOVE_THROUGH_DEFAULT_RADIUS = 0.3;

            /**
             * Stall detection for moveThroughPose.
             * If the robot moves less than STALL_THRESHOLD meters over STALL_TIMEOUT seconds,
             * the command exits to prevent a permanent block.
             */
            public static final double STALL_THRESHOLD = 0.02; // meters
            public static final double STALL_TIMEOUT   = 1.5;  // seconds

            /** Arrival threshold for driveToPoint / driveForwardMeters (meters). */
            public static final double ARRIVE_THRESHOLD = 0.05; // 5 cm

            /**
             * P gain for the APF deceleration ramp in driveToPoint / driveForwardMeters.
             * Deceleration begins at cruiseVelocity / ARRIVE_KP meters from the target.
             * At MAX_SPEED (5.04 m/s): ARRIVE_KP=2.5 → decel starts ~2.0 m out.
             */
            public static final double ARRIVE_KP = 2.5;

            /** Finish angle tolerance for pointToTarget / changeHeading (degrees). */
            public static final double HEADING_TOLERANCE_DEG = 3.0;

            /**
             * Lead time for motion compensation in maintainHeadingToTarget (seconds).
             * Increase if shots lag while moving; decrease if they lead too far.
             */
            public static final double AIM_LEAD_TIME = 0.25;
        }
    }

    // ============================================================
    // Elevator — single-motor linear elevator with state machine.
    // ============================================================

    public static final class kElevator {

        public static final int MOTOR_CAN_ID = RobotMap.ELEVATOR;

        // Named setpoints in inches — tune to your robot's geometry
        public static final double STOW_INCHES = 0.0;
        public static final double LOW_INCHES  = 12.0;
        public static final double MID_INCHES  = 24.0;
        public static final double HIGH_INCHES = 48.0;

        /** How close the elevator must be to consider the setpoint reached (inches). */
        public static final double TOLERANCE_INCHES = 0.5;

        /**
         * Inches of carriage travel per motor shaft rotation.
         * Depends on your spool diameter and gear ratio — measure and update.
         */
        public static final double INCHES_PER_ROTATION = 1.5;

        /**
         * Constant feedforward voltage applied to counteract gravity (volts).
         * Increase until the elevator holds position without drifting down.
         */
        public static final double GRAVITY_FF_VOLTS = 0.3;

        /** Maximum duty cycle for manual jogging. Keep low for safety. */
        public static final double MANUAL_DUTY = 0.3;

        public static final MotorConfig CONFIG =
            MotorConfig.builder(ControlMode.POSITION)
                .inverted(false)
                .withPID(5.0, 0.0, 0.0)
                .withVoltageLimits(8, -8)
                .withStatorCurrentLimit(Amps.of(60.0))
                .withStallCurrentThreshold(80.0)
                .withStallVelocityThreshold(kStall.VELOCITY_THRESHOLD / 360.0)
                .withStallTime(kStall.TIME_SEC)
                .withSimVelocityRps(2.0)
                .withBrake()
                .build();
    }

    // ============================================================
    // Shooter — single-motor flywheel.
    // ============================================================

    public static final class kShooter {

        public static final int MOTOR_CAN_ID = RobotMap.SHOOTER;

        /** Default shooting speed in RPM. */
        public static final double DEFAULT_RPM = 3000.0;

        /**
         * How close the flywheel must be to target RPM before the feeder
         * is allowed to run. Wider = faster but less consistent shots.
         */
        public static final double RPM_TOLERANCE = 100.0;

        public static final MotorConfig CONFIG =
            MotorConfig.builder(ControlMode.VELOCITY)
                .inverted(false)
                .withPID(0.5, 0.0, 0.0)
                .withFeedforward(0.0, 0.12, 0.0)
                .withStatorCurrentLimit(Amps.of(80.0))
                .withSimVelocityRps(50.0)
                .build();
    }

    // ============================================================
    // Feeder — single-motor ball feeder.
    // ============================================================

    public static final class kFeeder {

        public static final int MOTOR_CAN_ID = RobotMap.FEEDER;

        /** Feeding speed in RPM (positive = toward shooter). */
        public static final double FEED_RPM  =  500.0;

        /** Reverse speed in RPM for ejecting jammed pieces. */
        public static final double VOMIT_RPM = -250.0;

        public static final MotorConfig CONFIG =
            MotorConfig.builder(ControlMode.VELOCITY)
                .inverted(false)
                .withPID(0.5, 0.0, 0.0)
                .withFeedforward(0.0, 0.12, 0.0)
                .withStatorCurrentLimit(Amps.of(60.0))
                .withSimVelocityRps(8.33)
                .build();
    }

    // ============================================================
    // Intake — single-motor torque-controlled roller intake.
    // ============================================================

    public static final class kIntake {

        public static final int MOTOR_CAN_ID = RobotMap.INTAKE;

        // Torque commands in amps (TorqueCurrentFOC).
        // The motor delivers this exact torque regardless of hopper fill level,
        // preventing the current spikes that caused brownouts under load.
        // Tune TORQUE_AMPS_INTAKE down if brownouts persist;
        // tune up if the roller struggles against a full hopper.
        public static final double TORQUE_AMPS_INTAKE  =  15.0;
        public static final double TORQUE_AMPS_REVERSE =  -8.0;

        public static final MotorConfig CONFIG =
            MotorConfig.builder(ControlMode.TORQUE)
                .inverted(false)
                .withStatorCurrentLimit(Amps.of(20.0))       // hard cap above commanded torque
                .withSupplyCurrentLimit(Amps.of(40.0))       // prevents battery brownout
                .withPeakTorqueCurrent(20.0, -10.0)          // TODO: tune after measuring on robot
                .withSimVelocityRps(50.0)
                .build();
    }

    // ============================================================
    // Arm Motor — dual-motor intake arm mechanism.
    // ============================================================

    public static final class kArmMotor {

        public static final double MAX_ANGLE_DEGREES       = 140.0;
        public static final double MIN_ANGLE_DEGREES       =   0.0;
        public static final double DEPLOYED_ANGLE_DEGREES  = 138.0;
        public static final double RETRACTED_ANGLE_DEGREES =  82.0;

        /** How close the arm must be to consider its target reached (degrees). */
        public static final double ANGLE_TOLERANCE_DEGREES =   2.0;

        public static final double MANUAL_POSITIVE_POWER =  0.4;
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
                .withStallCurrentThreshold(100.0)
                .withStallVelocityThreshold(kStall.VELOCITY_THRESHOLD / 360.0)
                .withStallTime(kStall.TIME_SEC)
                .withSimVelocityRps(0.1)
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
                .withSimVelocityRps(0.1)
                .withBrake()
                .build();
    }

    // ============================================================
    // Stall Detection — shared thresholds used by motor configs.
    //
    // A stall is declared when all three conditions hold simultaneously:
    //   VELOCITY_THRESHOLD — mechanism velocity (deg/sec) is below this
    //   EFFORT_THRESHOLD   — applied voltage (V) is above this
    //   TIME_SEC           — both conditions have persisted this long
    //
    // Raise VELOCITY_THRESHOLD or lower EFFORT_THRESHOLD to detect stalls
    // more eagerly. Raise TIME_SEC to reduce false positives from brief
    // slowdowns, backlash, or load spikes.
    // ============================================================

    public static final class kStall {
        public static final double VELOCITY_THRESHOLD = 1.0;   // deg/sec
        public static final double EFFORT_THRESHOLD   = 2.0;   // volts
        public static final double TIME_SEC           = 0.015;  // seconds
    }

    // ============================================================
    // QuestNav — Meta Quest headset vision configuration.
    // ============================================================

    public static final class kQuest {

        /**
         * Transform from robot center to the Quest's physical mounting point.
         *
         * <p>The Quest reports its own 3D pose at the headset, not at the robot
         * center. This offset is applied to convert between the two:
         * <ul>
         *   <li>Robot center → Quest mount: apply ROBOT_TO_QUEST (forward)</li>
         *   <li>Quest mount → Robot center: apply ROBOT_TO_QUEST.inverse()</li>
         * </ul>
         *
         * <p>Measured from the 2026 field season. Re-measure if the mount changes.
         * x = -0.202 m (behind center), y = 0.304 m (left of center),
         * z = 0.39 m (height), yaw = 90° (headset faces robot-left).
         */
        public static final Transform3d ROBOT_TO_QUEST = new Transform3d(
            -0.202, 0.304, 0.39,
            new Rotation3d(0, 0, Math.toRadians(90))
        );
    }
}
