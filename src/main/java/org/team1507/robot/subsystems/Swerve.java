//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;

import org.team1507.lib.core.logging.Telemetry;
import org.team1507.lib.core.swerve.SwerveModule1507;

import static org.team1507.robot.Constants.kSwerve.kTunning.*;

public final class Swerve extends SubsystemBase {

    // ------------------------------------------------------------
    // Modules
    // ------------------------------------------------------------

    private final SwerveModule1507 frontLeft;
    private final SwerveModule1507 frontRight;
    private final SwerveModule1507 backLeft;
    private final SwerveModule1507 backRight;

    // ------------------------------------------------------------
    // Kinematics & odometry
    // ------------------------------------------------------------

    private final SwerveDriveKinematics kinematics;
    private final SwerveDrivePoseEstimator poseEstimator;

    // ------------------------------------------------------------
    // Sensors
    // ------------------------------------------------------------

    private final Pigeon2 pigeon;
    private final StatusSignal<Angle> yaw;

    // ------------------------------------------------------------
    // Cached state
    // ------------------------------------------------------------

    private Pose2d pose = new Pose2d();

    private final SwerveModuleState[] moduleStates = new SwerveModuleState[4];
    private final SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];

    private final double maxSpeedMetersPerSecond;
    private final double maxAngularMetersPerSecond;

    /**
     * Temporary pose used by driveForwardMeters.
     * Computed at command start and stored here so it persists across execute() loops.
     */
    private Pose2d temporaryTargetPose = new Pose2d();

    // ------------------------------------------------------------
    // Simulated Data
    // ------------------------------------------------------------

    private double simHeadingRadians = 0.0;

    // ============================================================
    // Constructor
    // ============================================================

    public Swerve(
        SwerveModule1507 frontLeft,
        SwerveModule1507 frontRight,
        SwerveModule1507 backLeft,
        SwerveModule1507 backRight,
        Translation2d flLocation,
        Translation2d frLocation,
        Translation2d blLocation,
        Translation2d brLocation,
        Pigeon2 pigeon,
        double maxSpeedMetersPerSecond,
        double maxAngularMetersPerSecond,
        Matrix<N3, N1> odometryStdDevs,
        Matrix<N3, N1> visionStdDevs
    ) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
        this.pigeon = pigeon;
        this.maxSpeedMetersPerSecond = maxSpeedMetersPerSecond;
        this.maxAngularMetersPerSecond = maxAngularMetersPerSecond;

        this.yaw = pigeon.getYaw();

        this.kinematics = new SwerveDriveKinematics(
            flLocation, frLocation, blLocation, brLocation
        );

        this.poseEstimator = new SwerveDrivePoseEstimator(
            kinematics,
            getHeading(),
            getModulePositions(),
            pose,
            odometryStdDevs,
            visionStdDevs
        );

        Telemetry.set("Swerve/Initialized", true);
    }

    // ============================================================
    // Periodic
    // ============================================================

    @Override
    public void periodic() {
        if (RobotBase.isSimulation()) {
            // Refresh yaw signal so getHeading() picks up sim state writes
            BaseStatusSignal.refreshAll(yaw);
        } else {
            frontLeft.refreshSignals();
            frontRight.refreshSignals();
            backLeft.refreshSignals();
            backRight.refreshSignals();
            BaseStatusSignal.refreshAll(yaw);
        }

        pose = poseEstimator.update(
            getHeading(),
            getModulePositions()
        );

        Telemetry.set("Swerve", pose);
        Telemetry.set("Swerve/DriveStalled", isAnyDriveStalled());
        Telemetry.set("Swerve/SteerStalled", isAnySteerStalled());
        Telemetry.set("ModuleStates", getModuleStates());
    }

    @Override
    public void simulationPeriodic() {
        frontLeft.simulationUpdate(0.02);
        frontRight.simulationUpdate(0.02);
        backLeft.simulationUpdate(0.02);
        backRight.simulationUpdate(0.02);

        ChassisSpeeds speeds = kinematics.toChassisSpeeds(getModuleStates());

        // ADD THIS temporarily
        Telemetry.set("Swerve/Sim/OmegaRadsPerSec", speeds.omegaRadiansPerSecond);
        Telemetry.set("Swerve/Sim/VxMps", speeds.vxMetersPerSecond);
        Telemetry.set("Swerve/Sim/VyMps", speeds.vyMetersPerSecond);
        Telemetry.set("Swerve/Sim/HeadingDeg", Math.toDegrees(simHeadingRadians));

        simHeadingRadians += speeds.omegaRadiansPerSecond * 0.02;
        pigeon.getSimState().setRawYaw(simHeadingRadians * 180.0 / Math.PI);
    }

    // ============================================================
    // Low-Level Control
    //
    // These are the raw driving methods called by commands.
    // Students generally don't call these directly — use a command.
    // ============================================================
 
    /**
     * Drives using field-relative ChassisSpeeds (x = forward, y = left).
     * The kinematics layer converts these to individual module states.
     */
    public void drive(ChassisSpeeds speeds) {
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, maxSpeedMetersPerSecond);
 
        frontLeft.setDesiredState(states[0]);
        frontRight.setDesiredState(states[1]);
        backLeft.setDesiredState(states[2]);
        backRight.setDesiredState(states[3]);
    }
 
    /**
     * Drives using robot-relative ChassisSpeeds.
     * Used by movement commands that already handle the field→robot conversion.
     */
    public void driveRobotRelative(ChassisSpeeds speeds) {
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, maxSpeedMetersPerSecond);
 
        frontLeft.setDesiredState(states[0]);
        frontRight.setDesiredState(states[1]);
        backLeft.setDesiredState(states[2]);
        backRight.setDesiredState(states[3]);
    }
 
    /** Stops all modules immediately. */
    public void stop() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }
 
    /**
     * Sets all modules to the X configuration (FL/BR at +45°, FR/BL at -45°).
     * Creates a cross pattern that strongly resists pushing.
     * Called every loop by lockCommand() to hold the configuration continuously.
     */
    public void lock() {
        frontLeft.setDesiredState(new SwerveModuleState(0.0,  Rotation2d.fromDegrees(45)));
        frontRight.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45)));
        backLeft.setDesiredState(new SwerveModuleState(0.0,   Rotation2d.fromDegrees(-45)));
        backRight.setDesiredState(new SwerveModuleState(0.0,  Rotation2d.fromDegrees(45)));
    }

    // ============================================================
    // Observation
    // ============================================================

    public Pose2d getPose() {
        return pose;
    }

    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(
            yaw.getValue().in(Degrees)
        );
    }

    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    /**
     * Returns the robot's velocity in field-relative coordinates.
     * Used by maintainHeadingToTarget for motion compensation.
     */
    public ChassisSpeeds getFieldRelativeSpeeds() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(
            kinematics.toChassisSpeeds(getModuleStates()), getHeading()
        );
    }
 
    // Temporary pose storage for driveForwardMeters.
    public void setTemporaryTargetPose(Pose2d p) {
        this.temporaryTargetPose = p;
    }

    public Pose2d getTemporaryTargetPose() {
        return temporaryTargetPose;
    }

    public double getMaxSpeed() {
        return maxSpeedMetersPerSecond;
    }

    public double getMaxAngular() {
        return maxAngularMetersPerSecond;
    }

    // ============================================================
    // Vision & Localization
    // ============================================================

    public void addVisionMeasurement(
        Pose2d visionPose,
        double timestampSeconds,
        Matrix<N3, N1> stdDevs
    ) {
        poseEstimator.addVisionMeasurement(
            visionPose,
            timestampSeconds,
            stdDevs
        );
    }

    public void zeroHeading() {
        pigeon.setYaw(0.0);
    }

    public void resetPose(Pose2d pose) {
        this.pose = pose;
        poseEstimator.resetPosition(
            getHeading(),
            getModulePositions(),
            pose
        );
    }

    // ============================================================
    // Fault interpretation
    // ============================================================

    public boolean isAnyDriveStalled() {
        if (edu.wpi.first.wpilibj.RobotBase.isSimulation()) return false;

        return frontLeft.isDriveStalled()
            || frontRight.isDriveStalled()
            || backLeft.isDriveStalled()
            || backRight.isDriveStalled();
    }

    public boolean isAnySteerStalled() {
        if (edu.wpi.first.wpilibj.RobotBase.isSimulation()) return false;

        return frontLeft.isSteerStalled()
            || frontRight.isSteerStalled()
            || backLeft.isSteerStalled()
            || backRight.isSteerStalled();
    }

    // ============================================================
    // Internal Helpers
    //
    // Private math utilities shared across multiple commands.
    // ============================================================
 
    /**
     * Simple proportional heading controller.
     * Returns an angular velocity (rad/s) to steer from current → desired rotation.
     *
     * The error is wrapped to [-π, π] so the robot always takes the shortest path.
     * HEADING_KP (5.5) means 1 radian of error = 5.5 rad/s of correction.
     */
    private double computeOmega(Rotation2d current, Rotation2d desired) {
        double error = MathUtil.angleModulus(desired.minus(current).getRadians());
        return error * HEADING_KP;
    }
 
    /**
     * Computes the direction the robot must face to look directly at a target.
     * Returns a Rotation2d pointing from robot's current XY → target's XY.
     * The target's own rotation field is ignored.
     */
    private Rotation2d computeHeadingToTarget(Pose2d robot, Pose2d target) {
        double dx = target.getX() - robot.getX();
        double dy = target.getY() - robot.getY();
        return new Rotation2d(Math.atan2(dy, dx));
    }

    private SwerveModuleState[] getModuleStates() {
        moduleStates[0] = frontLeft.getState();
        moduleStates[1] = frontRight.getState();
        moduleStates[2] = backLeft.getState();
        moduleStates[3] = backRight.getState();
        return moduleStates;
    }

    private SwerveModulePosition[] getModulePositions() {
        modulePositions[0] = frontLeft.getPosition();
        modulePositions[1] = frontRight.getPosition();
        modulePositions[2] = backLeft.getPosition();
        modulePositions[3] = backRight.getPosition();
        return modulePositions;
    }

    // ╔══════════════════════════════════════════════════════════════════╗
    // ║                        COMMANDS                                  ║
    // ║                                                                  ║
    // ║  All swerve commands live here, directly on the subsystem.       ║
    // ║  They use run() / runOnce() / andThen() from SubsystemBase,      ║
    // ║  which automatically registers this subsystem as a requirement.  ║
    // ║                                                                  ║
    // ║  SECTIONS:                                                       ║
    // ║    1. Basic Drive (teleop)                                       ║
    // ║    2. Heading Control (pointing, aiming)                         ║
    // ║    3. Autonomous Movement (driveToPoint, moveThroughPose, etc.)  ║
    // ║    4. Utility (lock, resetPose)                                  ║
    // ╚══════════════════════════════════════════════════════════════════╝
 
 
    // ─────────────────────────────────────────────────────────────────
    // 1. BASIC DRIVE
    // ─────────────────────────────────────────────────────────────────
 
    /**
     * Default teleop drive command.
     * Accepts a ChassisSpeeds supplier so Robot.java can compute speeds
     * from controller inputs each loop iteration.
     *
     *   swerve.setDefaultCommand(swerve.driveCommand(() -> computeSpeeds()));
     */
    public Command driveCommand(Supplier<ChassisSpeeds> speeds) {
        return run(() -> drive(speeds.get()))
            .withName("Swerve.drive");
    }
 
    /**
     * Drives at fixed ChassisSpeeds. Used by auto commands.
     */
    public Command driveCommand(ChassisSpeeds speeds) {
        return run(() -> drive(speeds))
            .withName("Swerve.driveFixed");
    }
 
    /**
     * Drives at the given ChassisSpeeds for a fixed number of seconds, then stops.
     * Called by AutoBuilder.driveForTime().
     */
    public Command driveForTime(ChassisSpeeds speeds, double seconds) {
        return run(() -> drive(speeds))
            .withTimeout(seconds)
            .finallyDo(interrupted -> stop())
            .withName("Swerve.driveForTime");
    }
 
    /** Stops all modules. Instantaneous (runOnce). */
    public Command stopCommand() {
        return runOnce(this::stop)
            .withName("Swerve.stop");
    }
 
    /** Resets the robot's pose estimate to a given field position. */
    public Command resetPoseCommand(Pose2d pose) {
        return runOnce(() -> resetPose(pose))
            .withName("Swerve.resetPose");
    }

 
    // ─────────────────────────────────────────────────────────────────
    // 2. HEADING CONTROL
    // ─────────────────────────────────────────────────────────────────
 
    /**
     * Rotates the robot in place until it faces a fixed field position.
     *
     * The robot looks toward the XY of targetPose — the target's own rotation
     * is ignored. Finishes within HEADING_TOLERANCE_DEG (5°).
     *
     * Use case: snap to face the speaker/hub before shooting.
     *
     * @param targetPose  the field position to face toward
     */
    public Command pointToTarget(Pose2d targetPose) {
        return run(() -> {
            Pose2d current     = getPose();
            Rotation2d desired = computeHeadingToTarget(current, targetPose);
            drive(new ChassisSpeeds(0.0, 0.0,
                computeOmega(current.getRotation(), desired)));
        })
        .until(() -> {
            Pose2d current     = getPose();
            Rotation2d desired = computeHeadingToTarget(current, targetPose);
            return Math.abs(MathUtil.angleModulus(
                current.getRotation().minus(desired).getRadians()
            )) < Math.toRadians(HEADING_TOLERANCE_DEG);
        })
        .finallyDo(interrupted -> stop())
        .withName("Swerve.pointToTarget");
    }
 
    /**
     * Rotates the robot in place to continuously face a dynamic target.
     * The target pose is re-read from the supplier every loop.
     *
     * Use case: continuously track a moving vision target or live shooter setpoint.
     *
     * @param targetPoseSupplier  supplier that returns the current target pose
     */
    public Command pointToTarget(Supplier<Pose2d> targetPoseSupplier) {
        return run(() -> {
            Pose2d current     = getPose();
            Rotation2d desired = computeHeadingToTarget(current, targetPoseSupplier.get());
            drive(new ChassisSpeeds(0.0, 0.0,
                computeOmega(current.getRotation(), desired)));
        })
        .until(() -> {
            Pose2d current     = getPose();
            Rotation2d desired = computeHeadingToTarget(current, targetPoseSupplier.get());
            return Math.abs(MathUtil.angleModulus(
                current.getRotation().minus(desired).getRadians()
            )) < Math.toRadians(HEADING_TOLERANCE_DEG);
        })
        .finallyDo(interrupted -> stop())
        .withName("Swerve.pointToTargetDynamic");
    }
 
    /**
     * Rotates the robot to match the heading stored in a target Pose2d,
     * ignoring that pose's XY position entirely.
     *
     * Differs from pointToTarget: this snaps to a specific angle (e.g. 90°),
     * not a direction toward a field location. Useful for correcting heading
     * drift after moveThroughPose, or for aligning with a wall.
     *
     * Finishes within HEADING_TOLERANCE_DEG (5°).
     *
     * @param targetPose  the pose whose .getRotation() defines the desired heading
     */
    public Command changeHeading(Pose2d targetPose) {
        return run(() -> {
            double omega = computeOmega(getPose().getRotation(), targetPose.getRotation());
            drive(new ChassisSpeeds(0.0, 0.0, omega));
        })
        .until(() ->
            Math.abs(MathUtil.angleModulus(
                getPose().getRotation().minus(targetPose.getRotation()).getRadians()
            )) < Math.toRadians(HEADING_TOLERANCE_DEG)
        )
        .finallyDo(interrupted -> stop())
        .withName("Swerve.changeHeading");
    }
 
    /**
     * Drives with driver-supplied translation while automatically aiming at a target.
     *
     * Includes motion compensation — the aim leads the robot's velocity so projectiles
     * arrive correctly even while moving. Tune AIM_LEAD_TIME at the top of this file.
     *
     * This command NEVER finishes on its own. Bind with whileTrue() so it cancels
     * when the button is released, returning control to the default drive command.
     *
     * Use case: driver holds a button to auto-aim while keeping full translation control.
     *
     * @param targetPoseSupplier  field target to aim at (re-read every loop)
     * @param xSupplier           driver forward/back input (m/s, field-relative)
     * @param ySupplier           driver strafe input (m/s, field-relative)
     */
    public Command maintainHeadingToTarget(
        Supplier<Pose2d> targetPoseSupplier,
        Supplier<Double> xSupplier,
        Supplier<Double> ySupplier
    ) {
        return run(() -> {
            Pose2d currentPose  = getPose();
            Pose2d targetPose   = targetPoseSupplier.get();
            ChassisSpeeds field = getFieldRelativeSpeeds();
 
            // Shift the aim point forward in time to compensate for robot motion.
            // If the robot moves right at 2 m/s and lead time is 0.25 s,
            // the compensated target shifts 0.5 m right — correcting for
            // the time the game piece takes to travel to the target.
            Translation2d compensated = targetPose.getTranslation().minus(
                new Translation2d(
                    field.vxMetersPerSecond * AIM_LEAD_TIME,
                    field.vyMetersPerSecond * AIM_LEAD_TIME
                )
            );
 
            Rotation2d desiredHeading = compensated
                .minus(currentPose.getTranslation())
                .getAngle();
 
            double omega = computeOmega(currentPose.getRotation(), desiredHeading);
            drive(new ChassisSpeeds(xSupplier.get(), ySupplier.get(), omega));
        })
        .finallyDo(interrupted -> stop())
        .withName("Swerve.maintainHeadingToTarget");
    }
 
 
    // ─────────────────────────────────────────────────────────────────
    // 3. AUTONOMOUS MOVEMENT
    // ─────────────────────────────────────────────────────────────────
 
    /**
     * Drives toward a target pose at a fixed velocity and stops when within
     * ARRIVE_THRESHOLD (5 cm) of the target XY.
     *
     * Rotation toward the target pose's heading is corrected proportionally
     * throughout the move.
     *
     * @param targetPose  the field pose to drive to
     * @param velocity    translation speed (m/s)
     * @param stopAtEnd   true = stop when done; false = leave velocity applied (for chaining)
     */
    public Command driveToPoint(Pose2d targetPose, double velocity, boolean stopAtEnd) {
        return run(() -> {
            Pose2d current     = getPose();
            Rotation2d heading = current.getRotation();
 
            // Unit vector toward target
            double dx       = targetPose.getX() - current.getX();
            double dy       = targetPose.getY() - current.getY();
            double distance = Math.hypot(dx, dy);
 
            double ux = (distance > ARRIVE_THRESHOLD) ? dx / distance : 0.0;
            double uy = (distance > ARRIVE_THRESHOLD) ? dy / distance : 0.0;
 
            // Heading correction
            double headingError = MathUtil.angleModulus(
                targetPose.getRotation().minus(heading).getRadians()
            );
            double omega = headingError * 5.0;
 
            driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(
                ux * velocity, uy * velocity, omega, heading
            ));
        })
        .until(() ->
            getPose().getTranslation()
                     .getDistance(targetPose.getTranslation()) < ARRIVE_THRESHOLD
        )
        .finallyDo(interrupted -> { if (stopAtEnd) stop(); })
        .withName("Swerve.driveToPoint");
    }
 
    /**
     * Drives through a waypoint at constant speed without slowing down.
     *
     * Unlike driveToPoint, the command finishes as soon as the robot enters the
     * pass radius — there is no deceleration. This allows smooth, high-speed
     * paths through multiple chained waypoints.
     *
     * Rotation is controlled by a PID toward the target pose's heading.
     * Tune THETA_KP / KI / KD and MOVE_THROUGH_DEFAULT_RADIUS at the top of this file.
     *
     * Stall detection prevents the robot from getting permanently stuck if
     * it hits an obstacle — the command auto-exits after STALL_TIMEOUT seconds
     * of insufficient movement.
     *
     * @param targetPose  the waypoint pose to pass through
     * @param maxSpeed    translation speed (m/s)
     * @param maxAngular  maximum rotation speed (rad/s), clamps PID output
     * @param passRadius  distance to consider the waypoint "passed" (meters)
     */
    @SuppressWarnings("resource")
    public Command moveThroughPose(
        Pose2d targetPose,
        double maxSpeed,
        double maxAngular,
        double passRadius
    ) {
        PIDController thetaPID = new PIDController(THETA_KP, THETA_KI, THETA_KD);
        thetaPID.enableContinuousInput(-Math.PI, Math.PI);
 
        // Stall tracker: [lastX, lastY, lastMoveTimestamp]
        final double[] stall = new double[3];
 
        return runOnce(() -> {
            Pose2d p = getPose();
            stall[0] = p.getX();
            stall[1] = p.getY();
            stall[2] = Timer.getFPGATimestamp();
            thetaPID.reset();
        })
        .andThen(run(() -> {
            Pose2d current = getPose();
 
            // Normalized direction vector toward waypoint
            double dx       = targetPose.getX() - current.getX();
            double dy       = targetPose.getY() - current.getY();
            double distance = Math.hypot(dx, dy);
            double dirX     = dx / (distance + 1e-9);
            double dirY     = dy / (distance + 1e-9);
 
            // PID rotation toward target heading, clamped to maxAngular
            double omega = MathUtil.clamp(
                thetaPID.calculate(
                    current.getRotation().getRadians(),
                    targetPose.getRotation().getRadians()
                ),
                -maxAngular, maxAngular
            );
 
            driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(
                dirX * maxSpeed, dirY * maxSpeed, omega, current.getRotation()
            ));
 
            // Update stall tracker if the robot is actually moving
            double moveDx = Math.abs(current.getX() - stall[0]);
            double moveDy = Math.abs(current.getY() - stall[1]);
            if (moveDx > STALL_THRESHOLD || moveDy > STALL_THRESHOLD) {
                stall[0] = current.getX();
                stall[1] = current.getY();
                stall[2] = Timer.getFPGATimestamp();
            }
        }))
        .until(() -> {
            Pose2d current = getPose();
 
            // Done: robot entered pass radius
            if (current.getTranslation().getDistance(targetPose.getTranslation()) < passRadius)
                return true;
 
            // Done: stall timeout expired
            double dx = Math.abs(current.getX() - stall[0]);
            double dy = Math.abs(current.getY() - stall[1]);
            return dx < STALL_THRESHOLD
                && dy < STALL_THRESHOLD
                && (Timer.getFPGATimestamp() - stall[2]) > STALL_TIMEOUT;
        })
        .finallyDo(interrupted -> stop())
        .withName("Swerve.moveThroughPose");
    }
 
    /**
     * Convenience overload of moveThroughPose using the default pass radius.
     * Use this when you don't need to tune the radius per waypoint.
     */
    public Command moveThroughPose(Pose2d targetPose, double maxSpeed, double maxAngular) {
        return moveThroughPose(targetPose, maxSpeed, maxAngular, MOVE_THROUGH_DEFAULT_RADIUS);
    }
 
    /**
     * Drives forward a fixed distance along the robot's current heading.
     *
     * The target pose is computed from the robot's starting position and heading
     * at the moment the command initializes. Heading is held with proportional
     * correction throughout. Finishes within ARRIVE_THRESHOLD (5 cm).
     *
     * Use case: fallback auto when vision is offline — just drive a known distance.
     *
     * @param distanceMeters  distance to drive (meters, positive = forward)
     * @param velocity        translation speed (m/s)
     * @param stopAtEnd       true = stop when done
     */
    public Command driveForwardMeters(double distanceMeters, double velocity, boolean stopAtEnd) {
        return runOnce(() -> {
            // Capture starting pose and compute the target at command start
            Pose2d current  = getPose();
            Rotation2d hdg  = current.getRotation();
            setTemporaryTargetPose(new Pose2d(
                current.getX() + distanceMeters * hdg.getCos(),
                current.getY() + distanceMeters * hdg.getSin(),
                hdg
            ));
        })
        .andThen(run(() -> {
            Pose2d current = getPose();
            Pose2d target  = getTemporaryTargetPose();
            Rotation2d hdg = current.getRotation();
 
            double dx       = target.getX() - current.getX();
            double dy       = target.getY() - current.getY();
            double distance = Math.hypot(dx, dy);
 
            double ux = (distance > ARRIVE_THRESHOLD) ? dx / distance : 0.0;
            double uy = (distance > ARRIVE_THRESHOLD) ? dy / distance : 0.0;
 
            double headingError = MathUtil.angleModulus(
                target.getRotation().minus(hdg).getRadians()
            );
            double omega = headingError * 5.0;
 
            driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(
                ux * velocity, uy * velocity, omega, hdg
            ));
        }))
        .until(() ->
            getPose().getTranslation()
                     .getDistance(getTemporaryTargetPose().getTranslation()) < ARRIVE_THRESHOLD
        )
        .finallyDo(interrupted -> { if (stopAtEnd) stop(); })
        .withName("Swerve.driveForwardMeters");
    }
 
 
    // ─────────────────────────────────────────────────────────────────
    // 4. UTILITY
    // ─────────────────────────────────────────────────────────────────
 
    /**
     * Locks all modules in the X configuration to resist pushing.
     *
     * FL and BR point to +45°, FR and BL point to -45°. The crossed pattern
     * makes it very hard to push the robot out of position.
     *
     * The command runs continuously and must be interrupted to exit.
     * Bind with whileTrue() so the lock releases when the button is released.
     *
     * Use case: hold position while shooting, resist defense, stay on a slope.
     */
    public Command lockCommand() {
        return run(this::lock)
            .finallyDo(interrupted -> stop())
            .withName("Swerve.lock");
    }
}
