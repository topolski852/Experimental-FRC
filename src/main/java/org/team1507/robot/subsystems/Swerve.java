//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.Supplier;

import org.team1507.lib.core.logging.Telemetry;
import org.team1507.lib.core.swerve.SwerveModule1507;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;

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

    // ------------------------------------------------------------
    // Simulated Data
    // ------------------------------------------------------------

    private double simHeadingRadians = 0.0;

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
        Matrix<N3, N1> odometryStdDevs,
        Matrix<N3, N1> visionStdDevs
    ) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
        this.pigeon = pigeon;
        this.maxSpeedMetersPerSecond = maxSpeedMetersPerSecond;

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
    // Control
    // ============================================================

    public void drive(ChassisSpeeds speeds) {
        SwerveModuleState[] states =
            kinematics.toSwerveModuleStates(speeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(
            states, maxSpeedMetersPerSecond
        );

        frontLeft.setDesiredState(states[0]);
        frontRight.setDesiredState(states[1]);
        backLeft.setDesiredState(states[2]);
        backRight.setDesiredState(states[3]);
    }

    public void stop() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    // ============================================================
    // Commands
    // ============================================================

    public Command driveCommand(Supplier<ChassisSpeeds> speeds) {
        return run(() -> drive(speeds.get()))
            .withName("Swerve.drive()");
    }

    public Command driveCommand(ChassisSpeeds speeds) {
        return run(() -> drive(speeds))
            .withName("Swerve.drive()");
    }

    public Command stopCommand() {
        return runOnce(this::stop)
            .withName("Swerve.stop()");
    }
    
    public Command resetPoseCommand(Pose2d pose) {
        return runOnce(() -> resetPose(pose))
            .withName("Swerve.resetPose()");
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

    // ============================================================
    // Vision
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
        if (edu.wpi.first.wpilibj.RobotBase.isSimulation()) {
            return false;
        }

        return frontLeft.isDriveStalled()
            || frontRight.isDriveStalled()
            || backLeft.isDriveStalled()
            || backRight.isDriveStalled();
    }

    public boolean isAnySteerStalled() {
        if (edu.wpi.first.wpilibj.RobotBase.isSimulation()) {
            return false;
        }

        return frontLeft.isSteerStalled()
            || frontRight.isSteerStalled()
            || backLeft.isSteerStalled()
            || backRight.isSteerStalled();
    }

    // ============================================================
    // Internal helpers
    // ============================================================

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
}
