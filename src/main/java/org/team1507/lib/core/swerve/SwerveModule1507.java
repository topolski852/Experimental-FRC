//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.lib.core.swerve;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.sim.CANcoderSimState;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotBase;

import org.team1507.lib.core.impl.ctre.Motor1507;
import org.team1507.lib.core.logging.Telemetry;

public final class SwerveModule1507 {

    public record MathConfig(
        double driveGearRatio,
        double steerGearRatio,
        double couplingRatio,
        double wheelRadiusMeters
    ) {
        public double wheelCircumferenceMeters() {
            return 2.0 * Math.PI * wheelRadiusMeters;
        }
    }

    private final String name;

    private final Motor1507 drive;
    private final Motor1507 steer;
    private final CANcoder encoder;

    private final Rotation2d encoderOffset;
    private final MathConfig math;
    private final double driveMetersScale;

    private Rotation2d lastAngle = Rotation2d.kZero;

    // Sim state
    private double simSteerAngleRotations = 0.0;
    private double simDriveVelocityRps = 0.0;
    private double simDrivePositionRotations = 0.0;

    private final StatusSignal<Angle> absPosition;
    private final StatusSignal<AngularVelocity> azimuthVelocity;

    private final BaseStatusSignal[] allSignals;

    public SwerveModule1507(
        String name,
        Motor1507 drive,
        Motor1507 steer,
        CANcoder encoder,
        Rotation2d encoderOffset,
        MathConfig math,
        double driveMetersScale
    ) {
        this.name = name;
        this.drive = drive;
        this.steer = steer;
        this.encoder = encoder;
        this.encoderOffset = encoderOffset;
        this.math = math;
        this.driveMetersScale = driveMetersScale;

        this.absPosition = encoder.getAbsolutePosition();
        this.azimuthVelocity = encoder.getVelocity();

        BaseStatusSignal.setUpdateFrequencyForAll(
            100.0,
            absPosition,
            azimuthVelocity
        );

        BaseStatusSignal[] driveSignals = drive.getSignals(); // 6
        BaseStatusSignal[] steerSignals = steer.getSignals(); // 6
        this.allSignals = new BaseStatusSignal[14];
        System.arraycopy(driveSignals, 0, allSignals, 0, 6);
        System.arraycopy(steerSignals, 0, allSignals, 6, 6);
        allSignals[12] = absPosition;
        allSignals[13] = azimuthVelocity;

        Telemetry.set(key("Drive/MetersScale"), driveMetersScale);
        Telemetry.set(key("Initialized"), true);
    }

    // ============================================================
    // Control
    // ============================================================

    public void setDesiredState(SwerveModuleState desired) {
        Rotation2d current = getAngle();
        SwerveModuleState optimized = new SwerveModuleState(
            desired.speedMetersPerSecond,
            desired.angle
        );

        optimized.optimize(current);

        Rotation2d targetAngle =
            Math.abs(optimized.speedMetersPerSecond) < 0.01
                ? lastAngle
                : optimized.angle;

        double driveRps =
            metersPerSecondToDriveMotorRps(optimized.speedMetersPerSecond);

        drive.setVelocityRPS(driveRps);

        steer.setPositionVoltage(
            targetAngle.getRotations(),
            0.0
        );

        // In sim, instantly snap to the target so getAngle() returns
        // the correct value next loop — simulates a perfect steer controller
        if (RobotBase.isSimulation()) {
            simSteerAngleRotations = targetAngle.getRotations();
            simDriveVelocityRps = driveRps / math.driveGearRatio();
        }

        lastAngle = targetAngle;

        Telemetry.set(key("Drive/TargetMps"), optimized.speedMetersPerSecond);
        Telemetry.set(key("Drive/TargetMotorRps"), driveRps);
        Telemetry.set(key("Steer/TargetAngleRad"), targetAngle.getRadians());
    }

    public void stop() {
        drive.stop();
        steer.stop();

        if (RobotBase.isSimulation()) {
            simDriveVelocityRps = 0.0;
        }
    }

    // ============================================================
    // Simulation Update
    // ============================================================

    /**
     * Called every loop from Swerve.simulationPeriodic().
     * Updates the CANcoder sim state so getAngle() returns real values,
     * and integrates drive position.
     */
    public void simulationUpdate(double dtSeconds) {
        // Update CANcoder sim state so absPosition signal reads correctly
        CANcoderSimState encoderSim = encoder.getSimState();
        encoderSim.setRawPosition(
            simSteerAngleRotations + encoderOffset.getRotations()
        );
        encoderSim.setVelocity(0.0);

        // Integrate drive position
        simDrivePositionRotations += simDriveVelocityRps * dtSeconds;
    }

    // ============================================================
    // Observation
    // ============================================================

    public Rotation2d getAngle() {
        if (RobotBase.isSimulation()) {
            return Rotation2d.fromRotations(simSteerAngleRotations);
        }
        double rotations = absPosition.getValue().in(Rotations);
        return Rotation2d.fromRotations(rotations).minus(encoderOffset);
    }

    private double azimuthRotationsRaw() {
        if (RobotBase.isSimulation()) {
            return simSteerAngleRotations + encoderOffset.getRotations();
        }
        return absPosition.getValue().in(Rotations);
    }

    private double azimuthRpsRaw() {
        return RobotBase.isSimulation() ? 0.0
            : azimuthVelocity.getValue().in(RotationsPerSecond);
    }

    public SwerveModuleState getState() {
        double mps = RobotBase.isSimulation()
            ? simDriveVelocityRps * math.wheelCircumferenceMeters()
            : wheelRpsToMetersPerSecond(correctedWheelRps()) * driveMetersScale;

        return new SwerveModuleState(mps, getAngle());
    }

    public SwerveModulePosition getPosition() {
        double meters = RobotBase.isSimulation()
            ? simDrivePositionRotations * math.wheelCircumferenceMeters()
            : wheelRotationsToMeters(correctedWheelRotations()) * driveMetersScale;

        return new SwerveModulePosition(meters, getAngle());
    }

    public BaseStatusSignal[] getAllSignals() {
        return allSignals;
    }

    /** @deprecated Replaced by {@link #getAllSignals()} — Swerve now batches all signals in one call. */
    @Deprecated
    public void refreshSignals() {
        // no-op — Swerve.periodic() calls BaseStatusSignal.refreshAll(allSignals) for all modules
    }

    // ============================================================
    // CAN Bus Access (for bus optimization only)
    // ============================================================

    /** Returns the drive motor's Phoenix 6 device handle. Used to optimize CAN bus utilization. */
    public com.ctre.phoenix6.hardware.ParentDevice getDriveDevice() { return drive.getDevice(); }

    /** Returns the steer motor's Phoenix 6 device handle. Used to optimize CAN bus utilization. */
    public com.ctre.phoenix6.hardware.ParentDevice getSteerDevice() { return steer.getDevice(); }

    /** Returns the CANcoder's Phoenix 6 device handle. Used to optimize CAN bus utilization. */
    public com.ctre.phoenix6.hardware.ParentDevice getEncoderDevice() { return encoder; }

    // ============================================================
    // Faults
    // ============================================================

    public boolean isDriveStalled() {
        return drive.isStalled();
    }

    public boolean isSteerStalled() {
        return steer.isStalled();
    }

    // ============================================================
    // Core math
    // ============================================================

    private double correctedDriveMotorRotations() {
        return drive.getRotorPosition()
            - (azimuthRotationsRaw() * math.couplingRatio());
    }

    private double correctedDriveMotorRps() {
        return drive.getRotorVelocity()
            - (azimuthRpsRaw() * math.couplingRatio());
    }

    private double correctedWheelRotations() {
        return correctedDriveMotorRotations() / math.driveGearRatio();
    }

    private double correctedWheelRps() {
        return correctedDriveMotorRps() / math.driveGearRatio();
    }

    private double wheelRotationsToMeters(double wheelRotations) {
        return wheelRotations * math.wheelCircumferenceMeters();
    }

    private double wheelRpsToMetersPerSecond(double wheelRps) {
        return wheelRps * math.wheelCircumferenceMeters();
    }

    private double metersPerSecondToDriveMotorRps(double mps) {
        double wheelRps = mps / math.wheelCircumferenceMeters();
        return (wheelRps * math.driveGearRatio())
            + (azimuthRpsRaw() * math.couplingRatio());
    }

    private String key(String field) {
        return "Swerve/" + name + "/" + field;
    }
}
