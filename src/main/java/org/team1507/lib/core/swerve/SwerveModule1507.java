package org.team1507.lib.core.swerve;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

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

    private final StatusSignal<Angle> absPosition;
    private final StatusSignal<AngularVelocity> azimuthVelocity;

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
            targetAngle.getRotations() * math.steerGearRatio(),
            0.0
        );

        lastAngle = targetAngle;

        Telemetry.set(key("Drive/TargetMps"), optimized.speedMetersPerSecond);
        Telemetry.set(key("Drive/TargetMotorRps"), driveRps);
        Telemetry.set(key("Steer/TargetAngleRad"), targetAngle.getRadians());
    }

    public void stop() {
        drive.stop();
        steer.stop();
    }

    // ============================================================
    // Observation
    // ============================================================

    public Rotation2d getAngle() {
        double rotations = absPosition.getValue().in(Rotations);
        return Rotation2d.fromRotations(rotations).minus(encoderOffset);
    }

    private double azimuthRotationsRaw() {
        return absPosition.getValue().in(Rotations);
    }

    private double azimuthRpsRaw() {
        return azimuthVelocity.getValue().in(RotationsPerSecond);
    }

    public SwerveModuleState getState() {
        double mps =
            wheelRpsToMetersPerSecond(correctedWheelRps())
            * driveMetersScale;

        return new SwerveModuleState(mps, getAngle());
    }

    public SwerveModulePosition getPosition() {
        double meters =
            wheelRotationsToMeters(correctedWheelRotations())
            * driveMetersScale;

        return new SwerveModulePosition(meters, getAngle());
    }

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
