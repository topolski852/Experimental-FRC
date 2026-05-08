//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╔╝╚═╝  ╚═╝╚══════╝
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.lib.core.vision;

import java.util.function.Consumer;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

import org.team1507.lib.core.framework.Subsystem1507;
import org.team1507.lib.core.logging.Telemetry;

/**
 * Reads pose data from a Meta Quest running QuestNav and feeds it into
 * the swerve drive's pose estimator.
 *
 * <p>QuestNav is a Meta Quest VR headset running software that tracks the
 * robot's position on the field using visual SLAM. This subsystem wraps the
 * official {@link QuestNav} library to integrate with Team 1507's swerve drive.
 *
 * <h2>Setup — add the vendor library first</h2>
 * <ol>
 *   <li>In VS Code, open the WPILib Command Palette (Ctrl+Shift+P).</li>
 *   <li>Select <b>WPILib: Manage Vendor Libraries → Install new libraries (online)</b>.</li>
 *   <li>Paste the QuestNav vendor URL:
 *       {@code https://maven.questnav.gg/releases/gg/questnav/questnavlib-java/latest/questnavlib.json}</li>
 * </ol>
 *
 * <h2>Setup — wiring in Robot.java</h2>
 * <pre>
 *   // After creating the swerve subsystem, create the QuestNav subsystem.
 *   // The :: syntax passes swerve's methods as callbacks (method references).
 *   questNav = new QuestNavSubsystem(
 *       swerve::addVisionMeasurement,   // called each loop with fresh pose data
 *       swerve::resetPose               // called to snap odometry to Quest pose
 *   );
 *
 *   // Bind the pose reset to a button (press before auto or when placing the robot):
 *   driver.start().onTrue(questNav.resetPoseFromQuestCommand());
 * </pre>
 *
 * <h2>Mounting offset</h2>
 * <p>The Quest reports its OWN pose, not the robot's center pose. If the Quest is
 * not mounted at the robot's center, you must supply a mounting offset so this
 * subsystem can convert Quest pose → robot center pose. The offset is a
 * {@link Transform3d} from the robot center to the Quest's mounting point.
 * If the Quest is roughly at the center, leave it as {@code Transform3d.kZero}.
 */
public final class QuestNavSubsystem extends Subsystem1507 {

    // ============================================================
    // Tuning
    // ============================================================

    /**
     * Confidence for vision measurements when the Quest is tracking well.
     * Format: [x (m), y (m), heading (rad)] — lower values = trust vision more.
     * Tune these on the actual field. Start conservative (0.5) and decrease if tracking is good.
     */
    private static final double[] STD_DEVS = { 0.1, 0.1, 0.05 };

    // ============================================================
    // Callbacks passed in from Robot.java
    // ============================================================

    /** Feeds a vision pose into the drivetrain's pose estimator. */
    private final VisionConsumer onMeasurement;

    /** Resets the robot's odometry pose directly. */
    private final Consumer<Pose2d> onReset;

    // ============================================================
    // QuestNav library instance
    // ============================================================

    private final QuestNav questNav = new QuestNav();

    /**
     * Transform from robot center to the Quest's physical mounting point on the robot.
     * Used to convert the Quest's reported pose into the robot's center pose.
     * Leave as {@code Transform3d.kZero} if the Quest is mounted at (or very close to)
     * the robot's center of rotation.
     */
    private final Transform3d mountingOffset;

    // ============================================================
    // Constructor
    // ============================================================

    /**
     * Creates a QuestNav subsystem assuming the Quest is mounted at the robot's center.
     *
     * @param onMeasurement called each loop with a fresh pose estimate — pass
     *                      {@code swerve::addVisionMeasurement}
     * @param onReset       called when resetting odometry from the Quest — pass
     *                      {@code swerve::resetPose}
     */
    public QuestNavSubsystem(VisionConsumer onMeasurement, Consumer<Pose2d> onReset) {
        this(onMeasurement, onReset, Transform3d.kZero);
    }

    /**
     * Creates a QuestNav subsystem with a mounting offset from the robot's center.
     *
     * @param onMeasurement  called each loop with a fresh pose estimate — pass
     *                       {@code swerve::addVisionMeasurement}
     * @param onReset        called when resetting odometry from the Quest — pass
     *                       {@code swerve::resetPose}
     * @param mountingOffset transform from robot center to the Quest's mounting point on the robot
     */
    public QuestNavSubsystem(VisionConsumer onMeasurement, Consumer<Pose2d> onReset,
                             Transform3d mountingOffset) {
        super("QuestNav");
        this.onMeasurement  = onMeasurement;
        this.onReset        = onReset;
        this.mountingOffset = mountingOffset;

        // Register state-change callbacks on the QuestNav library instance
        questNav.onConnected(
            () -> System.out.println("[QuestNav] Connected to Quest headset."));
        questNav.onDisconnected(
            () -> DriverStation.reportWarning("[QuestNav] Quest headset disconnected!", false));
        questNav.onTrackingLost(
            () -> DriverStation.reportWarning("[QuestNav] Tracking lost — check Quest view.", false));
        questNav.onTrackingAcquired(
            () -> System.out.println("[QuestNav] Tracking acquired."));
        questNav.onLowBattery(
            20,
            level -> DriverStation.reportWarning("[QuestNav] Quest battery low: " + level + "%", false));
    }

    // ============================================================
    // Periodic — runs every robot loop
    // ============================================================

    @Override
    public void periodic() {
        // Must be called every loop to process command responses and fire callbacks
        questNav.commandPeriodic();

        // Publish debug info to SmartDashboard / AdvantageScope
        Telemetry.set("QuestNav/Connected", questNav.isConnected());
        Telemetry.set("QuestNav/Tracking",  questNav.isTracking());
        questNav.getBatteryPercent().ifPresent(
            b -> Telemetry.set("QuestNav/Battery", (double) b));

        // Skip vision updates if the Quest is not reliably tracking
        if (!questNav.isConnected() || !questNav.isTracking()) {
            return;
        }

        // Process all new frames received since the last loop
        for (PoseFrame frame : questNav.getAllUnreadPoseFrames()) {
            // Convert Quest's 3D pose to the robot center's 2D pose on the field.
            // If the Quest is at the center (default), this is a no-op transformation.
            Pose3d robotPose3d = questPoseToRobotPose(frame.questPose3d());
            Pose2d robotPose2d = robotPose3d.toPose2d();

            // Use dataTimestamp — this is the NT reception time, which is what
            // the SwerveDrivePoseEstimator expects for accurate interpolation.
            onMeasurement.addMeasurement(
                robotPose2d,
                frame.dataTimestamp(),
                VecBuilder.fill(STD_DEVS[0], STD_DEVS[1], STD_DEVS[2])
            );

            Telemetry.set("QuestNav/Pose", robotPose2d);
        }
    }

    // ============================================================
    // Commands
    // ============================================================

    /**
     * Resets robot odometry to the current pose reported by the Quest, and tells
     * the Quest headset where it is on the field.
     *
     * <p>Run this before autonomous or when placing the robot at a known field location.
     * The Quest must already be tracking for this to produce a valid pose.
     *
     * <p>Bind it to a button:
     * {@code driver.start().onTrue(questNav.resetPoseFromQuestCommand());}
     *
     * @return a one-shot command that snaps odometry to the Quest's current pose
     */
    public Command resetPoseFromQuestCommand() {
        return Commands.runOnce(() -> {
            if (!questNav.isConnected()) {
                DriverStation.reportWarning("[QuestNav] Cannot reset pose — Quest not connected.", false);
                return;
            }

            // Read the most recent single frame for the reset
            PoseFrame[] frames = questNav.getAllUnreadPoseFrames();
            if (frames.length == 0) {
                DriverStation.reportWarning("[QuestNav] Cannot reset pose — no frame data available.", false);
                return;
            }

            PoseFrame latest = frames[frames.length - 1];
            Pose3d robotPose3d = questPoseToRobotPose(latest.questPose3d());
            Pose2d robotPose2d = robotPose3d.toPose2d();

            // Reset the robot's internal odometry
            onReset.accept(robotPose2d);

            // Tell the Quest where it is on the field so it can continue tracking
            // in the field coordinate system
            questNav.setPose(robotPose3d.transformBy(mountingOffset.inverse()));

            System.out.printf("[QuestNav] Pose reset → x=%.2f, y=%.2f, heading=%.1f°%n",
                robotPose2d.getX(), robotPose2d.getY(),
                robotPose2d.getRotation().getDegrees());
        }).withName("QuestNav.resetPose");
    }

    // ============================================================
    // Accessors
    // ============================================================

    /**
     * Returns true when the Quest headset is connected to the robot.
     * Check this before relying on vision for auto-targeting or pose resets.
     */
    public boolean isConnected() {
        return questNav.isConnected();
    }

    /**
     * Returns true when the Quest is actively tracking its position on the field.
     * Vision updates are paused automatically when tracking is lost.
     */
    public boolean isTracking() {
        return questNav.isTracking();
    }

    // ============================================================
    // Internal helpers
    // ============================================================

    /**
     * Converts the Quest's reported pose (at the Quest's mounting point) to the
     * robot's center pose using the configured mounting offset.
     */
    private Pose3d questPoseToRobotPose(Pose3d questPose) {
        // If mountingOffset is zero (default), this returns questPose unchanged.
        // Otherwise it applies the inverse transform to go from Quest space → robot center.
        return questPose.transformBy(mountingOffset.inverse());
    }
}
