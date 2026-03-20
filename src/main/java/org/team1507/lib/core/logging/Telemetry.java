//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.lib.core.logging;

import edu.wpi.first.wpilibj.Timer;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import java.util.Map;

/**
 * Centralized telemetry publisher.
 *
 * <p>{@code Telemetry} is responsible for periodically publishing registered
 * {@link InputField}s according to their declared {@link TelemetryRate}.
 *
 * <p>This class owns all scheduling and timing logic. Value production and
 * NetworkTables interaction are handled by {@link InputField}.
 */
public final class Telemetry {

    /**
     * Logging period (in seconds) associated with each telemetry rate.
     */
    private static final Map<TelemetryRate, Double> PERIOD_SEC = Map.of(
        TelemetryRate.FAST,   0.02, // 50 Hz
        TelemetryRate.NORMAL, 0.10, // 10 Hz
        TelemetryRate.SLOW,   0.50  // 2 Hz
    );

    /**
     * Registered telemetry fields, grouped by logging rate.
     */
    private static final Map<TelemetryRate, List<InputField<?>>> fields =
        new EnumMap<>(TelemetryRate.class);

    /**
     * Timestamp of the last update for each telemetry rate.
     */
    private static final Map<TelemetryRate, Double> lastUpdateTime =
        new EnumMap<>(TelemetryRate.class);

    private Telemetry() {}

    /**
     * Registers an {@link InputField} for telemetry publishing.
     *
     * @param field the telemetry field to register
     */
    public static void register(InputField<?> field) {
        fields
            .computeIfAbsent(field.getRate(), r -> new ArrayList<>())
            .add(field);
    }

    /**
     * Registers multiple {@link InputField}s for telemetry publishing.
     *
     * <p>This is a convenience overload that allows a group of telemetry fields
     * to be registered in a single call. Each field is registered according to
     * its declared {@link TelemetryRate}.
     *
     * @param fields the telemetry fields to register
     */
    public static void register(InputField<?>... fields) {
        for (InputField<?> field : fields) {
            register(field);
        }
    }

    /**
     * Publishes registered telemetry fields whose logging period has elapsed.
     *
     * <p>This method should be called periodically (e.g. from
     * {@code robotPeriodic}). Each telemetry rate is evaluated independently.
     */
    public static void update() {
        double now = Timer.getFPGATimestamp();

        for (TelemetryRate rate : TelemetryRate.values()) {
            double last = lastUpdateTime.getOrDefault(rate, 0.0);
            double period = PERIOD_SEC.get(rate);

            if (now - last < period) {
                continue;
            }

            lastUpdateTime.put(rate, now);

            List<InputField<?>> rateFields = fields.get(rate);
            if (rateFields == null) {
                continue;
            }

            for (InputField<?> field : rateFields) {
                field.publish();
            }
        }
    }
}
