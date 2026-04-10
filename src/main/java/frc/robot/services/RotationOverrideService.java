package frc.robot.services;

import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Common interface to override the robot's rotation while driving.
 */
public class RotationOverrideService {
    private AtomicReference<Double> rotationOverride = new AtomicReference<>();

    /**
     * Set the rotation override.
     * 
     * @param override The rotation override for the drive system, as given to
     *                 {@link SwerveInputStream}.
     */
    public void setOverride(double override) {
        rotationOverride.set(override);
    }

    /**
     * Stop the rotation override.
     */
    public void clearOverride() {
        rotationOverride.set(null);
    }

    /**
     * Get the rotation override.
     * 
     * @return The rotation override, or Optional.empty() if no override is set.
     */
    public Optional<Double> getOverride() {
        var override = rotationOverride.get();
        if (override == null) {
            return Optional.empty();
        }

        return Optional.of(override);
    }
}
