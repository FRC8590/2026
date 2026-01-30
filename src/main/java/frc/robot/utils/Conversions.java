package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.Constants.ScoreLocation;

public class Conversions {
    /**
     * Converts meters per second to RPM
     * @param mps Meters per second
     * @param gearRatio Gear ratio between motor and mechanism
     * @param circumference Circumference of wheel/pulley in meters
     */

    /**
     * Converts an AprilTag ID and scoring location into a field-relative Pose2d for scoring.
     * Uses the AprilTagFieldLayout to determine the base position and applies appropriate offsets.
     * 
     * @param apriltagID The ID of the AprilTag near the desired scoring location (1-5)
     * @param scoreLocation The desired scoring position (LEFT or RIGHT) relative to the AprilTag
     * @return A Pose2d representing the scoring position and rotation in field coordinates
     * @throws IllegalArgumentException if the AprilTag ID is invalid or the pose cannot be calculated
     */
    public static Pose2d scoringGoalsToPose(int apriltagID, ScoreLocation scoreLocation) {
        try {
            // Get the AprilTag position from the field layout
            Translation2d tagPose = Constants.layout.getTagPose(apriltagID).get().getTranslation().toTranslation2d();
            
            // Calculate the scoring position offset
            double xOffset = 0.5; // Distance forward/back from tag
            double yOffset = switch (scoreLocation) {
                case LEFT2 -> 0.55;  // Distance left from tag
                case LEFT3 -> 0.55;  // Distance left from tag
                case RIGHT2 -> -0.55; // Distance right from tag
                case RIGHT3 -> -0.55; // Distance right from tag
                default -> throw new IllegalArgumentException("Invalid score location: " + scoreLocation);
            };

            // Create the scoring position by adding offsets to tag position
            Translation2d scoringPosition = new Translation2d(
                tagPose.getX() + xOffset,
                tagPose.getY() + yOffset
            );

            // Return pose with 180 degree rotation to face the scoring location
            return new Pose2d(scoringPosition, new Rotation2d(Math.PI));
        } catch (Exception e) {
            throw new IllegalArgumentException("Failed to get pose for AprilTag ID: " + apriltagID, e);
        }
    }
}