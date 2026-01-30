package frc.robot.constants;

public record DriveConstants(
    double wheelLockTime,
    double maxTranslationalSpeed,
    double maxRotationalSpeed,
    PIDConstants translationPID,
    PIDConstants rotationPID
) {
    public record PIDConstants(
        double kP,
        double kI,
        double kD
    ) {}

    public static final DriveConstants DEFAULT = new DriveConstants(
        10.0,   // wheelLockTime (seconds)
        4.0,    // maxTranslationalSpeed (m/s)
        2 * Math.PI, // maxRotationalSpeed (rad/s)
        new PIDConstants(0.7, 0, 0),      // translationPID
        new PIDConstants(0.4, 0, 0.01)    // rotationPID
    );
} 