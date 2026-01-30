package frc.robot.constants;

public record OperatorConstants(
    double deadband,
    double leftYDeadband,
    double rightXDeadband,
    double turnConstant
) {
    public static final OperatorConstants DEFAULT = new OperatorConstants(
        0.01,    // deadband
        0.01,    // leftYDeadband
        0.01,    // rightXDeadband
        6.0     // turnConstant
    );
} 