package frc.robot.constants;

public record IntakeConstants
    (
        int pivotMotorID,
        int intakeMotorID,
        IntakePIDConstants PID
    )
{
    // This will probably make the intake explode or something
    public record IntakePIDConstants
    (
        double kP,
        double kI,
        double kD
    )
    {}
    
    // please for the love of god, do not forget to change these
    public static final IntakeConstants DEFAULT = new IntakeConstants
    (
        0,
        0,
        new IntakePIDConstants(1, 1, 1)
    );
}
