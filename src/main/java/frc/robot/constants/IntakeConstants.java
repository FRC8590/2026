package frc.robot.constants;

public record IntakeConstants
    (
        int pivotMotorID,
        int intakeMotorID
    )
{
    // please for the love of god, do not froget to change these
    public static final IntakeConstants DEFAULT = new IntakeConstants
    (
        0,
        0
    );
}
