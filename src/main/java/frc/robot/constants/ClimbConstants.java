package frc.robot.constants;

public record ClimbConstants
    (
        int climbMotorID
    )
{
    // please for the love of god, do not froget to change these
    public static final ClimbConstants DEFAULT = new ClimbConstants
    (
        0
    );
}
