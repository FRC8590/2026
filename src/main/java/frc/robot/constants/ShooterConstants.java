package frc.robot.constants;

public record ShooterConstants
    (
        int frontMotorID,
        int backMotorID
    )
{   
    // please for the love of god, do not forget to change these
    public static final ShooterConstants DEFAULT = new ShooterConstants
    (
        0,
        0
    );
}
