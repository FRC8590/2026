package frc.robot.constants;

public record BeltConstants
    (
        int beltMotorID
    )
{
    // please for the love of god, do not froget to change these
    public static final BeltConstants DEFAULT = new BeltConstants
    (
        0
    );
}