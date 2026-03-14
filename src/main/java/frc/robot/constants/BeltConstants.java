package frc.robot.constants;

public record BeltConstants(
        int beltMotorID, int indexMotorID) {
    // please for the love of god, do not forget to change these
    public static final BeltConstants DEFAULT = new BeltConstants(
            11, 14);
}