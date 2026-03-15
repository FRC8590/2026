package frc.robot.commands;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class BeltRun extends ParallelCommandGroup {
    public BeltRun() {
        addCommands(
                Constants.belt.runBelt());

        addRequirements(getRequirements());
    }

}
