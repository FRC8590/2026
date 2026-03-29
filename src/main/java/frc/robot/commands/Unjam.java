package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Belt;
import frc.robot.subsystems.Indexer;

public class Unjam extends Command {
    private final Belt beltSystem;
    private final Indexer indexerSystem;

    public Unjam(Belt belt, Indexer indexer) {
        beltSystem = belt;
        indexerSystem = indexer;
        addRequirements(belt, indexer);
    }

    @Override
    public void execute() {
        beltSystem.runReversed();
        indexerSystem.runReversed();
    }

    @Override
    public void end(boolean interrupted) {
        beltSystem.stop();
        indexerSystem.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
