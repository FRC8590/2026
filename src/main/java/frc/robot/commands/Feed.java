package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Belt;
import frc.robot.subsystems.Indexer;

public class Feed extends Command {
    private final Belt beltSystem;
    private final Indexer indexerSystem;

    public Feed(Belt belt, Indexer indexer) {
        beltSystem = belt;
        indexerSystem = indexer;
        addRequirements(belt, indexer);
    }

    @Override
    public void execute() {
        beltSystem.run();
        indexerSystem.run();
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
