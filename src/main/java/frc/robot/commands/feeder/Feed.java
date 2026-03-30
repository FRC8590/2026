package frc.robot.commands.feeder;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Belt;
import frc.robot.subsystems.Indexer;
import lib.woodsonrobotics.SystemWrapper;

public class Feed extends Command {
    private final SystemWrapper<Belt> beltSystem;
    private final SystemWrapper<Indexer> indexerSystem;

    public Feed(SystemWrapper<Belt> belt, SystemWrapper<Indexer> indexer) {
        beltSystem = belt;
        indexerSystem = indexer;
        addRequirements(belt, indexer);
    }

    @Override
    public void execute() {
        beltSystem.get().ifPresent((belt) -> belt.run());
        indexerSystem.get().ifPresent((indexer) -> indexer.run());
    }

    @Override
    public void end(boolean interrupted) {
        beltSystem.get().ifPresent((belt) -> belt.stop());
        indexerSystem.get().ifPresent((indexer) -> indexer.stop());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
