package frc.robot.commands.feeder;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.feeder.Belt;
import frc.robot.subsystems.feeder.Indexer;
import lib.woodsonrobotics.SystemWrapper;

/**
 * Feed fuel into the shooter.
 * 
 * This will run both the belt and indexer at a preset speed.
 * This does not wait on the shooter at all! If the shooter
 * isn't at the correct speed when this is executed, then feeding
 * will still take place!
 * 
 * Both the indexer and belt will be stopped when this is finished.
 */
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
        beltSystem.ifEnabled(belt -> belt.run());
        indexerSystem.ifEnabled(indexer -> indexer.run());
    }

    @Override
    public void end(boolean interrupted) {
        beltSystem.ifEnabled(belt -> belt.stop());
        indexerSystem.ifEnabled(indexer -> indexer.stop());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
