package frc.robot.commands.feeder;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Belt;
import frc.robot.subsystems.Indexer;

import lib.woodsonrobotics.SystemWrapper;

/*
 * Unjam the feeder.
 * 
 * This runs both the belt and indexer in reverse in hopes that any
 * stuck fuel will be made unstuck.
 * 
 * Both the belt and indexer will be stopped when this is finished.
 * 
*/
public class Unjam extends Command {
    private final SystemWrapper<Belt> beltSystem;
    private final SystemWrapper<Indexer> indexerSystem;

    public Unjam(SystemWrapper<Belt> belt, SystemWrapper<Indexer> indexer) {
        beltSystem = belt;
        indexerSystem = indexer;
        addRequirements(belt, indexer);
    }

    @Override
    public void execute() {
        beltSystem.ifEnabled(belt -> belt.runReversed());
        indexerSystem.ifEnabled(indexer -> indexer.runReversed());
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
