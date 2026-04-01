package frc.robot.commands.shooter;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.feeder.Feed;
import frc.robot.services.vision.VisionService;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.feeder.Belt;
import frc.robot.subsystems.feeder.Indexer;
import frc.robot.subsystems.shooter.Shooter;
import lib.woodsonrobotics.SystemWrapper;

public class ShootOnMove extends ParallelDeadlineGroup {

    private final ShootWithRotationOverride internalShootCommand;

    public ShootOnMove(
            SystemWrapper<Shooter> shooter,
            SystemWrapper<? extends Swerve> drive,
            SystemWrapper<Belt> belt,
            SystemWrapper<Indexer> indexer,
            VisionService vision) {

        // We have to do this stupid trick because Java doesn't let me
        // create a variable before calling super().
        this(new ShootWithRotationOverride(shooter, drive, vision), shooter, belt, indexer);
    }

    private ShootOnMove(ShootWithRotationOverride shootCommand, SystemWrapper<Shooter> shooter,
            SystemWrapper<Belt> belt, SystemWrapper<Indexer> indexer) {
        super(
                shootCommand,
                new SequentialCommandGroup(
                        new PrintCommand("hello"),
                        new WaitUntilCommand(() -> shooter.get().map(Shooter::atRPM).orElse(false)),
                        new Feed(belt, indexer)));
        internalShootCommand = shootCommand;
    }

    public Supplier<Double> getRotationOverride() {
        return internalShootCommand.getRotationOverride();
    }
}