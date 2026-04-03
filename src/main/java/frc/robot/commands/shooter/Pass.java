package frc.robot.commands.shooter;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.feeder.Feed;
import frc.robot.services.vision.VisionService;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.feeder.Belt;
import frc.robot.subsystems.feeder.Indexer;
import frc.robot.subsystems.shooter.Shooter;
import lib.woodsonrobotics.SystemWrapper;

public class Pass extends ParallelDeadlineGroup {

    private final PassWithRotationOverride internalPassCommand;

    public Pass(
            SystemWrapper<Shooter> shooter,
            SystemWrapper<? extends Swerve> drive,
            SystemWrapper<Belt> belt,
            SystemWrapper<Indexer> indexer,
            VisionService vision) {

        // We have to do this stupid trick because Java doesn't let me
        // create a variable before calling super().
        this(new PassWithRotationOverride(shooter, drive, vision), shooter, belt, indexer);
    }

    private Pass(PassWithRotationOverride passCommand, SystemWrapper<Shooter> shooter,
            SystemWrapper<Belt> belt, SystemWrapper<Indexer> indexer) {
        super(
                passCommand,
                new SequentialCommandGroup(
                        new WaitUntilCommand(() -> {
                            boolean rpmReady = shooter.get().map(Shooter::atRPM).orElse(false);
                            boolean headingReady = passCommand.isHeadingAligned();
                            return rpmReady && headingReady;
                        }).withTimeout(4.0),
                        new Feed(belt, indexer)));
        internalPassCommand = passCommand;
    }

    public Supplier<Double> getRotationOverride() {
        return internalPassCommand.getRotationOverride();
    }
}
