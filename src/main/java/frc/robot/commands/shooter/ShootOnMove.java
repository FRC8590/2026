package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.feeder.Feed;
import frc.robot.services.RotationOverrideService;
import frc.robot.services.vision.VisionService;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.feeder.Belt;
import frc.robot.subsystems.feeder.Indexer;
import frc.robot.subsystems.shooter.Shooter;
import lib.woodsonrobotics.SystemWrapper;

public class ShootOnMove extends ParallelDeadlineGroup {

    public ShootOnMove(
            SystemWrapper<Shooter> shooter,
            SystemWrapper<? extends Swerve> drive,
            SystemWrapper<Belt> belt,
            SystemWrapper<Indexer> indexer,
            VisionService vision, RotationOverrideService rotationOverride) {

        // We have to do this stupid trick because Java doesn't let me
        // create a variable before calling super().
        this(new ShootWithRotationOverride(shooter, drive, vision, rotationOverride), shooter, belt, indexer);
    }

    private ShootOnMove(ShootWithRotationOverride shootCommand, SystemWrapper<Shooter> shooter,
            SystemWrapper<Belt> belt, SystemWrapper<Indexer> indexer) {
        super(
                shootCommand,
                new SequentialCommandGroup(
                        new WaitUntilCommand(() -> {
                            boolean rpmReady = shooter.get().map(Shooter::atRPM).orElse(false);
                            boolean headingReady = shootCommand.isHeadingAligned();
                            return rpmReady && headingReady;
                        }).withTimeout(2.0),
                        new Feed(belt, indexer)));
    }
}