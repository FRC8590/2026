package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Meters;

import org.ironmaple.simulation.IntakeSimulation;

import frc.robot.subsystems.drive.SimulatedSwerve;
import lib.woodsonrobotics.SystemWrapper;

public class SimulatedIntake extends Intake {
    private IntakeSimulation intakeSimulation;

    public IntakeSimulation getIntakeSimulation() {
        return intakeSimulation;
    }

    public SimulatedIntake(SystemWrapper<SimulatedSwerve> swerve) {
        super();

        var simulatedSwerve = swerve.get();
        if (simulatedSwerve.isEmpty()) {
            return;
        }

        var mapleSimDrive = simulatedSwerve.get().getMapleSimDrive();

        intakeSimulation = IntakeSimulation.OverTheBumperIntake(
                "Fuel",
                mapleSimDrive,
                Meters.of(0.7),
                Meters.of(0.2),
                IntakeSimulation.IntakeSide.BACK,
                30);
    }

    @Override
    public void run() {
        super.run();
        if (intakeSimulation != null) {
            intakeSimulation.startIntake();
        }
    }

    @Override
    public void stop() {
        super.stop();
        if (intakeSimulation != null) {
            intakeSimulation.stopIntake();
        }
    }
}