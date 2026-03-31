package frc.robot.subsystems.shooter;

public class SimulatedShooter extends Shooter {
    @Override
    public boolean atRPM() {
        // We want to test the encoder code and whatnot, so we execute it
        // during the simulation. We don't care about its value, though.
        super.atRPM();

        // Since the encoder will always be zero during simulations, we
        // just pretend it instantly gets up to speed.
        return getGoalRPM() > 0;
    }
}
