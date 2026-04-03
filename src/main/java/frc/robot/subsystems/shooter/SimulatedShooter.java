package frc.robot.subsystems.shooter;

public class SimulatedShooter extends Shooter {
    private double timeSetGoalRPM = -1;

    @Override
    public void setGoalRPM(double rpm) {
        // For RPM changes greater than 100 RPM, set a new timestamp to simulate
        // the shooter getting up to speed.
        if (Math.abs(getGoalRPM() - rpm) >= 100) {
            timeSetGoalRPM = System.currentTimeMillis();
        }
        super.setGoalRPM(rpm);
    }

    @Override
    public boolean atRPM() {
        System.out.println(System.currentTimeMillis() - timeSetGoalRPM);
        return super.atRPM()
                // Half a second of speed-up time
                || (System.currentTimeMillis() - timeSetGoalRPM) >= 500;
    }
}
