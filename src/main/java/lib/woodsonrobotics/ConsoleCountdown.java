package lib.woodsonrobotics;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

/* A countdown displayed in Shuffleboard.
 * step() must be called periodically to advance it. */
public class ConsoleCountdown {
    private Timer timer = new Timer();
    private double waitTime = -1;
    private GenericEntry shuffleboardEntry;
    private long lastPublishedSecond = -1;

    public ConsoleCountdown(String name) {
        shuffleboardEntry = Shuffleboard
                .getTab("Console")
                .add(name, 0)
                .getEntry();
    }

    /* Start the countdown for a given amount of time. */
    public void start(long waitTime) {
        this.waitTime = waitTime;
        lastPublishedSecond = -1;
        timer.reset();
        timer.start();
        shuffleboardEntry.setInteger(waitTime);
    }

    /* Stop the countdown. */
    public void stop() {
        timer.stop();
    }

    /*
     * Advance the countdown.
     * 
     * @return The remaining time on the countdown, which may be zero.
     */
    public double step() {
        double remainingTime = waitTime - timer.get();

        if (remainingTime <= 0) {
            if (lastPublishedSecond != 0) {
                stop();
                shuffleboardEntry.setInteger(0);
                lastPublishedSecond = 0;
            }
            return 0;
        }

        long secondsToDisplay = Math.round(remainingTime);
        if (secondsToDisplay != lastPublishedSecond) {
            shuffleboardEntry.setInteger(secondsToDisplay);
            lastPublishedSecond = secondsToDisplay;
        }

        return remainingTime;
    }
}