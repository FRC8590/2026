package lib.woodsonrobotics.telemetry.notify;

import java.util.HashMap;

/* Notify the drive team about something. */
public class DriveNotifier {
    private static HashMap<String, Long> messagesWithTimes = new HashMap<>();

    /* Something has gone horribly wrong in the code. */
    public static void internalError(String where, String what) {
        var driveMessage = new DriveMessage(where, what, DriveMessageType.ERROR);
        driveMessage.publish();
    }

    private static boolean shouldSkip(String message) {
        long lastSent = messagesWithTimes.getOrDefault(message, Long.valueOf(-1));
        long currentTime = System.currentTimeMillis();
        messagesWithTimes.put(message, currentTime);
        if (lastSent == -1) {
            return false;
        }

        return (currentTime - lastSent) <= 1000;

    }

    /* The driver did something wrong. */
    public static void operatorError(String message) {
        if (shouldSkip(message)) {
            return;
        }
        var driveMessage = new DriveMessage("Driver error", message, DriveMessageType.WARNING);
        driveMessage.publish();
    }

    /* A simple status update. */
    public static void inform(String message) {
        if (shouldSkip(message)) {
            return;
        }
        var driveMessage = new DriveMessage("Info", message, DriveMessageType.STATUS);
        driveMessage.publish();
    }

    public static void informWarning(String message) {
        if (shouldSkip(message)) {
            return;
        }
        var driveMessage = new DriveMessage("Warning", message, DriveMessageType.STATUS);
        driveMessage.publish();
    }
}
