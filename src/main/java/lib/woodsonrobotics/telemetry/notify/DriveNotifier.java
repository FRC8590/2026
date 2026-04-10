package lib.woodsonrobotics.telemetry.notify;

import java.util.HashMap;

/**
 * Notify the drive team about something.
 */
public class DriveNotifier {
    private static HashMap<String, Long> messagesWithTimes = new HashMap<>();

    /**
     * Something has gone horribly wrong in the code.
     * 
     * @param where The name of the function in which the error occurred.
     * @param what  The description of the error.
     */
    public static void internalError(String where, String what) {
        var driveMessage = new DriveMessage(where, what, DriveMessageType.ERROR);
        driveMessage.publish();
    }

    /**
     * Should this message be skipped?
     * 
     * If the same message has been recently published, then we skip it.
     * 
     * @param message The message, which will be cached as part of calling this
     *                function.
     * @return Whether the message should be skipped.
     */
    private static boolean shouldSkip(String message) {
        long lastSent = messagesWithTimes.getOrDefault(message, Long.valueOf(-1));
        long currentTime = System.currentTimeMillis();
        messagesWithTimes.put(message, currentTime);
        if (lastSent == -1) {
            return false;
        }

        return (currentTime - lastSent) <= 1000;

    }

    /**
     * The driver did something wrong.
     * 
     * @param message What the driver did incorrectly.
     */
    public static void operatorError(String message) {
        if (shouldSkip(message)) {
            return;
        }
        var driveMessage = new DriveMessage("Driver error", message, DriveMessageType.WARNING);
        driveMessage.publish();
    }

    /**
     * A simple status update.
     * 
     * @param message What happened?
     */
    public static void inform(String message) {
        if (shouldSkip(message)) {
            return;
        }
        var driveMessage = new DriveMessage("Info", message, DriveMessageType.STATUS);
        driveMessage.publish();
    }

    /**
     * A simple warning -- nothing fatal or a huge deal, but something worth noting.
     *
     * @param message What happened?
     */
    public static void informWarning(String message) {
        if (shouldSkip(message)) {
            return;
        }
        var driveMessage = new DriveMessage("Warning", message, DriveMessageType.STATUS);
        driveMessage.publish();
    }
}
