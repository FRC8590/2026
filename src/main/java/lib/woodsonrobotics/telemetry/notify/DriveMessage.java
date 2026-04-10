package lib.woodsonrobotics.telemetry.notify;

import java.io.PrintStream;

import lib.woodsonrobotics.telemetry.elastic.Elastic;
import lib.woodsonrobotics.telemetry.elastic.Elastic.Notification;
import lib.woodsonrobotics.telemetry.elastic.Elastic.NotificationLevel;

/**
 * A message to the drive team.
 * 
 * @param title   The title to be displayed as part of the message.
 * @param message The description to be displayed as part of the message.
 * @param type    The type of the message.
 */
public record DriveMessage(String title, String message, DriveMessageType type) {
    /**
     * Publish the message to Elastic.
     */
    private void publishToElastic() {
        NotificationLevel level;
        switch (type) {
            case STATUS:
                level = NotificationLevel.INFO;
                break;
            case WARNING:
                level = NotificationLevel.WARNING;
                break;
            case ERROR:
                level = NotificationLevel.ERROR;
                break;
            default:
                System.err.println("Invalid notification type: " + type);
                level = NotificationLevel.ERROR;
        }
        var notification = new Notification(level, title, message);
        Elastic.sendNotification(notification);
    }

    /**
     * Get the message as a single string.
     */
    private String messageAsString() {
        String typeString;

        if (type == DriveMessageType.STATUS) {
            typeString = "STATUS";
        } else if (type == DriveMessageType.WARNING) {
            typeString = "WARNING";
        } else {
            assert (type == DriveMessageType.ERROR);
            typeString = "ERROR";
        }
        return String.format("[%s] %s: %s", typeString, title, message);

    }

    /**
     * Publish the message as a string to the console, either in standard output or
     * standard error.
     */
    private void publishToConsole() {
        PrintStream stream;

        if (type == DriveMessageType.STATUS) {
            stream = System.out;
        } else {
            stream = System.err;
        }

        stream.println(messageAsString());
    }

    /**
     * Publish the message to relevant dashboards.
     */
    public void publish() {
        publishToElastic();
        publishToConsole();
    }
}
