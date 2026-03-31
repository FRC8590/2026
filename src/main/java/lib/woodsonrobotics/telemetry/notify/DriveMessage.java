package lib.woodsonrobotics.telemetry.notify;

import java.io.PrintStream;

import lib.woodsonrobotics.telemetry.elastic.Elastic;
import lib.woodsonrobotics.telemetry.elastic.Elastic.Notification;
import lib.woodsonrobotics.telemetry.elastic.Elastic.NotificationLevel;

/* A message to the drive team. */
public record DriveMessage(String title, String message, DriveMessageType type) {
    /* Publish the message to Elastic. */
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

    private void publishToConsole() {
        PrintStream stream;

        if (type == DriveMessageType.STATUS) {
            stream = System.out;
        } else {
            stream = System.err;
        }

        stream.println(messageAsString());
    }

    /* Publish the message to relevant dashboards. */
    public void publish() {
        publishToElastic();
        publishToConsole();
    }
}
