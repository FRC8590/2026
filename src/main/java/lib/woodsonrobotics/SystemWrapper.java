package lib.woodsonrobotics;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Predicate;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lib.woodsonrobotics.telemetry.notify.DriveNotifier;

public class SystemWrapper<T extends SubsystemBase> extends SubsystemBase {
    private final Supplier<T> systemSupplier;
    private T cachedSystem;
    private boolean isEnabled = true;

    /**
     * TODO: Riley: Peter, you need to add descriptions to these parameters, i dont know what they mean
     * Wrapper class around a subsystem.
     * 
     * This allows subsystems to be rebooted or disabled.
     * 
     * @param name
     * @param supplier
     */
    public SystemWrapper(String name, Supplier<T> supplier) {
        setName(name);
        this.systemSupplier = supplier;
        SmartDashboard.putData("Systems/" + name + "/Toggle",
                Commands.runOnce(this::toggleEnabled).withName("Toggle"));
        SmartDashboard.putData("Systems/" + name + "/Reboot", Commands.runOnce(this::reboot).withName("Reboot"));
        SmartDashboard.putBoolean("Systems/" + getName() + "/Status", true);
    }

    public void toggleEnabled() {
        if (isEnabled) {
            disable();
        } else {
            enable();
        }
    }

    /**
     * Get the subsystem, or nothing if the system is disabled.
     * 
     * Do not use the resulting subsystem in an addRequirements() call!
     * Instead, directly use the wrapper. Using the wrapped subsystem as a
     * requirement
     * can cause weird issues, because that instance may become invalid (such as if
     * the system is rebooted).
     */
    public Optional<T> get() {
        if (!isEnabled) {
            return Optional.empty();
        }

        if (cachedSystem == null) {
            cachedSystem = systemSupplier.get();
        }

        return Optional.of(cachedSystem);
    }

    /**
     * Reboot the subsystem. This creates a fresh new instance, which get() will
     * return.
     */
    public void reboot() {
        // Unregister the old instance from the scheduler so it
        // doesn't keep running its default command / periodic.
        DriveNotifier.informWarning("Rebooting " + getName());
        T newSystem;
        try {
            newSystem = systemSupplier.get();
        } catch (Exception e) {
            DriveNotifier.internalError("reboot", "Reboot of " + getName() + " failed");
            e.printStackTrace();
            return;
        }
        if (cachedSystem != null) {
            CommandScheduler.getInstance().unregisterSubsystem(cachedSystem);
        }
        cachedSystem = newSystem;
        enable();
    }

    /**
     * Disable the system.
     */
    public void disable() {
        DriveNotifier.inform("Disabled system " + getName());
        isEnabled = false;
        if (cachedSystem != null) {
            CommandScheduler.getInstance().unregisterSubsystem(cachedSystem);
        }
        cachedSystem = null;
        SmartDashboard.putBoolean("Systems/" + getName() + "/Status", false);
    }

    /**
     * Enable the system. Systems are enabled by default; you only need to call this
     * if the system was previously disabled by disable(). If disable() wasn't
     * called, this function will effectively do nothing.
     */
    public void enable() {
        DriveNotifier.inform("Enabled system " + getName());
        isEnabled = true;
        SmartDashboard.putBoolean("Systems/" + getName() + "/Status", true);
        // cachedSystem will be lazily created on next getSystem() call
    }

    /**
     * Get a command wrapper for this subsystem.
     * 
     * When the command is invoked, the wrapper will be reinvoked to
     * get the latest state of the subsystem.
     */
    public Command command(Function<T, Command> commandFactory) {
        return new FunctionalCommand(
                // onInit
                () -> {
                },
                // onExecute
                () -> {
                },
                // onEnd
                interrupted -> {
                },
                // isFinished
                () -> false,
                this // requirement is the wrapper
        ) {
            private Command inner = null;

            // When initialize() is called, we reload the subsystem and apply
            // it to the command. This makes rebooting possible. If the system
            // is disabled, we set it to null and don't touch it.
            @Override
            public void initialize() {
                inner = get()
                        .map(commandFactory)
                        .orElse(null);
                if (inner != null) {
                    inner.initialize();
                }
            }

            @Override
            public void execute() {
                if (inner != null) {
                    inner.execute();
                }
            }

            @Override
            public void end(boolean interrupted) {
                if (inner != null) {
                    inner.end(interrupted);
                }
                inner = null;
            }

            @Override
            public boolean isFinished() {
                if (inner == null) {
                    return true;
                }
                return inner.isFinished();
            }
        };
    }

    /**
     * Wrap a predicate into a BooleanSupplier. This is useful for subsystems
     * that are used with a WaitUntilCommand.
     */
    public BooleanSupplier condition(Predicate<T> predicate) {
        return () -> get()
                .map(predicate::test)
                .orElse(false); // if disabled, condition is never true
    }

    /** Equivalent to wrapper.get().ifPresent(action) */
    public void ifEnabled(Consumer<? super T> action) {
        var result = get();
        result.ifPresent(action);
    }
}