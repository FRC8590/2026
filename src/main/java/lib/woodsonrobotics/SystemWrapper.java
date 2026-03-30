package lib.woodsonrobotics;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Function;
import java.util.function.Predicate;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/* Wrapper class around a subsystem.
 * 
 * This allows subsystems to be rebooted or disabled.
*/
public class SystemWrapper<T extends SubsystemBase> extends SubsystemBase {
    private final Supplier<T> systemSupplier;
    private T cachedSystem;
    private boolean isEnabled = true;

    public SystemWrapper(String name, Supplier<T> supplier) {
        setName(name);
        this.systemSupplier = supplier;
    }

    public Optional<T> get() {
        if (!isEnabled) {
            return Optional.empty();
        }

        if (cachedSystem == null) {
            cachedSystem = systemSupplier.get();
        }

        return Optional.of(cachedSystem);
    }

    public void reboot() {
        // Unregister the old instance from the scheduler so it
        // doesn't keep running its default command / periodic.
        if (cachedSystem != null) {
            CommandScheduler.getInstance().unregisterSubsystem(cachedSystem);
        }
        cachedSystem = systemSupplier.get();
    }

    public void disable() {
        isEnabled = false;
        if (cachedSystem != null) {
            CommandScheduler.getInstance().unregisterSubsystem(cachedSystem);
        }
        cachedSystem = null;
    }

    public void enable() {
        isEnabled = true;
        // cachedSystem will be lazily created on next getSystem() call
    }

    /*
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

    /*
     * Wrap a predicate into a BooleanSupplier. This is useful for subsystems
     * that are used with a WaitUntilCommand.
     */
    public BooleanSupplier condition(Predicate<T> predicate) {
        return () -> get()
                .map(predicate::test)
                .orElse(false); // if disabled, condition is never true
    }
}