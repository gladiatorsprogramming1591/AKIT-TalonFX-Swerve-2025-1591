package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.DoubleSupplier;

public final class ElevatorCommands {

    private ElevatorCommands() {}

    /** Manual control command using a joystick axis (–1 to 1). */
    public static Command manualControl(
        ElevatorSubsystem elevator,
        DoubleSupplier axisSupplier
    ) {
        return Commands.run(
            () -> elevator.setManualPercent(axisSupplier.getAsDouble()),
            elevator
        );
    }

    /** Move elevator to a fixed height and end when at target. */
    public static Command moveToHeight(
        ElevatorSubsystem elevator,
        double targetMeters
    ) {
        return Commands.runOnce(
                () -> elevator.setTargetHeight(targetMeters),
                elevator
            )
            .andThen(
                Commands.waitUntil(elevator::atTarget)
            )
            .withName("ElevatorMoveToHeight(" + targetMeters + "m)");
    }

    /** Hold the current position until interrupted. */
    public static Command holdPosition(ElevatorSubsystem elevator) {
        return Commands.runOnce(
                elevator::holdPosition,
                elevator
            ).andThen(
                Commands.idle(elevator)
            )
            .withName("ElevatorHoldPosition");
    }

    /** Zero the elevator position. */
    public static Command zeroPosition(ElevatorSubsystem elevator) {
        return Commands.runOnce(elevator::zeroPosition, elevator)
            .withName("ElevatorZeroPosition");
    }
}
