package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.DoubleSupplier;

public final class ClimberCommands {

  private ClimberCommands() {}

  /** Manual control command using a joystick axis (–1 to 1). */
  public static Command winchCommand(ClimberSubsystem climber, DoubleSupplier axisSupplier) {
    return Commands.run(() -> climber.setWinchRollerMotor(axisSupplier.getAsDouble()));
  }

  /** Manual control command using a joystick axis (–1 to 1). */
  public static Command wheelCommand(ClimberSubsystem climber, DoubleSupplier axisSupplier) {
    return Commands.run(() -> climber.setWheelRollerMotor(axisSupplier.getAsDouble()));
  }
}
