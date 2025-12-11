package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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

  public static Command WinchAndWheelCommand(
      ClimberSubsystem climber, DoubleSupplier winchSupplier, DoubleSupplier wheelSupplier) {
    return Commands.run(
        () ->
            climber.setWinchAndWheelRollerMotors(
                winchSupplier.getAsDouble(), wheelSupplier.getAsDouble()),
        climber);
  }

  // public Command manualClimberMovement(ClimberSubsystem climber, DoubleSupplier winchSupplier,
  // DoubleSupplier wheelSupplier)
  // {
  //   return new DefaultCommand(climber, winchSupplier, wheelSupplier);
  // }

  class DefaultCommand extends ParallelCommandGroup {
    public DefaultCommand(
        ClimberSubsystem climber, DoubleSupplier winchSupplier, DoubleSupplier wheelSupplier) {
      addRequirements(climber);
      addCommands(
          Commands.run(() -> climber.setWinchRollerMotor(winchSupplier.getAsDouble())),
          Commands.run(() -> climber.setWheelRollerMotor(wheelSupplier.getAsDouble())));
    }
  }
}
