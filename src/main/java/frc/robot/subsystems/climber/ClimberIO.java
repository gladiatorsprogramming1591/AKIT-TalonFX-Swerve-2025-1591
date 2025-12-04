package frc.robot.subsystems.climber;

public interface ClimberIO {

  /** Container for telemetry / sensor data. */
  public static class ClimberIOInputs {
    public double angle = 0.0;
    public double velocity = 0.0;
    public double appliedVolts = 0.0;
    public double climberCurrentAmps = 0.0;
    public double winchCurrentAmps = 0.0;
    public double positionMeters = 0.0;
  }

  /** Update the given inputs with the latest sensor data. */
  default void updateInputs(ClimberIOInputs inputs) {}

  /** Set motor output as voltage. */
  default void setVoltage(double volts) {}

  /** Optional: zero the position sensor. */
  default void zeroPosition() {}
}
