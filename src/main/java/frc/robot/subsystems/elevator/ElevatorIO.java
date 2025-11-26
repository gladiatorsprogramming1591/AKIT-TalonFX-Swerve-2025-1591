package frc.robot.subsystems.elevator;

public interface ElevatorIO {

    /** Container for telemetry / sensor data. */
    public static class ElevatorIOInputs {
        public double positionMeters = 0.0;
        public double velocityMetersPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double leaderCurrentAmps = 0.0;
        public double followerCurrentAmps = 0.0;
    }

    /** Update the given inputs with the latest sensor data. */
    default void updateInputs(ElevatorIOInputs inputs) {}

    /** Set motor output as voltage. */
    default void setVoltage(double volts) {}

    /** Optional: zero the position sensor. */
    default void zeroPosition() {}
}

