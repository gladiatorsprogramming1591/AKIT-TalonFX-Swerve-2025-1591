package frc.robot.subsystems.elevator;

public final class ElevatorConstants {

    // CAN IDs
    public static final int LEADER_ID   = 2; // motor 2 = leader
    public static final int FOLLOWER_ID = 1; // motor 1 = follower

    // Current limit (amps)
    public static final double SUPPLY_CURRENT_LIMIT = 75.0;

    // Motion / mechanism constants (tune as needed)
    public static final double GEAR_RATIO = 10.0;        // motor rotations per elevator stage rotation
    public static final double DRUM_RADIUS_METERS = 0.02; // 2cm radius drum/pulley
    public static final double MASS_KG = 10.0;           // approximate mass of elevator + carriage

    // PID / control
    public static final double kP = 30.0;
    public static final double kI = 0.0;
    public static final double kD = 0.5;
    public static final double kS = 0.0;
    public static final double kG = 0.5;
    public static final double kV = 3.0;
    public static final double kA = 0.0;

    // Position limits (meters)
    public static final double MIN_HEIGHT_METERS = 0.0;
    public static final double MAX_HEIGHT_METERS = 1.0;

    // Max velocities/accels for profiled / motion control
    public static final double MAX_VELOCITY_MPS = 1.0;
    public static final double MAX_ACCELERATION_MPS2 = 2.0;

    // Sim constants
    public static final double SIM_CARRIAGE_MOMENT = 4.0; // arbitrary inertia

    private ElevatorConstants() {}
}
