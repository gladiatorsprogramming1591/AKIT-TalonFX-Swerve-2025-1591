package frc.robot.subsystems.elevator;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

public final class ElevatorConstants {

  // CAN IDs
  public static final int LEADER_ID = 2; // motor 2 = leader
  public static final int FOLLOWER_ID = 1; // motor 1 = follower

  // Current limit (amps)
  public static final int SUPPLY_CURRENT_LIMIT = 75;

  // Motion / mechanism constants (tune as needed)
  public static final double GEAR_RATIO = 10.0; // motor rotations per elevator stage rotation
  public static final double DRUM_RADIUS_METERS = 0.02; // 2cm radius drum/pulley
  public static final double MASS_KG = 10.0; // approximate mass of elevator + carriage

  // PID / control
  public static final double kP = 30.0;
  public static final double kI = 0.0;
  public static final double kD = 0.5;
  
  //Optional values
  public static final double kS = 0.0;
  public static final double kG = 0.5;
  public static final double kV = 3.0;
  public static final double kA = 0.0;

  // FF Values
  public static final double FF_UP = 0.623;
  public static final double FF_DOWN = 0.623;

  // Position limits (meters)
  public static final double MIN_HEIGHT_METERS = 0.0;
  public static final double MAX_HEIGHT_METERS = 1.0;

  // Max velocities/accels for profiled / motion control
  public static final double MAX_VELOCITY_MPS = 1.0;
  public static final double MAX_ACCELERATION_MPS2 = 2.0;

  public static final double INCHES_PER_INTERNAL_ROTATION = 22.0 / 9.0 / 4.0;
        // public static final double UPPER_SPROCKET_CIRCUMFERENCE_OLD = 1.7576 * Math.PI; // 22 tooth sprocket
        public static final double UPPER_SPROCKET_CIRCUMFERENCE = 22.0 / 4.0; // 22 tooth sprocket
        public static final double INCHES_PER_EXTERNAL_ROTATION = UPPER_SPROCKET_CIRCUMFERENCE / 2;
        public static final double INTERNAL_ROTS_PER_EXTERNAL_ROT = INCHES_PER_EXTERNAL_ROTATION / INCHES_PER_INTERNAL_ROTATION;
        public static final double INITIAL_HEIGHT_INCHES = 0;
        public static final double TOLERANCE_INCHES = 1.0;

  public static final double MAX_VEL_DOWN = 3500 / INTERNAL_ROTS_PER_EXTERNAL_ROT;
        public static final double MAX_ACCEL_DOWN = 5400 / INTERNAL_ROTS_PER_EXTERNAL_ROT * 1.75;
        public static final double ALLOWERD_ERR_DOWN = 1.0 / INTERNAL_ROTS_PER_EXTERNAL_ROT / 4;

  

  // Sim constants
  public static final double SIM_CARRIAGE_MOMENT = 4.0; // arbitrary inertia
  public static final SparkFlexConfig MOTOR_CONFIG = new SparkFlexConfig() {
            {
                idleMode(IdleMode.kBrake);
                smartCurrentLimit(ElevatorConstants.SUPPLY_CURRENT_LIMIT);
                inverted(true);
                // closedLoopRampRate(ElevatorConstants.RAMP_RATE); // Not used; negatively impacts deceleration while approaching hardstops.
                limitSwitch.reverseLimitSwitchEnabled(false);
                limitSwitch.forwardLimitSwitchEnabled(false);

                closedLoop.outputRange(-1.0, MAX_VELOCITY_MPS, ClosedLoopSlot.kSlot0) // kslot 0 is up (only kPosition is used)
                        .p(ElevatorConstants.kP, ClosedLoopSlot.kSlot0)
                        .i(ElevatorConstants.kI, ClosedLoopSlot.kSlot0)
                        .d(ElevatorConstants.kD, ClosedLoopSlot.kSlot0)
                        .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder);
                // closedLoop.maxMotion.maxVelocity(MAX_VEL_UP, ClosedLoopSlot.kSlot0) // We don't use maxMotion going up due to inconsistent results.
                //         .maxAcceleration(MAX_ACCEL_UP, ClosedLoopSlot.kSlot0)
                //         .allowedClosedLoopError(ALLOWERD_ERR_UP, ClosedLoopSlot.kSlot0)
                //         .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal, ClosedLoopSlot.kSlot0);

                closedLoop.outputRange(-1.0, 1, ClosedLoopSlot.kSlot1) // kslot 1 is down (kMAXMotionPositionControl is used)
                        .p(ElevatorConstants.kP, ClosedLoopSlot.kSlot1)
                        .i(ElevatorConstants.kI, ClosedLoopSlot.kSlot1)
                        .d(ElevatorConstants.kD, ClosedLoopSlot.kSlot1)
                        .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder);
                closedLoop.maxMotion.maxVelocity(MAX_VEL_DOWN, ClosedLoopSlot.kSlot1)
                        .maxAcceleration(MAX_ACCEL_DOWN, ClosedLoopSlot.kSlot1)
                        .allowedClosedLoopError(ALLOWERD_ERR_DOWN, ClosedLoopSlot.kSlot1);
            }
        };

        public static final double kSTOW = 0.4;
        public static final double kL1 = 2.75;
        public static final double kL2 = 7.04;
        public static final double kL3 = 15.3;
        public static final double kL4 = 26.4;
        public static final double kPROCESSOR = 0.5;
        public static final double kNET = 27.75;
        public static final double ALGAE_HIGH = 10.53;
        public static final double ALGAE_LOW = 2.75;
        
  private ElevatorConstants() {}

}
