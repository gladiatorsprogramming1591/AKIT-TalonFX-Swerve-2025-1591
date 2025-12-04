package frc.robot.subsystems.climber;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

public final class ClimberConstants {
  public static final int CLIMB_ROLLER_CAN_ID = 50;
  public static final int CLIMB_ROLLER_CURRENT_LIMIT = 80;
  public static final boolean CLIMB_ROLLER_MOTOR_INVERTED = true;
  public static final double CLIMB_ROLLER_RAMP_RATE = 0.1;
  public static final double CLIMB_ROLLER_P = 0.025;
  public static final double CLIMB_ROLLER_I = 0;
  public static final double CLIMB_ROLLER_D = 0.04;

  public static final SparkMaxConfig CLIMB_ROLLER_MOTOR_CONFIG =
      new SparkMaxConfig() {
        {
          idleMode(IdleMode.kBrake);
          smartCurrentLimit(ClimberConstants.CLIMB_ROLLER_CURRENT_LIMIT);
          inverted(ClimberConstants.CLIMB_ROLLER_MOTOR_INVERTED);
          openLoopRampRate(ClimberConstants.CLIMB_ROLLER_RAMP_RATE);
          closedLoop.p(ClimberConstants.CLIMB_ROLLER_P);
          closedLoop.i(ClimberConstants.CLIMB_ROLLER_I);
          closedLoop.d(ClimberConstants.CLIMB_ROLLER_D);
          // closedLoop.maxMotion.allowedClosedLoopError(3);
          // closedLoop.maxMotion.maxAcceleration(100000);
          // closedLoop.maxMotion.maxVelocity(120000);
          closedLoop.feedbackSensor(
              FeedbackSensor
                  .kAbsoluteEncoder); // TODO: Why is this on rollers? This would be a problem if we
          // used this PID controller.
          absoluteEncoder.positionConversionFactor(360);
        }
      };

  public static final int WINCH_CAN_ID = 60;
  public static final int WINCH_CURRENT_LIMIT = 80;
  public static final boolean WINCH_MOTOR_INVERTED = true;
  public static final double WINCH_RAMP_RATE = 0.1;
  public static final double WINCH_P = 0.025;
  public static final double WINCH_I = 0;
  public static final double WINCH_D = 0.04;

  public static final double WINCH_IN_LIMIT = 299.0;
  public static final double WINCH_OUT_LIMIT = 35.0;
  public static final SparkMaxConfig WINCH_MOTOR_CONFIG =
      new SparkMaxConfig() {
        {
          idleMode(IdleMode.kBrake);
          smartCurrentLimit(ClimberConstants.WINCH_CURRENT_LIMIT);
          inverted(ClimberConstants.WINCH_MOTOR_INVERTED);
          openLoopRampRate(ClimberConstants.WINCH_RAMP_RATE);
          closedLoop.p(ClimberConstants.WINCH_P);
          closedLoop.i(ClimberConstants.WINCH_I);
          closedLoop.d(ClimberConstants.WINCH_D);
          // closedLoop.maxMotion.allowedClosedLoopError(3);
          // closedLoop.maxMotion.maxAcceleration(100000);
          // closedLoop.maxMotion.maxVelocity(120000);
          closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
          absoluteEncoder.positionConversionFactor(360);
        }
      };

  private ClimberConstants() {}
}
