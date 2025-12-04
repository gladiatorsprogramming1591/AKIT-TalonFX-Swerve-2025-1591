package frc.robot.subsystems.climber;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ClimberIOReal implements ClimberIO {

  private final SparkBase climbRollerMotor =
      new SparkFlex(ClimberConstants.CLIMB_ROLLER_CAN_ID, MotorType.kBrushless);
  final SparkBase winchMotor = new SparkFlex(ClimberConstants.WINCH_CAN_ID, MotorType.kBrushless);
  AbsoluteEncoder climberEncoder;

  public ClimberIOReal() {
    configureMotors();
  }

  private void configureMotors() {
    // Leader configuration

    climbRollerMotor.configure(
        ClimberConstants.CLIMB_ROLLER_MOTOR_CONFIG,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
    winchMotor.configure(
        ClimberConstants.WINCH_MOTOR_CONFIG,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);

    climberEncoder = winchMotor.getAbsoluteEncoder();
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    // Convert motor rotations to meters:
    //   pos_rot * (2πr / gearRatio)
    inputs.angle = climberEncoder.getPosition();
    inputs.velocity = climberEncoder.getVelocity();

    inputs.winchCurrentAmps = winchMotor.getOutputCurrent();
  }
}
