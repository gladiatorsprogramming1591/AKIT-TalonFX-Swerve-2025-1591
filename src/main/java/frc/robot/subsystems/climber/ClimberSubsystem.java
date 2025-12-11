package frc.robot.subsystems.climber;

import com.revrobotics.spark.SparkBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {

  private final ClimberIO io;
  private final ClimberIO.ClimberIOInputs inputs = new ClimberIO.ClimberIOInputs();
  private SparkBase wheelRollerMotor;
  private SparkBase winchRollerMotor;

  private double targetHeightMeters = 0.0;
  private boolean closedLoopEnabled = false;

  public ClimberSubsystem(ClimberIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    // Telemetry
    SmartDashboard.putNumber("Climber/PositionMeters", inputs.positionMeters);
    SmartDashboard.putNumber("Climber/VelocityMPS", inputs.velocity);
    SmartDashboard.putNumber("Climber/AppliedVolts", inputs.appliedVolts);
    SmartDashboard.putBoolean("Climber/ClosedLoop", closedLoopEnabled);
    SmartDashboard.putNumber("Climber/TargetMeters", targetHeightMeters);
  }

  // ---- External API ----

  public void setWinchRollerMotor(double speed) {
    winchRollerMotor.set(speed);
  }

  public void setWheelRollerMotor(double speed) {
    wheelRollerMotor.set(speed);
  }

  public void setWinchAndWheelRollerMotors(double winchSpeed, double wheelSpeed) {
    setWinchRollerMotor(winchSpeed);
    setWheelRollerMotor(wheelSpeed);
  }
}
