package frc.robot.subsystems.climber;

import com.revrobotics.spark.SparkBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {

  private final ClimberIO io;
  private final ClimberIO.ClimberIOInputs inputs = new ClimberIO.ClimberIOInputs();
  private SparkBase climberRollerMotor;
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
    SmartDashboard.putNumber("Elevator/PositionMeters", inputs.positionMeters);
    SmartDashboard.putNumber("Elevator/VelocityMPS", inputs.velocity);
    SmartDashboard.putNumber("Elevator/AppliedVolts", inputs.appliedVolts);
    SmartDashboard.putBoolean("Elevator/ClosedLoop", closedLoopEnabled);
    SmartDashboard.putNumber("Elevator/TargetMeters", targetHeightMeters);
  }

  // ---- External API ----

  public void setWinchRollerMotor(double speed) {
    winchRollerMotor.set(speed);
  }

  public void setWheelRollerMotor(double speed) {
    winchRollerMotor.set(speed);
  }
}
