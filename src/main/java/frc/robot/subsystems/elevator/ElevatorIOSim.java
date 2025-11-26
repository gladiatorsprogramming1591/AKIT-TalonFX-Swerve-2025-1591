package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim implements ElevatorIO {

  private final ElevatorSim sim =
      new ElevatorSim(
          DCMotor.getFalcon500(2),
          ElevatorConstants.GEAR_RATIO,
          ElevatorConstants.SIM_CARRIAGE_MOMENT,
          ElevatorConstants.DRUM_RADIUS_METERS,
          ElevatorConstants.MIN_HEIGHT_METERS,
          ElevatorConstants.MAX_HEIGHT_METERS,
          true,
          0,
          null);

  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    sim.update(0.02); // 20ms loop

    inputs.positionMeters = sim.getPositionMeters();
    inputs.velocityMetersPerSec = sim.getVelocityMetersPerSecond();
    inputs.appliedVolts = appliedVolts;
    inputs.leaderCurrentAmps = sim.getCurrentDrawAmps();
    inputs.followerCurrentAmps = sim.getCurrentDrawAmps();
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    sim.setInput(appliedVolts);
  }

  @Override
  public void zeroPosition() {
    sim.setState(0.0, 0.0);
  }
}
