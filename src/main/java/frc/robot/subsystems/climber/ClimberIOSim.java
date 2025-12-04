package frc.robot.subsystems.climber;

import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

public class ClimberIOSim implements ClimberIO {

  DCMotor neo = DCMotor.getNEO(3);
  SparkFlex winchMotor = new SparkFlex(60, MotorType.kBrushless);
  SparkFlexSim winchMotorSim = new SparkFlexSim(winchMotor, null);

  private final SparkFlexSim sim = new SparkFlexSim(winchMotor, neo);

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    sim.iterate(sim.getVelocity(), RoboRioSim.getVInVoltage(), 0.02); // 20ms loop
    inputs.positionMeters = sim.getPosition();
    inputs.velocity = sim.getVelocity();
    inputs.appliedVolts = sim.getAppliedOutput();
    inputs.climberCurrentAmps = sim.getMotorCurrent();
  }

  @Override
  public void setVoltage(double volts) {
    volts = MathUtil.clamp(volts, -12.0, 12.0);
    sim.setBusVoltage(volts);
  }

  @Override
  public void zeroPosition() {
    sim.setPosition(0.0);
  }
}
