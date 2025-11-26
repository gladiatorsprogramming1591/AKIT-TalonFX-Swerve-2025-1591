package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ElevatorIOReal implements ElevatorIO {

    private final TalonFX leader =
        new TalonFX(ElevatorConstants.LEADER_ID);
    private final TalonFX follower =
        new TalonFX(ElevatorConstants.FOLLOWER_ID);

    public ElevatorIOReal() {
        configureMotors();
    }

    private void configureMotors() {
        // Leader configuration
        TalonFXConfiguration leaderConfig = new TalonFXConfiguration();
        leaderConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        leaderConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        leaderConfig.CurrentLimits.SupplyCurrentLimit = ElevatorConstants.SUPPLY_CURRENT_LIMIT;
        leaderConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        leader.getConfigurator().apply(leaderConfig);

        // Follower configuration
        TalonFXConfiguration followerConfig = new TalonFXConfiguration();
        followerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        // Inverted relative to leader – per your requirement
        followerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        followerConfig.CurrentLimits.SupplyCurrentLimit = ElevatorConstants.SUPPLY_CURRENT_LIMIT;
        followerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        follower.getConfigurator().apply(followerConfig);

        // Follow leader, with opposing direction
        follower.setControl(
            new Follower(leader.getDeviceID(), true)
        );
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        // Convert motor rotations to meters:
        //   pos_rot * (2πr / gearRatio)
        double motorRotations = leader.getPosition().getValueAsDouble();
        double motorRPS = leader.getVelocity().getValueAsDouble();

        double metersPerRotation =
            2.0 * Math.PI * ElevatorConstants.DRUM_RADIUS_METERS / ElevatorConstants.GEAR_RATIO;

        inputs.positionMeters = motorRotations * metersPerRotation;
        inputs.velocityMetersPerSec = motorRPS * metersPerRotation;

        inputs.appliedVolts = leader.getMotorVoltage().getValueAsDouble();
        inputs.leaderCurrentAmps = leader.getSupplyCurrent().getValueAsDouble();
        inputs.followerCurrentAmps = follower.getSupplyCurrent().getValueAsDouble();
    }

    @Override
    public void setVoltage(double volts) {
        leader.setVoltage(volts);
        // follower follows automatically
    }

    @Override
    public void zeroPosition() {
        leader.setPosition(0.0);
    }
}
