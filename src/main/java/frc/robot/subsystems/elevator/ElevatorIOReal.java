package frc.robot.subsystems.elevator;

import java.util.EnumMap;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.elevator.ElevatorIO.ElevatorIOInputs;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

public class ElevatorIOReal implements ElevatorIO {

  private final SparkFlex leader = new SparkFlex(ElevatorConstants.LEADER_ID, MotorType.kBrushless);
  private final SparkFlex follower = new SparkFlex(ElevatorConstants.FOLLOWER_ID, MotorType.kBrushless);

   DigitalInput lowerLimit;
    Trigger zeroTrigger;

    SparkFlex leaderLeft;
    SparkFlex followerRight;

    RelativeEncoder externalEncoder; // Through-bore encoder
    SparkClosedLoopController controller;
    SparkLimitSwitch bottomLimitSwitch;

    private double lastPos;
    private boolean printInternalEncZero = true; // Flag to indicate whether to print when we are zeroing during stow position
    private boolean printBothEncZero = true;

    EnumMap<elevatorPositions, Double> mapEnc = new EnumMap<>(elevatorPositions.class);

    public enum elevatorPositions {
        STOW,
        L1,
        L2,
        L3,
        L4,
        PROCESSOR, // same as stow height?
        NETSHOOT, // same as l4 height
        ALGAE_HIGH,
        ALGAE_LOW, 
        AUTO_L4
    }
  public ElevatorIOReal() {
    configureMotors();
  }

  private void configureMotors() {
    // Leader configuration
    leader.configure(ElevatorConstants.MOTOR_CONFIG,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        follower.configure(
                ElevatorConstants.MOTOR_CONFIG.follow(ElevatorConstants.LEADER_ID, true),
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters);

        externalEncoder = leaderLeft.getExternalEncoder();
        controller = leaderLeft.getClosedLoopController();

        bottomLimitSwitch = followerRight.getReverseLimitSwitch();
        zeroTrigger = new Trigger(this::isElevatorNotAtBottom);
        zeroTrigger.onFalse(zeroElevatorInternalEncCommand().alongWith(zeroElevatorExternalEncCommand()));

        lastPos = ElevatorConstants.kSTOW;

        mapEnc.put(elevatorPositions.STOW, ElevatorConstants.kSTOW);
        mapEnc.put(elevatorPositions.L1, ElevatorConstants.kL1);
        mapEnc.put(elevatorPositions.L2, ElevatorConstants.kL2);
        mapEnc.put(elevatorPositions.L3, ElevatorConstants.kL3);
        mapEnc.put(elevatorPositions.L4, ElevatorConstants.kL4);
        mapEnc.put(elevatorPositions.PROCESSOR, ElevatorConstants.kPROCESSOR);
        mapEnc.put(elevatorPositions.NETSHOOT, ElevatorConstants.kNET);
        mapEnc.put(elevatorPositions.ALGAE_HIGH, ElevatorConstants.ALGAE_HIGH);
        mapEnc.put(elevatorPositions.ALGAE_LOW, ElevatorConstants.ALGAE_LOW);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    // Convert motor rotations to meters:
    //   pos_rot * (2πr / gearRatio)
    double motorRotations = getExternalPositionRotations();
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

  private boolean isElevatorNotAtBottom() {
        return !bottomLimitSwitch.isPressed();
    }

  public Command zeroElevatorExternalEncCommand() {
    return new InstantCommand(() -> {
        System.out.println("ZeroExternalEncCommand");
        externalEncoder.setPosition(0);
    });
}

public void ElevatorToPositionExternalEnc(elevatorPositions positions) {
  lastPos = mapEnc.get(positions);
  setPositionExternalRotations(inchesToExternalRotations(lastPos));
}

public double inchesToExternalRotations(double inches) {
  return (inches - ElevatorConstants.INITIAL_HEIGHT_INCHES) / ElevatorConstants.INCHES_PER_EXTERNAL_ROTATION;
}

public void setPositionExternalRotations(double rotations) {
  if (rotations < getExternalPositionRotations()) {
      controller.setReference(rotations, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot1, ElevatorConstants.FF_DOWN);
      System.out.println("Lowering Elevator using kSlot1. Reference set to: " + rotations);
  } else {
      controller.setReference(rotations, ControlType.kPosition, ClosedLoopSlot.kSlot0, ElevatorConstants.FF_UP);
      System.out.println("Raising Elevator using kSlot0. Reference set to:  " + rotations);
  }
}

public double getExternalPositionRotations() {
  return externalEncoder.getPosition();
}

}
