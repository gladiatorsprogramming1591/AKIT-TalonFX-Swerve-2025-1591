package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {

    private final ElevatorIO io;
    private final ElevatorIO.ElevatorIOInputs inputs = new ElevatorIO.ElevatorIOInputs();

    private final ProfiledPIDController controller =
        new ProfiledPIDController(
            ElevatorConstants.kP,
            ElevatorConstants.kI,
            ElevatorConstants.kD,
            new TrapezoidProfile.Constraints(
                ElevatorConstants.MAX_VELOCITY_MPS,
                ElevatorConstants.MAX_ACCELERATION_MPS2
            )
        );

    private final ElevatorFeedforward feedforward =
        new ElevatorFeedforward(
            ElevatorConstants.kS,
            ElevatorConstants.kG,
            ElevatorConstants.kV,
            ElevatorConstants.kA
        );

    private double targetHeightMeters = 0.0;
    private boolean closedLoopEnabled = false;

    public ElevatorSubsystem(ElevatorIO io) {
        this.io = io;
        controller.setTolerance(0.01); // 1cm tolerance
        controller.setGoal(0.0);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);

        if (closedLoopEnabled) {
            controller.setGoal(
                MathUtil.clamp(
                    targetHeightMeters,
                    ElevatorConstants.MIN_HEIGHT_METERS,
                    ElevatorConstants.MAX_HEIGHT_METERS
                )
            );

            double pidOut =
                controller.calculate(inputs.positionMeters, controller.getGoal().position);

            double ff =
                feedforward.calculate(controller.getSetpoint().velocity);

            double volts = pidOut + ff;
            io.setVoltage(volts);
        }

        // Telemetry
        SmartDashboard.putNumber("Elevator/PositionMeters", inputs.positionMeters);
        SmartDashboard.putNumber("Elevator/VelocityMPS", inputs.velocityMetersPerSec);
        SmartDashboard.putNumber("Elevator/AppliedVolts", inputs.appliedVolts);
        SmartDashboard.putNumber("Elevator/LeaderCurrent", inputs.leaderCurrentAmps);
        SmartDashboard.putNumber("Elevator/FollowerCurrent", inputs.followerCurrentAmps);
        SmartDashboard.putBoolean("Elevator/ClosedLoop", closedLoopEnabled);
        SmartDashboard.putNumber("Elevator/TargetMeters", targetHeightMeters);
    }

    // ---- External API ----

    /** Open-loop manual control, disables closed-loop mode. */
    public void setManualPercent(double percent) {
        closedLoopEnabled = false;
        io.setVoltage(percent * 12.0);
    }

    /** Hold current position using closed-loop. */
    public void holdPosition() {
        closedLoopEnabled = true;
        targetHeightMeters = inputs.positionMeters;
        controller.reset(inputs.positionMeters);
    }

    /** Move to a target height in meters using closed-loop. */
    public void setTargetHeight(double meters) {
        closedLoopEnabled = true;
        targetHeightMeters = meters;
    }

    /** Returns current height in meters. */
    public double getHeightMeters() {
        return inputs.positionMeters;
    }

    /** Returns true if at the current target within tolerance. */
    public boolean atTarget() {
        return controller.atGoal();
    }

    /** Zero the elevator position at its current location. */
    public void zeroPosition() {
        io.zeroPosition();
        controller.reset(0.0);
    }
}
