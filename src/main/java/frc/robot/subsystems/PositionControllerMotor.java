package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ControlledMotorConstants;

// NEW: Imports for SysId and Units
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class PositionControllerMotor extends SubsystemBase {
    private final SparkMax positionControlledMotor;
    
    private final SparkClosedLoopController pidController;
    private final SysIdRoutine m_sysIdRoutine;

    public PositionControllerMotor() {
        // 1. Initialize Motor
        positionControlledMotor = new SparkMax(ControlledMotorConstants.MOTOR_ID, MotorType.kBrushless);        
        
        pidController = positionControlledMotor.getClosedLoopController();

        // 2. Configure Leader
        SparkMaxConfig motorConfig = new SparkMaxConfig();
        motorConfig.voltageCompensation(ControlledMotorConstants.MOTOR_VOLTAGE_COMP);
        motorConfig.smartCurrentLimit(ControlledMotorConstants.MOTOR_CURRENT_LIMIT);
        motorConfig.closedLoop.allowedClosedLoopError(.04, ClosedLoopSlot.kSlot0);
        // motorConfig.idleMode(IdleMode.kBrake);
        motorConfig.idleMode(IdleMode.kBrake);
        motorConfig.closedLoop
        .pid(201.35, 0, 1.8598, ClosedLoopSlot.kSlot0)
        .feedForward
            .kS(0.19118)
            .kV(0.11393)
            .kA(0.00983)
            // .kG(0.0) // kG is a linear gravity feedforward, for an elevator
            // .kS(0.14139, ClosedLoopSlot.kSlot0)
            // .kV(0.12189, ClosedLoopSlot.kSlot0)
            .kCos(0.012); // kCos is a cosine gravity feedforward, for an arm
            // .kCosRatio(cosRatio); // kCosRatio relates the encoder position to absolute position
        // motorConfig.closedLoop
        //     .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        //     // The PID values (kP, kI, kD)
        //     .pid(-14.14, 0, -0.074646, ClosedLoopSlot.kSlot0)
        //     // The Feedforward values (kS, kV) from SysId
        //     // .velocityFF(0.0) // Usually set to 0 when using kS/kV directly
        //     .feedForward
        //     .kS(0.14139, ClosedLoopSlot.kSlot0)
        //     .kV(0.12189, ClosedLoopSlot.kSlot0);

        motorConfig.softLimit
            .forwardSoftLimit(20.0) // rotations
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimit(0.0)
            .reverseSoftLimitEnabled(true);
        positionControlledMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        // 3. SysId Routine Setup
        m_sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                // Volts.per(Second).of(2),
                // Volts.of(4),
                // Seconds.of(2),
                // null

            ),
            new SysIdRoutine.Mechanism(
                // Unified Drive: Sending voltage to leader automatically sends it to follower
                (voltage) -> positionControlledMotor.setVoltage(voltage.in(Volts)),
                log -> {
                    // We only need to log the Leader's encoder because they are physically linked
                    log.motor("controlled-motor-system")
                       .voltage(Volts.of(positionControlledMotor.getAppliedOutput() * positionControlledMotor.getBusVoltage()))
                       .angularPosition(Rotations.of(positionControlledMotor.getEncoder().getPosition()))
                       .angularVelocity(RotationsPerSecond.of(positionControlledMotor.getEncoder().getVelocity() / 60.0));
                },
                this
            )
        );
    }

    // NEW: Command Factories to trigger tests from RobotContainer
    public Command sysIdQuasistatic(Direction direction) {
        return m_sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(Direction direction) {
        return m_sysIdRoutine.dynamic(direction);
    }

    @Override
    public void periodic() {}

    public Command goToPositionCommand(double targetRotations) {
        // Keep continuously sending the target so our default "hold position" command doesn't
        // immediately override it.
        return run(() -> {
            // Use kPosition for instant PID or kSmartMotion for a smooth profiled move
            pidController.setReference(targetRotations, SparkMax.ControlType.kPosition);
        })
        // The command is finished when the encoder is within a small range of the target.
        // Use a tighter tolerance so we don't end early and immediately fall back to the
        // hold position command.
        .until(() -> Math.abs(positionControlledMotor.getEncoder().getPosition() - targetRotations) < 0.05);
    }

    public Command goToPositionWithTimeout(double targetRotations, double timeout) {
        return goToPositionCommand(targetRotations)
            .withTimeout(timeout) // Interupts if it takes longer than timeout seconds
            .handleInterrupt(() -> {
                // Optional: Logic to run if the command times out (e.g., stop motor)
                positionControlledMotor.set(0);
                System.out.println("Position command timed out - potential jam!");
            });
    }

    public Command zeroEncoderCommand() {
        return runOnce(() -> positionControlledMotor.getEncoder().setPosition(0))
           .ignoringDisable(true); // Allows you to zero even when robot is disabled
    }

    public Command holdPositionCommand() {
        // We use a local variable to "lock in" the position when the command starts
        return defer(() -> {
            double currentPos = positionControlledMotor.getEncoder().getPosition();
            return run(() -> pidController.setReference(currentPos, SparkMax.ControlType.kPosition));
        });
    }

        public Command holdPositionCommandBreak() {
        // We use a local variable to "lock in" the position when the command starts
           double currentPos = positionControlledMotor.getEncoder().getPosition();
            return run(() -> pidController.setReference(currentPos, SparkMax.ControlType.kPosition));
        
    }


    public Command nudgePositionCommand(double deltaRotations) {
        // Keep commanding the target while the command is active so the subsystem doesn't
        // immediately fall back to its default holding behavior.
        double newTarget = positionControlledMotor.getEncoder().getPosition() + deltaRotations;
        return run(() -> {
            pidController.setReference(newTarget, SparkMax.ControlType.kPosition);
        })
        .until(() -> Math.abs(positionControlledMotor.getEncoder().getPosition() - newTarget) < 0.05);
    }


}
