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

public class ControlledMotor extends SubsystemBase {
    private final SparkMax controlledMotor;
    
    private final SparkClosedLoopController pidController;
    private final SysIdRoutine m_sysIdRoutine;

    public ControlledMotor() {
        // 1. Initialize Motor
        controlledMotor = new SparkMax(ControlledMotorConstants.MOTOR_ID, MotorType.kBrushless);        
        
        pidController = controlledMotor.getClosedLoopController();

        // 2. Configure Leader
        SparkMaxConfig leaderConfig = new SparkMaxConfig();
        leaderConfig.voltageCompensation(ControlledMotorConstants.MOTOR_VOLTAGE_COMP);
        leaderConfig.smartCurrentLimit(ControlledMotorConstants.MOTOR_CURRENT_LIMIT);
        leaderConfig.idleMode(IdleMode.kBrake);
        leaderConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            // The PID values (kP, kI, kD)
            .pid(0.056503, 0, 0, ClosedLoopSlot.kSlot0)
            // The Feedforward values (kS, kV) from SysId
            .velocityFF(0.0) // Usually set to 0 when using kS/kV directly
            .feedForward
            .kS(0.07054, ClosedLoopSlot.kSlot0)
            .kV(0.12245, ClosedLoopSlot.kSlot0);

        controlledMotor.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // 3. SysId Routine Setup
        m_sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                // Unified Drive: Sending voltage to leader automatically sends it to follower
                (voltage) -> controlledMotor.setVoltage(voltage.in(Volts)),
                log -> {
                    // We only need to log the Leader's encoder because they are physically linked
                    log.motor("controlled-motor-system")
                       .voltage(Volts.of(controlledMotor.getAppliedOutput() * controlledMotor.getBusVoltage()))
                       .angularPosition(Rotations.of(controlledMotor.getEncoder().getPosition()))
                       .angularVelocity(RotationsPerSecond.of(controlledMotor.getEncoder().getVelocity() / 60.0));
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

    public void runSecondMotor(double speed){
        controlledMotor.set(speed);
    }

    public Command BackwardSlowSpin() {
        return this.runOnce(() -> { runSecondMotor(-.15);});
    }

    public Command SpinStop() {
        return this.runOnce(() -> { controlledMotor.stopMotor(); });
    }

    public void stop() {
        controlledMotor.stopMotor();
    }

    public Command ForwordSlowSpin() {
        return this.runOnce(() -> { runSecondMotor(.15);});
    }

    public void setVelocity(double rpm) {
        pidController.setReference(rpm, SparkBase.ControlType.kVelocity);
    }
}
