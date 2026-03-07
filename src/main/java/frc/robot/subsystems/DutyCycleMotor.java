package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DutyCycleMotorConstants;

public class DutyCycleMotor extends SubsystemBase {

    private final SparkMax dutyCycleMotor;
    /**
     * This subsytem that controls the motor.
     */
    public DutyCycleMotor () {

        // Set up the motor as a brushed motor
        dutyCycleMotor = new SparkMax(DutyCycleMotorConstants.SINGLE_MOTOR_ID, MotorType.kBrushless);

        // Set can timeout. Because this project only sets parameters once on
        // construction, the timeout can be long without blocking robot operation. Code
        // which sets or gets parameters during operation may need a shorter timeout.
        dutyCycleMotor.setCANTimeout(250);

        // Create and apply configuration for motor. Voltage compensation helps
        // the motor behave the same as the battery
        // voltage dips. The current limit helps prevent breaker trips or burning out
        // the motor in the event the practice stalls.
        SparkMaxConfig practiceConfig = new SparkMaxConfig();
        practiceConfig.voltageCompensation(DutyCycleMotorConstants.SINGLE_MOTOR_VOLTAGE_COMP);
        practiceConfig.smartCurrentLimit(DutyCycleMotorConstants.SINGLE_MOTOR_CURRENT_LIMIT);
        // practiceConfig.idleMode(IdleMode.kBrake);
        practiceConfig.idleMode(IdleMode.kCoast);
        dutyCycleMotor.configure(practiceConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
    }

    /**
     *  This is a method that makes the practice spin to your desired speed.
     *  Positive values make it spin forward and negative values spin it in reverse.
     * 
     * @param speedmotor speed from -1.0 to 1, with 0 stopping it
     */
    public void runDutyCycleMotor(double speed){
        dutyCycleMotor.set(speed);
    }

    // Change your command methods to use runEnd
    public Command ForwordSpin() {
        return this.runEnd(
            () -> runDutyCycleMotor(0.5), // Run at 50% speed while held
            () -> runDutyCycleMotor(0.0)  // Stop the motor when released
        );
    }

    public Command BackwardFastSpin() {
        return this.runEnd(
            () -> runDutyCycleMotor(-0.6), // Run reverse while held
            () -> runDutyCycleMotor(0.0)   // Stop the motor when released
        );
    }

    public Command BackwardSlowSpin() {
        return this.runEnd(
            () -> runDutyCycleMotor(-0.2),
            () -> runDutyCycleMotor(0.0)
        );
    }

    // The run once will have the motor continue to run until commanded to stop
    public Command ContinuousSlowForwardSpin() {
        return this.runOnce(
            () -> runDutyCycleMotor(-0.2)
        );
    }

}