package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SingleMotorConstants;

public class SingleMotor extends SubsystemBase {

    private final SparkMax SingleMotor;
    /**
     * This subsytem that controls the motor.
     */
    public SingleMotor () {

        // Set up the motor as a brushed motor
        SingleMotor = new SparkMax(SingleMotorConstants.SINGLE_MOTOR_ID, MotorType.kBrushless);

        // Set can timeout. Because this project only sets parameters once on
        // construction, the timeout can be long without blocking robot operation. Code
        // which sets or gets parameters during operation may need a shorter timeout.
        SingleMotor.setCANTimeout(250);

        // Create and apply configuration for motor. Voltage compensation helps
        // the motor behave the same as the battery
        // voltage dips. The current limit helps prevent breaker trips or burning out
        // the motor in the event the practice stalls.
        SparkMaxConfig practiceConfig = new SparkMaxConfig();
        practiceConfig.voltageCompensation(SingleMotorConstants.SINGLE_MOTOR_VOLTAGE_COMP);
        practiceConfig.smartCurrentLimit(SingleMotorConstants.SINGLE_MOTOR_CURRENT_LIMIT);
        practiceConfig.idleMode(IdleMode.kBrake);
        SingleMotor.configure(practiceConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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
    public void runSingleMotor(double speed){
        SingleMotor.set(speed);
    }

    public Command BackwardSlowSpin() {
        return this.runOnce(() -> { runSingleMotor(-.15);});
    }

    public Command SpinStop() {
        return this.runOnce(() -> { runSingleMotor(0);});
    }

    public Command ForwordSlowSpin() {
        return this.runOnce(() -> { runSingleMotor(15);});
    }
    
    public Command BackwardFastSpin() {
        return this.runOnce(() -> { runSingleMotor(5);});
    }

}