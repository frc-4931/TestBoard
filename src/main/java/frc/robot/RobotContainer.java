// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.DutyCycleMotor;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.subsystems.VelocityControlledMotor;

enum ControlType {
DUTY_CYCLE,         // Percent output (-1.0 to 1.0)
VELOCITY_CONTROL,   // RPM based
POSITION_CONTROL    // Rotation/Encoder based
}

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Select the type of motor to use
  private final ControlType controlType = ControlType.DUTY_CYCLE;

  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private VelocityControlledMotor velocityControlledMotor;
  private DutyCycleMotor dutyCycleMotor;

//   private final VelocityControlledMotor velocityControlledMotor = new VelocityControlledMotor();
//   private final DutyCycleMotor dutyCycleMotor = new DutyCycleMotor();
  
  final         CommandXboxController driverXbox = new CommandXboxController(0);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (controlType) {
      case DUTY_CYCLE:
        dutyCycleMotor = new DutyCycleMotor();
        break;
      case VELOCITY_CONTROL:
        // Note: requires PID controller setup on the SparkMax
        velocityControlledMotor = new VelocityControlledMotor();
        break;
      case POSITION_CONTROL:
          break;
    }
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
        switch (controlType) {
      case DUTY_CYCLE:
        driverXbox.a().whileTrue(dutyCycleMotor.ForwordSpin());
        driverXbox.b().whileTrue(dutyCycleMotor.BackwardSlowSpin());
        driverXbox.x().whileTrue(dutyCycleMotor.BackwardFastSpin());
        break;
      case VELOCITY_CONTROL:
        driverXbox.a().whileTrue(velocityControlledMotor.BackwardSlowSpin());
        driverXbox.b().whileTrue(velocityControlledMotor.ForwordSlowSpin());
        driverXbox.povUp().whileTrue(
          velocityControlledMotor.sysIdQuasistatic(Direction.kForward)
          .onlyIf(DriverStation::isTest)
          );
        driverXbox.povDown().whileTrue(
          velocityControlledMotor.sysIdQuasistatic(Direction.kReverse)
          .onlyIf(DriverStation::isTest)
          );
        driverXbox.povRight().whileTrue(
          velocityControlledMotor.sysIdDynamic(Direction.kForward)
          .onlyIf(DriverStation::isTest)
          );
        driverXbox.povLeft().whileTrue(
          velocityControlledMotor.sysIdDynamic(Direction.kReverse)
          .onlyIf(DriverStation::isTest)
          );
        break;
      case POSITION_CONTROL:
          break;
    }
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

    // These will only execute if the Robot is physically put into "Test Mode"

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
