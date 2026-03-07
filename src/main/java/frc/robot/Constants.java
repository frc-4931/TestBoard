// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class DutyCycleMotorConstants {
    public static final int SINGLE_MOTOR_ID = 10;
    public static final double SINGLE_MOTOR_VOLTAGE_COMP = 10.0;
    public static final int SINGLE_MOTOR_CURRENT_LIMIT = 60;
  }

  public static final class ControlledMotorConstants {
        public static final int MOTOR_ID = 10;
        public static final int MOTOR_CURRENT_LIMIT = 60;
        public static final double MOTOR_VOLTAGE_COMP = 10;
        public static final double MOTOR_SPIN = -.5;
  }
}
