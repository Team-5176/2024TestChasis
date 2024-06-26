// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;
import swervelib.parser.PIDFConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag

  public static final class Auton
  {

    public static final PIDFConfig TranslationPID = new PIDFConfig(0.7, 0, 0);
    public static final PIDFConfig angleAutoPID   = new PIDFConfig(0.4, 0, 0.01);

    public static final double MAX_ACCELERATION = 2;
  }

  public static final class Drivebase
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class DriverConstants
  {

    // Joystick Deadband
    public static final double LEFT_X_DEADBAND  = 0.01;
    public static final double LEFT_Y_DEADBAND  = 0.01;
    public static final double RIGHT_X_DEADBAND = 0.01;
    public static final double TURN_CONSTANT    = 6;
  }

  public static class PassengerConstants
  {

    public static final double LEFT_Y_DEADBAND  = 0.01;
    public static final double RIGHT_Y_DEADBAND  = 0.01;

  }

  public static final class ClimberConstants
  {

    public static final int leftClimberMotor = 11;
    public static final int rightClimberMotor = 12;

  }

  public static final class ArmConstants
  {
    public static final int pivotMotor = 10;
    public static final double gearRatio = 1/300;

    public static final int topShooterMotor = 13;
    public static final int bottomShooterMotor = 15;
    public static final int intakeMotor = 14;

    public static final int ThroughBoreChannel = 0;
    public static final double encoderOffset = -110;

    public static final PIDController PIVOT_CONTROLLER = new PIDController(0, 0, 0);

    public static final double IntakeAngle = 0;
    
    public static final double StowAngle = 90;

    public static final double AmpAngle = 115;
  }

}