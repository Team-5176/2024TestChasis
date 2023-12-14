// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.ThriftyEncoder;

public class SwerveModule {
  private static final double kWheelRadius = 0.0508; // in m
  private static final int kEncoderResolution = 2048;

  private static final double kModuleMaxAngularVelocity = Drivetrain.kMaxAngularSpeed;
  private static final double kModuleMaxAngularAcceleration =
      2 * Math.PI * 6; // radians per second squared

  public final CANSparkMax m_driveMotor;
  public final CANSparkMax m_turningMotor;


  public final ThriftyEncoder m_turningEncoder;
  public final RelativeEncoder m_driveEncoder;

  // Gains determined by guess and check method
  private final PIDController m_drivePIDController = new PIDController(0.1, 0.001, 0);

  // Gains determined by guess and check method
  private final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(
          3.0,
          0.01,
          0.05,
          // Constraints without these multipliers made response time far too slow, and changing the constants to reasonable numbers made overall movement too jerky
          // Should probably eventually make this cleaner
          new TrapezoidProfile.Constraints(
              kModuleMaxAngularVelocity * 300, kModuleMaxAngularAcceleration * 100));

  
  
  // The gains for this feedForward were determined using the SysId toolsuite, which calculates gains based on motor voltage, hence the conversion
  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(0.63988 * Constants.VOLTAGE_TO_PERCENT_POWER, 2.2288 * Constants.VOLTAGE_TO_PERCENT_POWER); 


  // ks = power level where motor first starts turning
  // kv = constant relating rotation speed to (input power - ks)
  // These values were determined experimentally
  private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(0.1, 0.00167);
  
  private String name;
  /**
   * Creates SwerveModule object
   * @param name used for putting debug information onto SmartDashboard
   * @param driveMotorId CAN ids for motor controllers
   * @param turningMotorId CAN ids for motor controllers
   * @param MA3AnalogId analog in port for this swerve module's MA3 encoder
   * @param maxv Maximum voltage before rollover of analog encoder
   * @param calibrationK offset calibration constant for each encoder
   */
  public SwerveModule(
      String name, // used for putting debug information onto SmartDashboard
      int driveMotorId, //CAN ids for motor controllers 
      int turningMotorId, 
      int MA3AnalogId, // analog in port for this swerve module's MA3 encoder
      double maxv, // Maximum voltage before rollover of analog encoder
      double calibrationK //offset calibration constant for each encoder
      ) {

    this.name = name;
    m_driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
    m_turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);
    m_turningEncoder = new ThriftyEncoder(MA3AnalogId, calibrationK);

    m_driveEncoder = m_driveMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
    m_driveEncoder.setPositionConversionFactor(Constants.DRIVE_COVNV_FACT);
    m_driveEncoder.setVelocityConversionFactor(Constants.DRIVE_COVNV_FACT);

    m_driveMotor.burnFlash();
    m_turningMotor.burnFlash();

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  private double getdriveVelocity(){
    //TODO: Make sure that this gets changed to work wtih the neo relative encoder
    double rotationsPerSecondofWheel = m_driveEncoder.getVelocity(); // Rev library automatically calculates velocity if given conversion factor(done in constructor)
    return rotationsPerSecondofWheel * kWheelRadius * 3.14159 * 2; // speed of the wheel treads in meters/second
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    
    return new SwerveModuleState(getdriveVelocity(), m_turningEncoder.getRotation());
  }

  public SwerveModulePosition getPosition(){
      //TODO: Make sure that this gets changed to work wtih the neo relative encoder
    double rotationsWheel = m_driveEncoder.getPosition(); // Rev library automatically calculates position if given conversion factor(done in constructor)
    return new SwerveModulePosition(rotationsWheel, m_turningEncoder.getRotation());
  }
  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state =
        SwerveModuleState.optimize(desiredState, m_turningEncoder.getRotation());

    
    // Calculate the drive output from the drive PID controller.
    final double driveOutput =
        m_drivePIDController.calculate(getdriveVelocity(), state.speedMetersPerSecond);

    final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

    
    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput =
        m_turningPIDController.calculate(m_turningEncoder.getRotation().getRadians(), state.angle.getRadians());

    final double turnFeedforward =
        m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

    

    //Print debug information
    SmartDashboard.putNumber(name + " velocity", getdriveVelocity());
    SmartDashboard.putNumber(name + " desiredVelocity", state.speedMetersPerSecond);
    SmartDashboard.putNumber(name + " desiredAngle", state.angle.getDegrees());
    SmartDashboard.putNumber(name + " currentAngle", getState().angle.getDegrees());
    SmartDashboard.putNumber(name + " drive output", driveOutput + driveFeedforward);
    //SmartDashboard.putNumber(name + " voltage", m_turningEncoder.voltage);

    //set motor powers
    m_driveMotor.set(-(driveOutput + driveFeedforward));
    m_turningMotor.set(turnOutput + turnFeedforward);
    
  }
}
