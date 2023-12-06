// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import java.util.Collection;
import java.util.Map;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.util.LidarLite;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.util.MA3AnalogEncoder;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.RelativeEncoder;



public class Robot extends TimedRobot {
  public static final XboxController m_controller = new XboxController(0);
  public static final XboxController m_copilot_controller = new XboxController(1);
  public static Drivetrain m_swerve;
  private final ObjectManipulatorSubsystem manipulator = new ObjectManipulatorSubsystem();
  private final ManipulatorCommand manipulatorCommand = new ManipulatorCommand(manipulator);
  private Auto auto;// = new Auto(manipulator, m_swerve, 0);

  // Slew rate limiters to make joystick inputs more gentle; 1/2 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(2);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(2);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(2);
  
  //LidarSensor
  private final LidarLite Lidar = new LidarLite(new DigitalInput(2));


  //variables to be populated from SmartDashboard, determine what auto to do
  private boolean balance = false;

  public static boolean isAuto;
  

  
  public Robot(){
    manipulator.setDefaultCommand(manipulatorCommand);
    m_swerve = new Drivetrain();    
    
  }

  @Override
  public void robotInit() {
    PathPlannerServer.startServer(5811);
    //Send these values to SmartDashboard so that they can be used to choose what auto to do. 
    //SmartDashboard.putBoolean("Attempt Charging Station", false);
    //SmartDashboard.putNumber("Starting position", 0);
    //gets alliance color from driverStation and sets IS_BLUE acordingly;
    if(DriverStation.getAlliance() == Alliance.Red)
      Constants.IS_BLUE = false;
    else
      Constants.IS_BLUE = true;
  }

  @Override
  public void robotPeriodic(){
    CommandScheduler.getInstance().run();

  }

  @Override
  public void autonomousInit(){
    balance = SmartDashboard.getBoolean("Attempt Charging Station", false);
    isAuto = true;
    
    auto = new Auto( 0, manipulator);

    //gets alliance color from driverStation and sets IS_BLUE acordingly;
    if(DriverStation.getAlliance() == Alliance.Red)
      Constants.IS_BLUE = false;
    else
      Constants.IS_BLUE = true;

    
    //m_swerve = new Drivetrain(new Pose2d());
    
    //auto.setRoute((int)Math.round(SmartDashboard.getNumber("Starting position", 0)));
    //schedule the autonomous command
    auto.schedule();
  }

  @Override
  public void autonomousPeriodic() {
    isAuto = true;
    m_swerve.updateOdometry();
    
  }

  @Override
  public void teleopInit(){
    manipulatorCommand.designateStep = 0;
    isAuto = false;
    if(m_swerve == null){
      //initiate swerve based on starting position specified in SmartDashboard. I rounded before converting to int just in case there are any double shenanigans
      m_swerve = new Drivetrain();
    }
    m_swerve.navx.reset();
  }

  @Override
  public void teleopPeriodic() {
    //orch.play();
    driveWithJoystick(false);
    double angle = m_swerve.getHeading();
    isAuto = false;
    m_swerve.updateOdometry();
    SmartDashboard.putNumber("Pos x", m_swerve.getPose().getX());
    SmartDashboard.putNumber("Pos y", m_swerve.getPose().getY());
    SmartDashboard.putNumber("Heading", angle);
    SmartDashboard.putNumber("navx raw heading", m_swerve.navx.getAngle());
    SmartDashboard.putBoolean("Navx connected", m_swerve.navx.isConnected());
    SmartDashboard.putBoolean("is blue", Constants.IS_BLUE);   
    SmartDashboard.putNumber("FL_ENCODER", m_swerve.m_frontLeft.m_turningEncoder.getRotation().getDegrees());
    SmartDashboard.putNumber("FR_ENCODER", m_swerve.m_frontRight.m_turningEncoder.getRotation().getDegrees());
    SmartDashboard.putNumber("BL_ENCODER", m_swerve.m_backLeft.m_turningEncoder.getRotation().getDegrees());
    SmartDashboard.putNumber("BR_ENCODER", m_swerve.m_backRight.m_turningEncoder.getRotation().getDegrees());
  }


  // These two methods get the left stick controller input and add a smooth deadzone
  private double getLeftY(){
    double response = 0;
    if(m_controller.getLeftY() > Constants.CONTROLLER_DRIVE_DEADZONE){
      response = (m_controller.getLeftY() - Constants.CONTROLLER_DRIVE_DEADZONE);
    }  else if(m_controller.getLeftY() < -Constants.CONTROLLER_DRIVE_DEADZONE){
      response = (m_controller.getLeftY() + Constants.CONTROLLER_DRIVE_DEADZONE);
    }
    return -response;
  }
  private double getLeftX(){
    double response = 0;
    if(m_controller.getLeftX() > Constants.CONTROLLER_DRIVE_DEADZONE){
      response = (m_controller.getLeftX() - Constants.CONTROLLER_DRIVE_DEADZONE);
    }  else if(m_controller.getLeftX() < -Constants.CONTROLLER_DRIVE_DEADZONE){
      response = (m_controller.getLeftX() + Constants.CONTROLLER_DRIVE_DEADZONE);
    }
    return -response;
  }

  private void driveWithJoystick(boolean fieldRelative) {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    var xSpeed = -m_xspeedLimiter.calculate(getLeftY()) * Drivetrain.kMaxSpeed;
    
    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    var ySpeed = -m_yspeedLimiter.calculate(getLeftX()) * Drivetrain.kMaxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    var rot = -m_rotLimiter.calculate(m_controller.getRightX()) * Drivetrain.kMaxAngularSpeed;

    if(!Constants.IS_BLUE){
      xSpeed = -xSpeed;
      ySpeed = -ySpeed;
    }
    m_swerve.drive(xSpeed, ySpeed, rot, fieldRelative);
    
    SmartDashboard.putNumber("Lidar Readout", Lidar.getDistance());
    
  }


 
}
