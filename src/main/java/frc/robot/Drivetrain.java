// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Timer;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.server.PathPlannerServer;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.*;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SerialPort;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain extends SubsystemBase{
  public static final double kMaxSpeed = 3.0; // 3 meters per second
  public static final double kMaxAngularSpeed = Math.PI * .6; // 1/2 rotation per second

  private final Translation2d m_frontLeftLocation = new Translation2d(0.288925, 0.29845);
  private final Translation2d m_frontRightLocation = new Translation2d(0.288925, -0.29845);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.288925, 0.29845);
  private final Translation2d m_backRightLocation = new Translation2d(-0.288925, -0.29845);

  public double startingHeading = 0;

  public static final SwerveModule m_frontLeft = new SwerveModule("fl", Constants.FL_DRIVE_ID, Constants.FL_TURN_ID, Constants.FL_MA3_ID, Constants.DRIVE_MAX_V, Constants.FL_K);
  public static final SwerveModule m_frontRight = new SwerveModule("fr", Constants.FR_DRIVE_ID, Constants.FR_TURN_ID, Constants.FR_MA3_ID, Constants.DRIVE_MAX_V, Constants.FR_K);
  public static final SwerveModule m_backLeft = new SwerveModule("bl", Constants.BL_DRIVE_ID, Constants.BL_TURN_ID, Constants.BL_MA3_ID, Constants.DRIVE_MAX_V, Constants.BL_K);
  public static final SwerveModule m_backRight = new SwerveModule("br", Constants.BR_DRIVE_ID, Constants.BR_TURN_ID, Constants.BR_MA3_ID, Constants.DRIVE_MAX_V, Constants.BR_K);

  public static final AHRS navx = new AHRS(SerialPort.Port.kUSB);
  public final PhotonCamera camera1 = new PhotonCamera("Cam1");

  public PhotonCameraWrapper pcw;
  
  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  
  public SwerveDrivePoseEstimator m_poseEstimator;


  public Drivetrain() {
    //navx = new AHRS(I2C.Port.kMXP);
    pcw = new PhotonCameraWrapper();
    Pose2d initialPose;
    if(Constants.AUTO == 1){
      if(Constants.IS_BLUE){
        initialPose = Constants.AutonomousPaths.path1_1.getInitialHolonomicPose();
      }
      else{
        initialPose = Constants.AutonomousPaths.path1_1Red.getInitialHolonomicPose();
        SmartDashboard.putNumber("Initial pose x", initialPose.getX());
      }
    } 
    else{
      initialPose = new Pose2d();
      
    }

    startingHeading = initialPose.getRotation().getDegrees();

    Robot.m_swerve.navx.reset();
    /* Here we use SwerveDrivePoseEstimator so that we can fuse odometry readings. The numbers used
    below are robot specific, and should be tuned. */
    
    m_poseEstimator =
      new SwerveDrivePoseEstimator(
          m_kinematics,
          Rotation2d.fromDegrees(getHeading()),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
          },
          initialPose,
          VecBuilder.fill(0.02, 0.02, Units.degreesToRadians(5)),
          VecBuilder.fill(1.15, 1.15, Units.degreesToRadians(30)));
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    double heading = getHeading();
    if(!Constants.IS_BLUE && fieldRelative){
      xSpeed = -xSpeed;
      ySpeed = -ySpeed;
      rot = -rot;
    }
    var swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, Rotation2d.fromDegrees(heading))
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
    SmartDashboard.putNumber("Drive X", xSpeed);
    SmartDashboard.putNumber("Drive Y", ySpeed);
    SmartDashboard.putNumber("Drive Rot", rot);
    
  }

  public Pose2d getVelocity(){
    // Convert to chassis speeds
    ChassisSpeeds chassisSpeeds = m_kinematics.toChassisSpeeds(
      m_frontLeft.getState(), m_frontRight.getState(), m_backLeft.getState(), m_backRight.getState());

    // Getting individual speeds
    double forward = chassisSpeeds.vxMetersPerSecond;
    double sideways = chassisSpeeds.vyMetersPerSecond;
    Translation2d vel = new Translation2d(forward, sideways).rotateBy(Rotation2d.fromDegrees(getHeading()));

    return new Pose2d(vel, vel.getAngle());
  }

  public Pose2d getPose(){
    
    return m_poseEstimator.getEstimatedPosition();
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_poseEstimator.update(
        Rotation2d.fromDegrees(getHeading()),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
        });

      
      Optional<EstimatedRobotPose> result =
              pcw.getEstimatedGlobalPose(m_poseEstimator.getEstimatedPosition());

      if (result.isPresent()) {
          EstimatedRobotPose camPose = result.get();
          m_poseEstimator.addVisionMeasurement(
                  camPose.estimatedPose.toPose2d(), camPose.timestampSeconds);
      }
       
    /* 
    // Also apply vision measurements. We use 0.3 seconds in the past as an example -- on
    // a real robot, this must be calculated based either on latency or timestamps.
    if(Vision.hasNewTarget){
      Pose2d estimate = Vision.getPositionEstimate();
      double distance = Math.sqrt((estimate.getX() - getPose().getX())*(estimate.getX() - getPose().getX()) + (estimate.getY() - getPose().getY())*(estimate.getY() - getPose().getY()));
      SmartDashboard.putNumber("Distance error", distance);
      if(distance < 1.0){
        m_poseEstimator.addVisionMeasurement(
            Vision.getPositionEstimate(),
            Vision.timestamp,
            VecBuilder.fill(0.01, 0.01, 0.01)); //This 3d vector represents the standard deviation of vision position estimates, as x (m), y (m), and theta (radians)
            // currently these values are ones I chose as arbitrary, small numbers, but eventually it might be better to actually quantify these values
      }
    }
    */
    
  }

  public PIDController xController = new PIDController(1.0, 0.04, 0.00);
  public PIDController yController = new PIDController(1.0, 0.04, 0.00);
  public PIDController rotController = new PIDController(1.0, 0.02, 0.00);

  public void matchPath(PathPlannerState state){

    Pose2d targetPose = state.poseMeters; // remember, rotation isn't actually robot rotation, it is the velocity angle. For robot rotation, use holonomic
    Translation2d vel = new Translation2d(state.velocityMetersPerSecond, 0);
    vel = vel.rotateBy(targetPose.getRotation());

    //add togather velocity that trajectory wants us to go, and pid loop correction for positional drift over time.
    double driveX = vel.getX() + xController.calculate(getPose().getX(), targetPose.getX());
    double driveY = vel.getY() + yController.calculate(getPose().getY(), targetPose.getY());

    double holonomicAngle = state.holonomicRotation.getDegrees();

    double heading = getHeading();

    while(Math.abs(holonomicAngle - heading) > 180){
      if(holonomicAngle < heading){
        holonomicAngle += 360;
      } else{
        heading += 360;
      }
    }

    double driveRot = state.holonomicAngularVelocityRadPerSec + rotController.calculate(Math.toRadians(heading), Math.toRadians(holonomicAngle));

    //sends path information to the server so that it can be visualized on the driverstation
    PathPlannerServer.sendPathFollowingData(new Pose2d(targetPose.getTranslation(), state.holonomicRotation), getPose());
    
    drive(driveX, driveY, driveRot, true);
  }

  public double getHeading(){
    return -navx.getAngle() + startingHeading;
  }

  public void reset(Pose2d initialPose){
    startingHeading = initialPose.getRotation().getDegrees();
    m_poseEstimator.resetPosition(Rotation2d.fromDegrees(getHeading()), new SwerveModulePosition[] { m_frontLeft.getPosition(), m_frontRight.getPosition(), m_backLeft.getPosition(), m_backRight.getPosition()}, initialPose);
  }

}
