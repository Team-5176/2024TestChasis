// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    // 0.085297 is a constant that relates motor voltage to input percentage. It was experimentally determined for this motor type.
    public static final double VOLTAGE_TO_PERCENT_POWER = 0.085297;

    public final static double DRIVE_MAX_V = 4.79;

    public final static int FL_DRIVE_ID = 5;
    public final static int BL_DRIVE_ID = 7;
    public final static int BR_DRIVE_ID = 1;
    public final static int FR_DRIVE_ID = 3;

    public final static int FL_TURN_ID = 4;
    public final static int BL_TURN_ID = 2;
    public final static int BR_TURN_ID = 8;
    public final static int FR_TURN_ID = 6;

    public final static int FL_MA3_ID = 1;
    public final static int FR_MA3_ID = 2;
    public final static int BR_MA3_ID = 3;
    public final static int BL_MA3_ID = 0;

    public static final double ROTATION_DEADZONE = 0.1;
    public static final double ROTATION_COMPENSATOR_DELAY = 0.5; 

    public static final double AUTO_SPEED_LIMIT = 0.4;
    //90 - reportedAngle
    //reported angle is == what is reported with the wheel straight and the big part facing out
    /*public final static double FL_K = 90.0 - 123.93;
    public final static double FR_K = 90.0 - 328.8;
    public final static double BR_K = 90.0 - 91.8;
    public final static double BL_K = 90.0 - 203.75;*/

    public final static double CONTROLLER_DRIVE_DEADZONE = 0.1;
    public final static double FL_K = -20.146355 + 94;//-98.532904;//-158.347000;//90.0 - 197.247845;//339.726057;//340.826983; //i offset this one by 180 degrees, hopefully that does something
    public final static double FR_K = 126.514780;//117.432138;//90.0 - 195.412968;
    public final static double BR_K = 65.963834 + 180.0;//75.321707;//114.218204;//90.0 - 137.342005;
    public final static double BL_K = 31.651631 + 180.0;//-107.979622;//131.101973;//90.0 - 149.176963;

    public static final double SWERVE_DRIVE_MULTIPLIER = 1;
	public static final double SWERVE_TELEOP_MULTIPLIER = 1;//0.4;
	public static boolean ARE_WE_IN_TELEOP = false;

    //object manipulator constants
    public static final int PIVOT_ID = 9;
    public static final int LWHEELS_ID = 11;
    public static final int RWHEELS_ID = 12;
    public static final double INTAKE_START_POS = 90;
    public static final double INTAKE_MAX_POS = 200;
    public static final double INTAKE_MIN_POS = 80;


    /*pilot controller outputs
    public static final int ELEVATOR_UP = 2;
    public static final int ELEVATOR_DOWN = 3;
    public static final int CLOSE_GRABBER = 6;
    public static final int OPEN_GRABBER = 5;
    public static final int PIVOT_FORWARD = 4;
    public static final int PIVOT_BACK = 1; */
    public static final int EXECUTE_AUTO = 7; //TODO: pick actual binding
    public static final int STOP_AUTO = 8;
    public static final int SUCK_IN = 2;
    public static final int PUSH_OUT = 3;
    

    /*copilot controllor
    public static final int MOVE_ELEVATOR_MAX = 6;
    public static final int MOVE_ELEVATOR_MIN = 5;
    public static final int STOW_ELEVATOR = 4;
    public static final int EXTEND = 7;
    public static final int RETRACT = 8;
    public static final int DESIGNATE_DOWN = 1; //TODO: Pick actual bindings
    public static final int DESIGNATE_UP = 4;
    public static final int DESIGNATE_LEFT = 3;
    public static final int DESIGNATE_RIGHT = 2;
    public static final int DESIGNATE_RESET = 9; */
    public static final int PIVOT_OUT = 1;
    public static final int PIVOT_IN = 4;

    //
    

    static class VisionConstants {
        static final Transform3d robotToCam =
                new Transform3d(
                    //0.127
                        new Translation3d(0.127, 0.0, 0.4318),
                        new Rotation3d(
                                0, 0, //2.64
                                Math.toRadians(6.0))); // Cam mounted facing forward, half a meter forward of center, half a meter up
        // from center.
        static final String cameraName = "Cam1";
    }

    static class ScoringPositions {
        static final Pose2d[] blueScorePoints = {
            // order is left to right from driver's point of view, with 
            new Pose2d(1.93, 4.97, Rotation2d.fromDegrees(180)),
            new Pose2d(1.93, 4.41, Rotation2d.fromDegrees(180)),
            new Pose2d(1.93, 3.87, Rotation2d.fromDegrees(180)),
            new Pose2d(1.93, 3.33, Rotation2d.fromDegrees(180)),
            new Pose2d(1.93, 2.75, Rotation2d.fromDegrees(180)),
            new Pose2d(1.93, 4.41, Rotation2d.fromDegrees(180)),
            new Pose2d(1.93, 2.17, Rotation2d.fromDegrees(180)),
            new Pose2d(1.93, 1.64, Rotation2d.fromDegrees(180)),
            new Pose2d(1.93, 1.07, Rotation2d.fromDegrees(180)),
            new Pose2d(1.93, .59, Rotation2d.fromDegrees(180)),
            new Pose2d(1.56, 7.34, Rotation2d.fromDegrees(0))

        };

        static final Pose2d[] redScorePoints = {

            new Pose2d(14.6, 4.97, Rotation2d.fromDegrees(0)),
            new Pose2d(14.6, 4.41, Rotation2d.fromDegrees(0)),
            new Pose2d(14.6, 3.87, Rotation2d.fromDegrees(0)),
            new Pose2d(14.6, 3.33, Rotation2d.fromDegrees(0)),
            new Pose2d(14.6, 2.75, Rotation2d.fromDegrees(0)),
            new Pose2d(14.6, 4.41, Rotation2d.fromDegrees(0)),
            new Pose2d(14.6, 2.17, Rotation2d.fromDegrees(0)),
            new Pose2d(14.6, 1.64, Rotation2d.fromDegrees(0)),
            new Pose2d(14.6, 1.07, Rotation2d.fromDegrees(0)),
            new Pose2d(14.6, .59, Rotation2d.fromDegrees(0)),
            new Pose2d(.87, 7.45, Rotation2d.fromDegrees(180))

        };
        
    }

    // Auto 1: Starts on blue driver right, places cube high, gets mobility
    // Auto 2: Starts middle, places cube high, tries to cross charging station
    // Auto 3: 
    // places cube middle then exits over charging stations
    public static int AUTO = 1;
    public static boolean IS_BLUE = false;
    static class AutonomousPaths {
        static final PathPlannerTrajectory path1_1 = PathPlanner.loadPath("1-1", new PathConstraints(2.5, 1.0));
        static final PathPlannerTrajectory path1_2 = PathPlanner.loadPath("1-2", new PathConstraints(2.5, 1.0));
        static final PathPlannerTrajectory path2_1 = PathPlanner.loadPath("2-1", new PathConstraints(2.5, 1.0));

        static final PathPlannerTrajectory path1_1Red = PathPlanner.loadPath("1-1 Red", new PathConstraints(2.5, 1.0));
        static final PathPlannerTrajectory path1_2Red = PathPlanner.loadPath("1-2 Red", new PathConstraints(2.5, 1.0));
        static final PathPlannerTrajectory path2_1Red = PathPlanner.loadPath("2-1 Red", new PathConstraints(2.5, 1.0));
    
        static final PathPlannerTrajectory path3_1 = PathPlanner.loadPath("3-1", new PathConstraints(2.5, 1.0));
        static final PathPlannerTrajectory path3_1Red = PathPlanner.loadPath("3-1 Red", new PathConstraints(2.5, 1.0));
    }
}
