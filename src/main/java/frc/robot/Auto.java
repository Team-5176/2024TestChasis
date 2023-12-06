package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
//import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Auto extends CommandBase{
    //private Drivetrain m_swerve;
    
    private int route = 0;

    private Timer timeMan = new Timer();
    private double stateStartTime;
    private boolean isFinished = false;
    private ObjectManipulatorSubsystem manipulator;
    
    public Auto(int pathNumber, ObjectManipulatorSubsystem sub){
        manipulator = sub;
        route = pathNumber;
    }

    // This will load the file "Example Path.path" and generate it with a max velocity of 4 m/s and a max acceleration of 3 m/s^2
    
    private PathPlannerTrajectory path1;
    private PathPlannerTrajectory path2;
    private PathPlannerTrajectory path3;

    @Override
    public void initialize(){
        timeMan.start();
        stateStartTime = Timer.getFPGATimestamp();

        Robot.m_swerve.navx.reset();

        //SmartDashboard.putNumber("navx raw heading", Robot.m_swerve.navx.getAngle());

        if(Constants.IS_BLUE){
            if(Constants.AUTO == 1){
                path1 = Constants.AutonomousPaths.path1_1;
                //path2 = Constants.AutonomousPaths.path1_2;
            }
            else if(Constants.AUTO == 2){
                path1 = Constants.AutonomousPaths.path2_1;
            }
            else if(Constants.AUTO == 3){
                path1 = Constants.AutonomousPaths.path3_1;
            }
        }
        else{
             if(Constants.AUTO == 1){
                path1 = Constants.AutonomousPaths.path1_1Red;
            //path2 = Constants.AutonomousPaths.path1_2Red;
            }
            else if(Constants.AUTO == 2){
                path1 = Constants.AutonomousPaths.path2_1Red;
            }
            else if(Constants.AUTO == 3){
                path1 = Constants.AutonomousPaths.path3_1Red;
            }
        }

            
        Pose2d initialPose = path1.getInitialHolonomicPose();
        
        Robot.m_swerve.reset(initialPose);
        
        
        //reset PID loops (probably not actually needed)
        Robot.m_swerve.xController.reset();
        Robot.m_swerve.yController.reset();
        Robot.m_swerve.rotController.reset();


    }

    

    private int state = 0;

    private boolean started = false; // implemented this when we had a weird delay between initialize and the first execute. Probably not needed any more, but I am going to leave it
    @Override
    public void execute(){
        if(!started){
            //initially used for debuging
            //SmartDashboard.putNumber("time error", Timer.getFPGATimestamp() - stateStartTime);
            stateStartTime = Timer.getFPGATimestamp();
            started = true;
        }


        SmartDashboard.putNumber("Pos x", Robot.m_swerve.getPose().getX());
        SmartDashboard.putNumber("Pos y", Robot.m_swerve.getPose().getY());
        SmartDashboard.putNumber("Heading", Robot.m_swerve.getHeading());
        //SmartDashboard.putNumber("navx raw heading", Robot.m_swerve.navx.getAngle());
        

        if(true){
            auto1();
        }
         
       /* else if(Constants.AUTO == 2){
            auto2();
        }
        else if (Constants.AUTO == 3){
            auto3();
        } */
        
        
        //if navx is disconnected, stop autonomous
        if(!Robot.m_swerve.navx.isConnected()){
            isFinished = true;
        }
    }

    private void auto3(){
        if(state == 0){
            //SmartDashboard.putNumber("sdfjaoisdjfoaiejoijwofw", Timer.getFPGATimestamp() - stateStartTime);
            if(Timer.getFPGATimestamp() - stateStartTime > 2.0){
                
            }
            if(Timer.getFPGATimestamp() - stateStartTime > 3.50){
                state ++;
                stateStartTime = Timer.getFPGATimestamp();
            }
        }
        else if(state == 1){ //wait 3 seconds while the extendor extends and the grabber pivots out before openning the grabber and releasing the first cone
            if(Timer.getFPGATimestamp() - stateStartTime > 2.0){
        
            }
            if(Timer.getFPGATimestamp() - stateStartTime > 3.0){
                
                
                // These two things are done after every state change
                state ++;
                stateStartTime = Timer.getFPGATimestamp();
                //
            }
        }
        else if(state == 2){ //waits half a second so that the cone can drop before pulling the extendor in
            if(Timer.getFPGATimestamp() - stateStartTime > .5){ 
                state ++;
                stateStartTime = Timer.getFPGATimestamp();
            }
        }
        else if (state == 3){ // Begin driving to the first cube
            Robot.m_swerve.matchPath((PathPlannerState)path1.sample(Timer.getFPGATimestamp() - stateStartTime));
            if(Timer.getFPGATimestamp() - stateStartTime > 2.0){ // After it has been driving for two seconds, set the elevator all the way down and picot back the intake
                //manipulator.setElevator(Constants.ELEVATOR_MIN - 2);
            }
            if(path1.getTotalTimeSeconds() < Timer.getFPGATimestamp() - stateStartTime){ // After the path has been completed, close the grabber
                
                state += 1;
                stateStartTime = Timer.getFPGATimestamp();
            }
        }
        /*else if(state == 4){
            Robot.m_swerve.matchPath((PathPlannerState)path2.sample(Timer.getFPGATimestamp() - stateStartTime));
            if(Timer.getFPGATimestamp() - stateStartTime > 2){
                manipulator.setElevator(Constants.ELEVATOR_MAX + 2);
                manipulator.pivotForward();
                manipulator.setExtendor(10.0);
            }
            if(path2.getTotalTimeSeconds() < Timer.getFPGATimestamp() - stateStartTime){
                manipulator.openPincher();
                state ++;
                stateStartTime = Timer.getFPGATimestamp();
            }
        }*/
         else{ //After the autonomous is completed, exit the autonomous command
            isFinished = true;
        }
        //
    }

    private void auto1(){
        if(state == 0){
            //SmartDashboard.putNumber("sdfjaoisdjfoaiejoijwofw", Timer.getFPGATimestamp() - stateStartTime);
            if(Timer.getFPGATimestamp() - stateStartTime > 0){
                manipulator.setPivotSetPoint(170);
                            }
            if(Timer.getFPGATimestamp() - stateStartTime > .5){
                manipulator.pushOut();
                state ++;
                stateStartTime = Timer.getFPGATimestamp();
            }
        }
        else if(state == 1){ //wait 3 seconds while the extendor extends and the grabber pivots out before openning the grabber and releasing the first cone
            if(Timer.getFPGATimestamp() - stateStartTime > .5){
                manipulator.hold();
                manipulator.setPivotSetPoint(75);
                state++;
                state++;
                stateStartTime = Timer.getFPGATimestamp();
            }
        }
        else if(state == 2){ //waits half a second so that the cone can drop before pulling the extendor in
            if(Timer.getFPGATimestamp() - stateStartTime > .5){ 

                state ++;
                stateStartTime = Timer.getFPGATimestamp();
            }
        }
        else if (state == 3){ // Begin driving to the first cube
            Robot.m_swerve.matchPath((PathPlannerState)path1.sample(Timer.getFPGATimestamp() - stateStartTime));
            if(Timer.getFPGATimestamp() - stateStartTime > 2.0){ // After it has been driving for two seconds, set the elevator all the way down and picot back the intake
                //manipulator.setElevator(Constants.ELEVATOR_MIN - 2);
            }
            if(path1.getTotalTimeSeconds() < Timer.getFPGATimestamp() - stateStartTime){ // After the path has been completed, close the grabber
                state += 1;
                stateStartTime = Timer.getFPGATimestamp();
            }
        }
        /*else if(state == 4){
            Robot.m_swerve.matchPath((PathPlannerState)path2.sample(Timer.getFPGATimestamp() - stateStartTime));
            if(Timer.getFPGATimestamp() - stateStartTime > 2){
                manipulator.setElevator(Constants.ELEVATOR_MAX + 2);
                manipulator.pivotForward();
                manipulator.setExtendor(10.0);
            }
            if(path2.getTotalTimeSeconds() < Timer.getFPGATimestamp() - stateStartTime){
                manipulator.openPincher();
                state ++;
                stateStartTime = Timer.getFPGATimestamp();
            }
        }*/
         else{ //After the autonomous is completed, exit the autonomous command
            isFinished = true;
        }
        //
    }

    private void auto2(){
        if(state == 0){
            if(Timer.getFPGATimestamp() - stateStartTime > 1.0){

            }
        }
        else if(state == 1){
            if(Timer.getFPGATimestamp() - stateStartTime >3.0){
                state++;
                stateStartTime=Timer.getFPGATimestamp();
            }
        }

       else if(state == 2){
            if(Timer.getFPGATimestamp() - stateStartTime >.5){

                state++;
                stateStartTime=Timer.getFPGATimestamp();
            }
        }

       else if(state == 3){
            Robot.m_swerve.matchPath((PathPlannerState)path1.sample(Timer.getFPGATimestamp() - stateStartTime));
            if(path1.getTotalTimeSeconds() < Timer.getFPGATimestamp() - stateStartTime){ // After the path has been completed, close the grabber
                state ++;
                stateStartTime = Timer.getFPGATimestamp();
            }

        }
        else{
            isFinished = true;
        }
    }
    private void auto100000(){
        if(state == 0){
            if(Timer.getFPGATimestamp() - stateStartTime > 2.0){

                state ++;
                stateStartTime = Timer.getFPGATimestamp();
            }
        }
        else if (state == 1){ // Begin driving to the first cube
            Robot.m_swerve.matchPath((PathPlannerState)path1.sample(Timer.getFPGATimestamp() - stateStartTime));
            if(Timer.getFPGATimestamp() - stateStartTime > 2.0){ // After it has been driving for two seconds, set the elevator all the way down and picot back the intake

            }
            if(path1.getTotalTimeSeconds() < Timer.getFPGATimestamp() - stateStartTime){ // After the path has been completed, close the grabber
                state += 1;
                stateStartTime = Timer.getFPGATimestamp();
            }
        }else if(state == 2){
            Robot.m_swerve.matchPath((PathPlannerState)path2.sample(Timer.getFPGATimestamp() - stateStartTime));
            if(Timer.getFPGATimestamp() - stateStartTime > 2){

            }
            if(path2.getTotalTimeSeconds() < Timer.getFPGATimestamp() - stateStartTime){
                state ++;
                stateStartTime = Timer.getFPGATimestamp();
            }
        }
         else{ //After the autonomous is completed, exit the autonomous command
            isFinished = true;
        }
        //
    }


    @Override
    public boolean isFinished() {
        return isFinished;
    }
}
