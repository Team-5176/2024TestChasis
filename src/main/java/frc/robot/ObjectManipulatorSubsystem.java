package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import java.lang.reflect.Constructor;
import java.util.function.ToDoubleBiFunction;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ObjectManipulatorSubsystem extends SubsystemBase{
    
    private final CANSparkMax PivotNeo = new CANSparkMax(Constants.PIVOT_ID, MotorType.kBrushless);
    private final CANSparkMax LWheelsNeo = new CANSparkMax(Constants.LWHEELS_ID, MotorType.kBrushless);
    private final CANSparkMax RWheelsNeo = new CANSparkMax(Constants.RWHEELS_ID, MotorType.kBrushless);

    private final RelativeEncoder PivotEncoder = PivotNeo.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
    private final RelativeEncoder LWheelsEncoder = LWheelsNeo.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
    private final RelativeEncoder RWheelsEncoder = RWheelsNeo.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
    
    private final SimpleMotorFeedforward PivotFeedforwardController = new SimpleMotorFeedforward(0 * Constants.VOLTAGE_TO_PERCENT_POWER, 0 * Constants.VOLTAGE_TO_PERCENT_POWER);

    private boolean manualControl; 

    private final ProfiledPIDController PivotPIDController = 
        new ProfiledPIDController(
            3.0,
            0.0,
            0.0,
            new TrapezoidProfile.Constraints(
                1, 5));


    private final PIDController WheelsPIDController = new PIDController(0.05, 0.0, 0.0);

    double pivotSetPoint = 0;

    //there is a nonzero chance i'll need limit switches so put them here if i do


    public ObjectManipulatorSubsystem(){
        PivotNeo.setIdleMode(IdleMode.kBrake);
        LWheelsNeo.setIdleMode(IdleMode.kBrake);
        RWheelsNeo.setIdleMode(IdleMode.kBrake);
        LWheelsNeo.setInverted(true);
    }


    public void suckIn(){
        LWheelsNeo.set(.1);
        RWheelsNeo.set(.1);
    }

    public void pushOut(){
        LWheelsNeo.set(-.25);
        RWheelsNeo.set(-.25);

    }

    public void hold(){
        LWheelsNeo.set(0);
        RWheelsNeo.set(0);
    }

    public void pivotOut(double speed){
        manualControl = true;
        PivotNeo.set(speed);
    }

    public void pivotIn(double speed){
        manualControl = true;
        PivotNeo.set(-speed);
    }

    public void pivotLock(){
        PivotNeo.set(0);
    }


    //Returns intake position in degrees, vertical is 90;
    public double getPivotPos(){
        double pos = Constants.INTAKE_START_POS;
        pos += (PivotEncoder.getPosition()*360)/64;
        return pos;
    }

    public void setPivotSetPoint(double pivotSetPoint) {
        this.pivotSetPoint = pivotSetPoint;
    }


    //returns the speed of the intake wheels between -1 and 1, assumes both sides share the same speed, just inverted
    public double getSpeed(){
        return LWheelsNeo.get();
    }

    public void setPivotPos(double pos){
        manualControl = false;
        pivotSetPoint = pos;
    }

    public void periodic(){
        

        double PivotPid = PivotPIDController.calculate(getPivotPos(), pivotSetPoint);

        if(Robot.isAuto == true){
            PivotNeo.set(PivotPid);
        }



    }
}
