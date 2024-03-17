package frc.robot.subsystems.ArmSubsytem;

import javax.lang.model.util.ElementScanner14;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends PIDSubsystem{

    private DutyCycleEncoder throughBoreEncoder;

    private TalonFX pivotController;
    private CANSparkMax intakeController;
    private CANSparkMax topShooterController;
    private CANSparkMax bottomShooterController;

    private int setpoint = 0;
    // 0 - Intake
    // 1 - Stow
    // 2 - Amp

    public ArmSubsystem(){
        super(ArmConstants.PIV_CONTROLLER);
        throughBoreEncoder = new DutyCycleEncoder(ArmConstants.ThroughBoreChannel);
        throughBoreEncoder.setDistancePerRotation(360);
        throughBoreEncoder.setDutyCycleRange(1/1025, 1024/1025);
        throughBoreEncoder.setPositionOffset(ArmConstants.encoderOffset);


        intakeController = new CANSparkMax(ArmConstants.intakeMotor, MotorType.kBrushless);
        topShooterController = new CANSparkMax(ArmConstants.topShooterMotor, MotorType.kBrushless);
        bottomShooterController = new CANSparkMax(ArmConstants.bottomShooterMotor, MotorType.kBrushless);
        pivotController = new TalonFX(ArmConstants.pivotMotor);
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        pivotController.set(output);
    }

    @Override
    protected double getMeasurement() {
        return throughBoreEncoder.getDistance();
    }

    public void incrementSetpoint() {
         if(setpoint < 0)
            setpoint = 0;
        
        if(setpoint >= 2)
            setpoint = 0;
        else 
            setpoint++;
    }

    public void decrementSetpoint() {
        if(setpoint > 2)
            setpoint = 2;
        
        if(setpoint <= 0)
            setpoint = 2;
        else 
            setpoint++;
    }

    public Command getDefualt() {
        return run(() -> {
            topShooterController.set(.5);
            bottomShooterController.set(.5);
            switch (setpoint) {
                case 0:
                    setSetpoint(0);
                    break;
                case 1:
                    setSetpoint(90);
                    break;
                case 2:
                    setSetpoint(115);
                    break;
            
                default:
                    System.out.println("I do not know how this happened but somehow [setpoint] isn't 0, 1, or 2");
                    break;
            }
        });
    }

    public Command setIntakeSpeed(double speed) {
        return run(() -> {
            intakeController.set(speed);
        });
    }

    public Command setShooterSpeed(double speed) {
        return run(() -> {
            topShooterController.set(speed);
            bottomShooterController.set(speed);
        });
    }
    //TODO: Please make this work
    public Command aim() {
        return new Command() {
            
        };
    }
    
}