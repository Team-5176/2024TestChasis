package frc.robot.subsystems.Climber;

import frc.robot.Constants.ClimberConstants;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class climberSubsystem extends SubsystemBase {
    
    private TalonFX leftTalon = new TalonFX(ClimberConstants.leftClimberMotor);
    private TalonFX rightTalon = new TalonFX(ClimberConstants.rightClimberMotor);

    public climberSubsystem(){

    }

    public Command getClimberCommand(DoubleSupplier leftClimberAxis, DoubleSupplier rightClimberAxis){
        return run(() -> {
            if(Math.abs(leftClimberAxis.getAsDouble()) > 0 || Math.abs(rightClimberAxis.getAsDouble()) > 0){
                leftTalon.set(leftClimberAxis.getAsDouble());
                rightTalon.set(rightClimberAxis.getAsDouble());
                }
            else{
                leftTalon.setControl(new StaticBrake());
                rightTalon.setControl(new StaticBrake());
            }
        });
    }

}
