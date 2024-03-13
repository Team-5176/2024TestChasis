package frc.robot.subsystems.Climber;

import frc.robot.Constants.ClimberConstants;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class climberSubsystem extends SubsystemBase {
    
    private TalonFX leftTalon = new TalonFX(ClimberConstants.leftClimberMotor);
    private TalonFX rightTalon = new TalonFX(ClimberConstants.rightClimberMotor);

    private boolean brakeEnabled = false;

    public climberSubsystem(){
        brakeEnabled = false;
    }

    public void toggleBrake(){

        brakeEnabled = !brakeEnabled;

        if(brakeEnabled){
            leftTalon.setControl(new StaticBrake());
            rightTalon.setControl(new StaticBrake());
        }

    }

    public void changeHeight(DoubleSupplier leftClimberAxis, DoubleSupplier rightClimberAxis){
        leftTalon.set(leftClimberAxis.getAsDouble());
        rightTalon.set(rightClimberAxis.getAsDouble());
    }

}
