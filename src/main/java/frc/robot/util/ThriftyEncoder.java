// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


//Credit to https://github.com/entech281/Robot2024
package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;

/**
 * The {@code ThriftyEncoder} class contains fields and methods pertaining to
 * the function of the absolute encoder.
 */
public class ThriftyEncoder {
    private AnalogInput analogInput;
    private boolean inverted;
    private double positionOffset;

    public ThriftyEncoder(int port, double calibrationK) {
        this.analogInput = new AnalogInput(port);
        this.positionOffset = calibrationK;
        this.inverted = false;
        this.positionOffset = 0.0;
    }

    /**
     * Returns the current raw position of the absolute encoder.
     *
     * @return the current raw position of the absolute encoder in radians.
     */
    public Rotation2d getRotation() {
        return new Rotation2d(((inverted ? -1.0 : 1.0) //for whoever gets this next this is a ternary operator, basically a very simple if statement. if inverted == true, the val on the left of the : is used, of inverted == false, the val on the right is used
                * ((analogInput.getAverageVoltage() / RobotController.getVoltage5V()) * (Math.PI * 2) - Math.PI))-positionOffset);
    }

    /**
     * Inverts the absolute encoder.
     * 
     * @param inverted flag indicating if inverted.
     */
    public void setInverted(boolean inverted) {
        this.inverted = inverted;
    }

    /**
     * Sets the position offset between the raw position and the virtual position.
     * 
     * @param offset offset in radians
     */
    public void setPositionOffset(double offset) {
        positionOffset = offset;
    }

    /**
     * Returns the position offset between the raw position and the virtual
     * position.
     *
     * @return the position offset in radians.
     */
    public double getPositionOffset() {
        return positionOffset;
    }

    /**
     * Returns the virtual position of the absolute encoder (raw position minus
     * offset).
     *
     * @return the virtual position in radians.
     */

}