package frc.robot.util;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.math.geometry.Rotation2d;

// https://github.com/Team254/FRC-2016-Public/blob/master/src/com/team254/lib/util/MA3AnalogEncoder.java

/**
 * A 10-bit analog MA3 absolute encoder.
 * http://cdn.usdigital.com/assets/datasheets/MA3_datasheet.pdf
 */
public class MA3AnalogEncoder {

    public final AnalogInput mAnalogInput;

    protected Rotation2d rotation_ = new Rotation2d();
    protected Rotation2d home_ = new Rotation2d();
    protected int num_rotations_ = 0;
    protected double maxv;
    protected double calibrationK;

    public CrashTrackingRunnable read_thread_ = new CrashTrackingRunnable() {
        @Override
        public void runCrashTracked() {
            Rotation2d new_rotation = new Rotation2d(2 * Math.PI * mAnalogInput.getVoltage() / maxv); 

            // Check for rollover
            synchronized (MA3AnalogEncoder.this) {
                /* Not sure what this chunk does, don't think it is neccesary
                double relative_angle = rotation_.getRadians() 
                        + rotation_.inverse().rotateBy(new_rotation).getRadians();
                if (relative_angle > Math.PI) {
                    ++num_rotations_;
                } else if (relative_angle < -Math.PI) {
                    --num_rotations_;
                } */

                rotation_ = new_rotation;
            }
        }
    };
    //*/
    public MA3AnalogEncoder(int port, double maxv, double calibrationK) {
        mAnalogInput = new AnalogInput(port);
        this.maxv = maxv;
        this.calibrationK = calibrationK;
    }

    /*  public synchronized Rotation2dUtil getCalibratedAngle() {
        return home_.rotateBy(rotation_);
    }

    public synchronized void zero() {
        num_rotations_ = 0;
        home_ = rotation_.inverse();
    }

    public synchronized Rotation2dUtil getRawAngle() {
        return rotation_;
    }

    public synchronized double getContinuousAngleDegrees() {
        return getRawAngle().getDegrees() + num_rotations_ * 360.0 + home_.getDegrees();
    }

    public synchronized double get5176Angle() {
        double degrees = getRawAngle().getDegrees();
        // sanity checks
        if (degrees > 180d) degrees = 180d;
        if (degrees < -180d) degrees = -180d;
        double corrected = degrees + 180d + calibrationK;
        if (corrected < 0d) {
            corrected += 360d;
        } else if (corrected > 360d) {
            corrected -= 360d;
        }
        return corrected;
    }
*/
    public double voltage = 0.0;
    public Rotation2d getRotation(){
        //return rotation_;
        voltage = mAnalogInput.getVoltage();
        Rotation2d rawAngle = new Rotation2d(2 * Math.PI * voltage / maxv); //raw encoder rotation, the 2 is negative because SwerveModule reads counterclockwise as increasing
        Rotation2d calibratedAngle = Rotation2d.fromDegrees(rawAngle.getDegrees() - calibrationK);
        if(calibratedAngle.getDegrees() > 180){
            calibratedAngle = Rotation2d.fromDegrees(-180 + (calibratedAngle.getDegrees() - 180));
        }
        else if(calibratedAngle.getDegrees() < -180){
            calibratedAngle = Rotation2d.fromDegrees(180 + (calibratedAngle.getDegrees() + 180));
        }
        return calibratedAngle;
    }


}