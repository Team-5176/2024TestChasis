package frc.robot.util;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalSource;

public class LidarLite{

//Use this for the offset from the edge of the robot and or to make up for differences in the Sensor
private static final int CALIBRATION_OFFSET = 0;

private Counter counter;

private int printedWarningCount = 5;



//takes the DIO pin on the roboRIO that the LIDARLite is attached to ex: new DigitalSource(PinNumberOnRIO)
public LidarLite (DigitalSource source) {
	counter = new Counter(source);
    counter.setMaxPeriod(1.0);
    // Configure for measuring rising to falling pulses
    counter.setSemiPeriodMode(true);
    counter.reset();
}

//returns the distance the sensor detects in meters
public double getDistance() {

    double meters;

        //if we havent seen a pulse we don't have a measurement, and the sensor likely isnt plugged in
    if (counter.get() < 1) {
		if (printedWarningCount-- > 0) {
			System.out.println("LidarLitePWM: waiting for distance measurement");
		}
		return 0;
    }
        meters = (counter.getPeriod() * 1000.0) + CALIBRATION_OFFSET;
	return meters;
	

}

}