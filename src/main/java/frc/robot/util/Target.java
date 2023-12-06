package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

public class Target {
    public double x;
    public double y;
   // public double angle;
    public int id;
    public Pose3d pose;
    public Target(double x, double y, double z, int id){
        this.id = id;
        pose = new Pose3d(x, y, z, new Rotation3d());
    }
}
