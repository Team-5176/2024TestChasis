/*
 * MIT License
 *
 * Copyright (c) 2022 PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package frc.robot.util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.VisionConstants;
import java.io.IOException;
import java.io.UncheckedIOException;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;
 
 public class PhotonCameraWrapper {
     private PhotonCamera photonCamera;
     private PhotonPoseEstimator photonPoseEstimator;
 
     public PhotonCameraWrapper() {
         // Change the name of your camera here to whatever it is in the PhotonVision UI.
         photonCamera = new PhotonCamera(VisionConstants.cameraName);
 
         try {
             // Attempt to load the AprilTagFieldLayout that will tell us where the tags are on the field.
             AprilTagFieldLayout fieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
             // Create pose estimator
             photonPoseEstimator =
                     new PhotonPoseEstimator(
                             fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, photonCamera, VisionConstants.robotToCam);
             photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
         } catch (UncheckedIOException e) {
             // The AprilTagFieldLayout failed to load. We won't be able to estimate poses if we don't know
             // where the tags are.
             DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
             photonPoseEstimator = null;
         }
     }
 
     /**
      * @param estimatedRobotPose The current best guess at robot pose
      * @return an EstimatedRobotPose with an estimated pose, the timestamp, and targets used to create
      *     the estimate
      */
     public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
         if (photonPoseEstimator == null) {
             // The field layout failed to load, so we cannot estimate poses.
             return Optional.empty();
         }

         

         var result = photonCamera.getLatestResult();
        
        if(result.hasTargets()){ //checks if target exists
            // Get the current best target.
            PhotonTrackedTarget target = result.getBestTarget();
            
            Transform3d bestCameraToTarget = target.getBestCameraToTarget();

            
            double distance = Math.sqrt(bestCameraToTarget.getX()*bestCameraToTarget.getX() + bestCameraToTarget.getY()*bestCameraToTarget.getY());
            SmartDashboard.putNumber("Distance?", distance);
            if(distance > 5.0){
                return Optional.empty();
            }
        }
         photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
         return photonPoseEstimator.update();
     }
 }