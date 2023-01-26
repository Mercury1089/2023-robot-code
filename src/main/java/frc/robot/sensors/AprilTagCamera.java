// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/** Wrapper for PhotonCamera class */
public class AprilTagCamera extends PhotonCamera {

    private static final String DEFAULT_CAM_NAME = "photonCamera";
    private static final double DEFAULT_CAM_X = 0.5; // .5m forward of center
    private static final double DEFAULT_CAM_Y = 0.0; // centered in robot Y
    private static final double DEFAULT_CAM_Z = 0.5; // .5m up from center
    private final double CAMERA_HEIGHT = 0.0; // height on robot (meters)
    private final double TARGET_HEIGHT = 0.36; // may need to change 
    private final int CAMERA_PITCH = 0; // tilt of our camera (radians)

    private AprilTagFieldLayout fieldLayout;
    private PhotonPoseEstimator estimator;

    public AprilTagCamera() {
        this(DEFAULT_CAM_NAME, DEFAULT_CAM_X, DEFAULT_CAM_Y, DEFAULT_CAM_Z);
    }

    public AprilTagCamera(String camName, double camX, double camY, double camZ) {
        super(camName);
        try {
            fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
        } catch (IOException e) {
            fieldLayout = null;
        }
        Transform3d robotToCam = new Transform3d(
            new Translation3d(camX, camY, camZ), new Rotation3d(0, 0, 0)
        );
        estimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.AVERAGE_BEST_TARGETS, this, robotToCam);
    }

    public double getDistanceToTarget() {
        PhotonPipelineResult result = getLatestResult();
        if (result.hasTargets()) {
            double range = PhotonUtils.calculateDistanceToTargetMeters(
                CAMERA_HEIGHT, TARGET_HEIGHT, CAMERA_PITCH, 
                Units.degreesToRadians(result.getBestTarget().getPitch()));
            return range;
        }
        return 0.0;
    }

    public Optional<EstimatedRobotPose> getGlobalPose() {
        return estimator.update();
    } 

}

