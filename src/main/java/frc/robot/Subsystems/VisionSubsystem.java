// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase {
  private final PhotonCamera camera = new PhotonCamera(VisionConstants.CAMERA_NAME);
  private final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);
  private final Transform3d cameraToRobot = new Transform3d(VisionConstants.CAMERA_TO_ROBOT_X, VisionConstants.CAMERA_TO_ROBOT_Y, VisionConstants.CAMERA_TO_ROBOT_Z, new Rotation3d());
  private final Field2d simulatedField = new Field2d();

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem() {}

  @Override
  public void periodic() {

    //////////////////////////////////////////////////////////////////////
    // PhotonVision
    PhotonTrackedTarget target = getTarget();

    if (target != null) {
      SmartDashboard.putNumber("Vision_Target_ID", target.getFiducialId());
      SmartDashboard.putNumber("Vision_Target_Yaw", target.getYaw());
      SmartDashboard.putNumber("Vision_Target_Pitch", target.getPitch());
      SmartDashboard.putNumber("Vision_Target_Area", target.getArea());
      SmartDashboard.putNumber("Vision_Target_Skew", target.getSkew());

      Pose3d estimatedPose = estimateRobotPose();
      if (estimatedPose != null) {
        simulatedField.setRobotPose(estimatedPose.toPose2d());
        SmartDashboard.putData("PoseFromAprilTag", simulatedField);
      }
    }
    //////////////////////////////////////////////////////////////////////
  }

  /**
   * Get the latest unread result.
   * @return Latest result.
   */
  private PhotonPipelineResult getLatestResult() {
    return camera.getLatestResult();
  }

  /**
   * Get the latest best apriltag target.
   * @return Best target, null if not found.
   */
  public PhotonTrackedTarget getTarget() {
    return getLatestResult().hasTargets() ? getLatestResult().getBestTarget() : null;
  }

  /**
   * Get the target with the corresponding fiducial ID.
   * @param fiducialId The attached ID
   * @return Target, null if not found.
   */
  public PhotonTrackedTarget getTarget(int fiducialId) {
    PhotonPipelineResult result = getLatestResult();
    if (!result.hasTargets()) {
      for (PhotonTrackedTarget target : result.getTargets()) {
        if (target.getFiducialId() == fiducialId) {
          return target;
        }
      }
    }
    return null;
  }

  /**
   * Estimates the robot's position based on a tracked april tag
   * @return {@code Pose3d} containing an estimate robot position. Returns null if position could not be estimated.
   */
  public Pose3d estimateRobotPose() {
    PhotonTrackedTarget target = getTarget();

    if (target == null || target.getFiducialId() == -1) {
      return null;
    }

    Optional<Pose3d> tagPose = aprilTagFieldLayout.getTagPose(target.getFiducialId());
    if (tagPose.isEmpty()) {
      return null;
    }

    return PhotonUtils.estimateFieldToRobotAprilTag(
      target.getBestCameraToTarget(),
      tagPose.get(),
      cameraToRobot
    );
  }
}
