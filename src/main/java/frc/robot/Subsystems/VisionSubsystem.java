// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase {
  private final PhotonCamera camera = new PhotonCamera(VisionConstants.CAMERA_NAME);

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem() {}

  @Override
  public void periodic() {
    PhotonPipelineResult result = getLatestResult();

    SmartDashboard.putNumber("Vision_Target_Yaw", 0);
    SmartDashboard.putNumber("Vision_Target_Pitch", 0);
    SmartDashboard.putNumber("Vision_Target_Area", 0);
    SmartDashboard.putNumber("Vision_Target_Skew", 0);
    SmartDashboard.putString("Vision_Target_Pose", "[]");
    SmartDashboard.putString("Vision_Target_Corners", "[]");

    if (result == null) {
      SmartDashboard.putString("Vision_Target_Status", "Result not found.");
      SmartDashboard.putBoolean("Vision_Target_Status_Bool", false);
      return;
    } else if (!result.hasTargets()) {
      SmartDashboard.putString("Vision_Target_Status", "Result found, no targets found.");
      SmartDashboard.putBoolean("Vision_Target_Status_Bool", false);
      return;
    }

    SmartDashboard.putString("Vision_Target_Status", "Result found, receiving target data.");
    SmartDashboard.putBoolean("Vision_Target_Status_Bool", true);
    PhotonTrackedTarget target = result.getBestTarget();

    SmartDashboard.putNumber("Vision_Target_Yaw", target.getYaw());
    SmartDashboard.putNumber("Vision_Target_Pitch", target.getPitch());
    SmartDashboard.putNumber("Vision_Target_Area", target.getArea());
    SmartDashboard.putNumber("Vision_Target_Skew", target.getSkew());
    SmartDashboard.putString("Vision_Target_Pose", target.getBestCameraToTarget().toString());
    SmartDashboard.putString("Vision_Target_Corners", target.getDetectedCorners().toString());
  }

  // public List<PhotonPipelineResult> getUnreadResults() {
  //   return camera.getAllUnreadResults();
  // }

  public PhotonPipelineResult getLatestResult() {
    // List<PhotonPipelineResult> unreadResults = getUnreadResults();
    // if (unreadResults.isEmpty()) {
    //   return null;
    // }
    // return getUnreadResults().get(unreadResults.size()-1);
    return camera.getLatestResult();
  }

  public void takeSnapshot() {
    camera.takeOutputSnapshot();
  }
}