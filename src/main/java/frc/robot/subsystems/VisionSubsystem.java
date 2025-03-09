// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;

// import com.revrobotics.*;
// import com.revrobotics.;

public class VisionSubsystem extends SubsystemBase{

    private final PhotonCamera leftLime = new PhotonCamera("Left Limelight");
    
    private Transform3d BOTTOM_CAMERA_TO_CENTER = new Transform3d(
        new Translation3d(VisionConstants.camXBottom, VisionConstants.camYBottom, VisionConstants.camZBottom), 
        new Rotation3d(0,0,0));
    private Transform3d TOP_CAMERA_TO_CENTER = new Transform3d(
        new Translation3d(VisionConstants.camXTop, VisionConstants.camYTop, VisionConstants.camZTop), 
        new Rotation3d(0,0,0));
    
    // private final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    // Construct PhotonPoseEstimator
    // private PhotonPoseEstimator bottomPhotonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, BOTTOM_CAMERA_TO_CENTER);
    // private PhotonPoseEstimator topPhotonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, TOP_CAMERA_TO_CENTER);

    private SwerveDrivePoseEstimator botPoseEstimator;
    private final Supplier<Rotation2d> botRotation2D;
    private final Supplier<SwerveModulePosition[]> botModulePositions;

    private double yaw;
    private double pitch;
    private double targetRange;


    public VisionSubsystem(Supplier<Rotation2d> rotationSupplier,
        Supplier<SwerveModulePosition[]> modulePositionSupplier) {
    
        this.botRotation2D = rotationSupplier;
        this.botModulePositions = modulePositionSupplier;

        botPoseEstimator = new SwerveDrivePoseEstimator(
            DriveConstants.kDriveKinematics,
            rotationSupplier.get(),
            modulePositionSupplier.get(),
            new Pose2d());

        leftLime.setLED(VisionLEDMode.kDefault);
    }

    public void periodic() {
        // SmartDashboard.putNumber("Left Camera Yaw Angle", getLeftAngles()[0]);
        // SmartDashboard.putNumber("Left Camera Pitch Angle", getLeftAngles()[1]);
        // SmartDashboard.putNumber("Left Camera To AprilTag Distance", getLeftAngles()[2]);

        botPoseEstimator.update(botRotation2D.get(), botModulePositions.get());
        
    }

    public PhotonTrackedTarget getBestTarget() {
        var result = leftLime.getLatestResult();
        if(result.hasTargets()) {
            // Get a list of currently tracked targets.
            return result.getBestTarget();
        }
        return null;
    }

    //Localization
    // public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    //     photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    //     return photonPoseEstimator.update(bottomLime.getLatestResult());
    // }

    // public void update() {
    //     final Optional<EstimatedRobotPose> bottomOptionalEstimatedPose = bottomPhotonPoseEstimator.update(leftLime.getLatestResult());
    //     if (bottomOptionalEstimatedPose.isPresent()) {
    //         final EstimatedRobotPose estimatedPose = bottomOptionalEstimatedPose.get();          
    //         botPoseEstimator.addVisionMeasurement(estimatedPose.estimatedPose.toPose2d(), estimatedPose.timestampSeconds);
    //     }

    //     final Optional<EstimatedRobotPose> topOptionalEstimatedPose = topPhotonPoseEstimator.update(rightLime.getLatestResult());
    //     if (topOptionalEstimatedPose.isPresent()) {
    //         final EstimatedRobotPose estimatedPose = topOptionalEstimatedPose.get();          
    //         botPoseEstimator.addVisionMeasurement(estimatedPose.estimatedPose.toPose2d(), estimatedPose.timestampSeconds);
    //     }

    //     botPoseEstimator.update(botRotation2D.get(), botModulePositions.get());
    // }

    // public Pose2d getCurrentPose() {
    //     update();
    //     return botPoseEstimator.getEstimatedPosition();
    // }

    // [0] = yaw, [1] = pitch
    public double[] getLeftAngles() {
        var results = leftLime.getAllUnreadResults();
        if(!results.isEmpty()) {
            var result = results.get(results.size() - 1);
            if(result.hasTargets() && result.getBestTarget().pitch != 0) {
                PhotonTrackedTarget bestTarget = result.getBestTarget();
                // System.out.println(bestTarget.toString());
                yaw = bestTarget.yaw;
                pitch = bestTarget.pitch;
                targetRange = PhotonUtils.calculateDistanceToTargetMeters(
                                        0.333, // Measured with a tape measure, or in CAD.
                                        0.174625, 
                                        Units.degreesToRadians(Math.PI / 2), // Measured with a protractor, or in CAD.
                                        Units.degreesToRadians(bestTarget.pitch));
                // return new double[] {yaw, pitch};
            }
        } return new double[] {yaw, pitch, targetRange};
    }

} 