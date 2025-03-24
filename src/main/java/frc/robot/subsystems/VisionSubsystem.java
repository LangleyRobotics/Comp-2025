package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.LimelightHelpers.PoseEstimate;
import swervelib.SwerveDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.measure.AngularVelocity;



public class VisionSubsystem extends SubsystemBase {



  double xSpeed;
  double ySpeed;
  double tangentSpeed;
  double normalSpeed;

  boolean hasTarget = false;

  boolean isRed;
  

  LimelightHelpers limelightHelp;


  /** Creates a new VisionSubsystem. */
  public VisionSubsystem() {
    LimelightHelpers.setLEDMode_ForceOff("limelight-left");
    LimelightHelpers.setLEDMode_ForceOff("limelight-right");

    // DIO_1 = new DigitalInput(1);


    if (DriverStation.isFMSAttached()) {
      if (DriverStation.getAlliance().isPresent()) {
        isRed = DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
      }
    }


  }

  
  public boolean getTV() {
    return LimelightHelpers.getTV("Left Limelight");
  }

  public double getTX() {
    return LimelightHelpers.getTX("Left Limelight");
  }

  public void updatePoseEstimator(SwerveDrive swerve) {
    PoseEstimate  poseEst = getEstimatedGlobalPose("Left Limelight");
      if (poseEst != null) {
        swerve.addVisionMeasurement(poseEst.pose, poseEst.timestampSeconds);
      }
  }

  /*called by drive and stuff*/
  public void updatePosesEstimator(SwerveDrive swerve) {
    double maxta = 0.4;
    String camera = null;
    String[] limelights = {"Left Limelight", "Right Limelight"}; // , "limelight-rear"
    for (String limelight: limelights) {
      if (LimelightHelpers.getTV(limelight) && LimelightHelpers.getTA(limelight) > maxta) {
        maxta = LimelightHelpers.getTA(limelight);
        camera = limelight;
      }
    }
    if (camera != null) {
      PoseEstimate poseEst = getEstimatedGlobalPose(camera);
      swerve.addVisionMeasurement(poseEst.pose, poseEst.timestampSeconds);
      hasTarget = true;
    } else {
      hasTarget = false;
      
    }
    SmartDashboard.putBoolean("target??", hasTarget);
  }


  /**
   * Building this out for hopefully a quick test. to try mega tag 2
   * @param swerve
   */
  
    
  public PoseEstimate getEstimatedGlobalPose(String limelight) {
    if (LimelightHelpers.getTV(limelight)) {
      hasTarget = true;
      PoseEstimate poseEst = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelight);
     
      SmartDashboard.putBoolean("limelightTV", LimelightHelpers.getTV(limelight));
      SmartDashboard.putNumber("limelightXPosition", poseEst.pose.getX());
      SmartDashboard.putNumber("limelightYPosition", poseEst.pose.getY());
      return poseEst;
    }
    SmartDashboard.putBoolean("limelightTV", LimelightHelpers.getTV(limelight));
    SmartDashboard.putNumber("limelightX", new PoseEstimate().pose.getX());
    SmartDashboard.putNumber("limelightY", new PoseEstimate().pose.getY());
    return new PoseEstimate(); 
  }

  public PoseEstimate[] getEstimatedGlobalPose(String[] limelights) {
    PoseEstimate[] poseEsts = new PoseEstimate[limelights.length];
    int num = 0;
    for (String limelight : limelights) {
      if (LimelightHelpers.getTV(limelight)) {
        PoseEstimate poseEst = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelight);
        poseEsts[num] = poseEst;
      }
      else {
        poseEsts[num] = null;
      }
      num++;
    }
    return poseEsts;
     
  }
    
  @Override
  public void periodic() {
    // This method will be called once per scheduler run   
  }

  
  


}