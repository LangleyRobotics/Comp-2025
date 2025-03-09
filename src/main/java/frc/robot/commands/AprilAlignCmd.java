package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.commands.SwerveControllerCmd;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Command that makes the robot go to a tag idiot;
 */
public class AprilAlignCmd extends Command {
    private VisionSubsystem m_vision_subsystem;
    private DriveSubsystem m_swerve_subsystem;

    private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(
            VisionConstants.kMaxSpeed, VisionConstants.kMaxAcceleration);
    private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(
            VisionConstants.kMaxSpeed, VisionConstants.kMaxAcceleration);
    private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(
            VisionConstants.kMaxAngularSpeed, VisionConstants.kMaxAngularAcceleration);

    private final ProfiledPIDController xController = new ProfiledPIDController(VisionConstants.kXYP,
            VisionConstants.kXYI, VisionConstants.kXYI, X_CONSTRAINTS);
    private final ProfiledPIDController yController = new ProfiledPIDController(VisionConstants.kXYP,
            VisionConstants.kXYI, VisionConstants.kXYI, Y_CONSTRAINTS);
    private final ProfiledPIDController omegaController = new ProfiledPIDController(VisionConstants.kRP,
            VisionConstants.kRI, VisionConstants.kRI, OMEGA_CONSTRAINTS);

    private double xSpeed;
    private double ySpeed;
    private double turningSpeed;

    private int targetTag;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    private boolean isFinished;

    // private static final int TAG_TO_CHASE = 5;
    // private static final Transform3d TAG_TO_GOAL = new Transform3d(
    // new Translation3d(1.0, 0.0, 0.0),
    // new Rotation3d(0.0, 0.0, Math.PI));

    /**
     * Creates a new GoToTag.
     *
     * @param vision_subsystem Vision subsystem
     * @param swerve_subsystem Swerve subsystem
     * @param target_tag       The tag id to drive to
     */
    public AprilAlignCmd(VisionSubsystem vision_subsystem, DriveSubsystem swerve_subsystem, int target_tag) {
        m_vision_subsystem = vision_subsystem;
        m_swerve_subsystem = swerve_subsystem;

        xController.setTolerance(VisionConstants.kPositionTolerance);
        yController.setTolerance(VisionConstants.kPositionTolerance);
        omegaController.setTolerance(Units.degreesToRadians(VisionConstants.kAngleTolerance));
        omegaController.enableContinuousInput(-Math.PI, Math.PI);

        targetTag = target_tag;

        if (targetTag == -1) {
            isFinished = true;
        }

        // Use addRequirements() here to declare subsystem dependencies.
        this.xLimiter = new SlewRateLimiter(Constants.kMaxAccelerationMetersPerSecondSquared);
        this.yLimiter = new SlewRateLimiter(Constants.kMaxAccelerationMetersPerSecondSquared);
        this.turningLimiter = new SlewRateLimiter(Constants.kMaxAngularAccelerationRadiansPerSecondSquared);
        addRequirements(m_vision_subsystem);
        addRequirements(swerve_subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // lastTarget(or whatever we call it)=null

    }

    /*
     * The fact that this is the execute code seems a little overkill
     * Seems like there could be a better way to do this.
     */
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // 1. get current pos
        Pose2d currentPose2d = m_swerve_subsystem.getPose();
        // 2. get target pos off of current targeted apriltag
        Pose2d targetPose2d = new Pose2d(); // just in case of errors

        targetPose2d = getTargetPose(VisionConstants.kAprilTags[targetTag - 1]);

        TrapezoidProfile.State targetxState = new TrapezoidProfile.State(targetPose2d.getX(), 0);
        TrapezoidProfile.State targetyState = new TrapezoidProfile.State(targetPose2d.getY(), 0);
        TrapezoidProfile.State targetomegaState = new TrapezoidProfile.State(targetPose2d.getRotation().getRadians(),
                0);
        // 3. set speeds using pid
        xSpeed = xController.calculate(currentPose2d.getX(), targetxState, X_CONSTRAINTS);
        ySpeed = yController.calculate(currentPose2d.getY(), targetyState, Y_CONSTRAINTS);
        turningSpeed = omegaController.calculate(currentPose2d.getRotation().getRadians(), targetomegaState,
                OMEGA_CONSTRAINTS);
        // 4. Output speeds to wheels
       

        // 2. Apply deadband
        xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;

        // 3. Make the driving smoother
        xSpeed = xLimiter.calculate(xSpeed) * Constants.kMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * Constants.kMaxSpeedMetersPerSecond;
        turningSpeed = turningLimiter.calculate(turningSpeed)
                * Constants.kMaxAngularSpeedRadiansPerSecond;

        // 4. Construct desired chassis speeds
        ChassisSpeeds chassisSpeeds;
        
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        

        // 5. Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        // 6. Output each module states to wheels
        m_swerve_subsystem.setModuleStates(moduleStates);
 

        // Smart dash varibs. The string ones proably work
        // SmartDashboard.putNumber("Automove/X-Speed", xSpeed);
        // SmartDashboard.putNumber("Automove/Y-Speed", ySpeed);
        // SmartDashboard.putNumber("Automove/Turn-Speed", turningSpeed);
        // SmartDashboard.putString("Automove/Target-Pose", targetPose2d.toString());
        // SmartDashboard.putString("Automove/Current-Pose", currentPose2d.toString());
    }

    /**
     * @param interrupted
     */
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return isFinished;
    }

    private Pose2d getTargetPose(Pose2d tag) {
        Pose2d targetPose = new Pose2d(
                tag.getX(),
                tag.getY(),
                tag.getRotation().rotateBy(new Rotation2d(Math.PI)));

        return targetPose;
    }
}