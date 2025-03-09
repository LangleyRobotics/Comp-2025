package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;


import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OuttakeConstants;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.MathMethods;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.ConfigurationFailedException;
public class OuttakeSubsystem extends SubsystemBase {

    private final TalonFX outtakeMotor = new TalonFX(OuttakeConstants.kOuttakeMotorPort);

    double goal = 0;
    double offset = 0;
    private LaserCan heimdal;
    private LaserCan tyr;
    private boolean allGood=false;
    private boolean moveForward=false;

    public OuttakeSubsystem() {


        // Configures the kraken like how REV Hardware Client would for Sparkmaxes

        heimdal = new LaserCan(0);
        tyr = new LaserCan(1);
        TalonFXConfigurator outtakeConfig = outtakeMotor.getConfigurator();
        outtakeConfig.apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
        outtakeConfig.apply(
                new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(80)
                        .withStatorCurrentLimitEnable(true));
    }


    @Override
    public void periodic() {
        // SmartDashboard.putNumber("Outtake Talon Position", outtakeMotor.getPosition().getValueAsDouble());
        // SmartDashboard.putNumber("Outtake Talon Voltage", outtakeMotor.getMotorVoltage().getValueAsDouble());

        LaserCan.Measurement haveCoral = heimdal.getMeasurement();
        LaserCan.Measurement tooFarBack = tyr.getMeasurement();

        if(haveCoral!=null && haveCoral.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT){
            allGood = haveCoral.distance_mm < 20;
        }
        else{
            allGood = false;
        }

        if(tooFarBack != null&&tooFarBack.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT){
            moveForward = tooFarBack.distance_mm < 20;

        }
        else{
            moveForward = false;
        }

        

    }




    public void setOuttakeMotor(double velocity){
        
         outtakeMotor.set(-velocity);
     }
    
     public void stopOuttakeMotor(){
         outtakeMotor.set(0);
     }
     public void setOuttakeIntakeMotor(double velocity){
        if(allGood && !moveForward) {
            stopOuttakeMotor();
        }
        else {
            outtakeMotor.set(-velocity);
        }
    }
     public boolean getAllGood(){
        return allGood;
    }
    public boolean getMoveForward(){
        return moveForward;
    }

}