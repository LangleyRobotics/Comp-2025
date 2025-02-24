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

public class OuttakeSubsystem extends SubsystemBase {

    private final TalonFX outtakeMotor = new TalonFX(OuttakeConstants.kOuttakeMotorPort);

    double goal = 0;
    double offset = 0;


    public OuttakeSubsystem() {


        // Configures the kraken like how REV Hardware Client would for Sparkmaxes


        TalonFXConfigurator outtakeConfig = outtakeMotor.getConfigurator();
        outtakeConfig.apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
        outtakeConfig.apply(
                new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(80)
                        .withStatorCurrentLimitEnable(true));
    }


    @Override
    public void periodic() {
        SmartDashboard.putNumber("Outtake Talon Position", outtakeMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Outtake Talon Voltage", outtakeMotor.getMotorVoltage().getValueAsDouble());
    }




    public void setOuttakeMotor(double velocity){
         outtakeMotor.set(velocity);
     }
    
     public void stopOuttakeMotor(){
         outtakeMotor.set(0);
     }

}