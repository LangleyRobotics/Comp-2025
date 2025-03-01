// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import au.grapplerobotics.LaserCan;

import java.util.function.Supplier;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.*;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.MathMethods;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.ConfigurationFailedException;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

public class ElevatorSubsystem extends SubsystemBase{
    private LaserCan heimdal;
    private LaserCan tyr;
    private boolean allGood = false;
    private boolean moveForward = false;
    private final TalonFX elevatorMotorRight = new TalonFX(ElevatorConstants.kElevatorMotorRightPort);
    private final TalonFX elevatorMotorLeft = new TalonFX(ElevatorConstants.kElevatorMotorLeftPort);

    private final ElevatorFeedforward elevatorFeedForward = new ElevatorFeedforward(ElevatorConstants.kS_Elevator, ElevatorConstants.kG_Elevator, ElevatorConstants.kV_Elevator, ElevatorConstants.kA_Elevator);
    private final PIDController elevatorPIDController = new PIDController(ElevatorConstants.kP_Elevator, ElevatorConstants.kI_Elevator, ElevatorConstants.kD_Elevator);
    private final ProfiledPIDController profiledPIDController = new ProfiledPIDController(
        ElevatorConstants.kP_Elevator,
        ElevatorConstants.kI_Elevator,
        ElevatorConstants.kD_Elevator,
        // The motion profile constraints
        new TrapezoidProfile.Constraints(200, 100));

    double goal = 0;
    double absEncoderRaw = 0;
    double offset = 0;
    

    public ElevatorSubsystem() {
        // Configures the encoder to return a distance for every rotation
        elevatorPIDController.setTolerance(ElevatorConstants.deadbandAngle);
        elevatorMotorRight.setPosition(0);
        elevatorMotorLeft.setPosition(0);
        heimdal = new LaserCan(0);
        tyr = new LaserCan(1);
        TalonFXConfigurator elevatorConfigRight = elevatorMotorRight.getConfigurator();
        elevatorConfigRight.apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
        elevatorConfigRight.apply(
                new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(80)
                        .withStatorCurrentLimitEnable(true));

        TalonFXConfigurator elevatorConfigLeft = elevatorMotorLeft.getConfigurator();
        elevatorConfigLeft.apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
        elevatorConfigLeft.apply(
                new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(80)
                        .withStatorCurrentLimitEnable(true));
        elevatorMotorLeft.setControl(new Follower(ElevatorConstants.kElevatorMotorRightPort,true));
    }


    public void periodic() {
        SmartDashboard.putNumber("Elevator Talon Position", elevatorMotorRight.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Elevator Talon Voltage", elevatorMotorLeft.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Elevator Goal Position", goal);
        LaserCan.Measurement haveCoral = heimdal.getMeasurement();
        LaserCan.Measurement tooFarBack = tyr.getMeasurement();
        if(haveCoral!=null&&haveCoral.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT){
            allGood = haveCoral.distance_mm < 20;
            SmartDashboard.putNumber("Coral To Elevator Distance", haveCoral.distance_mm);
        }
        else{
            allGood=false;
        }

        if(tooFarBack!=null&&tooFarBack.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT){
            moveForward = tooFarBack.distance_mm < 20;
            SmartDashboard.putNumber("Coral To Outtake Distance", tooFarBack.distance_mm);

        }
        else{
            moveForward=false;
        }
        SmartDashboard.putBoolean("Coral In Elevator", moveForward);
        SmartDashboard.putBoolean("Coral In Outtake", allGood);
        
        // Use the output (and optionally the setpoint) here
        // double feedforward = 0;
        // elevatorMotorLeft.setVoltage(output + feedforward);
        // elevatorMotorRight.setVoltage(-(output + feedforward));
        
        double motorVoltage = profiledPIDController.calculate(getElevatorPosition(), goal);// + elevatorFeedForward.calculate();
        //try elevatorMotor.setPosition(double position);
        if(!moveForward){
        elevatorMotorRight.setVoltage(motorVoltage);
        }
        else{
            elevatorMotorRight.setVoltage(0);
        }
        //elevatorMotorRight.setVoltage(-motorVoltage);
    }
    public boolean getAllGood(){
        return allGood;
    }
    public boolean getMoveForward(){
        return moveForward;
   }
    public void setElevatorMotorNoBounds(double velocity) {
        //elevatorMotorRight.set(velocity);
        elevatorMotorRight.set(velocity);
    }

    //Manually controlling angle of elevator (operator controller during teleop)
    public void setElevatorMotor(double velocity){

        try {
            if (getElevatorPosition() < ElevatorConstants.kMaxElevatorPosition || 
                getElevatorPosition() > ElevatorConstants.kMinElevatorPosition) {
                    //motors are facing opposite directions
                   // elevatorMotorRight.set(velocity);
                    elevatorMotorRight.set(-velocity);
                } else {
                    //elevatorMotorRight.set(0);
                    elevatorMotorRight.set(0);
                }
        } catch(Exception e) {
            // System.out.println("Error: Elevator Motor is Set to a value out of valid range [-1.0, 1.0]");
        }
    }


    public void stopElevatorMotor(){
        //elevatorMotorRight.set(0);
        elevatorMotorRight.set(0);
    }

    public double getElevatorPosition() {
        return elevatorMotorLeft.getPosition().getValueAsDouble();
    }

    public void setElevatorPosition(double position) {
        elevatorMotorLeft.setPosition(position);
    }

    public void pidElev() {
        // Use the output (and optionally the setpoint) here
        // double feedforward = 0;
        // elevatorMotorLeft.setVoltage(output + feedforward);
        // elevatorMotorRight.setVoltage(-(output + feedforward));
        double motorVoltage = profiledPIDController.calculate(getElevatorPosition(), goal);// + elevatorFeedForward.calculate();
        //try elevatorMotor.setPosition(double position);
        elevatorMotorRight.setVoltage(motorVoltage);
        //elevatorMotorRight.setVoltage(-motorVoltage);
    }
    public void setGoal(double setpoint) {
        this.goal = setpoint;
    }
    public double getGoal() {
        return goal;
    }

    public void setPosition(double setpoint) {
        profiledPIDController.setGoal(setpoint);
    }

    public Command updatePosition(Supplier<Double> setpoint) {
            return new RunCommand(() -> setPosition(setpoint.get()));
    }
    public boolean isAtSetpoint() {
        return profiledPIDController.atGoal();
    }
}
