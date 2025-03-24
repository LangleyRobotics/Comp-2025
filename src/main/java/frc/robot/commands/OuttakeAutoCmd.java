// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OuttakeConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.OuttakeSubsystem;

/** Add your docs here. */
public class OuttakeAutoCmd extends Command{
    
    private final OuttakeSubsystem outtakeSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final Supplier<Boolean> intaking;
  
  
    public OuttakeAutoCmd(OuttakeSubsystem outtakeSubsystem, IntakeSubsystem intakeSubsystem,
    Supplier<Boolean> intaking) {
      this.outtakeSubsystem = outtakeSubsystem;
      this.intakeSubsystem = intakeSubsystem;
      this.intaking = intaking;
  
      addRequirements(outtakeSubsystem, intakeSubsystem);
    }
  
  
    @Override
    public void initialize() {
  
    }
  
  
    @Override
    public void execute() {
      if(intaking.get()) {
        intakeSubsystem.setIntakeMotor(IntakeConstants.kIntakeMotorSpeed);
        outtakeSubsystem.setOuttakeMotor(OuttakeConstants.kOuttakeMotorSpeedSlow);
      } else {
        outtakeSubsystem.setOuttakeMotor(OuttakeConstants.kOuttakeMotorSpeedFast);
      }

    }
      
    
  
  
    @Override
    public void end(boolean interrupted) {
      outtakeSubsystem.stopOuttakeMotor();
      intakeSubsystem.stopIntakeMotor();
    }
  
  
    @Override
    public boolean isFinished() {
      return (intaking.get() && outtakeSubsystem.getAllGood()) || 
             (!intaking.get() && !outtakeSubsystem.getAllGood());
    }
  }
  
