// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.OuttakeSubsystem;

/** Add your docs here. */
public class OuttakeAutoCmd extends Command{
    
    private final OuttakeSubsystem outtakeSubsystem;
    private final Supplier<Double> outtakePositiveDirFunction;
    private final Supplier<Double> outtakeNegativeDirFunction;
  
  
    public OuttakeAutoCmd(OuttakeSubsystem outtakeSubsystem,
    Supplier<Double> outtakePositiveDirFunction, Supplier<Double> outtakeNegativeDirFunction) {
      this.outtakeSubsystem = outtakeSubsystem;
      this.outtakePositiveDirFunction = outtakePositiveDirFunction;
      this.outtakeNegativeDirFunction = outtakeNegativeDirFunction;
  
      addRequirements(outtakeSubsystem);
    }
  
  
    @Override
    public void initialize() {
  
    }
  
  
    @Override
    public void execute() {
        if(outtakeNegativeDirFunction.get() > 0){
            outtakeSubsystem.setOuttakeMotor(-outtakeNegativeDirFunction.get());
        } else if(outtakePositiveDirFunction.get() > 0){
            outtakeSubsystem.setOuttakeMotor(outtakePositiveDirFunction.get());
        }
    }
      
    
  
  
    @Override
    public void end(boolean interrupted) {
      outtakeSubsystem.stopOuttakeMotor();
    }
  
  
    @Override
    public boolean isFinished() {
      return false;
    }
  }
  
