package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeAutoCmd extends Command{
    private final IntakeSubsystem intakeSubsystem;
    //direction int is negative 1 or 1.
    private final int direction;

    public IntakeAutoCmd(IntakeSubsystem intakeSubsystem, int direction) {
        this.intakeSubsystem = intakeSubsystem;
        this.direction = direction;

        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        int dir = direction;
        double velocity = IntakeConstants.kIntakeMotorSpeed * dir;

          intakeSubsystem.setIntakeMotor(velocity);
        

    }

    @Override
    public void end(boolean interrupted) {
      intakeSubsystem.stopIntakeMotor();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
