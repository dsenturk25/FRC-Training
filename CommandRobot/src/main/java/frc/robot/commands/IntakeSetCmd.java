package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeSetCmd extends Command {
    
    private final IntakeSubsystem intakeSubsystem;
    private boolean open;

    public IntakeSetCmd(IntakeSubsystem intakeSubsystem, boolean open) {
        this.intakeSubsystem = intakeSubsystem;
        this.open = open;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("IntakeSetCmd started!");
    }

    @Override
    public void execute() {
        intakeSubsystem.setPosition(open);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("IntakeSetCmd ended!");
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
