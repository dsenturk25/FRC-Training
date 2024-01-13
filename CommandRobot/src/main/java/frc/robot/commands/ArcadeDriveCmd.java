package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class ArcadeDriveCmd extends Command {

    private final DriveSubsystem driveSubsystem;
    private final Supplier<Double> speedFunction, turnFunction;

    public ArcadeDriveCmd(DriveSubsystem driveSubsystem, Supplier<Double> speedFunction, Supplier<Double> turnFunction) {
        this.driveSubsystem = driveSubsystem;
        this.speedFunction = speedFunction;
        this.turnFunction = turnFunction;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("ArcadeDriveCmd started!");
    }
    @Override
    public void execute() {
        double realTimeSpeed = speedFunction.get();
        double realTimeTurn = turnFunction.get();

        double left = realTimeSpeed + realTimeTurn;
        double right = realTimeSpeed - realTimeTurn;
        driveSubsystem.setMotors(left, right);
    }
    @Override
    public void end(boolean interrupted) {
        System.out.println("ArcadeDriveCmd ended!");
    }
    @Override
    public boolean isFinished() {
        return false;
    }

}
