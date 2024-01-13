// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

/** An example command that uses an example subsystem. */
public class ElevatorJoystickCmd extends Command {

  private final ElevatorSubsystem elevatorSubsystem;
  private final double speed;

  public ElevatorJoystickCmd(ElevatorSubsystem elevatorSubsystem, double speed) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.speed = speed;
    addRequirements(elevatorSubsystem);  // avoid conflicts between commands
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("ElevatorJoystickCmd started!");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevatorSubsystem.setMotor(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.setMotor(0);
    System.out.println("ElevatorJoystickCmd ended!");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;  // Command never ends unless interrupted
  }
}
