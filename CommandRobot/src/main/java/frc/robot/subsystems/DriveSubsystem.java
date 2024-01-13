// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  
  private final Spark driveLeftMotor = new Spark(0);
  private final Spark driveRightMotor = new Spark(1);
  private final Encoder leftEncoder = new Encoder(0, 1, false, EncodingType.k4X);
  private final Encoder rightEncoder = new Encoder(2, 3, false, EncodingType.k4X);
  private final double kEncoderTicks2Meter = 1.0 / 4096 * 0.128 * Math.PI;

  public double getEncoderMeters() {
    return (((leftEncoder.get() + -rightEncoder.get()) / 2) * kEncoderTicks2Meter);
  }
  
  /** Creates a new ExampleSubsystem. */
  public DriveSubsystem() {
    driveLeftMotor.setInverted(false);
    driveRightMotor.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Drive Encoder Value", getEncoderMeters());
  }

  public void setMotors(double leftSpeed, double rightSpeed) {
    driveLeftMotor.set(leftSpeed);
    driveRightMotor.set(-rightSpeed);
  }
}
