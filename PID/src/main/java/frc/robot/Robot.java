// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */


  private Spark leftMotor1 = new Spark(0);
  private Spark leftMotor2 = new Spark(0);
  private Spark rightMotor1 = new Spark(0);
  private Spark rightMotor2 = new Spark(0);

  private Joystick joy1 = new Joystick(0);

  private Encoder encoder = new Encoder(0, 1, true, EncodingType.k4X);  // manipulate reverseDirection in case there is anomaly in acceleration
  private final double kDriveTick2Meter = (1.0 / 128 * 6 * Math.PI / 12) / 0.3048;

  @Override
  public void robotInit() {}

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("encoder value", encoder.get() * kDriveTick2Meter);
  }

  @Override
  public void autonomousInit() {
    encoder.reset();
    errorSum = 0;
    lastTimeStamp = Timer.getFPGATimestamp();
    lastError = 0;
  }

  final double kP = 0.5;
  final double kI = 5;
  final double kD = 0.5;
  final double iLimit = 0.1;

  double setpoint = 0;
  double errorSum = 0;
  double lastTimeStamp = 0;  // timestamp of the latest calculation
  double lastError = 0;

  @Override
  public void autonomousPeriodic() {
    if (joy1.getRawButton(1)) {
      setpoint = 10;
    } else if (joy1.getRawButton(2)) {
      setpoint = 0;
    }

    // get sensor position
    double sensorPosition = encoder.get() * kDriveTick2Meter;

    // calculations 
    double error = setpoint - sensorPosition;
    double dt = Timer.getFPGATimestamp() - lastTimeStamp;

    if (Math.abs(error) < iLimit) {
      errorSum = error * dt;
    }

    double dE = error - lastError;

    double errorRate = dE / dt;

    double outputSpeed = kP * error + kI * errorSum + kD * errorRate;

    // output to motors
    leftMotor1.set(outputSpeed);
    leftMotor2.set(outputSpeed);
    rightMotor1.set(-outputSpeed);
    rightMotor2.set(-outputSpeed);

    // Update variables
    lastTimeStamp = Timer.getFPGATimestamp();
    lastError = error;
  }

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {}

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
