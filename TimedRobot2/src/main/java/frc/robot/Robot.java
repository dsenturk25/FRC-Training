// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kForward;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kOff;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kReverse;


import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
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

  private Talon leftMaster = new Talon(1);
  private Talon rightMaster = new Talon(2);

  private VictorSP leftSlave = new VictorSP(1);
  private VictorSP rightSlave = new VictorSP(2);

  private Talon armMaster = new Talon(3);
  private VictorSP armSlave = new VictorSP(5);

  private Talon rollerMotor = new Talon(0);

  private Compressor compressor = new Compressor(null);
  private DoubleSolenoid hatchIntake = new DoubleSolenoid(null, 0, 1);

  private Joystick driverJoystick = new Joystick(0);
  private Joystick operatorJoystick = new Joystick(1);

  private Encoder encoderLeft = new Encoder(0, 1);
  private Encoder encoderRight = new Encoder(0, 1);
  private Encoder encoderArm = new Encoder(0, 1);

  // Unit Conversion
  private final double kArmTick2Deg = 360.0 / 512 * 26 / 42 * 18 / 60 * 18 / 84;
  private final double kMotorTick2Meter = (1.0 / 4096 * 6 * Math.PI / 12) / 3.28084;

  private DifferentialDrive drive = new DifferentialDrive(leftMaster, rightMaster);

  @Override
  public void robotInit() {
    // inverted settings
    leftMaster.setInverted(true);
    rightMaster.setInverted(true);
    armMaster.setInverted(true);
    rollerMotor.setInverted(true);

    // Slave motors
    leftMaster.addFollower(leftSlave);
    rightMaster.addFollower(rightSlave);
    armMaster.addFollower(armSlave);

    leftSlave.setInverted(leftMaster.getInverted());
    rightSlave.setInverted(rightMaster.getInverted());
    armSlave.setInverted(armMaster.getInverted());

    // Encoder init
    encoderLeft.reset();
    encoderRight.reset();
    encoderArm.reset();

    // Pneumatics
    compressor.enableDigital();
    hatchIntake.set(kOff);

    drive.setDeadband(0.05);
  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Arm Encoder Deg", encoderArm.getRaw() / kArmTick2Deg);
    SmartDashboard.putNumber("Left Master Encoder Distance", encoderLeft.getRaw() / kMotorTick2Meter);
    SmartDashboard.putNumber("Right Master Encoder Distance", encoderRight.getRaw() / kMotorTick2Meter);
  }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {
    double leftPosition = encoderLeft.getRaw() / kMotorTick2Meter;
    double rightPosition = encoderRight.getRaw() / kMotorTick2Meter;
    double distance = (leftPosition + rightPosition) / 2;

    if (distance < 10) {
      drive.tankDrive(0.6, 0.6);
    } else {
      drive.tankDrive(0, 0);
    }
  }

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    double power = - driverJoystick.getRawAxis(1);
    double turn = driverJoystick.getRawAxis(4);

    drive.arcadeDrive(power, turn);

    double armPower = -operatorJoystick.getRawAxis(1);

    double armDeg = encoderArm.getRaw() / kArmTick2Deg;
    if (Math.abs(armPower) <= 0.05) {
      armPower = 0;
    }
    if (0 < armDeg && armDeg < (Math.PI / 4)) {
      armPower *= 0.5;
    } else {
      armPower = 0;
    }

    armMaster.set(armPower);

    double rollerPower = 0;
    if (operatorJoystick.getRawButton(1)) {
      rollerPower = 1;
    } else if (operatorJoystick.getRawButton(2)) {
      rollerPower = -1;
    }
    rollerMotor.set(rollerPower);


    if (operatorJoystick.getRawButton(3)) {
      hatchIntake.set(kReverse);
    } else {
      hatchIntake.set(kForward);
    }
  }

  @Override
  public void disabledInit() {
    leftMaster.stopMotor();
    rightMaster.stopMotor();
    leftSlave.stopMotor();
    rightSlave.stopMotor();
    armMaster.stopMotor();
    armSlave.stopMotor();
    rollerMotor.stopMotor();
  }

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
