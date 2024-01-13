package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    
    private final Spark elevatorMotor = new Spark(0);
    private final Encoder encoder = new Encoder(4, 5, false, EncodingType.k4X);

    private final double kEncoderTick2Meter = 1.0 / 4096.0 * 0.1 * Math.PI;

    public double getEncoderMeters() {
        return encoder.get() * kEncoderTick2Meter;
    }
    
    public ElevatorSubsystem() {}

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator Encoder Value", getEncoderMeters());
    }

    public void setMotor(double speed) {
        elevatorMotor.set(speed);
    }
}
