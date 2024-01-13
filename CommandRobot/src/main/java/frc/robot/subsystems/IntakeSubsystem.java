package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    
    private final Spark intakeLeftMotor = new Spark(3);
    private final Spark intakeRightMotor = new Spark(4);
    
    public IntakeSubsystem() {}

    public void setPosition(boolean open) {
        if (open) {
            intakeLeftMotor.set(-1);
            intakeRightMotor.set(-1);
        } else {
            intakeLeftMotor.set(1);
            intakeRightMotor.set(1);
        }
    }
}
