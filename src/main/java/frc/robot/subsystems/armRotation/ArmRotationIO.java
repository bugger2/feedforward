package frc.robot.subsystems.armRotation;

import org.littletonrobotics.junction.AutoLog;

public interface ArmRotationIO {
    @AutoLog
    public class ArmRotationInputs {
        double motorPosition;
        double motorInput;
        double armDegrees;
        double armDegreesPerSec;
    }

    public default void updateInputs(ArmRotationInputsAutoLogged inputs) {};
    public default void setF(double feedforward) {};
    public default void setSetpoint(double setpoint) {};
}
