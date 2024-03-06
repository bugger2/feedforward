package frc.robot.subsystems.armRotation;

import org.littletonrobotics.junction.Logger;

import com.teamtators.util.PIDFController;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class ArmRotationIOReal implements ArmRotationIO {
    Spark motor;
    DutyCycleEncoder encoder;
    PIDFController controller;

    double prevPosition;

    private static double ROTATIONS_TO_DEGREES = 1;
    
    public ArmRotationIOReal() {
        motor = new Spark(0);
        encoder = new DutyCycleEncoder(1);
        controller = new PIDFController(.2, 0, .06, 6.0475, .02);
        prevPosition = encoder.get();
    }

    @Override
    public void updateInputs(ArmRotationInputsAutoLogged inputs) {
        inputs.armDegreesPerSec = (encoder.get() - prevPosition)/.02/ROTATIONS_TO_DEGREES;
        inputs.armDegrees = encoder.get();
        inputs.motorPosition = encoder.get();
        inputs.motorInput = motor.get();

        double speed = controller.calculate(inputs.armDegrees);
        motor.setVoltage(speed);
        Logger.recordOutput("ArmRotation/ControllerOutput", speed);
    }

    @Override
    public void setF(double ff) {
        controller.setF(ff);
    }

    @Override
    public void setSetpoint(double setpoint) {
        controller.setSetpoint(setpoint);
    }
}
