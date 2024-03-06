package frc.robot.subsystems.armRotation;

import org.littletonrobotics.junction.Logger;

import com.teamtators.sim.VariableLengthArmSim;
import com.teamtators.util.PIDFController;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ArmRotationIOSim implements ArmRotationIO {
    VariableLengthArmSim sim;
    double lengthMeters = 1;
    double massKg = 6;

    PIDFController controller;

    public ArmRotationIOSim() {
        sim = new VariableLengthArmSim(
            DCMotor.getFalcon500Foc(1),
            10,
            SingleJointedArmSim.estimateMOI(lengthMeters, massKg),
            lengthMeters,
            -Math.PI/4,
            Math.PI/4,
            massKg,
            true
        );
        controller = new PIDFController(.2, 0, .06, 6.0475, .02);
    }

    @Override
    public void updateInputs(ArmRotationInputsAutoLogged inputs) {
        sim.update(.02);

        inputs.armDegreesPerSec = sim.getVelocityDegreesPerSec();
        inputs.armDegrees = sim.getAngleDegrees();
        inputs.motorPosition = sim.getAngleDegrees();

        double speed = controller.calculate(inputs.armDegrees);
        
        inputs.motorInput = speed;
        sim.setInput(speed);

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
