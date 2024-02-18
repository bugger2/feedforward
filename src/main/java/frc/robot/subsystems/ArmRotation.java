package frc.robot.subsystems;

import com.teamtators.sim.VariableLengthArmSim;
import com.teamtators.util.PIDFController;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmRotation extends SubsystemBase {
    private VariableLengthArmSim sim;
    private PIDFController controller;

    private Mechanism2d canvas;
    private MechanismLigament2d ligament;

    private double lengthMeters = 1;
    private double massKg = 6;

    private NetworkTable table = NetworkTableInstance.getDefault().getTable("sim");

    public ArmRotation() {
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

        canvas = new Mechanism2d(2, 2);
        ligament = canvas.getRoot("pivot", 0, 1)
            .append(new MechanismLigament2d("arm", 1, 0));
        SmartDashboard.putData("arm", canvas);

        controller = new PIDFController(.2, 0, .06, 6.0475, .02);

        table.getEntry("arm theta degrees").setDouble(0);
    }

    @Override
    public void periodic() {
        controller.setF(6.0475 * Math.cos(sim.getAngleRads()));
        controller.setSetpoint(table.getEntry("arm theta degrees").getDouble(0));

        sim.setInput(controller.calculate(sim.getAngleDegrees()));
        sim.update(.02);

        ligament.setAngle(sim.getAngleDegrees());
    }
}