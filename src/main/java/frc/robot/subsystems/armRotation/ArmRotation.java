package frc.robot.subsystems.armRotation;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmRotation extends SubsystemBase {
    ArmRotationIO io;
    ArmRotationInputsAutoLogged inputs = new ArmRotationInputsAutoLogged();
    
    private Mechanism2d canvas;
    private MechanismLigament2d ligament;

    private double lengthMeters = 1;
    private double massKg = 6;

    private NetworkTable table = NetworkTableInstance.getDefault().getTable("sim");

    public ArmRotation() {
        switch(Constants.ROBOT_MODE) {
            case REAL:
                io = new ArmRotationIOReal();
                break;
            case SIM:
                io = new ArmRotationIOSim();
                break;
            case REPLAY:
                io = new ArmRotationIO() {};
                break;
        }
        
        canvas = new Mechanism2d(2, 2);
        ligament = canvas.getRoot("pivot", 0, 1)
            .append(new MechanismLigament2d("arm", 1, 0));
        SmartDashboard.putData("arm", canvas);

        table.getEntry("arm theta degrees").setDouble(0);
    }

    @Override
    public void periodic() {
        io.setF(6.0475 * Math.cos(Math.toRadians(inputs.armDegrees)));
        // io.setSetpoint(table.getEntry("arm theta degrees").getDouble(0));

        // set motor speeds and update the inputs
        io.updateInputs(inputs);

        ligament.setAngle(inputs.armDegrees);

        Logger.recordOutput("ArmRotation/Mechanism", canvas);
    }

    public Command goTo(double setpoint) {
        return this.runOnce(() -> io.setSetpoint(setpoint));
    }
}
