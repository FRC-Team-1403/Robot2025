package team1403.robot.commands;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.Constants;
import team1403.robot.subsystems.ElevatorSubsystem;
import team1403.robot.subsystems.WristSubsystem;

public class CoralMechanism extends Command {
    
    //Variable for local copy of Wrist subsystem
    private final WristSubsystem m_wrist;
    //Variable for local copy of Elevator subsystem
    private final ElevatorSubsystem m_elevator;
    //Variable for local copy of the mechanism object
    private final LoggedMechanism2d m_mechanism;
    //Variable for local copy of the mechanism root object
    private final LoggedMechanismRoot2d m_root;
    //Variable for the mechanism ligament elevator object
    private final LoggedMechanismLigament2d m_elevatorMech;
    //Variable for the mechanism ligament wrist object
    private final LoggedMechanismLigament2d m_wristMech;

    /**
     * Shows the mechanisms on Advantage Scope
     * 
     * @param wrist Wrist subsystem
     * @param elevator Elevator subsystem
     */
    public CoralMechanism(WristSubsystem wrist, ElevatorSubsystem elevator) {
        m_wrist = wrist;
        m_elevator = elevator;

        m_mechanism = new LoggedMechanism2d(2, 2);
        m_root = m_mechanism.getRoot("Bellypan", 1, 0);
        m_elevatorMech = m_root.append(new LoggedMechanismLigament2d("elevator", 0.4, 90));
        m_wristMech = m_elevatorMech.append(new LoggedMechanismLigament2d("wrist", 0.1, 
            0, 6, new Color8Bit(Color.kGreen)));

        if (Constants.DEBUG_MODE) SmartDashboard.putData(m_mechanism);
    }

    @Override
    public void execute() {
        //Updates the elevator height in advantage scope by increasing its length
        m_elevatorMech.setLength(0.2 + m_elevator.getPosition() / 20.0);
        //Updates the wrist angle in advantage scope by rotating the mechanism
        m_wristMech.setAngle(Rotation2d.fromRotations(m_wrist.getWristAngle() + 0.25));
        //Logs the mechanism
        if (Constants.DEBUG_MODE)
            Logger.recordOutput("CoralMechanism", m_mechanism);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
