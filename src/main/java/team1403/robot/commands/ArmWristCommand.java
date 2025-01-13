package team1403.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import team1403.robot.subsystems.Arm;
import team1403.robot.subsystems.Wrist;
import team1403.robot.Constants;

public class ArmWristCommand extends Command {
    private Arm m_arm;
    private Wrist m_wrist;
    private boolean m_up;
    private boolean m_down; 
    private CommandXboxController m_ops;

    public ArmWristCommand(Arm arm, Wrist wrist, CommandXboxController ops, boolean up, boolean down) {
        m_arm = arm;
        m_wrist = wrist;
        m_up = up;
        m_down = down;
        m_ops = ops;

        addRequirements(m_arm, m_wrist);
    }

    @Override
    public void execute() {
        if (m_up) {
            m_arm.setArmSetpoint(Constants.Arm.kDriveSetpoint);
            m_wrist.setWristSetpoint(Constants.Wrist.kDriveSetpoint);
        }
        if (m_down) {
            m_arm.setArmSetpoint(Constants.Arm.kIntakeSetpoint);
            m_wrist.setWristSetpoint(0);
        }
    }
}
