package team1403.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import org.littletonrobotics.junction.AutoLogOutput;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team1403.robot.Constants;

public class AlgaeWristSubsystem extends SubsystemBase {
    private final SparkMax m_algaeWristMotor;
    private final ProfiledPIDController m_pid;

    public AlgaeWristSubsystem() {
        m_algaeWristMotor = new SparkMax(Constants.CanBus.algaeWristMotorID, MotorType.kBrushless);
        m_pid = new ProfiledPIDController(Constants.AlgaeWrist.Kp, Constants.AlgaeWrist.Ki, Constants.AlgaeWrist.Kd, new TrapezoidProfile.Constraints(Constants.AlgaeIntake.maxVelo, Constants.AlgaeIntake.maxAccel));

    }
    
    public void setWristAngle(double targetAngle) {
        m_pid.setGoal(targetAngle);
    }

    @AutoLogOutput(key = "AlgaeIntake/Wrist Setpoint")
    public double getWristSetpoint() {
        return m_pid.getGoal().position;
    }

    @AutoLogOutput(key = "AlgaeIntake/Wrist Angle")
    public double getWristAngle() {
        return m_algaeWristMotor.getEncoder().getPosition() * 360.0;
    }

    @AutoLogOutput(key = "AlgaeIntake/Wrist is at Setpoint")
    public boolean isAtSetpoint() {
      return Math.abs(getWristAngle() - m_pid.getGoal().position) 
          < Units.degreesToRotations(5);
    }

    @Override
    public void periodic() {
        m_algaeWristMotor.set(m_pid.calculate(getWristAngle()));
    }

}
