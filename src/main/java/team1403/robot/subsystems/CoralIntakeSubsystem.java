package team1403.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.CANrange;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import team1403.robot.Constants;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralIntakeSubsystem extends SubsystemBase {
    private SparkMax m_intakeMotor;
    private CANrange m_CANRange;

    private LinearFilter m_filter = LinearFilter.singlePoleIIR(Constants.kLoopTime * 10, Constants.kLoopTime);

    public CoralIntakeSubsystem() {
        m_intakeMotor = new SparkMax(Constants.CanBus.intakeMotorID, MotorType.kBrushless);
        configMotors();
        m_CANRange = new CANrange(Constants.CanBus.CANRangeID);
    }

    private void configMotors() {
        SparkMaxConfig intakeConfig = new SparkMaxConfig();
        intakeConfig
            .idleMode(IdleMode.kBrake);
        intakeConfig.smartCurrentLimit(40);

        m_intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
   }

    public void setIntakeMotorSpeed(double speed) {
        m_intakeMotor.set(speed);
    }
    
    public double getIntakeSpeed() {
        return m_intakeMotor.get();
    }

    //TODO add in the distance threholds
    public double getDistance() {
        return m_CANRange.getDistance(true).getValue().in(Meters);
    }

    private double getFilteredCurrent() {
        return m_filter.calculate(m_intakeMotor.getOutputCurrent());
    }

    private boolean pieceIn() {
        return getDistance() < 0.37;
    }

    @Override
    public void periodic() {
        if (getIntakeSpeed() > 0.04) {
            if (getFilteredCurrent() > 40 && pieceIn()) {
                Constants.CoralIntake.hasPiece = true;
            }
        }

        Logger.recordOutput("Intake speed", getIntakeSpeed());
        Logger.recordOutput("Distance", getDistance());
        Logger.recordOutput("Has Piece", Constants.CoralIntake.hasPiece);
        Logger.recordOutput("Current current", getFilteredCurrent());
    }
}