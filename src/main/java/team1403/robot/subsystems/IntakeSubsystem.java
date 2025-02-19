package team1403.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.hardware.CANrange;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import team1403.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private SparkMax m_intakeMotor;
    private CANrange m_CANRange;

    public IntakeSubsystem(){
        m_intakeMotor = new SparkMax(Constants.CanBus.intakeMotorID, MotorType.kBrushless);
        m_CANRange = new CANrange(Constants.CanBus.CANRangeID);
    }

    public void setIntakeMotorSpeed(double speed) {
        m_intakeMotor.set(speed);
    }

    //TODO add in the distance threholds
    public double getDistance() {
        return m_CANRange.getDistance(true).getValue().in(Meters);
    }


    
}
