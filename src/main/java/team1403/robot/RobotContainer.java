// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team1403.robot;

import java.util.Set;

import org.ejml.dense.row.MatrixFeatures_CDRM;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import team1403.robot.commands.*;
import team1403.robot.subsystems.*;
import team1403.lib.RepeatNTimes;
import team1403.robot.Constants;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private ElevatorSubsystem m_elevator;
  private WristSubsystem m_wrist;
  private CoralIntakeSubsystem m_coralIntake;


  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController;
  private final CommandXboxController m_operatorController;

  private final PowerDistribution m_powerDistribution;

  private SendableChooser<Command> autoChooser = new SendableChooser<>();
  private Command m_pathFinder = Commands.none();
  private Command m_teleopCommand = Commands.none();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    m_driverController = new CommandXboxController(Constants.Driver.pilotPort);
    m_operatorController = new CommandXboxController(Constants.Operator.pilotPort);

    m_elevator = new ElevatorSubsystem();
    m_wrist = new WristSubsystem();
    m_coralIntake = new CoralIntakeSubsystem();

    // Enables power distribution logging
    m_powerDistribution = new PowerDistribution(Constants.CanBus.powerDistributionID, ModuleType.kRev);
    //m_operatorController.b().whileTrue(() -> m_intakeSubsystem.setIntakeMotorSpeed(0));
   // m_operatorController.a().whileTrue().new InstantCommand(() -> m_elevator.)
    


    SmartDashboard.putData(autoChooser);

    autoChooser.addOption("Elevator SysID QF", m_elevator.getSysIDQ(Direction.kForward));
    autoChooser.addOption("Elevator SysID QR", m_elevator.getSysIDQ(Direction.kReverse));
    autoChooser.addOption("Elevator SysID DF", m_elevator.getSysIDD(Direction.kForward));
    autoChooser.addOption("Elevator SysID DR", m_elevator.getSysIDD(Direction.kReverse));

   configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    // wrist
    m_wrist.setDefaultCommand(new WristCommand(m_wrist, 
      () -> m_operatorController.getHID().getXButton(),             // level 1 angle
      () -> m_operatorController.getHID().getAButton(),             // level 2 angle
      () -> m_operatorController.getHID().getBButton(),             // level 3 angle
      () -> m_operatorController.getHID().getYButton(),             // level 4 angle
      () -> m_operatorController.getHID().getRightBumperButton())); // source angle

    // elevator
    m_elevator.setDefaultCommand(new ElevatorCommand(m_elevator, 
      () -> m_driverController.getHID().getXButton(),   // level 1 / source
      () -> m_driverController.getHID().getAButton(),   // level 2
      () -> m_driverController.getHID().getBButton(),   // level 3
      () -> m_driverController.getHID().getYButton())); // level 4

    // coral intake

    // stop intake
    m_operatorController.leftBumper().whileTrue(new CoralIntakeSpeed(m_coralIntake, 0));
    // release coral
    new Trigger(() -> m_operatorController.getRightTriggerAxis() > 0.5).onTrue(
      new CoralIntakeSpeed(m_coralIntake, -0.3).withTimeout(.2));
    // wiggle
    new Trigger(() -> m_coralIntake.hasPiece()).toggleOnTrue(
      new RepeatNTimes(Commands.sequence(
        new CoralIntakeSpeed(m_coralIntake, -Constants.CoralIntake.wiggle).withTimeout(0.4),
        new CoralIntakeSpeed(m_coralIntake, Constants.CoralIntake.wiggle).withTimeout(0.4)
      ), 4)
    );
    // start intake
    new Trigger(() -> !m_coralIntake.hasPiece()).and(() -> 
    Constants.Elevator.Setpoints.current == Constants.Elevator.Setpoints.source
    && Constants.Wrist.Setpoints.current == Constants.Wrist.Setpoints.source / 360.0)
    .whileFalse(new CoralIntakeSpeed(m_coralIntake, 0.5));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }

  public Command getTeleopCommand() {
    return m_teleopCommand;
  }
}
