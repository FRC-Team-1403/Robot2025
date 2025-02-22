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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import team1403.lib.util.AutoUtil;
import team1403.lib.util.CougarUtil;
import team1403.robot.commands.*;
import team1403.robot.subsystems.*;
import team1403.robot.swerve.SwerveSubsystem;
import team1403.robot.vision.AprilTagCamera;
import team1403.robot.Constants;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private final candle m_candle = new candle();

  // private SwerveSubsystem m_swerve;
  // private Elevator m_elevator;
  // private IntakeSubsystem m_intake;
  // // private WristSubsystem m_wrist;
  // private final AlgaeIntake m_AlgaeIntake;
  // private ClimberSubsystem m_climberSubsystem;
  // // private StateMachine m_stateMachine;
  // private Command m_vibrationCmd;
  

  // // Replace with CommandPS4Controller or CommandJoystick if needed
  // private final CommandXboxController m_driverController;
  // private final CommandXboxController m_operatorController;

  // private final AlgaeEstimateSubystem test = new AlgaeEstimateSubystem();

  // private final PowerDistribution m_powerDistribution;

  // private SendableChooser<Command> autoChooser;

  // private Command m_teleopCommand;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    // m_driverController = new CommandXboxController(Constants.Driver.pilotPort);
    // m_operatorController = new CommandXboxController(Constants.Operator.pilotPort);
    // Blackbox.init();
    // m_swerve = new SwerveSubsystem();
    // m_elevator = new Elevator();
    // m_intake = new IntakeSubsystem();
    // m_climberSubsystem = new ClimberSubsystem();
    // m_AlgaeIntake = new AlgaeIntake();
    // m_wrist = new WristSubsystem();
    // m_vibrationCmd = new ControllerVibrationCommand(m_driverController.getHID(), 0.28, 1);
    // m_stateMachine = new StateMachine(m_intake, m_wrist, m_elevator, m_swerve, () -> m_vibrationCmd);
    


    // Enables power distribution logging
    // m_powerDistribution = new PowerDistribution(Constants.CanBus.powerDistributionID, ModuleType.kRev);
    //m_operatorController.b().whileTrue(() -> m_intakeSubsystem.setIntakeMotorSpeed(0));
   // m_operatorController.a().whileTrue().new InstantCommand(() -> m_elevator.)
    


    // autoChooser = AutoBuilder.buildAutoChooser();
    
    //avoid cluttering up auto chooser at competitions
    // if (Constants.ENABLE_SYSID) {
    //   autoChooser.addOption("Swerve SysID QF", m_swerve.getSysIDQ(Direction.kForward));
    //   autoChooser.addOption("Swerve SysID QR", m_swerve.getSysIDQ(Direction.kReverse));
    //   autoChooser.addOption("Swerve SysID DF", m_swerve.getSysIDD(Direction.kForward));
    //   autoChooser.addOption("Swerve SysID DR", m_swerve.getSysIDD(Direction.kReverse));
    //   autoChooser.addOption("Swerve SysID Steer QF", m_swerve.getSysIDSteerQ(Direction.kForward));
    //   autoChooser.addOption("Swerve SysID Steer QR", m_swerve.getSysIDSteerQ(Direction.kReverse));
    //   autoChooser.addOption("Swerve SysID Steer DF", m_swerve.getSysIDSteerD(Direction.kForward));
    //   autoChooser.addOption("Swerve SysID Steer DR", m_swerve.getSysIDSteerD(Direction.kReverse));
    // }

    // autoChooser.addOption("Choreo Auto", AutoUtil.loadChoreoAuto("test", m_swerve));
    // autoChooser.addOption("FivePieceCenter", AutoHelper.getFivePieceAuto(m_swerve));

    // SmartDashboard.putData("Auto Chooser", autoChooser);
    // if(Constants.DEBUG_MODE) {
    //   SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
    //   SmartDashboard.putData("Swerve Drive", m_swerve);
    //   SmartDashboard.putData("Power Distribution", m_powerDistribution);
    // }

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
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    // Setting default command of swerve subPsystem
    // red
    
    // m_swerve.setDefaultCommand(new DefaultSwerveCommand(
    //     m_swerve,
    //     () -> -m_driverController.getLeftX(),
    //     () -> -m_driverController.getLeftY(),
    //     () -> -m_driverController.getRightX(),
    //     () -> m_driverController.getHID().getYButtonPressed(),
    //     () -> m_driverController.getHID().getBButtonPressed(),
    //     () -> m_driverController.getHID().getXButton(),
    //     () -> m_driverController.getRightTriggerAxis(),
    //     () -> m_driverController.getLeftTriggerAxis()));

    // Command driverVibrationCmd = new ControllerVibrationCommand(m_driverController.getHID(), 0.28, 1);

    // //m_driverController.povRight().onTrue(Blackbox.reefSelect(ReefSelect.RIGHT));
    // //m_driverController.povLeft().onTrue(Blackbox.reefSelect(ReefSelect.LEFT));

    // m_driverController.rightBumper().onTrue(Blackbox.setAligningCmd(true, ReefSelect.RIGHT));

    // m_driverController.leftBumper().whileTrue(Blackbox.setAligningCmd(true,ReefSelect.LEFT));

    // //m_driverController.a().onTrue(new ControllerVibrationCommand(m_driverController.getHID(), 0.28, 1));
    // //SmartDashboard.putNumber("vibration", 0);

    // m_driverController.b().onTrue(m_swerve.runOnce(() -> m_swerve.zeroHeading()));

    // m_climberSubsystem.setDefaultCommand(new ClimberCommand(m_climberSubsystem, 
    //   () -> m_operatorController.getHID().getAButtonPressed(), 
    //   () -> m_operatorController.getHID().getBButtonPressed(), 0.1));
    // m_wristCommand = new WristCommand(m_wrist, m_operatorController.a(),
    //   m_operatorController.b(), m_operatorController.x());

    // m_wrist.setDefaultCommand(m_wristCommand);


    m_candle.setDefaultCommand(CandleCommand(m_candle));
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
   // return autoChooser.getSelected();
   return Commands.none();
  }

  public Command getTeleopCommand() {
    // return m_teleopCommand;
    return Commands.none();
  }
}
