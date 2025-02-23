// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team1403.robot;

import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
import team1403.lib.util.AutoUtil;
import team1403.lib.util.CougarUtil;
import team1403.robot.commands.AlignCommand;
import team1403.robot.commands.ControllerVibrationCommand;
import team1403.robot.commands.DefaultSwerveCommand;
import team1403.robot.subsystems.Blackbox;
import team1403.robot.subsystems.Blackbox.ReefSelect;
import team1403.robot.swerve.SwerveSubsystem;
import team1403.robot.swerve.TunerConstants;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final SwerveSubsystem m_swerve;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController;
  private final CommandXboxController m_operatorController;

  private final PowerDistribution m_powerDistribution;

  private SendableChooser<Command> m_autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_driverController = new CommandXboxController(Constants.Driver.pilotPort);
    m_operatorController = new CommandXboxController(Constants.Operator.pilotPort);
    // Configure the trigger bindings
    Blackbox.init();
    m_swerve = TunerConstants.createDrivetrain();
    // Enables power distribution logging
    m_powerDistribution = new PowerDistribution(Constants.CanBus.powerDistributionID, ModuleType.kRev);

    if (AutoBuilder.isConfigured()) m_autoChooser = AutoBuilder.buildAutoChooser();
    else
    {
      m_autoChooser = new SendableChooser<>();
      System.err.println("Auto builder wasn't configured!");
    }
    
    //avoid cluttering up auto chooser at competitions
    if (Constants.ENABLE_SYSID) {
      m_autoChooser.addOption("Swerve SysID QF", m_swerve.sysIdQuasistatic(Direction.kForward));
      m_autoChooser.addOption("Swerve SysID QR", m_swerve.sysIdQuasistatic(Direction.kReverse));
      m_autoChooser.addOption("Swerve SysID DF", m_swerve.sysIdDynamic(Direction.kForward));
      m_autoChooser.addOption("Swerve SysID DR", m_swerve.sysIdDynamic(Direction.kReverse));
    }

    // autoChooser.addOption("Choreo Auto", AutoUtil.loadChoreoAuto("test", m_swerve));
    // autoChooser.addOption("FivePieceCenter", AutoHelper.getFivePieceAuto(m_swerve));

    SmartDashboard.putData("Auto Chooser", m_autoChooser);
    if(Constants.DEBUG_MODE) {
      SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
      SmartDashboard.putData("Power Distribution", m_powerDistribution);
    }

    if(TunerConstants.SWERVE_DEBUG_MODE) {
      SmartDashboard.putData("Swerve Drive", m_swerve);
    }

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
    
    m_swerve.setDefaultCommand(new DefaultSwerveCommand(
        m_swerve,
        () -> -m_driverController.getLeftX(),
        () -> -m_driverController.getLeftY(),
        () -> -m_driverController.getRightX(),
        () -> m_driverController.getHID().getYButtonPressed(),
        () -> m_driverController.getHID().getXButton(),
        () -> m_driverController.getRightTriggerAxis(),
        () -> m_driverController.getLeftTriggerAxis()));

    Command vibrationCmd = new ControllerVibrationCommand(m_driverController.getHID(), 0.28, 1);

    //m_driverController.povRight().onTrue(Blackbox.reefSelect(ReefSelect.RIGHT));
    //m_driverController.povLeft().onTrue(Blackbox.reefSelect(ReefSelect.LEFT));

    m_driverController.rightBumper().whileTrue(new DeferredCommand(() -> {
      Blackbox.reefSelect(ReefSelect.RIGHT);
      Pose2d currentPose = m_swerve.getPose();
      Pose2d target = Blackbox.getNearestAlignPositionReef(currentPose);
      if (target == null) return Commands.none();  
      return Commands.sequence(
        AutoUtil.pathFindToPose(target),
        new AlignCommand(m_swerve, target).finallyDo((interrupted) -> {
          if(!interrupted) vibrationCmd.schedule();
        })
      );
     }, Set.of(m_swerve)));

     m_driverController.leftBumper().whileTrue(new DeferredCommand(() -> {
      Blackbox.reefSelect(ReefSelect.LEFT);
      Pose2d currentPose = m_swerve.getPose();
      Pose2d target = Blackbox.getNearestAlignPositionReef(currentPose);
      if (target == null) return Commands.none();
      return Commands.sequence(
        AutoUtil.pathFindToPose(target),
        new AlignCommand(m_swerve, target).finallyDo((interrupted) -> {
          if(!interrupted) vibrationCmd.schedule();
        })
      );
     }, Set.of(m_swerve)));

    //m_driverController.a().onTrue(new ControllerVibrationCommand(m_driverController.getHID(), 0.28, 1));
    //SmartDashboard.putNumber("vibration", 0);
    m_driverController.b().onTrue(m_swerve.runOnce(() -> m_swerve.resetShallowHeading()));

    m_driverController.b().debounce(2.0).onTrue(Commands.sequence(
      m_swerve.runOnce(() -> m_swerve.resetShallowHeading(Rotation2d.kZero)),
      m_swerve.runOnce(() -> m_swerve.resetRotation(Rotation2d.kZero)),
      new DeferredCommand(() -> vibrationCmd, Set.of()) //empty set, no requirements
    ));
  }
   
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return m_autoChooser.getSelected();
  }
}
