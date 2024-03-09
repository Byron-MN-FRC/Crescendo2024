// RobotBuilder Version: 6.1
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: RobotContainer.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autos.exampleAuto;
import frc.robot.commands.ActivateIntake;
import frc.robot.commands.AutonomousCommand;
import frc.robot.commands.ClearIntake;
import frc.robot.commands.DriveToTag;
import frc.robot.commands.ResetState;
import frc.robot.commands.RetractIntake;
import frc.robot.commands.RunClimb;
import frc.robot.commands.ShootHighSpeakerEnd;
import frc.robot.commands.ShootHighSpeakerStart;
import frc.robot.commands.ShootLowAmpEnd;
import frc.robot.commands.ShootLowAmpStart;
import frc.robot.commands.SpampIntake;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.autonAcquire;
import frc.robot.commands.autonAmp;
import frc.robot.commands.autonShoot;
import frc.robot.subsystems.Acquisition;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Spamp;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot
 * (including subsystems, commands, and button mappings) should be declared
 * here.
 */
public class RobotContainer {

  private static RobotContainer m_robotContainer = new RobotContainer();

  // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
  // The robot's subsystems

  // Joysticks

  // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
  /* Controllers */

  public final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController accessory = new CommandXboxController(1);

  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  /* Driver Buttons */
  private final JoystickButton zeroGyro = new JoystickButton(driver.getHID(), XboxController.Button.kX.value);

  /* Subsystems */
  private final Swerve s_Swerve = new Swerve();
  public final Acquisition m_acquisition = new Acquisition();
  public final Climb m_climb = new Climb();
  public final Spamp m_spamp = new Spamp();
  public final Vision m_final = new Vision();
  public PneumaticHub ph = new PneumaticHub(20);

  /* Path Planner */
  private final SendableChooser<Command> autoChooser;
  public Field2d field = new Field2d();


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */

  private RobotContainer() {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=SMARTDASHBOARD
    // Smartdashboard Subsystems

    // SmartDashboard Buttons
    SmartDashboard.putData("Autonomous Command", new AutonomousCommand());
    
   // SmartDashboard.put

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=SMARTDASHBOARD
    SmartDashboard.putNumber("left", accessory.getRawAxis(1));
    SmartDashboard.putNumber("right", accessory.getRawAxis(1));

    // Register Named Commands 
    NamedCommands.registerCommand("autonShootCommand", new autonShoot(m_spamp));
    NamedCommands.registerCommand("autonAcquireCommand", new autonAcquire(m_acquisition));
    NamedCommands.registerCommand("autonAmpCommand", new autonAmp(m_spamp));

    ph.enableCompressorAnalog(100, 120);
    SmartDashboard.putNumber("PressureLim", 45);

    field = new Field2d();
    SmartDashboard.putData("Field", field);
    // Logging callback for current robot pose
    PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
      // Do whatever you want with the pose here
      field.setRobotPose(pose);

    //SmartDashboard.putNumber("Battery Voltage",powerDistribution.getVoltage());
    });

    // Logging callback for target robot pose
    PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
      // Do whatever you want with the pose here
      field.getObject("target pose").setPose(pose);
    });

    // Logging callback for the active path, this is sent as a list of poses
    PathPlannerLogging.setLogActivePathCallback((poses) -> {
      // Do whatever you want with the poses here
      field.getObject("path").setPoses(poses);
    });

    s_Swerve.setDefaultCommand(
        new TeleopSwerve(
            s_Swerve,
            () -> -driver.getRawAxis(translationAxis),
            () -> -driver.getRawAxis(strafeAxis),
            () -> -driver.getRawAxis(rotationAxis),
            () -> false));
    // robotCentric.getAsBoolean() ---> false
    autoChooser = AutoBuilder.buildAutoChooser();
    autoChooser.addOption("DrawS", new exampleAuto(s_Swerve));
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=SUBSYSTEM_DEFAULT_COMMAND

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=SUBSYSTEM_DEFAULT_COMMAND

    // Configure autonomous sendable chooser
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=AUTONOMOUS

    autoChooser.setDefaultOption("Autonomous Command", new AutonomousCommand());

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=AUTONOMOUS

    SmartDashboard.putData("Auto Mode", autoChooser);
  }

  public static RobotContainer getInstance() {
    return m_robotContainer;
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=BUTTONS
    // Create some buttons

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=BUTTONS

    /* Driver Buttons */
    zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));

    /* Aquisition buttons */
    driver.rightTrigger(.5)
        .onFalse(new RetractIntake(m_acquisition).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

    driver.rightTrigger(.5)
        .onTrue(new ActivateIntake(m_acquisition).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

    /* Spamp Buttons */
    final JoystickButton btnSpampIntake = new JoystickButton(driver.getHID(), XboxController.Button.kY.value);
    btnSpampIntake.whileTrue(new SpampIntake(m_spamp).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

    final JoystickButton btnEndAmp = new JoystickButton(driver.getHID(), XboxController.Button.kLeftBumper.value);
    btnEndAmp.onFalse(new ShootLowAmpEnd(m_spamp).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

    final JoystickButton btnStartAmp = new JoystickButton(driver.getHID(), XboxController.Button.kLeftBumper.value);
    btnStartAmp.onTrue(new ShootLowAmpStart(m_spamp).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

    final JoystickButton btnEndSpeaker = new JoystickButton(driver.getHID(), XboxController.Button.kRightBumper.value);
    btnEndSpeaker.onFalse(new ShootHighSpeakerEnd(m_spamp).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

    final JoystickButton btnStartSpeaker = new JoystickButton(driver.getHID(),
        XboxController.Button.kRightBumper.value);
    btnStartSpeaker.onTrue(new ShootHighSpeakerStart(m_spamp).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
  
    final JoystickButton btnDriveToTag = new JoystickButton(driver.getHID(),
        XboxController.Button.kA.value);
    btnDriveToTag.whileTrue(new DriveToTag(
            s_Swerve, 
            () -> -driver.getRawAxis(translationAxis),
            () -> -driver.getRawAxis(strafeAxis),
            () -> -driver.getRawAxis(rotationAxis),
            () -> false
    ).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
  
    /* Accessory Buttons */
    final JoystickButton btnClearIntake = new JoystickButton(accessory.getHID(), XboxController.Button.kA.value);
      btnClearIntake.whileTrue(new ClearIntake(m_acquisition).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

    final JoystickButton btnResetState = new JoystickButton(accessory.getHID(), XboxController.Button.kB.value);
      btnResetState.onTrue(new ResetState(m_acquisition, m_spamp).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    accessory.leftTrigger(0.5)
        .whileTrue(new RunClimb(m_climb));

    SmartDashboard.putData("Pathfind Center Field", AutoBuilder.pathfindToPose(
      new Pose2d(6.0, 0.0, Rotation2d.fromDegrees(0)), 
      new PathConstraints(
        2.0, 2.0, 
        Units.degreesToRadians(360), Units.degreesToRadians(540)
      ), 
      0, 
      2.0
    ));
    }
  

  // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS

  // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // The selected command will be run in autonomous
    return autoChooser.getSelected();
  }

}
