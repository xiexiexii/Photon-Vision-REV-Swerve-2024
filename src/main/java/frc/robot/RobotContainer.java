// Imports stuff (again!)

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.Controls;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.LEDColorChangeCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

// This class is where the bulk of the robot should be declared.  Since Command-based is a
// "declarative" paradigm, very little robot logic should actually be handled in the Robot
// periodic methods (other than the scheduler calls).  Instead, the structure of the robot
// (including subsystems, commands, and button mappings) should be declared here.

public class RobotContainer {

  // Robot's Subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final LEDSubsystem m_LEDSubsystem = new LEDSubsystem();
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  // Controllers
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  // The container for the robot. Contains subsystems, OI devices, and commands.
  public RobotContainer() {

    // Configure the button bindings
    configureButtonBindings();

    // DEPRECATED: Calibrates Gyro
    m_robotDrive.calibrateGyro();

    // Configure Default Commands
    m_robotDrive.setDefaultCommand(
      
        // The left stick on the controller controls robot translation.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(

            // Joystick input to tele-op control
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(-m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(-m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true, true), m_robotDrive));

    SmartDashboard.putData("AutoMode", m_chooser);

    // Named Command Configuration
    NamedCommands.registerCommand("Change LED Color", new LEDColorChangeCommand(m_LEDSubsystem));

    // Autos
    m_chooser.addOption("Curvy yay", m_robotDrive.getAuto("Curvy yay"));
    m_chooser.addOption("Move and Spin", m_robotDrive.getAuto("Move and Spin"));

    // Load a Choreo trajectory as a PathPlannerPath
    PathPlannerPath Test = PathPlannerPath.fromChoreoTrajectory("Test");

    // Choreo Autos
    m_chooser.addOption("Choreo Test 1", m_robotDrive.getAuto("Test"));

  }

  // Use this method to define your button to command mappings. Buttons can be
  // created by instantiating a edu.wpi.first.wpilibj.GenericHID or one of its
  // subclasses edu.wpi.first.wpilibj.Joystick or XboxController, and then
  // passing it to a JoystickButton.

  private void configureButtonBindings() {
    new JoystickButton(m_driverController, Controls.setXValue)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));
            // Sets wheels in an X position to prevent movement
            // A button driver
  }

  public Command getAutonomousCommand() {
    return m_chooser.getSelected();   
  } 
}