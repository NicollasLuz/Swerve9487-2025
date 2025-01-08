// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
// import com.pathplanner.lib.config.ReplanningConfig;
// import com.pathplanner.lib.config.HolonomicPathFollowerConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.SwerveSubsystem;

/** Add your docs here. */
public class ConfigAuto {

    SwerveSubsystem swerve;
    final boolean enableFeedforward = true;
    
    
    public ConfigAuto(SwerveSubsystem swerve) {
        this.swerve = swerve;
    }
    
    public void setupPathPlanner()
    {   
        RobotConfig config;
        config = RobotConfig.fromGUISettings();

        AutoBuilder.configure(
            swerve::getPose, // Robot pose supplier
            swerve::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            swerve::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speedsRobotRelative, moduleFeedForwards)-> {
                if (enableFeedforward)
                {
                  swerve.drive(
                      speedsRobotRelative,
                      swerve.getKinematics().toSwerveModuleStates(speedsRobotRelative),
                      moduleFeedForwards.linearForces()
                                   );
                } else
                {
                  swerve.setChassisSpeeds(speedsRobotRelative);
                }
              }, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new PPHolonomicDriveController( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                                         new PIDConstants(5.0, 0.0, 0.0),
                                         // Translation PID constants
                                         new PIDConstants(swerve.getSwerveController().config.headingPIDF.p,
                                                          swerve.getSwerveController().config.headingPIDF.i,
                                                          swerve.getSwerveController().config.headingPIDF.d),
                                         // Rotation PID constants
                                         4.5
            ),
            config,
            () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                    var alliance = DriverStation.getAlliance();
                    return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
                },
            swerve // Reference to this subsystem to set requirements
                                  );
    }

}
