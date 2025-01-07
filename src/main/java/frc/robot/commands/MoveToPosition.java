package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class MoveToPosition extends Command{

    private final SwerveSubsystem swerveSubsystem;
    private final Pose2d targetPose2d;
    private static final double TOLERANCIA = 0.1;


    public MoveToPosition(SwerveSubsystem swerveSubsystem, double x, double y){
        this.swerveSubsystem = swerveSubsystem;
        this.targetPose2d =  new Pose2d(new Translation2d(x,y), swerveSubsystem.getPose().getRotation());
        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute(){

        //Descobre onde o robô ta na arena
        Pose2d posicaoAtual = swerveSubsystem.getPose();

        //Calcula a diferença entre a posição em que o robô está e a posição em que queremos chegar
        Translation2d erro = targetPose2d.getTranslation().minus(posicaoAtual.getTranslation());


        //
        double vX = erro.getX();
        double vY = erro.getY();


        double velocidadeMax = Math.hypot(vX, vY);

        if (velocidadeMax > 1.0) {
            vX /= velocidadeMax;
            vY /= velocidadeMax;
        }

        swerveSubsystem.drive(new Translation2d(vX, vY), 0, true);
    }

    @Override
    public boolean isFinished(){
        

        Translation2d erro = targetPose2d.getTranslation().minus(swerveSubsystem.getPose().getTranslation());
        return erro.getNorm() < TOLERANCIA;
    }
    
    @Override
    public void end(boolean interromper){
        swerveSubsystem.drive(new Translation2d(0, 0), 0, true);
    }
}
