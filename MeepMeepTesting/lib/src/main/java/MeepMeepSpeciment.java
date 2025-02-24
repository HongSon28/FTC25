import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepSpeciment {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 30, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(18,18)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-25, 62, Math.toRadians(270)))
                .splineTo(new Vector2d(-52,56),Math.toRadians(90))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(-30,26),Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(-30,10),Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(-46,10),Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(-46,56),Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(-46,10),Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(-55,10),Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(-55,56),Math.toRadians(270))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
