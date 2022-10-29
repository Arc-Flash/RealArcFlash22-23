package com.example.meepmeep;

import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.*;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.acmerobotics.roadrunner.geometry.Pose2d;

public class MeepTesting {
    public static void main(String args[]) {
        MeepMeep meep = new MeepMeep(800);

        RoadRunnerBotEntity ArcFlashBot = new DefaultBotBuilder(meep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(100, 100, Math.toRadians(180), Math.toRadians(180), 13)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-38, -60, Math.toRadians(90) ))
                                .strafeRight(26)
                                .forward(48)
                                .strafeLeft(43)
                                .turn(Math.toRadians(90))
                                .waitSeconds(.5)

                                .turn(Math.toRadians(-60))
                                .back(5)
                                .waitSeconds(.5)
                                .forward(5)
                                .turn(Math.toRadians(60))
                                .waitSeconds(.5)

                                .turn(Math.toRadians(-60))
                                .back(5)
                                .waitSeconds(.5)
                                .forward(5)
                                .turn(Math.toRadians(60))
                                .waitSeconds(.5)

                                .turn(Math.toRadians(-60))
                                .back(5)
                                .waitSeconds(.5)
                                .forward(5)
                                .turn(Math.toRadians(60))
                                .waitSeconds(.5)

                                .turn(Math.toRadians(-60))
                                .back(5)
                                .waitSeconds(.5)
                                .forward(5)
                                .turn(Math.toRadians(60))
                                .waitSeconds(.5)

                                .turn(Math.toRadians(-60))
                                .back(5)
                                .waitSeconds(.5)
                                .forward(5)
                                .turn(Math.toRadians(60))
                                .waitSeconds(.5)

                                .forward(4)
                                .turn(Math.toRadians(90))
                                .forward(50)


                                .build()
                );

                meep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setTheme(new ColorSchemeBlueDark())
                .setBackgroundAlpha(1f)
                .setDarkMode(true)
                .addEntity(ArcFlashBot)
                        .start();






    }
}