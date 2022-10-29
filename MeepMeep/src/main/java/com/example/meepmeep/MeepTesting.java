package com.example.meepmeep;

import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.acmerobotics.roadrunner.geometry.Pose2d;

public class MeepTesting {
    public static void main(String args[]) {
        MeepMeep meep = new MeepMeep(800);

        RoadRunnerBotEntity ArcFlashBot = new DefaultBotBuilder(meep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 12)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(38, 60, Math.toRadians(-90) ))
                                .strafeRight(26)
                                .forward(50)
                                .strafeLeft(42)
                                .turn(Math.toRadians(90))
                                .turn(Math.toRadians(-33.8))
                                .strafeLeft(10)
                                .turn(Math.toRadians(33.8))
                                .strafeRight(8)
                                .back(6)
                                .turn(Math.toRadians(-33.8))
                                .strafeLeft(10)
                                .turn(Math.toRadians(33.8))
                                .strafeRight(8)
                                .back(6)
                                .turn(Math.toRadians(-33.8))
                                .strafeLeft(10)
                                .turn(Math.toRadians(33.8))
                                .strafeRight(8)
                                .back(6)
                                .turn(Math.toRadians(-33.8))
                                .strafeLeft(10)
                                .turn(Math.toRadians(33.8))
                                .strafeRight(8)
                                .back(6)
                                .turn(Math.toRadians(-33.8))
                                .strafeLeft(10)
                                .turn(Math.toRadians(33.8))
                                .strafeRight(8)
                                .turn(Math.toRadians(90))
                                .forward(45)
                                .turn(Math.toRadians(45))
                                .back(3)
                                .build()
                );

                meep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setTheme(new ColorSchemeRedDark())
                .setBackgroundAlpha(1f)
                .setDarkMode(true)
                .addEntity(ArcFlashBot)
                        .start();






    }
}