package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class CloseRouteTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(500);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                         drive.trajectorySequenceBuilder(new Pose2d(48,-25,Math.toRadians(180)))
                                 .splineToLinearHeading(new Pose2d(27,-33, Math.toRadians(0)), Math.toRadians(180))
                                 .setTangent(Math.toRadians(180))
                                 .splineToLinearHeading(new Pose2d(0,-33, Math.toRadians(0)), Math.toRadians(0))
                                 .splineToLinearHeading(new Pose2d(12, -33, Math.toRadians(0)), Math.toRadians(0))
                                 .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}