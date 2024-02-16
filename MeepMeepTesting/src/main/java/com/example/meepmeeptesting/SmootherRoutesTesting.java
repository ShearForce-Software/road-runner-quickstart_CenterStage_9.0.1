package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class SmootherRoutesTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(500);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        // BlueFarStackAuto
                        drive.trajectorySequenceBuilder(new Pose2d(-35.5, 62.5, Math.toRadians(270)))
                                .splineToLinearHeading(new Pose2d(-35.5, 33, Math.toRadians(270)), Math.toRadians(270))
                                // Position - 1 floor
                                .splineToLinearHeading (new Pose2d(-27, 33, Math.toRadians(180)), Math.toRadians(0))//CHANGED
                                .splineToLinearHeading(new Pose2d(-34.5, 33, Math.toRadians(180)), Math.toRadians(0))
                                .setTangent(Math.toRadians(270))
                                .splineToLinearHeading(new Pose2d(-56, 11.5, Math.toRadians(180)), Math.toRadians(180))

                                // Position - 2 floor
                                //.splineToLinearHeading(new Pose2d(-38.5, 12.5, Math.toRadians(270)), Math.toRadians(270))
                                // Position - 3 floor
                                //.splineToLinearHeading(new Pose2d(-37.0, 20.5, Math.toRadians(315)), Math.toRadians(315))
                                // Drive to stack
                                //.splineToLinearHeading(new Pose2d(-38.5, 12, Math.toRadians(180)), Math.toRadians(0))
                                //.splineToLinearHeading(new Pose2d(-56, 12, Math.toRadians(180)), Math.toRadians(0))
                                //.setTangent(Math.toRadians(270))
                            //    .splineToLinearHeading(new Pose2d(-34.5,12, Math.toRadians(0)), Math.toRadians(270))
                             //   .splineToLinearHeading(new Pose2d(-56,12, Math.toRadians(180)), Math.toRadians(180))
                               // .splineToLinearHeading(new Pose2d(-56,12, Math.toRadians(180)), Math.toRadians(180))//CHANGED
                                // Drive to backboard area
                                //.setTangent(0)
                                //.splineToLinearHeading(new Pose2d(-30, 9, Math.toRadians(180)), Math.toRadians(0))
                                //.splineToLinearHeading(new Pose2d(-30, 9, Math.toRadians(180)), Math.toRadians(0))
                                //.splineToLinearHeading(new Pose2d(50, 9, Math.toRadians(180)), Math.toRadians(0))
                                // Position - 1 board
                                .setTangent(0)
                                .splineToLinearHeading(new Pose2d(50, 11.5, Math.toRadians(180)), Math.toRadians(0))
                                // Position - 2 board
                                //.setTangent(0)
                                //.splineToLinearHeading(new Pose2d(50, 29, Math.toRadians(180)), Math.toRadians(0))
                                // Position - 3 board
                                //.setTangent(Math.toRadians(90))
                                .setTangent(Math.toRadians(90))
                                .splineToLinearHeading(new Pose2d(50, 42, Math.toRadians(180)), Math.toRadians(90))
                                //drive back to stack
                                //.setReversed(true)
                                .setTangent(Math.toRadians(180))
                                .splineToLinearHeading(new Pose2d(30,11.5, Math.toRadians(180)), Math.toRadians(180))
                                //.setTangent(Math.toRadians(180))
                                .splineToLinearHeading(new Pose2d(-50,11.5, Math.toRadians(180)), Math.toRadians(180))
                                // Drive to backboard area
                                .setTangent(0)
                                //.splineToLinearHeading(new Pose2d(-30, 9, Math.toRadians(180)), Math.toRadians(0))
                                .splineToLinearHeading(new Pose2d(50, 11.5, Math.toRadians(180)), Math.toRadians(0))
                                // Position - 1 board
                                //.setTangent(0)
                                //.splineToLinearHeading(new Pose2d(50, 34, Math.toRadians(180)), Math.toRadians(0))
                                // Position - 2 board
                                //.setTangent(0)
                                //.splineToLinearHeading(new Pose2d(50, 29, Math.toRadians(180)), Math.toRadians(0))
                                // Position - 3 board
                                .setTangent(Math.toRadians(90))
                                .splineToLinearHeading(new Pose2d(50, 36, Math.toRadians(180)), Math.toRadians(270))
                                // Parking Position
                                .splineToLinearHeading(new Pose2d(48, 15, Math.toRadians(270)), Math.toRadians(270))
                                //.lineTo(new Vector2d(47, 55)) //lineToX(47)
                                //.splineToLinearHeading(new Pose2d(48, 15, Math.toRadians(270)), Math.toRadians(270))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}