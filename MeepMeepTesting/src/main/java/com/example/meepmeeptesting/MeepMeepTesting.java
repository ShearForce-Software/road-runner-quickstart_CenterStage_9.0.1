package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->


                        // BlueRightAuto
                        drive.trajectorySequenceBuilder(new Pose2d(-35.5, 62.5, Math.toRadians(270)))
                                .splineToLinearHeading(new Pose2d(-38.5, 33, Math.toRadians(270)), Math.toRadians(270))
                                // Position - 1 floor
                                .splineToLinearHeading (new Pose2d(-27, 33, Math.toRadians(180)), Math.toRadians(180))
                                .splineToLinearHeading(new Pose2d(-34.5, 32, Math.toRadians(180)), Math.toRadians(180))
                                // Position - 2 floor
                              //  .splineToLinearHeading(new Pose2d(-38.5, 12.5, Math.toRadians(270)), Math.toRadians(270))
                                // Position - 3 floor
                                //.splineToLinearHeading (new Pose2d(-38.5, 20.5, Math.toRadians(315)), Math.toRadians(315))
                                // Drive to stack
                                .setTangent(270)
                                .splineToLinearHeading(new Pose2d(-38.5, 12, Math.toRadians(180)), Math.toRadians(0))
                                .splineToLinearHeading(new Pose2d(-54, 12, Math.toRadians(180)), Math.toRadians(0))
                                // Drive to backboard area
                                .setTangent(0)
                                .splineToLinearHeading(new Pose2d(-30, 9, Math.toRadians(180)), Math.toRadians(0))
                                .splineToLinearHeading(new Pose2d(30, 9, Math.toRadians(180)), Math.toRadians(0))
                                // Position - 1 board
                                .setTangent(0)
                                .splineToLinearHeading(new Pose2d(50, 34, Math.toRadians(180)), Math.toRadians(0))
                                // Position - 2 board
                                //.setTangent(0)
                                //.splineToLinearHeading(new Pose2d(50, 28, Math.toRadians(180)), Math.toRadians(0))
                                // Position - 3 board
                                //.setTangent(0)
                                //.splineToLinearHeading(new Pose2d(50, 22, Math.toRadians(180)), Math.toRadians(0))
                                // Parking Position
                                .splineToLinearHeading(new Pose2d(48, 15, Math.toRadians(270)), Math.toRadians(270))



                        // BlueLeftAuto
                        //drive.trajectorySequenceBuilder(new Pose2d(12, 62.5, Math.toRadians(270)))
                                // Position 1 - board
                                //.splineToLinearHeading(new Pose2d(38, 40, Math.toRadians(180)), Math.toRadians(270))
                                //.splineToLinearHeading(new Pose2d(46, 38, Math.toRadians(180)), Math.toRadians(180))
                                // Position 2 - board
                                //.splineToLinearHeading(new Pose2d(38, 34, Math.toRadians(180)), Math.toRadians(270))
                                //.splineToLinearHeading(new Pose2d(46, 33, Math.toRadians(180)), Math.toRadians(180))
                                // Position 3 - board
                                //.splineToLinearHeading(new Pose2d(38, 34, Math.toRadians(180)), Math.toRadians(270))
                                //.splineToLinearHeading(new Pose2d(46, 33, Math.toRadians(180)), Math.toRadians(180))

                                // Position 1 - floor
                                //.splineToLinearHeading (new Pose2d(10.5, 35, Math.toRadians(180)), Math.toRadians(180))
                                // Position 2 - floor
                                //.splineToLinearHeading(new Pose2d(16, 30, Math.toRadians(180)), Math.toRadians(180))
                                //.splineToLinearHeading(new Pose2d(12, 35, Math.toRadians(90)), Math.toRadians(180))
                                // Position 3 - floor
                                //.splineToLinearHeading(new Pose2d(-12, 35, Math.toRadians(180)), Math.toRadians(180))

                                // Parking Position
//                                .splineToLinearHeading(new Pose2d(24,35,Math.toRadians(180)), Math.toRadians(180))
//                                .setTangent(0)
//                                .splineToLinearHeading(new Pose2d(48,60,Math.toRadians(270)), Math.toRadians(0))


                        /*
                        // RedLeftAuto
                        drive.trajectorySequenceBuilder(new Pose2d(-35.5,-62.5,Math.toRadians(90)))
                                .splineToLinearHeading(new Pose2d(-38.5, -33, Math.toRadians(90)), Math.toRadians(90))
                                // Position 1 - floor
                                .splineToLinearHeading (new Pose2d(-38.5, -20.5, Math.toRadians(45)), Math.toRadians(45))
                                // Position 2 - floor
                                //.splineToLinearHeading(new Pose2d(-38.5, -12.5, Math.toRadians(90)), Math.toRadians(90))
                                // Position 3 - floor
                                //.splineToLinearHeading (new Pose2d(-27, -33, Math.toRadians(180)), Math.toRadians(180))
                                //.splineToLinearHeading(new Pose2d(-34.5, -32, Math.toRadians(180)), Math.toRadians(180))
                                // Drive to the backboard area
                                .splineToLinearHeading(new Pose2d(-38,-9, Math.toRadians(90)), Math.toRadians(90))
                                .splineToLinearHeading(new Pose2d(-30,-9, Math.toRadians(180)), Math.toRadians(0))
                                .splineToLinearHeading(new Pose2d(30,-9, Math.toRadians(180)), Math.toRadians(0))
                                // Position 1 - board
                                .setTangent(0)
                                .splineToLinearHeading(new Pose2d(50, -22, Math.toRadians(180)), Math.toRadians(0))
                                // Position 2 - board
                                //.setTangent(0)
                                //.splineToLinearHeading(new Pose2d(50, -28, Math.toRadians(180)), Math.toRadians(0))
                                // Position 3 - board
                                //.setTangent(0)
                                //.splineToLinearHeading(new Pose2d(50, -34, Math.toRadians(180)), Math.toRadians(0))
                                // Parking Position
                                .splineToLinearHeading(new Pose2d(48,-15, Math.toRadians(90)), Math.toRadians(90))
*/

                                // RedRightAuto
                         /*drive.trajectorySequenceBuilder(new Pose2d(12, -62.5, Math.toRadians(90)))
                                // Position 1 - board
                                .splineToLinearHeading(new Pose2d(38, -22, Math.toRadians(180)), Math.toRadians(90))
                                .splineToLinearHeading(new Pose2d(50, -22, Math.toRadians(180)), Math.toRadians(180))
                                // Position 2 - board
                                //.splineToLinearHeading(new Pose2d(38, -28, Math.toRadians(180)), Math.toRadians(90))
                                //.splineToLinearHeading(new Pose2d(50, -28, Math.toRadians(180)), Math.toRadians(180))
                                // Position 3 - board
                                //.splineToLinearHeading(new Pose2d(38, -34, Math.toRadians(180)), Math.toRadians(90))
                                //.splineToLinearHeading(new Pose2d(50, -34, Math.toRadians(180)), Math.toRadians(180))
                                // Position 1 - floor
                                .splineToLinearHeading(new Pose2d(-12, -35, Math.toRadians(180)), Math.toRadians(180))
                                // Position 2 - floor
                                //.splineToLinearHeading(new Pose2d(12, -35, Math.toRadians(180)), Math.toRadians(180))
                                //.splineToLinearHeading(new Pose2d(12, -36, Math.toRadians(270)), Math.toRadians(180))
                                // Position 3 - floor
                                //.splineToLinearHeading (new Pose2d(10.5, -35, Math.toRadians(180)), Math.toRadians(180))
                                // Parking Position
                                .splineToLinearHeading(new Pose2d(24,-35,Math.toRadians(180)), Math.toRadians(180))
                                .setTangent(0)
                                .splineToLinearHeading(new Pose2d(48,-60,Math.toRadians(90)), Math.toRadians(0))


*/
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}