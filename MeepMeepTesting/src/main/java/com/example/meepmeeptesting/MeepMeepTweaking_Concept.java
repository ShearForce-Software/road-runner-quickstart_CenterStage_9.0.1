package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTweaking_Concept {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(500);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(17, 15)
                .followTrajectorySequence(drive ->


                        // BlueRightAuto
                        //drive.trajectorySequenceBuilder(new Pose2d(-35.5, 62.5, Math.toRadians(270)))
                          //      .splineToLinearHeading(new Pose2d(-38.5, 33, Math.toRadians(270)), Math.toRadians(270))
                                // Position - 1 floor
                            //    .splineToLinearHeading (new Pose2d(-27, 33, Math.toRadians(180)), Math.toRadians(180))
                              //  .splineToLinearHeading(new Pose2d(-34.5, 32, Math.toRadians(180)), Math.toRadians(180))
                                // Position - 2 floor
                              //  .splineToLinearHeading(new Pose2d(-38.5, 12.5, Math.toRadians(270)), Math.toRadians(270))
                                // Position - 3 floor
                                //.splineToLinearHeading (new Pose2d(-38.5, 20.5, Math.toRadians(315)), Math.toRadians(315))
                                // Drive to stack
                                //.setTangent(270)
                                //.splineToLinearHeading(new Pose2d(-38.5, 12, Math.toRadians(180)), Math.toRadians(0))
                                //.splineToLinearHeading(new Pose2d(-54, 12, Math.toRadians(180)), Math.toRadians(0))
                                // Drive to backboard area
                                //.setTangent(0)
                                //.splineToLinearHeading(new Pose2d(-30, 9, Math.toRadians(180)), Math.toRadians(0))
                                //.splineToLinearHeading(new Pose2d(30, 9, Math.toRadians(180)), Math.toRadians(0))
                                // Position - 1 board
                                //.setTangent(0)
                                //.splineToLinearHeading(new Pose2d(50, 34, Math.toRadians(180)), Math.toRadians(0))
                                // Position - 2 board
                                //.setTangent(0)
                                //.splineToLinearHeading(new Pose2d(50, 28, Math.toRadians(180)), Math.toRadians(0))
                                // Position - 3 board
                                //.setTangent(0)
                                //.splineToLinearHeading(new Pose2d(50, 22, Math.toRadians(180)), Math.toRadians(0))
                                // Parking Position
                                //.splineToLinearHeading(new Pose2d(48, 15, Math.toRadians(270)), Math.toRadians(270))



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


                        // RedLeftAuto
                        drive.trajectorySequenceBuilder(new Pose2d(-35.5,-62.5,Math.toRadians(90)))

                                // Position 1 - floor
                                //.splineToLinearHeading (new Pose2d(-38.5, -20.5, Math.toRadians(45)), Math.toRadians(45))
                                //.waitSeconds(2)

                                // Position 2 - floor
                                //.splineToLinearHeading(new Pose2d(-38.5, -33, Math.toRadians(90)), Math.toRadians(90))
                                //.splineToLinearHeading(new Pose2d(-38.5, -14.5, Math.toRadians(90)), Math.toRadians(90))
                                //.waitSeconds(2)

                                // Position 3 - floor
                                .splineToLinearHeading(new Pose2d(-38.5, -33, Math.toRadians(90)), Math.toRadians(90))
                                .splineToLinearHeading (new Pose2d(-27, -33, Math.toRadians(180)), Math.toRadians(180))
                                .splineToLinearHeading(new Pose2d(-34.5, -32, Math.toRadians(180)), Math.toRadians(180))
                                .waitSeconds(2)

                                //Drive to stack
                                .splineToLinearHeading(new Pose2d(-45, -11.5, Math.toRadians(180)), Math.toRadians(180))
                                .splineToLinearHeading(new Pose2d(-58,-11.5, Math.toRadians(180)), Math.toRadians(180))
                                .waitSeconds(2)

                                // Drive to the backboard area
                                .setTangent(Math.toRadians(0))
                                .splineToLinearHeading(new Pose2d(-30,-9, Math.toRadians(180)), Math.toRadians(0))
                                .splineToLinearHeading(new Pose2d(30,-9, Math.toRadians(180)), Math.toRadians(0))

                                //Drive to board position to clear fallen pixels or move partner out of way
                                .splineToLinearHeading(new Pose2d(50,-17, Math.toRadians(180)), Math.toRadians(0))

                                //Strafe to Position 1 - board
                                //.setTangent(Math.toRadians(270))
                                //.splineToLinearHeading(new Pose2d(50,-29, Math.toRadians(180)), Math.toRadians(270))
                                //.waitSeconds(1)

                                //Strafe to Position 2 - board
                                //.setTangent(Math.toRadians(270))
                                //.splineToLinearHeading(new Pose2d(50,-35, Math.toRadians(180)), Math.toRadians(270))
                                //.waitSeconds(1)

                                //Strafe to Position 3 - board
                                .setTangent(Math.toRadians(270))
                                .splineToLinearHeading(new Pose2d(50,-41, Math.toRadians(180)), Math.toRadians(270))
                                .waitSeconds(1)

                                // Position 1 - board
//                                .setTangent(0)
//                                .splineToLinearHeading(new Pose2d(50, -28, Math.toRadians(180)), Math.toRadians(0))
//                                .waitSeconds(1)

                                // Position 2 - board

                                //.splineToLinearHeading(new Pose2d(50, -34, Math.toRadians(180)), Math.toRadians(0))
                                //.waitSeconds(1)

                                // Position 3 - board

                                //.splineToLinearHeading(new Pose2d(50, -40, Math.toRadians(180)), Math.toRadians(0))
                                //.waitSeconds(1)


                                //Drive to stack
 //                               .setTangent(Math.toRadians(180))
 //                               .splineToLinearHeading(new Pose2d(30,-9, Math.toRadians(180)), Math.toRadians(180))
 //                               .splineToLinearHeading(new Pose2d(-38.5, -9, Math.toRadians(180)), Math.toRadians(180))
 //                               .splineToLinearHeading(new Pose2d(-58,-11.5, Math.toRadians(180)), Math.toRadians(180))
 //                               .waitSeconds(2)

                                //Drive to board
 //                               .setTangent(Math.toRadians(0))
 //                               .splineToLinearHeading(new Pose2d(-30,-9, Math.toRadians(180)), Math.toRadians(0))
 //                               .splineToLinearHeading(new Pose2d(30,-9, Math.toRadians(180)), Math.toRadians(0))

                                // Position 2 - board
//                                .splineToLinearHeading(new Pose2d(50, -34, Math.toRadians(180)), Math.toRadians(0))

                                // Parking Position
                                .setTangent(Math.toRadians(180))
                                .splineToLinearHeading(new Pose2d(42,-20,Math.toRadians(90)), Math.toRadians(90))
                                //.setTangent(Math.toRadians(0))
                                //.splineToLinearHeading(new Pose2d(38, -34, Math.toRadians(180)), Math.toRadians(0))
                                //.splineToLinearHeading(new Pose2d(45,-15, Math.toRadians(90)), Math.toRadians(90))


                                // RedRightAuto
/*                         drive.trajectorySequenceBuilder(new Pose2d(12, -62.5, Math.toRadians(90)))
                                // Left Spike Strip
                                    // Position 1 - floor
                                 .splineToLinearHeading(new Pose2d(12.5, -33, Math.toRadians(90)), Math.toRadians(90))
                                 .splineToLinearHeading (new Pose2d(0, -33, Math.toRadians(0)), Math.toRadians(0))
                                 .splineToLinearHeading(new Pose2d(12.5, -33, Math.toRadians(0)), Math.toRadians(0))
                                 .waitSeconds(2)
                                    //Drive to board position to clear fallen pixels or move partner out of way
                                 .splineToLinearHeading(new Pose2d(50,-17, Math.toRadians(180)), Math.toRadians(0))
                                 .setTangent(Math.toRadians(270))
                                    //Strafe to Position 1 - board
                                 .splineToLinearHeading(new Pose2d(50,-29, Math.toRadians(180)), Math.toRadians(270))
                                    //Drive to stack
                                 .setTangent(Math.toRadians(180))
                                 .splineToLinearHeading(new Pose2d(24,-11.5, Math.toRadians(180)), Math.toRadians(180))
                                 .splineToLinearHeading(new Pose2d(-58,-11.5, Math.toRadians(180)), Math.toRadians(180))
                                 .waitSeconds(2)
                                    //Return to board
                                 .setTangent(Math.toRadians(0))
                                 .splineToLinearHeading(new Pose2d(24,-11.5, Math.toRadians(180)), Math.toRadians(0))
                                    //Position 1 - board
                                 .splineToLinearHeading(new Pose2d(50,-29, Math.toRadians(180)), Math.toRadians(0))
                                    //Drive to stack
                                 .setTangent(Math.toRadians(180))
                                 .splineToLinearHeading(new Pose2d(24,-11.5, Math.toRadians(180)), Math.toRadians(180))
                                 .splineToLinearHeading(new Pose2d(-58,-11.5, Math.toRadians(180)), Math.toRadians(180))
                                 .waitSeconds(2)
                                    //Return to board
                                 .setTangent(Math.toRadians(0))
                                 .splineToLinearHeading(new Pose2d(24,-11.5, Math.toRadians(180)), Math.toRadians(0))
                                    //Position 1 - board
                                 .splineToLinearHeading(new Pose2d(50,-29, Math.toRadians(180)), Math.toRadians(0))
                                    // Parking Position
                                 .setTangent(Math.toRadians(180))
                                 .splineToLinearHeading(new Pose2d(42,-20,Math.toRadians(90)), Math.toRadians(90))
*/
/*                                // Left Spike Strip - strafe
                                    // Position 1 - floor
                                 .splineToLinearHeading(new Pose2d(12.5, -33, Math.toRadians(90)), Math.toRadians(90))
                                 .splineToLinearHeading (new Pose2d(0, -33, Math.toRadians(0)), Math.toRadians(0))
                                 .splineToLinearHeading(new Pose2d(12.5, -33, Math.toRadians(0)), Math.toRadians(0))
                                 .waitSeconds(2)
                                    //Drive to board position to clear fallen pixels or move partner out of way
                                 .splineToLinearHeading(new Pose2d(50,-17, Math.toRadians(180)), Math.toRadians(0))
                                 .setTangent(Math.toRadians(270))
                                    //Strafe to Position 1 - board
                                 .splineToLinearHeading(new Pose2d(50,-29, Math.toRadians(180)), Math.toRadians(270))
                                    //Drive to stack
                                 .setTangent(Math.toRadians(180))
                                 .splineToLinearHeading(new Pose2d(24,-11.5, Math.toRadians(180)), Math.toRadians(180))
                                 .splineToLinearHeading(new Pose2d(-58,-11.5, Math.toRadians(180)), Math.toRadians(180))
                                 .waitSeconds(2)
                                    //Return to board
                                 .setTangent(Math.toRadians(0))
                                 .splineToLinearHeading(new Pose2d(24,-11.5, Math.toRadians(180)), Math.toRadians(0))
                                 .splineToLinearHeading(new Pose2d(50,-17, Math.toRadians(180)), Math.toRadians(0))
                                 .setTangent(Math.toRadians(270))
                                 //Strafe to Position 1 - board
                                 .splineToLinearHeading(new Pose2d(50,-29, Math.toRadians(180)), Math.toRadians(270))

                                 //Drive to stack
                                 .setTangent(Math.toRadians(180))
                                 .splineToLinearHeading(new Pose2d(24,-11.5, Math.toRadians(180)), Math.toRadians(180))
                                 .splineToLinearHeading(new Pose2d(-58,-11.5, Math.toRadians(180)), Math.toRadians(180))
                                 .waitSeconds(2)
                                 //Return to board
                                 .setTangent(Math.toRadians(0))
                                 .splineToLinearHeading(new Pose2d(24,-11.5, Math.toRadians(180)), Math.toRadians(0))
                                 .splineToLinearHeading(new Pose2d(50,-17, Math.toRadians(180)), Math.toRadians(0))
                                 .setTangent(Math.toRadians(270))
                                 //Strafe to Position 1 - board
                                 .splineToLinearHeading(new Pose2d(50,-29, Math.toRadians(180)), Math.toRadians(270))
                                 // Parking Position
                                 .setTangent(Math.toRadians(180))
                                 .splineToLinearHeading(new Pose2d(42,-20,Math.toRadians(90)), Math.toRadians(90))
*/
  /*                               // Center Spike Strip
                                 // Position 2 - floor
                                 .splineToLinearHeading(new Pose2d(12.5, -11, Math.toRadians(90)), Math.toRadians(90))
                                 .waitSeconds(2)
                                 //Drive to board position to clear fallen pixels or move partner out of way
                                 .setTangent(Math.toRadians(0))
                                 .splineToLinearHeading(new Pose2d(50,-17, Math.toRadians(180)), Math.toRadians(0))
                                 .setTangent(Math.toRadians(270))
                                 //Strafe to Position 1 - board
                                 .splineToLinearHeading(new Pose2d(50,-34, Math.toRadians(180)), Math.toRadians(270))
                                 //Drive to stack
                                 .setTangent(Math.toRadians(180))
                                 .splineToLinearHeading(new Pose2d(24,-11.5, Math.toRadians(180)), Math.toRadians(180))
                                 .splineToLinearHeading(new Pose2d(-58,-11.5, Math.toRadians(180)), Math.toRadians(180))
                                 .waitSeconds(2)
                                 //Return to board
                                 .setTangent(Math.toRadians(0))
                                 .splineToLinearHeading(new Pose2d(24,-11.5, Math.toRadians(180)), Math.toRadians(0))
                                 //Position 2 - board
                                 .splineToLinearHeading(new Pose2d(50,-34, Math.toRadians(180)), Math.toRadians(0))
                                 //Drive to stack
                                 .setTangent(Math.toRadians(180))
                                 .splineToLinearHeading(new Pose2d(24,-11.5, Math.toRadians(180)), Math.toRadians(180))
                                 .splineToLinearHeading(new Pose2d(-58,-11.5, Math.toRadians(180)), Math.toRadians(180))
                                 .waitSeconds(2)
                                 //Return to board
                                 .setTangent(Math.toRadians(0))
                                 .splineToLinearHeading(new Pose2d(24,-11.5, Math.toRadians(180)), Math.toRadians(0))
                                 //Position 2 - board
                                 .splineToLinearHeading(new Pose2d(50,-34, Math.toRadians(180)), Math.toRadians(0))
                                 // Parking Position
                                 .setTangent(Math.toRadians(180))
                                 .splineToLinearHeading(new Pose2d(42,-25,Math.toRadians(90)), Math.toRadians(90))
*/
/*                                 // Right Spike Strip
                                 // Position 3 - floor
                                 .splineToLinearHeading(new Pose2d(23, -11, Math.toRadians(90)), Math.toRadians(90))
                                 .waitSeconds(2)
                                 //Drive to board position to clear fallen pixels or move partner out of way
                                 .setTangent(Math.toRadians(0))
                                 .splineToLinearHeading(new Pose2d(50,-17, Math.toRadians(180)), Math.toRadians(0))
                                 .setTangent(Math.toRadians(270))
                                 //Strafe to Position 1 - board
                                 .splineToLinearHeading(new Pose2d(50,-40, Math.toRadians(180)), Math.toRadians(270))
                                 //Drive to stack
                                 .setTangent(Math.toRadians(180))
                                 .splineToLinearHeading(new Pose2d(24,-11.5, Math.toRadians(180)), Math.toRadians(180))
                                 .splineToLinearHeading(new Pose2d(-58,-11.5, Math.toRadians(180)), Math.toRadians(180))
                                 .waitSeconds(2)
                                 //Return to board
                                 .setTangent(Math.toRadians(0))
                                 .splineToLinearHeading(new Pose2d(24,-11.5, Math.toRadians(180)), Math.toRadians(0))
                                 //Position 2 - board
                                 .splineToLinearHeading(new Pose2d(50,-40, Math.toRadians(180)), Math.toRadians(0))
                                 //Drive to stack
                                 .setTangent(Math.toRadians(180))
                                 .splineToLinearHeading(new Pose2d(24,-11.5, Math.toRadians(180)), Math.toRadians(180))
                                 .splineToLinearHeading(new Pose2d(-58,-11.5, Math.toRadians(180)), Math.toRadians(180))
                                 .waitSeconds(2)
                                 //Return to board
                                 .setTangent(Math.toRadians(0))
                                 .splineToLinearHeading(new Pose2d(24,-11.5, Math.toRadians(180)), Math.toRadians(0))
                                 //Position 2 - board
                                 .splineToLinearHeading(new Pose2d(50,-40, Math.toRadians(180)), Math.toRadians(0))
                                 // Parking Position
                                 .setTangent(Math.toRadians(180))
                                 .splineToLinearHeading(new Pose2d(42,-29,Math.toRadians(90)), Math.toRadians(90))
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