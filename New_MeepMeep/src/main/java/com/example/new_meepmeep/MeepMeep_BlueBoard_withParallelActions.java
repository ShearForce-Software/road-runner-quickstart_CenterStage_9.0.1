package com.example.new_meepmeep;


import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeep_BlueBoard_withParallelActions {
    static Pose2d startPose;
    static Pose2d stackPose;
    static Pose2d deliverToFloorPose;
    static Pose2d deliverToBoardPose;
    static Action FloorTraj;
    static Action BoardTraj2;
    static Action DriveToStack;
    static RoadRunnerBotEntity myBot;
    static int autoPosition = 1;
    static VelConstraint speedUpVelocityConstraint;
    static AccelConstraint speedUpAccelerationConstraint;
    static VelConstraint slowDownVelocityConstraint;
    static AccelConstraint slowDownAccelerationConstraint;

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(500);

        startPose = new Pose2d(12, 62.5, Math.toRadians(270));
        stackPose = new Pose2d(-55.5, 36, Math.toRadians(180));

        // Define some custom constraints to use when wanting to go faster than defaults
        speedUpVelocityConstraint = new TranslationalVelConstraint(90.0);
        speedUpAccelerationConstraint = new ProfileAccelConstraint(-70.0, 70.0);
        slowDownVelocityConstraint = new TranslationalVelConstraint(15);
        slowDownAccelerationConstraint = new ProfileAccelConstraint(-30, 30);

        // Define the standard constraints to use for this robot
        myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.PI * .8, Math.PI, 15)
                .setDimensions(18, 15)
                .build();

        // ******************************************
        /* Specify which Position will be run */
        // ******************************************
        autoPosition = 2;

        // Build up the start to board delivery trajectory
        BlueBoardDecision();

        // Build up the Board to Floor Trajectory
        BlueRightPurplePixelDecision();

        // Build up the Floor to Stack Trajectory
        if (autoPosition == 1)
        {
            DriveToStack = myBot.getDrive().actionBuilder(deliverToFloorPose)
                    .splineToLinearHeading(new Pose2d(12,62, Math.toRadians(180)), Math.toRadians(180))
                    .strafeToLinearHeading(new Vector2d(-55.5,62), Math.toRadians(180))
                    .strafeToLinearHeading(new Vector2d(stackPose.position.x, stackPose.position.y), Math.toRadians(180))
                    .build();

        }
        else
        {
            DriveToStack = myBot.getDrive().actionBuilder(deliverToFloorPose)
                    .strafeToLinearHeading(new Vector2d(12,62), Math.toRadians(180))
                    .strafeToLinearHeading(new Vector2d(-55.5,62), Math.toRadians(180))
                    .strafeToLinearHeading(new Vector2d(stackPose.position.x, stackPose.position.y), Math.toRadians(180))
                    .build();
        }
        /////
       // Action DriveBackToStack2 = myBot.getDrive().actionBuilder(new Pose2d(48,11.5,Math.toRadians(180)))
        //        .strafeToLinearHeading(new Vector2d(stackPose.position.x, stackPose.position.y), Math.toRadians(180), speedUpVelocityConstraint)
         //       .build();

        // Build up the Stack to Board Position 1 Trajectory
        Action BoardTrajFinal = myBot.getDrive().actionBuilder(stackPose)
                //.setTangent(0)
                //.splineToLinearHeading(new Pose2d(-30, 11.5, Math.toRadians(180)), Math.toRadians(0))
                //.splineToLinearHeading(new Pose2d(47.5, 11.5, Math.toRadians(180)), Math.toRadians(0))
                //.setTangent(Math.toRadians(270))5
                //.splineToLinearHeading(deliverToBoardPose, Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(-55.5, 62), Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(47,62), Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(47,40), Math.toRadians(180))
                .build();

        // Build up the Board position 3 to Parking Trajectory
        //Action Park = myBot.getDrive().actionBuilder(new Pose2d(46, deliverToBoardPose.position.y, Math.toRadians(180)))
        Action Park = myBot.getDrive().actionBuilder(new Pose2d(47, 40, Math.toRadians(180)))
                .splineToLinearHeading(new Pose2d(48, 60, Math.toRadians(270)), Math.toRadians(90))
                //.strafeToLinearHeading(new Vector2d(48, 10), Math.toRadians(90))
                //.turnTo(Math.toRadians(90))
                .build();

        myBot.runAction(new SequentialAction(
                // Drive to Board Position
                BoardTraj2,
                // simulate waiting for board delivery and april tag correction
                new SleepAction(0.6),
                FloorTraj,
                // simulate waiting for purple pixel delivery & resetArm
                new SleepAction(1.0),
                DriveToStack,
                // simulate waiting to get to the white pixel pickup location
                new SleepAction(0.7),
                // simulate waiting to pick up the pixels
                new SleepAction(1.05),
                // Drive to Board Position 3
                BoardTrajFinal,
                // simulate waiting for board delivery
                new SleepAction(0.6),
                // Drive to the parking spot
                Park
                // simulate waiting for slides down, and servo stop
                //, new SleepAction(0.8)
                ));

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }

        static public void BlueRightPurplePixelDecision() {
            //***POSITION 1***
            if (autoPosition == 1) {
                deliverToFloorPose = new Pose2d(12, 33, Math.toRadians(0));
                FloorTraj = myBot.getDrive().actionBuilder(deliverToBoardPose)
                        .splineToLinearHeading(new Pose2d(27,33, Math.toRadians(0)), Math.toRadians(180))
                        //.setTangent(Math.toRadians(180))
                        .lineToX(0)
                        .strafeToLinearHeading(new Vector2d(deliverToFloorPose.position.x, deliverToFloorPose.position.y), Math.toRadians(0))
                        //.splineToLinearHeading(new Pose2d(0,33, Math.toRadians(0)), Math.toRadians(0))
                        //.splineToLinearHeading(deliverToFloorPose, Math.toRadians(0))
                        .build();
            }
            //***POSITION 3***
            else if (autoPosition == 3) {
                deliverToFloorPose = new Pose2d(12, 36, Math.toRadians(180));
                FloorTraj = myBot.getDrive().actionBuilder(deliverToBoardPose)
                        .setTangent(Math.toRadians(180))
                        .splineToLinearHeading (deliverToFloorPose, Math.toRadians(180))
                        .build();
            }
            //***POSITION 2***
            else {
                deliverToFloorPose = new Pose2d(12, 36, Math.toRadians(270));
                FloorTraj = myBot.getDrive().actionBuilder(deliverToBoardPose)
                        .splineToLinearHeading(new Pose2d(12, 30, Math.toRadians(-90)), Math.toRadians(180))
                        .splineToLinearHeading(deliverToFloorPose, Math.toRadians(270))
                        .build();
            }
        }

        static public void BlueBoardDecision() {
            // Look for potential errors
            //***POSITION 1***
            if (autoPosition == 1) {
                deliverToBoardPose = new Pose2d(47,42,Math.toRadians(180));
            }
            //***POSITION 3***
            else if (autoPosition == 3) {
                deliverToBoardPose = new Pose2d(47,30,Math.toRadians(180));
            }
            //***POSITION 2***
            else {
                deliverToBoardPose = new Pose2d(47,36,Math.toRadians(180));
            }
            BoardTraj2 = myBot.getDrive().actionBuilder(startPose)
                    .splineToLinearHeading(deliverToBoardPose, Math.toRadians(0))
                    .build();
        }
    }

