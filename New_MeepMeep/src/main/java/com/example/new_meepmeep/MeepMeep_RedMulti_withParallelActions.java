package com.example.new_meepmeep;


import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeep_RedMulti_withParallelActions {
    static Pose2d startPose;
    static Pose2d stackPose;
    static Pose2d deliverToFloorPose;
    static Pose2d deliverToBoardPose;
    static Pose2d deliverToBoardPosition1Pose;
    static Action FloorTraj;
    static Action DriveToStack;
    static Action BoardTraj2;
    static Action BoardTraj3;
    static Action BackToStackTraj;
    static Action ParkingTraj;
    static RoadRunnerBotEntity myBot;
    static int autoPosition = 1;
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(500);

        startPose = new Pose2d(-35.5,-62.5,Math.toRadians(90));
        stackPose = new Pose2d(-56, -12, Math.toRadians(180));

        myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(18, 15)
                .build();

        // ******************************************
        /* Specify which Position will be run */
        // ******************************************
        autoPosition = 1;

        // Build up the floor delivery trajectory
        RedLeftPurplePixelDecision();

        // Create the floor to Stack trajectory
        DriveToStack = myBot.getDrive().actionBuilder(deliverToFloorPose)
                .splineToLinearHeading(stackPose, Math.toRadians(180))
                .build();

        // Build up the Stack to Board Trajectory
        RedBoardDecision();

        // Build up the Board back to Stack Trajectory
        BackToStackTraj = myBot.getDrive().actionBuilder(deliverToBoardPose)
                .strafeToLinearHeading(new Vector2d(49, -11.5), Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(stackPose.position.x, stackPose.position.y), Math.toRadians(180))
                .build();

        // Build up the Stack to Board Position 1 Trajectory
        deliverToBoardPosition1Pose = new Pose2d(50,-28,Math.toRadians(180));
        BoardTraj3 = myBot.getDrive().actionBuilder(stackPose)
                .strafeToLinearHeading(new Vector2d(49, -11.5), Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(deliverToBoardPosition1Pose.position.x, deliverToBoardPosition1Pose.position.y), Math.toRadians(180))
                .build();

        // Build up the Board position 1 to Parking Trajectory
        ParkingTraj = myBot.getDrive().actionBuilder(deliverToBoardPosition1Pose)
                .splineToLinearHeading(new Pose2d(48,-15, Math.toRadians(90)), Math.toRadians(90))
                .build();

        myBot.runAction(new SequentialAction(
                // Drive to Floor Position
                FloorTraj,
                // simulate waiting for purple pixel delivery
                new SleepAction(0.5),
                DriveToStack,
                // simulate waiting for white pixel pickup
                new SleepAction(0.5),
                BoardTraj2,
                // simulate waiting for board delivery
                new SleepAction(0.5),
                // Drive back to the stack
                BackToStackTraj,
                // simulate waiting for white pixel pickup
                new SleepAction(0.5),
                // Drive to Board Position 1
                BoardTraj3,
                // simulate waiting for board delivery
                new SleepAction(0.5),
                // Drive to the parking spot
                ParkingTraj
                ));

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }

    static public void RedLeftPurplePixelDecision() {
        //***POSITION 1***
        if (autoPosition == 1) {
            deliverToFloorPose = new Pose2d(-39.5, -20.5, Math.toRadians(45));
            FloorTraj = myBot.getDrive().actionBuilder(startPose)
                    .splineToLinearHeading(new Pose2d(-38.5, -33, Math.toRadians(90)), Math.toRadians(90))
                    .splineToLinearHeading (deliverToFloorPose, Math.toRadians(45))
                    .build();
        }
        //***POSITION 3***
        else if (autoPosition == 3) {
            deliverToFloorPose = new Pose2d(-34.5, -32, Math.toRadians(180));
            FloorTraj = myBot.getDrive().actionBuilder(startPose)
                    .splineToLinearHeading(new Pose2d(-38.5, -33, Math.toRadians(90)), Math.toRadians(90))
                    .splineToLinearHeading(new Pose2d(-27, -33, Math.toRadians(180)), Math.toRadians(180))
                    .splineToLinearHeading(deliverToFloorPose, Math.toRadians(180))
                    .build();
        }
        //***POSITION 2***
        else {
            deliverToFloorPose = new Pose2d(-38.5, -12.5, Math.toRadians(90));
            FloorTraj = myBot.getDrive().actionBuilder(startPose)
                    .splineToLinearHeading(new Pose2d(-38.5, -33, Math.toRadians(90)), Math.toRadians(90))
                    .splineToLinearHeading(deliverToFloorPose, Math.toRadians(90))
                    .build();
        }
    }

    static public void RedBoardDecision() {
        // Look for potential errors
        //***POSITION 1***
        if (autoPosition == 1) {
            deliverToBoardPose = new Pose2d(50,-28,Math.toRadians(180));
            BoardTraj2 = myBot.getDrive().actionBuilder(stackPose)
                    .setTangent(0)
                    .splineToLinearHeading(new Pose2d(-30, -11.5, Math.toRadians(180)), Math.toRadians(0))
                    .splineToLinearHeading(new Pose2d(49, -11.5, Math.toRadians(180)), Math.toRadians(0))
                    .setTangent(Math.toRadians(270))
                    .splineToLinearHeading(deliverToBoardPose, Math.toRadians(270))
                    .build();
            //drive.actionBuilder(drive.  new Pose2d(50+control.rangeError, 36+control.yawError, Math.toRadians(180)))

        }
        //***POSITION 3***
        else if (autoPosition == 3) {
            deliverToBoardPose = new Pose2d(50,-38,Math.toRadians(180));
            BoardTraj2 = myBot.getDrive().actionBuilder(stackPose)
                    .setTangent(0)
                    .splineToLinearHeading(new Pose2d(-30, -11.5, Math.toRadians(180)), Math.toRadians(0))
                    .splineToLinearHeading(new Pose2d(49, -11.5, Math.toRadians(180)), Math.toRadians(0))
                    .setTangent(Math.toRadians(270))
                    .splineToLinearHeading(deliverToBoardPose, Math.toRadians(270))
                    .build();
            //drive.actionBuilder(drive.  new Pose2d(50+control.rangeError, 36+control.yawError, Math.toRadians(180)))
        }
        //***POSITION 2***
        else {
            deliverToBoardPose = new Pose2d(50,-33,Math.toRadians(180));
            BoardTraj2 = myBot.getDrive().actionBuilder(stackPose)
                    .setTangent(0)
                    .splineToLinearHeading(new Pose2d(-30, -11.5, Math.toRadians(180)), Math.toRadians(0))
                    .splineToLinearHeading(new Pose2d(49, -11.5, Math.toRadians(180)), Math.toRadians(0))
                    .setTangent(Math.toRadians(270))
                    .splineToLinearHeading(deliverToBoardPose, Math.toRadians(270))
                    .build();
            //drive.actionBuilder(drive.  new Pose2d(50+control.rangeError, 36+control.yawError, Math.toRadians(180)))
        }
    }


}