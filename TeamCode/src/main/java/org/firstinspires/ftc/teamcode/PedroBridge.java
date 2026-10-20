
package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import org.firstinspires.ftc.robotcore.external.ExportToBlocks;
import com.pedropathing.follower.Follower;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.api.Paths.*;

import com.pedropathing.paths.Path;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

public class PedroBridge extends BlocksOpModeCompanion {
    private static Follower follower;
    private static PoseFactory poseFactory;
    private static HashMap<String, Pose> poseMap;
    private static List<Path> pathList;


    @ExportToBlocks(
            comment = "Initializes auton path follower. Place this in the initialization section before setting path and points.",
            tooltip = "Initializes auton path follower.",
            parameterLabels = {}
    )
    public void initPedroPathing()
    {
        Scheduler.reset();
        HardwareMap hwMap = hardwareMap;
        if (hwMap != null) {
            poseMap = new HashMap<String, Pose>();
            poseFactory = PoseFactory.degrees();
            follower = Constants.create(hwMap);
            follower.setPose(poseFactory.of(0,0,0));
            pathList = new ArrayList<>();
        }

    }


    @ExportToBlocks(
            comment = "Create a point from an (X,Y,rot) input. Used to create a path.",
            tooltip = "Must add two points prior to creating a path.",
            parameterLabels = {"X (mm)", "Y (mm) Heading (degrees)"}
    )
    public static void addPoint(String name, double x, double y, double heading) {
        final Pose targetPose = poseFactory.of(x, y, heading);
        poseMap.put(name, targetPose);
    }
    @ExportToBlocks(
            comment = "Create a path from point 1 to point 2. Provide the names of both points in order.",
            tooltip = "Use the name of the path to specify which path you want. Case-sensitive",
            parameterLabels = {"Start Pose (name)", "End Pose (name)"}
    )
    public static void addPath(String startPoseName, String endPoseName) {
        Pose startPose = poseMap.get(startPoseName);
        Pose endPose = poseMap.get(endPoseName);

        if (startPose == null || endPose == null) {
            telemetry.addData("Path Log:", String.format("Point %s does not exist.", startPose == null ? startPoseName : endPoseName));
            return;
        }

        Path targetPath = line(startPose, endPose).linear(startPose, endPose);
        pathList.add(targetPath);

    }
    @ExportToBlocks(
            comment = "Clear the path list and start a path plan from scratch.",
            tooltip = "Previous paths will be dropped from the list if they exist",
            parameterLabels = {}
    )
    public static void clearPathPlan() {
        pathList.clear();
    }
    @ExportToBlocks(
            comment = "Schedule the paths you created within the scheduler. ",
            tooltip = "Run this block after you set your set of paths. Only run once per path plan.",
            parameterLabels = {}
    )
    public static void schedulePathPlan()
    {
        List<Command> pathCommands = new ArrayList<>();
        for (Path path : pathList) {
            pathCommands.add(follow(follower, path));
        }
        Command autonRoutine = sequential(
                pathCommands.toArray(new Command[0])
        );

        schedule(autonRoutine);
    }

    @ExportToBlocks(
            comment = "Test specific paths by passing in an index",
            tooltip = "Index starts at 0 and will run that path in the sequence",
            parameterLabels = {"Path # (index)"}
    )
    public static void testPath(int index) {
        Command pathCommand = follow(follower, pathList.get(index));
        schedule(pathCommand);
    }

    @ExportToBlocks(
            comment = "Update Pedro path planner calculations.",
            tooltip = "Place in loop block",
            parameterLabels = {}
    )
    public static void updatePedroPathing()
    {
        if (follower != null) {
            follower.update();
            Scheduler.execute();

            telemetry.addData("X", follower.pose().x());
            telemetry.addData("Y", follower.pose().y());
            telemetry.addData("Heading", follower.pose().heading());
            telemetry.addData("Follower Mode", follower.mode());
            telemetry.addData("Path Completion", follower.completion());
        }
        telemetry.update();
    }

    @ExportToBlocks(
            comment = "Checks to see if PathPlanner is busy",
            tooltip = "Returns true while the robot is driving on a path",
            parameterLabels = {}
    )
    public static boolean isBusy() {
        if (follower == null) return false;
        return follower.isBusy();
    }
}
