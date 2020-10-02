package org.firstinspires.ftc.teamcode.RobotComponents.Tasks;

import android.util.Pair;

import org.firstinspires.ftc.teamcode.RobotComponents.DriveTrain;
import org.firstinspires.ftc.teamcode.RobotComponents.Robot;

public class TaskMachine extends Thread {
    private TaskMap[] taskMaps;
    private Robot robot;
    private DriveTrain driveTrain;
    private Robot.TaskState currState;

    public TaskMachine(Robot r, DriveTrain d, TaskMap[] taskMaps) {
        this.taskMaps = taskMaps;
        robot = r;
        driveTrain = d;
        currState = r.getTaskState();
    }

    @Override
    public void run() {
        while(robot.getMyOpMode().opModeIsActive()) {
            if (robot.getTaskState() != currState) {
                currState = robot.getTaskState();
                setRobotStates(currState);
            }

            double currProgress = driveTrain.getProgress();

            for (TaskMap t : taskMaps) {
                Pair<Double, Task> expectedTask = t.getExpectedTask();
                if (currProgress > expectedTask.first) {
                    expectedTask.second.start();
                    t.incrementTask();
                }
            }
        }
    }

    private void setRobotStates(Robot.TaskState taskState) {
        for(TaskMap t : taskMaps) {
            t.setCurrentRobotState(taskState);
        }
    }
}
