package org.firstinspires.ftc.teamcode.robotComponents.Tasks;

import android.util.Pair;

import org.firstinspires.ftc.teamcode.robotComponents.Robot;

import java.util.HashMap;
import java.util.Map;
import java.util.SortedMap;
import java.util.TreeMap;

public class TaskMap {
    private int stateWithinRobotState = 0;
    private Robot.TaskState currentRobotState = Robot.TaskState.IDLE;

    private HashMap<Robot.TaskState, SortedMap<Double, Task>> tasks = new HashMap<>();

    public TaskMap() {
    }

    public void addTasks(Robot.TaskState taskState,  HashMap<Double, Task> tasksToAdd) {
        SortedMap<Double, Task> sortedTasks = new TreeMap<>();
        for(Map.Entry entry : tasksToAdd.entrySet()){
            sortedTasks.put((Double)entry.getKey(), (Task)entry.getValue());
        }
        tasks.put(taskState, sortedTasks);
    }

    public void setCurrentRobotState(Robot.TaskState t) {
        currentRobotState = t;
        stateWithinRobotState = 0;
    }


    public SortedMap<Double, Task> getCurrentTasks() {
        return tasks.get(currentRobotState);
    }

    public Pair<Double, Task> getExpectedTask() {
        Task[] array = (Task[])getCurrentTasks().entrySet().toArray();
        Double[] keyArray = (Double[])getCurrentTasks().keySet().toArray();
        Task task = null;
        double time = -1;
        if(stateWithinRobotState < array.length) {
            task = array[stateWithinRobotState];
            time = keyArray[stateWithinRobotState];
        }
        return new Pair<>(time, task);
    }

    public void incrementTask() {
        stateWithinRobotState++;
    }
}
