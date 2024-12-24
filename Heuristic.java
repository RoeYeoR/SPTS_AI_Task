import java.util.*;

public class Heuristic {

    public static int heuristic(String[][] state, String[][] goal) {
        int h = 0;
        Map<String, Integer> costs = Map.of("B", 1, "G", 3, "R", 10);

        // Track marble locations
        Map<String, int[]> stateLocations = new HashMap<>();
        Map<String, int[]> goalLocations = new HashMap<>();

        // Find locations of marbles in current state and goal state
        for (int i = 0; i < state.length; i++) {
            for (int j = 0; j < state[i].length; j++) {
                String marble = state[i][j];
                if (!marble.equals("_") && !marble.equals("X")) {
                    stateLocations.put(marble + i + j, new int[]{i, j});
                }
            }
        }

        for (int i = 0; i < goal.length; i++) {
            for (int j = 0; j < goal[i].length; j++) {
                String marble = goal[i][j];
                if (!marble.equals("_") && !marble.equals("X")) {
                    goalLocations.put(marble + i + j, new int[]{i, j});
                }
            }
        }

        // Calculate heuristic based on marble misplacements
        h = calculateHeuristic(state, goal);

        return h;
    }

    public static int calculateHeuristic(String[][] currentState, String[][] goalState) {
        int heuristic = 0;
        int rows = currentState.length;
        int cols = currentState[0].length;

        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                String marble = currentState[i][j];
                if (!marble.equals("_") && !marble.equals("X")) {
                    int[] goalPos = findGoalPosition(marble, goalState);
                    
                    // Calculate circular distance
                    int verticalDist = Math.min(
                        Math.abs(i - goalPos[0]), 
                        rows - Math.abs(i - goalPos[0])
                    );
                    int horizontalDist = Math.min(
                        Math.abs(j - goalPos[1]), 
                        cols - Math.abs(j - goalPos[1])
                    );
                    
                    heuristic += verticalDist + horizontalDist;
                }
            }
        }
        return heuristic;
    }

    private static int[] findGoalPosition(String marble, String[][] goalState) {
        for (int i = 0; i < goalState.length; i++) {
            for (int j = 0; j < goalState[i].length; j++) {
                if (goalState[i][j].equals(marble)) {
                    return new int[]{i, j};
                }
            }
        }
        return new int[]{-1, -1};
    }
}
