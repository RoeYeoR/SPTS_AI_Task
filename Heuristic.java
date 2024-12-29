import java.util.*;

public class Heuristic {

    public static int heuristic(String[][] currentState, String[][] goalState) {
        int h = 0;
        int rows = currentState.length;
        int cols = currentState[0].length;
        
        // For each marble in current state
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                String marble = currentState[i][j];
                if (!marble.equals("_") && !marble.equals("X")) {
                    // Find this marble in goal state
                    boolean found = false;
                    for (int gi = 0; gi < rows; gi++) {
                        for (int gj = 0; gj < cols; gj++) {
                            if (goalState[gi][gj].equals(marble)) {
                                // Calculate Manhattan distance
                                int dist = Math.abs(i - gi) + Math.abs(j - gj);
                                
                                // Consider circular moves
                                // For row distance
                                int circularRowDist = Math.min(i, gi) + (rows - Math.max(i, gi));
                                dist = Math.min(dist, circularRowDist + Math.abs(j - gj));
                                
                                // For column distance
                                int circularColDist = Math.min(j, gj) + (cols - Math.max(j, gj));
                                dist = Math.min(dist, Math.abs(i - gi) + circularColDist);
                                
                                // Add cost based on marble type and distance
                                switch (marble) {
                                    case "R": h += dist * 10; break;
                                    case "G": h += dist * 3; break;
                                    case "B": h += dist * 1; break;
                                }
                                found = true;
                                break;
                            }
                        }
                        if (found) break;
                    }
                }
            }
        }
        
        // Add penalty for marbles blocking paths
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                if (currentState[i][j].equals("_")) {
                    // Check if there are marbles that need to move through this space
                    for (int di = -1; di <= 1; di++) {
                        for (int dj = -1; dj <= 1; dj++) {
                            if (di == 0 && dj == 0) continue;
                            int ni = i + di;
                            int nj = j + dj;
                            if (ni >= 0 && ni < rows && nj >= 0 && nj < cols) {
                                String marble = currentState[ni][nj];
                                if (!marble.equals("_") && !marble.equals("X")) {
                                    // Check if this marble needs to move through the empty space
                                    for (int gi = 0; gi < rows; gi++) {
                                        for (int gj = 0; gj < cols; gj++) {
                                            if (goalState[gi][gj].equals(marble)) {
                                                if ((gi - ni) * di > 0 || (gj - nj) * dj > 0) {
                                                    // Marble needs to move through the empty space
                                                    switch (marble) {
                                                        case "R": h += 5; break;
                                                        case "G": h += 2; break;
                                                        case "B": h += 1; break;
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        
        return h;
    }
}
