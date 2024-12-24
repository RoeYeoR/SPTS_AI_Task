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
                                // Calculate minimum moves needed
                                int moveCount = 0;
                                if (i != gi) moveCount++;
                                if (j != gj) moveCount++;
                                
                                // Add cost based on marble type
                                switch (marble) {
                                    case "R": h += moveCount * 10; break;
                                    case "G": h += moveCount * 3; break;
                                    case "B": h += moveCount * 1; break;
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
        
        return h;
    }
}
