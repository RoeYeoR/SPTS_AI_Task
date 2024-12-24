public class Node {
    String[][] state;
    int g;  // Cost from start state
    int h;  // Heuristic estimate to goal state
    int f;  // Total cost (f = g + h)
    Node parent;  // Parent node
    String action;  // Action taken to reach this state
    Move parentMove;  // Move that led to this state

    // Constructor with state, parent, g, h, and action
    public Node(String[][] state, Node parent, int g, int h, String action) {
        this.state = state;
        this.g = g;
        this.h = h;
        this.f = g + h;  // f = g + h for A* search
        this.parent = parent;
        this.action = action;  // Action that led to this state
        
        // Find the move that led to this state
        if (parent != null) {
            this.parentMove = findParentMove(parent.state, state);
        }
    }

    // Helper method to find the move between parent and current state
    private Move findParentMove(String[][] parentState, String[][] currentState) {
        for (int i = 0; i < parentState.length; i++) {
            for (int j = 0; j < parentState[i].length; j++) {
                // Find the color that moved
                if (!parentState[i][j].equals(currentState[i][j]) && 
                    !parentState[i][j].equals("_") && 
                    !parentState[i][j].equals("X")) {
                    return new Move(i, j);
                }
            }
        }
        return null;
    }

    // Inner class to represent a move
    private static class Move {
        int i;
        int j;

        public Move(int i, int j) {
            this.i = i;
            this.j = j;
        }
    }
}
