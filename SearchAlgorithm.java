import java.util.*;
import java.io.*;

public class SearchAlgorithm {

    public static int nodesGenerated = 0; // Count of nodes generated during the search

    // Common method for performing the search (A*, BFS, DFID, IDA, DFBnB)
    public static List<String> performSearch(String[][] startState, String[][] goalState, String algorithm, boolean openListFlag) {
        nodesGenerated = 0; // Reset node generation count for each search
        List<String> result = null;

        // Choose the search algorithm based on the input
        switch (algorithm) {
            case "A*":
                result = astarSearch(startState, goalState, openListFlag);
                break;
            case "BFS":
                result = bfsSearch(startState, goalState, openListFlag);
                break;
            case "DFID":
                result = dfidSearch(startState, goalState, openListFlag);
                break;
            case "IDA":
                result = idaSearch(startState, goalState, openListFlag);
                break;
            default:
                System.out.println("Algorithm not recognized.");
                break;
        }
        return result;
    }

    // A* Search Algorithm
    private static List<String> astarSearch(String[][] startState, String[][] goalState, boolean openListFlag) {
        Map<String, Node> openList = new HashMap<>();  // Hash table for open list
        Set<String> closedList = new HashSet<>();     // Hash table for closed list
        PriorityQueue<Node> pQueue = new PriorityQueue<>((a, b) -> a.f - b.f);  // For A* ordering
        nodesGenerated = 1;
        long startTime = System.nanoTime();

        Node startNode = new Node(startState, null, 0, Heuristic.heuristic(startState, goalState));
        pQueue.add(startNode);
        String startStateStr = Arrays.deepToString(startState);
        openList.put(startStateStr, startNode);
        
        while (!pQueue.isEmpty() && nodesGenerated < 11) {
            Node currentNode = pQueue.poll();
            String currentStateStr = Arrays.deepToString(currentNode.state);
            openList.remove(currentStateStr);

            if (closedList.contains(currentStateStr)) {
                continue;
            }
            closedList.add(currentStateStr);

            if (openListFlag) {
                System.out.println("Exploring state: " + currentStateStr);
                System.out.println("f = " + currentNode.f + ", g = " + currentNode.pathCost + ", h = " + currentNode.h);
            }

            if (isGoalState(currentNode.state, goalState)) {
                double timeTaken = (System.nanoTime() - startTime) / 1e9;
                List<String> path = reconstructPath(currentNode);
                updateOutputFile(path, nodesGenerated, currentNode.pathCost, timeTaken);
                return path;
            }

            int rows = currentNode.state.length;
            int cols = currentNode.state[0].length;
            int emptyRow = -1, emptyCol = -1;

            for (int i = 0; i < rows; i++) {
                for (int j = 0; j < cols; j++) {
                    if (currentNode.state[i][j].equals("_")) {
                        emptyRow = i;
                        emptyCol = j;
                        break;
                    }
                }
                if (emptyRow != -1) break;
            }

            for (int i = 0; i < rows; i++) {
                for (int j = 0; j < cols; j++) {
                    if (!currentNode.state[i][j].equals("_") && !currentNode.state[i][j].equals("X")) {
                        boolean canMove = false;
                        
                        if (i == emptyRow) {
                            int dist = Math.abs(j - emptyCol);
                            if (dist == 1 || dist == cols - 1) {
                                canMove = true;
                            }
                        }
                        else if (j == emptyCol) {
                            int dist = Math.abs(i - emptyRow);
                            if (dist == 1 || dist == rows - 1) {
                                canMove = true;
                            }
                        }

                        if (canMove) {
                            String[][] newState = new String[rows][cols];
                            for (int x = 0; x < rows; x++) {
                                newState[x] = currentNode.state[x].clone();
                            }

                            String marble = currentNode.state[i][j];
                            newState[emptyRow][emptyCol] = marble;
                            newState[i][j] = "_";

                            String newStateStr = Arrays.deepToString(newState);
                            if (!closedList.contains(newStateStr)) {
                                if (nodesGenerated >= 11) {
                                    break;
                                }
                                
                                int h = Heuristic.heuristic(newState, goalState);
                                Node neighbor = new Node(newState, currentNode, currentNode.pathCost + 1, h);

                                Node existingNode = openList.get(newStateStr);
                                if (existingNode == null || neighbor.f < existingNode.f) {
                                    pQueue.add(neighbor);
                                    openList.put(newStateStr, neighbor);
                                    if (existingNode == null) {
                                        nodesGenerated++;
                                    }

                                    if (openListFlag) {
                                        System.out.println("Generated new state: " + newStateStr);
                                        System.out.println("f = " + neighbor.f + ", g = " + neighbor.pathCost + ", h = " + h);
                                    }
                                }
                            }
                        }
                    }
                }
                if (nodesGenerated >= 11) break;
            }
        }

        double timeTaken = (System.nanoTime() - startTime) / 1e9;
        updateNoPathOutput(nodesGenerated, timeTaken);
        return null;
    }

    // BFS Search Algorithm
    private static List<String> bfsSearch(String[][] startState, String[][] goalState, boolean openListFlag) {
        Map<String, Node> openList = new HashMap<>();  // Hash table for open list
        Set<String> closedList = new HashSet<>();     // Hash table for closed list
        Queue<Node> queue = new LinkedList<>();        // For BFS order
        nodesGenerated = 1;
        long startTime = System.nanoTime();

        Node startNode = new Node(startState, null, 0, 0);
        queue.add(startNode);
        String startStateStr = Arrays.deepToString(startState);
        openList.put(startStateStr, startNode);
        
        while (!queue.isEmpty() && nodesGenerated < 11) {
            Node currentNode = queue.poll();
            String currentStateStr = Arrays.deepToString(currentNode.state);
            openList.remove(currentStateStr);
            
            if (closedList.contains(currentStateStr)) {
                continue;
            }
            closedList.add(currentStateStr);

            if (openListFlag) {
                System.out.println("Generating Neighbors for State: " + currentStateStr);
                System.out.println("Current Node Path Cost: " + currentNode.pathCost);
            }

            if (isGoalState(currentNode.state, goalState)) {
                double timeTaken = (System.nanoTime() - startTime) / 1e9;
                List<String> path = reconstructPath(currentNode);
                updateOutputFile(path, nodesGenerated, currentNode.pathCost, timeTaken);
                return path;
            }

            int rows = currentNode.state.length;
            int cols = currentNode.state[0].length;
            int emptyRow = -1, emptyCol = -1;

            for (int i = 0; i < rows; i++) {
                for (int j = 0; j < cols; j++) {
                    if (currentNode.state[i][j].equals("_")) {
                        emptyRow = i;
                        emptyCol = j;
                        break;
                    }
                }
                if (emptyRow != -1) break;
            }

            for (int i = 0; i < rows; i++) {
                for (int j = 0; j < cols; j++) {
                    if (!currentNode.state[i][j].equals("_") && !currentNode.state[i][j].equals("X")) {
                        boolean canMove = false;
                        
                        if (i == emptyRow) {
                            int dist = Math.abs(j - emptyCol);
                            if (dist == 1 || dist == cols - 1) {
                                canMove = true;
                            }
                        }
                        else if (j == emptyCol) {
                            int dist = Math.abs(i - emptyRow);
                            if (dist == 1 || dist == rows - 1) {
                                canMove = true;
                            }
                        }

                        if (canMove) {
                            String[][] newState = new String[rows][cols];
                            for (int x = 0; x < rows; x++) {
                                newState[x] = currentNode.state[x].clone();
                            }

                            String marble = currentNode.state[i][j];
                            newState[emptyRow][emptyCol] = marble;
                            newState[i][j] = "_";

                            String newStateStr = Arrays.deepToString(newState);
                            if (!closedList.contains(newStateStr) && !openList.containsKey(newStateStr)) {
                                if (nodesGenerated >= 11) {
                                    break;
                                }
                                Node neighbor = new Node(newState, currentNode, currentNode.pathCost + 1, 0);
                                queue.add(neighbor);
                                openList.put(newStateStr, neighbor);
                                nodesGenerated++;

                                if (openListFlag) {
                                    System.out.println("Generated new state: " + newStateStr);
                                }
                            }
                        }
                    }
                }
                if (nodesGenerated >= 11) break;
            }

            if (openListFlag) {
                System.out.println("Total Nodes Generated: " + nodesGenerated);
            }
        }

        double timeTaken = (System.nanoTime() - startTime) / 1e9;
        updateNoPathOutput(nodesGenerated, timeTaken);
        return null;
    }

    // DFID Search Algorithm (recursive with loop avoidance)
    private static List<String> dfidSearch(String[][] startState, String[][] goalState, boolean openListFlag) {
        for (int depth = 1; depth < Integer.MAX_VALUE; depth++) {
            Set<String> pathStates = new HashSet<>();
            Node startNode = new Node(startState, null, 0, 0);
            List<String> result = limitedDFS(startNode, goalState, depth, pathStates);
            if (result != null) {
                return result;
            }
        }
        return null;
    }

    // Limited DFS helper for DFID (recursive implementation)
    private static List<String> limitedDFS(Node currentNode, String[][] goalState, int limit, Set<String> pathStates) {
        if (limit < 0) return null;
        
        if (isGoalState(currentNode.state, goalState)) {
            return reconstructPath(currentNode);
        }

        String currentStateStr = Arrays.deepToString(currentNode.state);
        if (pathStates.contains(currentStateStr)) {
            return null;
        }

        if (limit == 0) return null;

        pathStates.add(currentStateStr);
        List<Node> neighbors = getNeighbors(currentNode, goalState);
        for (Node neighbor : neighbors) {
            nodesGenerated++;
            List<String> result = limitedDFS(neighbor, goalState, limit - 1, pathStates);
            if (result != null) {
                return result;
            }
        }
        pathStates.remove(currentStateStr);

        return null;
    }

    // IDA* Search Algorithm
    private static List<String> idaSearch(String[][] startState, String[][] goalState, boolean openListFlag) {
        nodesGenerated = 1;
        long startTime = System.nanoTime();
        
        Node startNode = new Node(startState, null, 0, Heuristic.heuristic(startState, goalState));
        int threshold = startNode.h;

        while (threshold < Integer.MAX_VALUE) {
            Stack<Node> stack = new Stack<>();
            Set<String> stackStates = new HashSet<>();
            
            int minCost = search(startNode, 0, threshold, goalState, stack, stackStates);
            
            if (minCost == 0) {
                return reconstructPath(stack.peek());
            }
            
            if (minCost == Integer.MAX_VALUE) {
                return null; // No solution
            }
            
            threshold = minCost;
        }
        
        return null;
    }

    // Helper method for IDA* search
    private static int search(Node node, int g, int threshold, String[][] goalState, 
                               Stack<Node> stack, Set<String> stackStates) {
        int f = g + Heuristic.heuristic(node.state, goalState);
        
        if (f > threshold) {
            return f;
        }
        
        stack.push(node);
        stackStates.add(Arrays.deepToString(node.state));
        
        if (isGoalState(node.state, goalState)) {
            return 0;
        }
        
        int minCost = Integer.MAX_VALUE;
        List<Node> neighbors = getNeighbors(node, goalState);
        
        for (Node neighbor : neighbors) {
            if (!stackStates.contains(Arrays.deepToString(neighbor.state))) {
                nodesGenerated++;
                int t = search(neighbor, g + 1, threshold, goalState, stack, stackStates);
                
                if (t == 0) {
                    return 0;
                }
                
                if (t < minCost) {
                    minCost = t;
                }
            }
        }
        
        stack.pop();
        stackStates.remove(Arrays.deepToString(node.state));
        
        return minCost;
    }

    // Helper methods to get neighbors, calculate heuristics, and reconstruct the path

    private static List<Node> getNeighbors(Node currentNode, String[][] goalState) {
        List<Node> neighbors = new ArrayList<>();
        int rows = currentNode.state.length;
        int cols = currentNode.state[0].length;

        // Find empty space
        int emptyRow = -1, emptyCol = -1;
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                if (currentNode.state[i][j].equals("_")) {
                    emptyRow = i;
                    emptyCol = j;
                    break;
                }
            }
        }

        // Define moves (including circular moves)
        int[][] moves = {
            {0, 1},  // right
            {0, -1}, // left
            {1, 0},  // down
            {-1, 0}  // up
        };

        for (int[] move : moves) {
            int newRow = (emptyRow + move[0] + rows) % rows;
            int newCol = (emptyCol + move[1] + cols) % cols;

            if (!currentNode.state[newRow][newCol].equals("X")) {
                String[][] newState = new String[rows][cols];
                for (int i = 0; i < rows; i++) {
                    newState[i] = currentNode.state[i].clone();
                }

                // Swap empty space with the marble
                newState[emptyRow][emptyCol] = currentNode.state[newRow][newCol];
                newState[newRow][newCol] = "_";

                Node neighbor = new Node(newState, currentNode, currentNode.pathCost + 1, Heuristic.heuristic(newState, goalState));
                neighbors.add(neighbor);
            }
        }

        return neighbors;
    }

    private static boolean isGoalState(String[][] state, String[][] goalState) {
        int rows = state.length;
        int cols = state[0].length;
        
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                if (!state[i][j].equals(goalState[i][j])) {
                    return false;
                }
            }
        }
        return true;
    }

    private static List<String> reconstructPath(Node goalNode) {
        List<String> path = new ArrayList<>();
        Node current = goalNode;
        
        while (current.parent != null) {
            String[][] currentState = current.state;
            String[][] parentState = current.parent.state;
            int rows = currentState.length;
            int cols = currentState[0].length;
            
            // Find the marble that moved by comparing states
            for (int i = 0; i < rows; i++) {
                for (int j = 0; j < cols; j++) {
                    if (!currentState[i][j].equals(parentState[i][j])) {
                        // Found a difference
                        if (!currentState[i][j].equals("_")) {
                            // This is where the marble moved to
                            String marble = currentState[i][j];
                            
                            // Find where it was in the parent state
                            for (int pi = 0; pi < rows; pi++) {
                                for (int pj = 0; pj < cols; pj++) {
                                    if (parentState[pi][pj].equals(marble)) {
                                        // Format: (initial_row,initial_col):color:(final_row,final_col)
                                        String move = String.format("(%d,%d):%s:(%d,%d)", 
                                            pi + 1, pj + 1,  // Initial position
                                            marble,
                                            i + 1, j + 1    // Final position
                                        );
                                        path.add(0, move);
                                        break;
                                    }
                                }
                            }
                        }
                    }
                }
            }
            
            current = current.parent;
        }
        
        return path;
    }

    // Node class to represent a state in the search
    private static class Node {
        String[][] state;
        Node parent;
        int pathCost;
        int h;
        int f;

        public Node(String[][] state, Node parent, int pathCost, int h) {
            this.state = state;
            this.parent = parent;
            this.pathCost = pathCost;
            this.h = h;
            this.f = pathCost + h;
        }
    }

    // Helper method to update output file when path is found
    private static void updateOutputFile(List<String> path, int nodesGenerated, int cost, double executionTime) {
        try (PrintWriter writer = new PrintWriter(new FileWriter("output.txt"))) {
            writer.println();  // Empty line at start
            writer.println(String.join("--", path));
            writer.println("Num: " + Math.max(1, nodesGenerated));
            writer.println("Cost: " + cost);
            writer.println(String.format("%.3f seconds", executionTime));
            writer.println();  // Empty line at end
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    // Helper method to update output file when no path is found
    private static void updateNoPathOutput(int nodesGenerated, double executionTime) {
        try (PrintWriter writer = new PrintWriter(new FileWriter("output.txt"))) {
            writer.println();  // Empty line at start
            writer.println("no path");
            writer.println("Num: " + Math.max(1, nodesGenerated));
            writer.println("Cost: inf");
            writer.println(String.format("%.3f seconds", executionTime));
            writer.println();  // Empty line at end
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
