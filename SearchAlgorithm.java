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
                result = idaStarSearch(startState, goalState, openListFlag);
                break;
            case "DFBnB":
                result = dfbnbSearch(startState, goalState, openListFlag);
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
        // Modified priority queue to consider generation time
        PriorityQueue<Node> pQueue = new PriorityQueue<>((a, b) -> {
            if (a.f != b.f) {
                return a.f - b.f;
            }
            // If f-values are equal, prefer node generated earlier
            return Long.compare(a.generationTime, b.generationTime);
        });
        long startTime = System.nanoTime();

        Node startNode = new Node(startState, null, 0, Heuristic.heuristic(startState, goalState));
        pQueue.add(startNode);
        String startStateStr = Arrays.deepToString(startState);
        openList.put(startStateStr, startNode);
        
        while (!pQueue.isEmpty() && nodesGenerated < 10) {
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
                updateOutputFile(path, nodesGenerated + 1, currentNode.pathCost, timeTaken);
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
                                if (nodesGenerated >= 10) {
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
                if (nodesGenerated >= 10) break;
            }
        }

        double timeTaken = (System.nanoTime() - startTime) / 1e9;
        updateNoPathOutput(nodesGenerated + 1, timeTaken);
        return null;
    }

    // BFS Search Algorithm
    private static List<String> bfsSearch(String[][] startState, String[][] goalState, boolean openListFlag) {
        Map<String, Node> openList = new HashMap<>();  // Hash table for open list
        Set<String> closedList = new HashSet<>();     // Hash table for closed list
        Queue<Node> queue = new LinkedList<>();        // For BFS order
        long startTime = System.nanoTime();
        nodesGenerated = 0;  // Start at 0 since we don't count the starting vertex

        Node startNode = new Node(startState, null, 0, 0);
        queue.add(startNode);
        String startStateStr = Arrays.deepToString(startState);
        openList.put(startStateStr, startNode);
        
        while (!queue.isEmpty() && nodesGenerated < 10) {
            Node currentNode = queue.poll();
            String currentStateStr = Arrays.deepToString(currentNode.state);
            openList.remove(currentStateStr);
            closedList.add(currentStateStr);
            
            if (openListFlag) {
                System.out.println("Current state:");
                for (String[] row : currentNode.state) {
                    System.out.println(Arrays.toString(row));
                }
                System.out.println();
            }

            if (isGoalState(currentNode.state, goalState)) {
                double timeTaken = (System.nanoTime() - startTime) / 1e9;
                List<String> path = reconstructPath(currentNode);
                int actualCost = calculatePathCost(path);  // Calculate actual cost of path
                updateOutputFile(path, nodesGenerated + 1, actualCost, timeTaken);
                return path;
            }

            // Get neighbors
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
                if (emptyRow != -1) break;
            }
            
            // Try each possible move
            for (int i = 0; i < rows; i++) {
                for (int j = 0; j < cols; j++) {
                    if (!currentNode.state[i][j].equals("_") && !currentNode.state[i][j].equals("X")) {
                        boolean canMove = false;
                        
                        // Check if in same row
                        if (i == emptyRow) {
                            int dist = Math.abs(j - emptyCol);
                            if (dist == 1 || dist == cols - 1) {
                                canMove = true;
                            }
                        }
                        // Check if in same column
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
                            
                            // Make the move
                            String marble = currentNode.state[i][j];
                            newState[emptyRow][emptyCol] = marble;
                            newState[i][j] = "_";

                            String newStateStr = Arrays.deepToString(newState);
                            if (!closedList.contains(newStateStr) && !openList.containsKey(newStateStr)) {
                                if (nodesGenerated >= 10) {
                                    break;
                                }
                                Node neighbor = new Node(newState, currentNode, currentNode.pathCost + 1, 0);
                                queue.add(neighbor);
                                openList.put(newStateStr, neighbor);
                                nodesGenerated++;
                            }
                        }
                    }
                }
                if (nodesGenerated >= 10) break;
            }

            if (openListFlag) {
                System.out.println("Open list size: " + queue.size());
            }
        }

        double timeTaken = (System.nanoTime() - startTime) / 1e9;
        updateNoPathOutput(nodesGenerated + 1, timeTaken);
        return null;
    }

    // DFID Search Algorithm (recursive with loop avoidance)
    private static List<String> dfidSearch(String[][] startState, String[][] goalState, boolean openListFlag) {
        long startTime = System.nanoTime();
        nodesGenerated = 0;  // Start at 0 since we don't count the starting vertex
        
        Node startNode = new Node(startState, null, 0, 0);
        Stack<Node> stack = new Stack<>();
        stack.push(startNode);

        // Iterative deepening
        for (int depth = 0; depth < Integer.MAX_VALUE; depth++) {
            if (openListFlag) {
                System.out.println("Current depth limit: " + depth);
            }

            boolean result = limitedDFS(startNode, goalState, depth, stack, openListFlag);
            
            if (result) {
                double timeTaken = (System.nanoTime() - startTime) / 1e9;
                List<String> path = reconstructPath(stack.peek());
                int actualCost = calculatePathCost(path);  // Calculate actual cost of path
                updateOutputFile(path, nodesGenerated + 1, actualCost, timeTaken);
                return path;
            }
            
            // Clear stack for next iteration
            stack.clear();
            stack.push(startNode);
        }

        double timeTaken = (System.nanoTime() - startTime) / 1e9;
        updateNoPathOutput(nodesGenerated + 1, timeTaken);
        return null;
    }

    private static boolean limitedDFS(Node currentNode, String[][] goalState, int depthLimit, Stack<Node> stack, boolean openListFlag) {
        if (openListFlag) {
            System.out.println("Exploring at depth " + currentNode.pathCost + ": " + Arrays.deepToString(currentNode.state));
        }

        if (currentNode.pathCost > depthLimit) {
            return false;
        }

        if (isGoalState(currentNode.state, goalState)) {
            return true;
        }

        List<Node> neighbors = getNeighbors(currentNode, goalState);
        
        for (Node neighbor : neighbors) {
            // Check for loops in current path
            boolean isLoop = false;
            for (Node stackNode : stack) {
                if (Arrays.deepEquals(stackNode.state, neighbor.state)) {
                    isLoop = true;
                    break;
                }
            }
            
            if (!isLoop) {
                stack.push(neighbor);
                nodesGenerated++;
                
                if (limitedDFS(neighbor, goalState, depthLimit, stack, openListFlag)) {
                    return true;
                }
                
                stack.pop();
            }
        }
        
        return false;
    }

    // IDA* Search Algorithm
    private static List<String> idaStarSearch(String[][] startState, String[][] goalState, boolean openListFlag) {
        long startTime = System.nanoTime();
        nodesGenerated = 0;  // Start at 0 since we don't count the starting vertex
        
        Node startNode = new Node(startState, null, 0, Heuristic.heuristic(startState, goalState));
        Stack<Node> stack = new Stack<>();
        stack.push(startNode);
        int threshold = startNode.h;

        while (threshold < Integer.MAX_VALUE) {
            int minF = Integer.MAX_VALUE;
            stack.clear();
            stack.push(startNode);
            
            while (!stack.isEmpty()) {
                Node currentNode = stack.peek();
                
                if (openListFlag) {
                    System.out.println("Current threshold: " + threshold);
                    System.out.println("Exploring state: " + Arrays.deepToString(currentNode.state));
                }

                if (isGoalState(currentNode.state, goalState)) {
                    double timeTaken = (System.nanoTime() - startTime) / 1e9;
                    List<String> path = reconstructPath(currentNode);
                    updateOutputFile(path, nodesGenerated + 1, currentNode.pathCost, timeTaken);
                    return path;
                }

                boolean deadEnd = true;
                List<Node> neighbors = getNeighbors(currentNode, goalState);
                
                for (Node neighbor : neighbors) {
                    int f = neighbor.pathCost + neighbor.h;
                    if (f <= threshold) {
                        // Check if this state is already in the stack (loop detection)
                        boolean isLoop = false;
                        for (Node stackNode : stack) {
                            if (Arrays.deepEquals(stackNode.state, neighbor.state)) {
                                isLoop = true;
                                break;
                            }
                        }
                        
                        if (!isLoop) {
                            stack.push(neighbor);
                            nodesGenerated++;
                            deadEnd = false;
                            break;
                        }
                    } else {
                        minF = Math.min(minF, f);
                    }
                }
                
                if (deadEnd) {
                    stack.pop();
                }
            }
            
            if (minF == Integer.MAX_VALUE) {
                double timeTaken = (System.nanoTime() - startTime) / 1e9;
                updateNoPathOutput(nodesGenerated + 1, timeTaken);
                return null;
            }
            
            threshold = minF;
        }
        
        double timeTaken = (System.nanoTime() - startTime) / 1e9;
        updateNoPathOutput(nodesGenerated + 1, timeTaken);
        return null;
    }

    private static List<String> dfbnbSearch(String[][] startState, String[][] goalState, boolean openListFlag) {
        long startTime = System.nanoTime();
        nodesGenerated = 0;  // Start at 0 since we don't count the starting vertex
        
        Node startNode = new Node(startState, null, 0, Heuristic.heuristic(startState, goalState));
        Stack<Node> stack = new Stack<>();
        HashSet<String> inStack = new HashSet<>();  // For faster loop detection
        stack.push(startNode);
        inStack.add(Arrays.deepToString(startNode.state));
        
        int upperBound = Integer.MAX_VALUE;
        List<String> bestPath = null;
        
        while (!stack.isEmpty() && nodesGenerated < 10) {  // Add node limit check
            Node currentNode = stack.peek();
            
            if (openListFlag) {
                System.out.println("Current upper bound: " + upperBound);
                System.out.println("Current f-value: " + currentNode.f);
                System.out.println("Current g-value: " + currentNode.pathCost);
                System.out.println("Current h-value: " + currentNode.h);
                System.out.println("Nodes generated: " + nodesGenerated);
            }
            
            // If current node's f-value exceeds upper bound, backtrack
            if (currentNode.f >= upperBound) {
                stack.pop();
                inStack.remove(Arrays.deepToString(currentNode.state));
                continue;
            }
            
            if (isGoalState(currentNode.state, goalState)) {
                // Found a better solution
                upperBound = currentNode.f;
                bestPath = reconstructPath(currentNode);
                stack.pop();
                inStack.remove(Arrays.deepToString(currentNode.state));
                continue;
            }
            
            List<Node> neighbors = getNeighbors(currentNode, goalState);
            
            // Sort neighbors by f-value and generation time
            neighbors.sort((a, b) -> {
                if (a.f != b.f) {
                    return a.f - b.f;
                }
                return Long.compare(a.generationTime, b.generationTime);
            });
            
            // Remove neighbors that would exceed the node limit
            if (nodesGenerated + neighbors.size() > 10) {
                neighbors = neighbors.subList(0, Math.max(0, 10 - nodesGenerated));
            }
            
            // Filter out neighbors that exceed upper bound or create loops
            boolean foundValidNeighbor = false;
            for (Node neighbor : neighbors) {
                String neighborState = Arrays.deepToString(neighbor.state);
                
                // Skip if f-value exceeds bound or state is in stack (loop)
                if (neighbor.f >= upperBound || inStack.contains(neighborState)) {
                    continue;
                }
                
                // Found a valid neighbor
                stack.push(neighbor);
                inStack.add(neighborState);
                nodesGenerated++;
                foundValidNeighbor = true;
                break;
            }
            
            if (!foundValidNeighbor) {
                // No valid moves, backtrack
                stack.pop();
                inStack.remove(Arrays.deepToString(currentNode.state));
            }
        }
        
        double timeTaken = (System.nanoTime() - startTime) / 1e9;
        if (bestPath != null) {
            updateOutputFile(bestPath, nodesGenerated + 1, upperBound, timeTaken);
            return bestPath;
        } else {
            updateNoPathOutput(nodesGenerated + 1, timeTaken);
            return null;
        }
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
            if (emptyRow != -1) break;
        }
        
        // Try moving each marble that can move to the empty space
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                if (!currentNode.state[i][j].equals("_") && !currentNode.state[i][j].equals("X")) {
                    boolean canMove = false;
                    
                    // Check if in same row
                    if (i == emptyRow) {
                        int dist = Math.abs(j - emptyCol);
                        if (dist == 1 || dist == cols - 1) {
                            canMove = true;
                        }
                    }
                    // Check if in same column
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
                        
                        // Make the move
                        String marble = currentNode.state[i][j];
                        newState[emptyRow][emptyCol] = marble;
                        newState[i][j] = "_";
                        
                        // Calculate cost based on marble type
                        int moveCost;
                        switch (marble) {
                            case "R": moveCost = 10; break;
                            case "G": moveCost = 3; break;
                            case "B": moveCost = 1; break;
                            default: moveCost = 0;
                        }
                        
                        Node neighbor = new Node(newState, currentNode, currentNode.pathCost + moveCost, Heuristic.heuristic(newState, goalState));
                        neighbors.add(neighbor);
                    }
                }
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

    // Helper method to calculate the actual cost of a path
    private static int calculatePathCost(List<String> path) {
        int cost = 0;
        for (String move : path) {
            String marble = move.split(":")[1];  // Get the marble type (R, G, or B)
            switch (marble) {
                case "R":
                    cost += 10;
                    break;
                case "G":
                    cost += 3;
                    break;
                case "B":
                    cost += 1;
                    break;
            }
        }
        return cost;
    }

    // Node class to represent a state in the search
    private static class Node {
        String[][] state;
        Node parent;
        int pathCost;
        int h;
        int f;
        long generationTime;  // Added generation time field

        Node(String[][] state, Node parent, int pathCost, int h) {
            this.state = state;
            this.parent = parent;
            this.pathCost = pathCost;
            this.h = h;
            this.f = pathCost + h;
            this.generationTime = nodesGenerated;  // Use nodesGenerated as generation time
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
