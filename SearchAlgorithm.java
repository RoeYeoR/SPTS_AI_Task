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
                result = aStarSearch(startState, goalState, openListFlag);
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
    private static List<String> aStarSearch(String[][] startState, String[][] goalState, boolean openListFlag) {
        List<Node> closedList = new ArrayList<>();
        Map<String, Node> openMap = new HashMap<>();
        PriorityQueue<Node> openList = new PriorityQueue<>(Comparator.comparingInt(n -> n.f));
        int nodesGenerated = 1; // Always start with 1 node
        long startTime = System.nanoTime();

        Node startNode = new Node(startState, null, 0, 
            calculateHeuristic(startState, goalState), null);
        String startStateStr = Arrays.deepToString(startState);
        
        // Generate initial neighbors
        List<Node> neighbors = getNeighbors(startNode, goalState);
        nodesGenerated += neighbors.size();
        for (Node neighbor : neighbors) {
            String neighborStateStr = Arrays.deepToString(neighbor.state);
            
            // Calculate f value
            neighbor.h = calculateHeuristic(neighbor.state, goalState);
            neighbor.f = neighbor.g + neighbor.h;
            
            openList.add(neighbor);
            openMap.put(neighborStateStr, neighbor);
        }

        while (!openList.isEmpty()) {
            Node currentNode = openList.poll();
            String currentStateStr = Arrays.deepToString(currentNode.state);
            openMap.remove(currentStateStr);

            // Skip already explored states
            if (closedList.stream().anyMatch(n -> Arrays.deepEquals(n.state, currentNode.state))) {
                continue;
            }
            closedList.add(currentNode);

            // Check goal state
            if (Arrays.deepEquals(currentNode.state, goalState)) {
                long endTime = System.nanoTime();
                double executionTime = (endTime - startTime) / 1e9;
                updateOutputFile(reconstructPath(currentNode), nodesGenerated, currentNode.g, executionTime);
                return reconstructPath(currentNode);
            }

            // Generate neighbors
            List<Node> currentNeighbors = getNeighbors(currentNode, goalState);
            nodesGenerated += currentNeighbors.size();
            for (Node neighbor : currentNeighbors) {
                String neighborStateStr = Arrays.deepToString(neighbor.state);
                
                // Calculate f value
                neighbor.h = calculateHeuristic(neighbor.state, goalState);
                neighbor.f = neighbor.g + neighbor.h;

                // Check if already in closed list
                if (closedList.stream().anyMatch(n -> Arrays.deepEquals(n.state, neighbor.state))) {
                    continue;
                }

                // Check if better path exists in open map
                Node existingNode = openMap.get(neighborStateStr);
                if (existingNode == null || neighbor.f < existingNode.f) {
                    openList.add(neighbor);
                    openMap.put(neighborStateStr, neighbor);
                }
            }
        }

        // No solution found
        long endTime = System.nanoTime();
        double executionTime = (endTime - startTime) / 1e9;
        updateNoPathOutput(nodesGenerated, executionTime);
        return null;
    }

    // BFS Search Algorithm
    private static List<String> bfsSearch(String[][] startState, String[][] goalState, boolean openListFlag) {
        List<Node> closedList = new ArrayList<>();
        Queue<Node> queue = new LinkedList<>();
        int nodesGenerated = 1; // Always start with 1 node
        long startTime = System.nanoTime();

        Node startNode = new Node(startState, null, 0, 0, null);
        queue.add(startNode);
        
        while (!queue.isEmpty()) {
            Node currentNode = queue.poll();
            
            // Skip already explored states
            if (closedList.stream().anyMatch(n -> Arrays.deepEquals(n.state, currentNode.state))) {
                continue;
            }
            closedList.add(currentNode);

            // Check goal state
            if (Arrays.deepEquals(currentNode.state, goalState)) {
                long endTime = System.nanoTime();
                double executionTime = (endTime - startTime) / 1e9;
                updateOutputFile(reconstructPath(currentNode), nodesGenerated, currentNode.g, executionTime);
                return reconstructPath(currentNode);
            }

            // Generate neighbors
            List<Node> currentNeighbors = getNeighbors(currentNode, goalState);
            nodesGenerated += currentNeighbors.size();
            for (Node neighbor : currentNeighbors) {
                // Skip if already explored
                if (closedList.stream().noneMatch(n -> Arrays.deepEquals(n.state, neighbor.state))) {
                    queue.add(neighbor);
                }
            }
        }

        // No solution found
        long endTime = System.nanoTime();
        double executionTime = (endTime - startTime) / 1e9;
        updateNoPathOutput(nodesGenerated, executionTime);
        return null;
    }

    // Helper method to update output file when path is found
    private static void updateOutputFile(List<String> path, int nodesGenerated, int cost, double executionTime) {
        try (PrintWriter writer = new PrintWriter(new FileWriter("output.txt"))) {
            // Write path actions
            writer.println(String.join("--", path));
            writer.println("Num: " + Math.max(1, nodesGenerated));
            writer.println("Cost: " + cost);
            writer.println(String.format("%.3f seconds", executionTime));
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    // Helper method to update output file when no path is found
    private static void updateNoPathOutput(int nodesGenerated, double executionTime) {
        try (PrintWriter writer = new PrintWriter(new FileWriter("output.txt"))) {
            writer.println("path no");
            writer.println("Num: " + Math.max(1, nodesGenerated));
            writer.println("inf :Cost");
            writer.println(String.format("%.3f seconds", executionTime));
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    // DFID Search Algorithm (recursive with loop avoidance)
    private static List<String> dfidSearch(String[][] startState, String[][] goalState, boolean openListFlag) {
        for (int depth = 1; depth < Integer.MAX_VALUE; depth++) {
            Set<String> pathStates = new HashSet<>();
            Node startNode = new Node(startState, null, 0, 0, null);
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
        
        if (Arrays.deepEquals(currentNode.state, goalState)) {
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
        Node startNode = new Node(startState, null, 0, calculateHeuristic(startState, goalState), null);
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
        int f = g + calculateHeuristic(node.state, goalState);
        
        if (f > threshold) {
            return f;
        }
        
        stack.push(node);
        stackStates.add(Arrays.deepToString(node.state));
        
        if (Arrays.deepEquals(node.state, goalState)) {
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

    private static int calculateHeuristic(String[][] state, String[][] goalState) {
        int heuristic = 0;
        int rows = state.length;
        int cols = state[0].length;
        
        for (int i = 0; i < state.length; i++) {
            for (int j = 0; j < state[i].length; j++) {
                if (!state[i][j].equals("_") && !state[i][j].equals("X")) {
                    int[] goalPos = findGoalPosition(state[i][j], goalState);
                    
                    // Calculate circular distance
                    int verticalDistance = Math.min(
                        Math.abs(i - goalPos[0]),
                        rows - Math.abs(i - goalPos[0])
                    );
                    
                    int horizontalDistance = Math.min(
                        Math.abs(j - goalPos[1]),
                        cols - Math.abs(j - goalPos[1])
                    );
                    
                    heuristic += verticalDistance + horizontalDistance;
                }
            }
        }
        return heuristic;
    }

    private static int[] findGoalPosition(String color, String[][] goalState) {
        for (int i = 0; i < goalState.length; i++) {
            for (int j = 0; j < goalState[i].length; j++) {
                if (goalState[i][j].equals(color)) {
                    return new int[]{i, j};
                }
            }
        }
        return new int[]{-1, -1}; // Should not happen if goalState is valid
    }

    private static List<Node> getNeighbors(Node node, String[][] goalState) {
        List<Node> neighbors = new ArrayList<>();
        String[][] currentState = node.state;

        // Debugging print
        System.out.println("Generating Neighbors for State: " + Arrays.deepToString(currentState));
        System.out.println("Current Node Path Cost: " + node.g);

        // Identify colors that need to move
        for (int i = 0; i < currentState.length; i++) {
            for (int j = 0; j < currentState[i].length; j++) {
                if (currentState[i][j].equals("_")) {
                    // Try moving adjacent colors into the empty space
                    generateAdjacentMoves(currentState, i, j, neighbors, node, goalState);
                }
            }
        }

        // Debugging print
        System.out.println("Total Neighbors Generated: " + neighbors.size());

        return neighbors;
    }

    private static void generateAdjacentMoves(String[][] currentState, int emptyI, int emptyJ, 
                                               List<Node> neighbors, Node parentNode, String[][] goalState) {
        int[][] directions = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
        
        for (int[] dir : directions) {
            int newI = emptyI + dir[0];
            int newJ = emptyJ + dir[1];
            
            // Check if the new position is within bounds
            if (newI >= 0 && newI < currentState.length && 
                newJ >= 0 && newJ < currentState[0].length) {
                
                // Create a deep copy of the current state
                String[][] newState = new String[currentState.length][currentState[0].length];
                for (int i = 0; i < currentState.length; i++) {
                    newState[i] = Arrays.copyOf(currentState[i], currentState[i].length);
                }
                
                // Swap the empty space with the adjacent color
                if (!newState[newI][newJ].equals("X")) {
                    newState[emptyI][emptyJ] = newState[newI][newJ];
                    newState[newI][newJ] = "_";
                    
                    // Create a new node with the modified state
                    Node neighborNode = new Node(
                        newState, 
                        parentNode, 
                        parentNode.g + 1, 
                        0,  // Heuristic will be calculated later 
                        new Move(newI, newJ)
                    );
                    
                    neighbors.add(neighborNode);
                }
            }
        }
    }

    private static List<String> reconstructPath(Node goalNode) {
        List<String> path = new ArrayList<>();
        Node current = goalNode;
        
        while (current.parent != null) {
            // Create move description
            if (current.parent.parentMove != null) {
                int parentMoveI = current.parent.parentMove.i;
                int parentMoveJ = current.parent.parentMove.j;
                String movedColor = current.parent.state[parentMoveI][parentMoveJ];
                
                // Find initial and final positions
                int[] initialPos = findColorPosition(current.parent.state, movedColor);
                int[] finalPos = findColorPosition(current.state, movedColor);
                
                String move = String.format("(%d,%d):%s:(%d,%d)", 
                    initialPos[0] + 1, initialPos[1] + 1,
                    movedColor,
                    finalPos[0] + 1, finalPos[1] + 1
                );
                path.add(0, move);
            }
            current = current.parent;
        }
        
        return path;
    }

    // Helper method to find color position in a 2D array
    private static int[] findColorPosition(String[][] state, String color) {
        for (int i = 0; i < state.length; i++) {
            for (int j = 0; j < state[i].length; j++) {
                if (state[i][j].equals(color)) {
                    return new int[]{i, j};
                }
            }
        }
        return new int[]{-1, -1};  // Color not found
    }

    // Node class to represent a state in the search
    private static class Node {
        String[][] state;
        Node parent;
        int g;
        int h;
        int f;
        Move parentMove;

        public Node(String[][] state, Node parent, int g, int h, Move parentMove) {
            this.state = state;
            this.parent = parent;
            this.g = g;
            this.h = h;
            this.f = g + h;
            this.parentMove = parentMove;
        }
    }

    private static class Move {
        int i;
        int j;

        public Move(int i, int j) {
            this.i = i;
            this.j = j;
        }
    }
}
