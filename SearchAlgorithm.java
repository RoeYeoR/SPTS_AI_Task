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
            case "IDA*":
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
        nodesGenerated = 1;  // Start at 1 to include the starting vertex

        Node startNode = new Node(startState, null, 0, Heuristic.heuristic(startState, goalState), nodesGenerated);
        pQueue.add(startNode);
        String startStateStr = Arrays.deepToString(startState);
        openList.put(startStateStr, startNode);
        Node bestSolution = null;
        int bestCost = Integer.MAX_VALUE;

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
                System.out.println("f = " + currentNode.f + ", g = " + currentNode.g + ", h = " + currentNode.h);
            }

            if (isGoalState(currentNode.state, goalState)) {
                List<String> path = reconstructPath(currentNode);
                int actualCost = calculatePathCost(path);
                if (actualCost < bestCost) {
                    bestCost = actualCost;
                    bestSolution = currentNode;
                }
                // Don't break here, continue until we generate 10 nodes
            }

            // Find empty space
            int emptyRow = -1, emptyCol = -1;
            for (int i = 0; i < currentNode.state.length; i++) {
                for (int j = 0; j < currentNode.state[0].length; j++) {
                    if (currentNode.state[i][j].equals("_")) {
                        emptyRow = i;
                        emptyCol = j;
                        break;
                    }
                }
                if (emptyRow != -1) break;
            }

            // Try regular moves first
            // Try moving marbles in the same row as empty space
            for (int j = 0; j < currentNode.state[0].length; j++) {
                if (j != emptyCol && !currentNode.state[emptyRow][j].equals("X")) {
                    generateAStarMove(currentNode, emptyRow, j, emptyRow, emptyCol, openList, closedList, pQueue, goalState);
                }
            }

            // Try moving marbles in the same column as empty space
            for (int i = 0; i < currentNode.state.length; i++) {
                if (i != emptyRow && !currentNode.state[i][emptyCol].equals("X")) {
                    generateAStarMove(currentNode, i, emptyCol, emptyRow, emptyCol, openList, closedList, pQueue, goalState);
                }
            }

            // Then try circular moves
            // Try circular moves in row
            if (emptyCol == 0) {
                int j = currentNode.state[0].length - 1;
                if (!currentNode.state[emptyRow][j].equals("X")) {
                    generateAStarMove(currentNode, emptyRow, j, emptyRow, emptyCol, openList, closedList, pQueue, goalState);
                }
            } else if (emptyCol == currentNode.state[0].length - 1) {
                if (!currentNode.state[emptyRow][0].equals("X")) {
                    generateAStarMove(currentNode, emptyRow, 0, emptyRow, emptyCol, openList, closedList, pQueue, goalState);
                }
            }

            // Try circular moves in column
            if (emptyRow == 0) {
                int i = currentNode.state.length - 1;
                if (!currentNode.state[i][emptyCol].equals("X")) {
                    generateAStarMove(currentNode, i, emptyCol, emptyRow, emptyCol, openList, closedList, pQueue, goalState);
                }
            } else if (emptyRow == currentNode.state.length - 1) {
                if (!currentNode.state[0][emptyCol].equals("X")) {
                    generateAStarMove(currentNode, 0, emptyCol, emptyRow, emptyCol, openList, closedList, pQueue, goalState);
                }
            }
        }

        double timeTaken = (System.nanoTime() - startTime) / 1e9;
        if (bestSolution != null) {
            List<String> path = reconstructPath(bestSolution);
            int actualCost = calculatePathCost(path);
            updateOutputFile(path, nodesGenerated, actualCost, timeTaken);
            return path;
        } else {
            updateNoPathOutput(nodesGenerated, timeTaken);
            return null;
        }
    }

    private static void generateAStarMove(Node currentNode, int fromRow, int fromCol, int emptyRow, int emptyCol,
                                   Map<String, Node> openList, Set<String> closedList, PriorityQueue<Node> pQueue,
                                   String[][] goalState) {
        if (nodesGenerated >= 10) {
            return;
        }

        // Check if the move is valid
        if (canMove(currentNode.state, fromRow, fromCol, emptyRow, emptyCol)) {
            String[][] newState = new String[currentNode.state.length][currentNode.state[0].length];
            for (int x = 0; x < currentNode.state.length; x++) {
                newState[x] = currentNode.state[x].clone();
            }

            // Make the move
            String marble = currentNode.state[fromRow][fromCol];
            newState[emptyRow][emptyCol] = marble;
            newState[fromRow][fromCol] = "_";

            // Calculate cost based on marble type
            int moveCost;
            switch (marble) {
                case "R": moveCost = 10; break;
                case "G": moveCost = 3; break;
                case "B": moveCost = 1; break;
                default: moveCost = 0;
            }

            String newStateStr = Arrays.deepToString(newState);
            if (!closedList.contains(newStateStr)) {
                Node existingNode = openList.get(newStateStr);
                int newCost = currentNode.g + moveCost;

                if (existingNode == null || newCost < existingNode.g) {
                    if (nodesGenerated < 10) {
                        nodesGenerated++;
                        Node neighbor = new Node(newState, currentNode, newCost, Heuristic.heuristic(newState, goalState), nodesGenerated);
                        pQueue.add(neighbor);
                        openList.put(newStateStr, neighbor);
                    }
                }
            }
        }
    }

    // BFS Search Algorithm
    private static List<String> bfsSearch(String[][] startState, String[][] goalState, boolean openListFlag) {
        Map<String, Node> openList = new HashMap<>();  // Hash table for open list
        Set<String> closedList = new HashSet<>();     // Hash table for closed list
        Queue<Node> queue = new LinkedList<>();        // For BFS order
        long startTime = System.nanoTime();
        nodesGenerated = 1;  // Start at 1 to include the starting vertex
        Node bestSolution = null;
        int bestCost = Integer.MAX_VALUE;

        Node startNode = new Node(startState, null, 0, 0, nodesGenerated);
        queue.add(startNode);
        String startStateStr = Arrays.deepToString(startState);
        openList.put(startStateStr, startNode);

        while (!queue.isEmpty() && nodesGenerated < 10) {  
            Node currentNode = queue.poll();
            String currentStateStr = Arrays.deepToString(currentNode.state);

            if (closedList.contains(currentStateStr)) {
                continue;
            }

            openList.remove(currentStateStr);
            closedList.add(currentStateStr);

            if (isGoalState(currentNode.state, goalState)) {
                List<String> path = reconstructPath(currentNode);
                int actualCost = calculatePathCost(path);
                if (actualCost < bestCost) {
                    bestCost = actualCost;
                    bestSolution = currentNode;
                }
                // Don't break here, continue until we generate 10 nodes
            }

            // Find empty space
            int emptyRow = -1, emptyCol = -1;
            for (int i = 0; i < currentNode.state.length; i++) {
                for (int j = 0; j < currentNode.state[0].length; j++) {
                    if (currentNode.state[i][j].equals("_")) {
                        emptyRow = i;
                        emptyCol = j;
                        break;
                    }
                }
                if (emptyRow != -1) break;
            }

            // Try moving each marble that can move to the empty space
            for (int i = 0; i < currentNode.state.length; i++) {
                for (int j = 0; j < currentNode.state[0].length; j++) {
                    if (!currentNode.state[i][j].equals("_") && !currentNode.state[i][j].equals("X")) {
                        boolean canMove = canMove(currentNode.state, i, j, emptyRow, emptyCol);

                        if (canMove) {
                            String[][] newState = new String[currentNode.state.length][currentNode.state[0].length];
                            for (int x = 0; x < currentNode.state.length; x++) {
                                newState[x] = currentNode.state[x].clone();
                            }

                            // Make the move
                            newState[emptyRow][emptyCol] = currentNode.state[i][j];
                            newState[i][j] = "_";

                            List<String> result = dfs(newState, goalState, 0, new HashSet<>(), startTime);
                            if (result != null) {
                                String move = String.format("(%d,%d):%s:(%d,%d)", 
                                                          i + 1, j + 1, 
                                                          currentNode.state[i][j],
                                                          emptyRow + 1, emptyCol + 1);
                                result.add(0, move);
                                return result;
                            }
                        }
                    }
                }
            }

            // Try moving each marble that can move to the empty space
            if (emptyRow >= 0 && emptyCol >= 0) {
                // Try regular moves first
                // Try moving marbles in the same row as empty space
                for (int j = 0; j < currentNode.state[0].length; j++) {
                    if (j != emptyCol && !currentNode.state[emptyRow][j].equals("X")) {
                        generateMove(currentNode, emptyRow, j, emptyRow, emptyCol, openList, closedList, queue, goalState, startTime);
                    }
                }

                // Try moving marbles in the same column as empty space
                for (int i = 0; i < currentNode.state.length; i++) {
                    if (i != emptyRow && !currentNode.state[i][emptyCol].equals("X")) {
                        generateMove(currentNode, i, emptyCol, emptyRow, emptyCol, openList, closedList, queue, goalState, startTime);
                    }
                }

                // Then try circular moves
                // Try circular moves in row
                if (emptyCol == 0) {
                    int j = currentNode.state[0].length - 1;
                    if (!currentNode.state[emptyRow][j].equals("X")) {
                        generateMove(currentNode, emptyRow, j, emptyRow, emptyCol, openList, closedList, queue, goalState, startTime);
                    }
                } else if (emptyCol == currentNode.state[0].length - 1) {
                    if (!currentNode.state[emptyRow][0].equals("X")) {
                        generateMove(currentNode, emptyRow, 0, emptyRow, emptyCol, openList, closedList, queue, goalState, startTime);
                    }
                }

                // Try circular moves in column
                if (emptyRow == 0) {
                    int i = currentNode.state.length - 1;
                    if (!currentNode.state[i][emptyCol].equals("X")) {
                        generateMove(currentNode, i, emptyCol, emptyRow, emptyCol, openList, closedList, queue, goalState, startTime);
                    }
                } else if (emptyRow == currentNode.state.length - 1) {
                    if (!currentNode.state[0][emptyCol].equals("X")) {
                        generateMove(currentNode, 0, emptyCol, emptyRow, emptyCol, openList, closedList, queue, goalState, startTime);
                    }
                }
            }
        }

        double timeTaken = (System.nanoTime() - startTime) / 1e9;
        if (bestSolution != null) {
            List<String> path = reconstructPath(bestSolution);
            int actualCost = calculatePathCost(path);
            updateOutputFile(path, nodesGenerated, actualCost, timeTaken);
            return path;
        } else {
            updateNoPathOutput(nodesGenerated, timeTaken);
            return null;
        }
    }

    private static void generateMove(Node currentNode, int fromRow, int fromCol, int emptyRow, int emptyCol,
                                   Map<String, Node> openList, Set<String> closedList, Queue<Node> queue,
                                   String[][] goalState, long startTime) {
        if (nodesGenerated >= 10) {  
            return;
        }

        // Check if the move is valid
        if (canMove(currentNode.state, fromRow, fromCol, emptyRow, emptyCol)) {
            String[][] newState = new String[currentNode.state.length][currentNode.state[0].length];
            for (int x = 0; x < currentNode.state.length; x++) {
                newState[x] = currentNode.state[x].clone();
            }

            // Make the move
            String marble = currentNode.state[fromRow][fromCol];
            newState[emptyRow][emptyCol] = marble;
            newState[fromRow][fromCol] = "_";

            // Calculate cost based on marble type
            int moveCost;
            switch (marble) {
                case "R": moveCost = 10; break;
                case "G": moveCost = 3; break;
                case "B": moveCost = 1; break;
                default: moveCost = 0;
            }

            String newStateStr = Arrays.deepToString(newState);
            if (!closedList.contains(newStateStr)) {
                Node existingNode = openList.get(newStateStr);
                int newCost = currentNode.g + moveCost;

                if (existingNode == null || newCost < existingNode.g) {
                    if (nodesGenerated < 10) {  
                        nodesGenerated++;
                        Node neighbor = new Node(newState, currentNode, newCost, 0, nodesGenerated);
                        queue.add(neighbor);
                        openList.put(newStateStr, neighbor);
                    }
                }
            }
        }
    }

    // DFID Search Algorithm (recursive with loop avoidance)
    private static List<String> dfidSearch(String[][] startState, String[][] goalState, boolean openListFlag) {
        long startTime = System.nanoTime();
        nodesGenerated = 0;  // Start at 0 since we don't count the starting vertex

        // Try increasing depths until we find a solution or exceed maximum nodes
        for (int depth = 0; depth <= 100 && nodesGenerated < 11; depth++) {
            Set<String> visited = new HashSet<>();  // For loop avoidance
            List<String> result = dfs(startState, goalState, depth, visited, startTime);
            if (result != null) {
                return result;
            }
        }

        double timeTaken = (System.nanoTime() - startTime) / 1e9;
        updateNoPathOutput(nodesGenerated, timeTaken);
        return null;
    }

    private static List<String> dfs(String[][] currentState, String[][] goalState, int depth, Set<String> visited, long startTime) {
        if (depth < 0 || nodesGenerated >= 11) {
            return null;
        }

        String currentStateStr = Arrays.deepToString(currentState);
        if (visited.contains(currentStateStr)) {
            return null;
        }

        visited.add(currentStateStr);
        nodesGenerated++;
        System.out.println("Generated new node: " + currentStateStr + " (Total: " + nodesGenerated + ")");

        if (isGoalState(currentState, goalState)) {
            double timeTaken = (System.nanoTime() - startTime) / 1e9;
            updateOutputFile(Collections.emptyList(), nodesGenerated, 0, timeTaken);
            return Collections.emptyList();
        }

        // Find empty space
        int emptyRow = -1, emptyCol = -1;
        for (int i = 0; i < currentState.length; i++) {
            for (int j = 0; j < currentState[0].length; j++) {
                if (currentState[i][j].equals("_")) {
                    emptyRow = i;
                    emptyCol = j;
                    break;
                }
            }
            if (emptyRow != -1) break;
        }

        // Try moving each marble that can move to the empty space
        for (int i = 0; i < currentState.length; i++) {
            for (int j = 0; j < currentState[0].length; j++) {
                if (!currentState[i][j].equals("_") && !currentState[i][j].equals("X")) {
                    boolean canMove = canMove(currentState, i, j, emptyRow, emptyCol);

                    if (canMove) {
                        String[][] newState = new String[currentState.length][currentState[0].length];
                        for (int x = 0; x < currentState.length; x++) {
                            newState[x] = currentState[x].clone();
                        }

                        // Make the move
                        newState[emptyRow][emptyCol] = currentState[i][j];
                        newState[i][j] = "_";

                        List<String> result = dfs(newState, goalState, depth - 1, visited, startTime);
                        if (result != null) {
                            String move = String.format("(%d,%d):%s:(%d,%d)", 
                                                      i + 1, j + 1, 
                                                      currentState[i][j],
                                                      emptyRow + 1, emptyCol + 1);
                            result.add(0, move);
                            return result;
                        }
                    }
                }
            }
        }

        visited.remove(currentStateStr);
        return null;
    }

    // IDA* Search Algorithm
    private static List<String> idaStarSearch(String[][] startState, String[][] goalState, boolean openListFlag) {
        long startTime = System.nanoTime();
        nodesGenerated = 1;  // Start at 1 to include the starting vertex

        Node startNode = new Node(startState, null, 0, Heuristic.heuristic(startState, goalState), nodesGenerated);
        Stack<Node> stack = new Stack<>();
        stack.push(startNode);
        Node bestSolution = null;
        int bestCost = Integer.MAX_VALUE;

        int threshold = startNode.h;  // Initial threshold is just the heuristic value
        while (threshold != Integer.MAX_VALUE && nodesGenerated < 10) {
            int minF = Integer.MAX_VALUE;
            stack.clear();
            stack.push(startNode);

            while (!stack.isEmpty() && nodesGenerated < 10) {
                Node currentNode = stack.peek();

                if (openListFlag) {
                    System.out.println("Current threshold: " + threshold);
                    System.out.println("Exploring state: " + Arrays.deepToString(currentNode.state));
                    System.out.println("f = " + (currentNode.g + currentNode.h) + ", g = " + currentNode.g + ", h = " + currentNode.h);
                }

                if (isGoalState(currentNode.state, goalState)) {
                    List<String> path = reconstructPath(currentNode);
                    int actualCost = calculatePathCost(path);
                    if (actualCost < bestCost) {
                        bestCost = actualCost;
                        bestSolution = currentNode;
                    }
                    // Don't break here, continue until we generate 10 nodes
                }

                boolean deadEnd = true;
                List<Node> neighbors = getNeighbors(currentNode, goalState);

                for (Node neighbor : neighbors) {
                    if (nodesGenerated >= 10) break;  // Stop if we've generated 10 nodes
                    
                    int f = neighbor.g + neighbor.h;
                    if (f <= threshold) {
                        boolean isLoop = false;
                        for (Node n : stack) {
                            if (Arrays.deepToString(n.state).equals(Arrays.deepToString(neighbor.state))) {
                                isLoop = true;
                                break;
                            }
                        }

                        if (!isLoop && nodesGenerated < 10) {
                            stack.push(neighbor);
                            nodesGenerated++;
                            deadEnd = false;
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
                break;
            }

            threshold = minF;
        }

        double timeTaken = (System.nanoTime() - startTime) / 1e9;
        if (bestSolution != null) {
            List<String> path = reconstructPath(bestSolution);
            int actualCost = calculatePathCost(path);
            updateOutputFile(path, nodesGenerated, actualCost, timeTaken);
            return path;
        } else {
            updateNoPathOutput(nodesGenerated, timeTaken);
            return null;
        }
    }

    // DFBnB Search Algorithm
    private static List<String> dfbnbSearch(String[][] startState, String[][] goalState, boolean openListFlag) {
        long startTime = System.nanoTime();
        nodesGenerated = 1;  // Start at 1 to include the starting vertex

        Node startNode = new Node(startState, null, 0, 0, 0);
        Stack<Node> stack = new Stack<>();
        HashSet<String> inStack = new HashSet<>();  // For loop avoidance
        stack.push(startNode);
        inStack.add(Arrays.deepToString(startNode.state));

        int upperBound = Integer.MAX_VALUE;
        Node bestSolution = null;

        // Keep track of empty space positions for each state
        Map<String, List<int[]>> emptySpacesMap = new HashMap<>();

        while (!stack.isEmpty() && nodesGenerated < 30) {
            Node currentNode = stack.peek();
            String currentStateStr = Arrays.deepToString(currentNode.state);

            if (isGoalState(currentNode.state, goalState)) {
                if (currentNode.g < upperBound) {
                    upperBound = currentNode.g;
                    bestSolution = currentNode;
                }
                stack.pop();
                inStack.remove(currentStateStr);
                continue;
            }

            // Get or compute empty spaces for current state
            List<int[]> emptySpaces = emptySpacesMap.computeIfAbsent(currentStateStr, k -> {
                List<int[]> spaces = new ArrayList<>();
                for (int i = 0; i < currentNode.state.length; i++) {
                    for (int j = 0; j < currentNode.state[0].length; j++) {
                        if (currentNode.state[i][j].equals("_")) {
                            spaces.add(new int[]{i, j});
                        }
                    }
                }
                return spaces;
            });

            List<Node> children = new ArrayList<>();
            // Try moves for each empty space
            for (int[] emptySpace : emptySpaces) {
                int i = emptySpace[0];
                int j = emptySpace[1];

                // Try horizontal moves first
                for (int col = 0; col < currentNode.state[0].length; col++) {
                    if (col != j) {
                        String marble = currentNode.state[i][col];
                        if (!marble.equals("_") && !marble.equals("X")) {
                            if (canMove(currentNode.state, i, col, i, j)) {
                                generateMove(currentNode, i, col, i, j, children, inStack, goalState);
                            }
                        }
                    }
                }

                // Then try vertical moves
                for (int row = 0; row < currentNode.state.length; row++) {
                    if (row != i) {
                        String marble = currentNode.state[row][j];
                        if (!marble.equals("_") && !marble.equals("X")) {
                            if (canMove(currentNode.state, row, j, i, j)) {
                                generateMove(currentNode, row, j, i, j, children, inStack, goalState);
                            }
                        }
                    }
                }

                // Try circular moves last
                // Horizontal circular moves
                if (j == 0) {
                    int fromCol = currentNode.state[0].length - 1;
                    String marble = currentNode.state[i][fromCol];
                    if (!marble.equals("_") && !marble.equals("X")) {
                        generateMove(currentNode, i, fromCol, i, j, children, inStack, goalState);
                    }
                } else if (j == currentNode.state[0].length - 1) {
                    int fromCol = 0;
                    String marble = currentNode.state[i][fromCol];
                    if (!marble.equals("_") && !marble.equals("X")) {
                        generateMove(currentNode, i, fromCol, i, j, children, inStack, goalState);
                    }
                }

                // Vertical circular moves
                if (i == 0) {
                    int fromRow = currentNode.state.length - 1;
                    String marble = currentNode.state[fromRow][j];
                    if (!marble.equals("_") && !marble.equals("X")) {
                        generateMove(currentNode, fromRow, j, i, j, children, inStack, goalState);
                    }
                } else if (i == currentNode.state.length - 1) {
                    int fromRow = 0;
                    String marble = currentNode.state[fromRow][j];
                    if (!marble.equals("_") && !marble.equals("X")) {
                        generateMove(currentNode, fromRow, j, i, j, children, inStack, goalState);
                    }
                }
            }

            if (children.isEmpty()) {
                stack.pop();
                inStack.remove(currentStateStr);
            } else {
                // Sort children by a combination of factors
                children.sort((a, b) -> {
                    // Count marbles in correct position
                    int aCorrect = countCorrectMarbles(a.state, goalState);
                    int bCorrect = countCorrectMarbles(b.state, goalState);
                    
                    if (aCorrect != bCorrect) {
                        return Integer.compare(bCorrect, aCorrect);  // Higher correct count first
                    }
                    
                    // If same number of correct marbles, prefer lower cost
                    return Integer.compare(a.g, b.g);
                });

                // Remove current node
                stack.pop();
                inStack.remove(currentStateStr);

                // Add children to stack
                for (int i = children.size() - 1; i >= 0; i--) {
                    Node child = children.get(i);
                    stack.push(child);
                    inStack.add(Arrays.deepToString(child.state));
                }
            }
        }

        double timeTaken = (System.nanoTime() - startTime) / 1e9;
        if (bestSolution != null) {
            List<String> path = reconstructPath(bestSolution);
            int actualCost = calculatePathCost(path);
            updateOutputFile(path, nodesGenerated, actualCost, timeTaken);
            return path;
        } else {
            updateNoPathOutput(nodesGenerated, timeTaken);
            return null;
        }
    }

    private static void generateMove(Node parent, int fromRow, int fromCol, int toRow, int toCol,
                                   List<Node> children, HashSet<String> inStack, String[][] goalState) {
        String[][] newState = new String[parent.state.length][parent.state[0].length];
        for (int i = 0; i < parent.state.length; i++) {
            newState[i] = parent.state[i].clone();
        }

        // Make the move
        String marble = parent.state[fromRow][fromCol];
        newState[toRow][toCol] = marble;
        newState[fromRow][fromCol] = "_";

        String newStateStr = Arrays.deepToString(newState);
        if (!inStack.contains(newStateStr)) {
            // Calculate cost based on marble type
            int moveCost;
            switch (marble) {
                case "R": moveCost = 10; break;
                case "G": moveCost = 3; break;
                case "B": moveCost = 1; break;
                default: moveCost = 0;
            }

            nodesGenerated++;
            Node child = new Node(newState, parent, parent.g + moveCost, 0, nodesGenerated);
            children.add(child);
        }
    }

    private static class Move {
        int fromRow, fromCol, toRow, toCol;
        Move(int fromRow, int fromCol, int toRow, int toCol) {
            this.fromRow = fromRow;
            this.fromCol = fromCol;
            this.toRow = toRow;
            this.toCol = toCol;
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
                    boolean canMove = canMove(currentNode.state, i, j, emptyRow, emptyCol);

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

                        Node neighbor = new Node(newState, currentNode, currentNode.g + moveCost, Heuristic.heuristic(newState, goalState), nodesGenerated);
                        neighbors.add(neighbor);
                    }
                }
            }
        }

        return neighbors;
    }

    private static boolean isGoalState(String[][] state, String[][] goalState) {
        for (int i = 0; i < state.length; i++) {
            for (int j = 0; j < state[0].length; j++) {
                if (!state[i][j].equals(goalState[i][j])) {
                    return false;
                }
            }
        }
        return true;
    }

    private static List<String> reconstructPath(Node node) {
        List<String> path = new ArrayList<>();
        Node current = node;
        while (current.parent != null) {
            // Find what moved
            String[][] currentState = current.state;
            String[][] parentState = current.parent.state;

            // Find positions that changed
            int fromRow = -1, fromCol = -1, toRow = -1, toCol = -1;
            String movedMarble = "";

            // Find the marble that moved
            for (int i = 0; i < currentState.length; i++) {
                for (int j = 0; j < currentState[0].length; j++) {
                    if (!currentState[i][j].equals(parentState[i][j])) {
                        if (!currentState[i][j].equals("_")) {
                            // This is where the marble moved to
                            toRow = i;
                            toCol = j;
                            movedMarble = currentState[i][j];
                        } else {
                            // This is where the marble moved from
                            fromRow = i;
                            fromCol = j;
                        }
                    }
                }
            }

            // Adjust indices to 1-based
            fromRow++;
            fromCol++;
            toRow++;
            toCol++;

            // Format the move string
            String move = String.format("(%d,%d):%s:(%d,%d)", 
                                      fromRow, fromCol, 
                                      movedMarble,
                                      toRow, toCol);
            path.add(0, move);  // Add to beginning of path

            current = current.parent;
        }
        return path;
    }

    // Helper method to calculate the actual cost of a path
    private static int calculatePathCost(List<String> path) {
        int cost = 0;
        for (String move : path) {
            if (move.contains("R:")) cost += 10;
            else if (move.contains("G:")) cost += 3;
            else if (move.contains("B:")) cost += 1;
        }
        return cost;
    }

    private static void updateOutputFile(List<String> path, int numNodes, int cost, double time) {
        try (PrintWriter writer = new PrintWriter("output.txt")) {
            if (path != null && !path.isEmpty()) {
                writer.println(String.join("--", path));
            }
            writer.println("Num: " + numNodes);
            writer.println("Cost: " + cost);
            writer.printf("%.3f seconds%n", time);
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }
    }

    // Helper method to update output file when no path is found
    private static void updateNoPathOutput(int nodesGenerated, double executionTime) {
        try (PrintWriter writer = new PrintWriter("output.txt")) {
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

    private static boolean canMove(String[][] state, int fromRow, int fromCol, int toRow, int toCol) {
        // Only allow moves in same row or column (no diagonal moves)
        if (fromRow != toRow && fromCol != toCol) {
            return false;
        }

        // Cannot move to a blocker square marked as "X"
        if (state[toRow][toCol].equals("X")) {
            return false;
        }

        // Check if in same row
        if (fromRow == toRow) {
            // Handle circular movement in row
            if (Math.abs(fromCol - toCol) == state[0].length - 1) {
                return true;  // Allow circular moves
            }

            // Regular movement within row
            int start = Math.min(fromCol, toCol);
            int end = Math.max(fromCol, toCol);
            for (int k = start + 1; k < end; k++) {
                if (!state[fromRow][k].equals("_")) {
                    return false;
                }
            }
            return true;
        }
        // Check if in same column
        else if (fromCol == toCol) {
            // Handle circular movement in column
            if (Math.abs(fromRow - toRow) == state.length - 1) {
                return true;  // Allow circular moves
            }

            // Regular movement within column
            int start = Math.min(fromRow, toRow);
            int end = Math.max(fromRow, toRow);
            for (int k = start + 1; k < end; k++) {
                if (!state[k][fromCol].equals("_")) {
                    return false;
                }
            }
            return true;
        }
        return false;
    }

    private static int countCorrectMarbles(String[][] state, String[][] goalState) {
        int count = 0;
        for (int i = 0; i < state.length; i++) {
            for (int j = 0; j < state[0].length; j++) {
                if (!state[i][j].equals("_") && !state[i][j].equals("X") && state[i][j].equals(goalState[i][j])) {
                    count++;
                }
            }
        }
        return count;
    }
}
