import java.io.*;
import java.util.*;

public class Ex1 {
    public static void main(String[] args) throws IOException {
        // Parsing input from the file
        Map<String, Object> input = FileReaderUtil.parseInput("input.txt");
        String algorithm = (String) input.get("algorithm");
        boolean timeFlag = (boolean) input.get("timeFlag");
        boolean openListFlag = (boolean) input.get("openListFlag");
        String[][] startState = (String[][]) input.get("startState");
        String[][] goalState = (String[][]) input.get("goalState");

        long startTime = System.currentTimeMillis();  // Start the timer

        List<String> solutionPath = null;
        int solutionCost = -1;

        // Use the performSearch method from SearchAlgorithm to execute the chosen algorithm
        solutionPath = SearchAlgorithm.performSearch(startState, goalState, algorithm, openListFlag);

        // If no solution found, output the failure message
        if (solutionPath == null) {
            String output = SolutionFormatter.formatSolution(null, solutionCost, SearchAlgorithm.nodesGenerated, startTime, timeFlag);
            FileReaderUtil.writeOutput(output, "output.txt");
            return;
        }

        // Calculate the cost of the solution
        solutionCost = SolutionFormatter.calculateSolutionCost(solutionPath);

        // Format the output based on the solution
        String output = SolutionFormatter.formatSolution(solutionPath, solutionCost, SearchAlgorithm.nodesGenerated, startTime, timeFlag);
        FileReaderUtil.writeOutput(output, "output.txt");
    }
}
