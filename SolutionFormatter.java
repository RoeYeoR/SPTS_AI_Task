import java.util.List;
import java.util.Map;

public class SolutionFormatter {

    // Method to format the solution and return the output string
    public static String formatSolution(List<String> solutionPath, int solutionCost, int nodesGenerated, long startTime, boolean timeFlag) {
        StringBuilder output = new StringBuilder();
        
        if (solutionPath != null && !solutionPath.isEmpty()) {
            // Path found - format according to requirements
            output.append(String.join("--", solutionPath)).append("\n");
            output.append("Num: ").append(nodesGenerated).append("\n");
            output.append("Cost: ").append(solutionCost);
            
            // Add time if required
            if (timeFlag) {
                double seconds = (System.currentTimeMillis() - startTime) / 1000.0;
                output.append("\n").append(String.format("%.3f", seconds)).append(" seconds");
            }
        } else {
            // No path found
            output.append("no path\n");
            output.append("Num: ").append(nodesGenerated).append("\n");
            output.append("Cost: inf");
            
            // Add time if required
            if (timeFlag) {
                double seconds = (System.currentTimeMillis() - startTime) / 1000.0;
                output.append("\n").append(String.format("%.3f", seconds)).append(" seconds");
            }
        }

        return output.toString();
    }

    // Method to calculate the total cost of the solution path
    public static int calculateSolutionCost(List<String> solutionPath) {
        int totalCost = 0;
        Map<String, Integer> marbleCosts = Map.of("B", 1, "G", 3, "R", 10);

        for (String action : solutionPath) {
            // Example of action format: (2,2):B:(2,3)
            String[] parts = action.split(":");

            // Ensure the action has at least two parts before accessing the marble
            if (parts.length >= 2) {
                String marble = parts[1];  // Extract marble (B, G, or R)
                totalCost += marbleCosts.getOrDefault(marble, 0);  // Add cost for this marble
            } else {
                System.out.println("Invalid action format: " + action);
            }
        }

        return totalCost;
    }
}
