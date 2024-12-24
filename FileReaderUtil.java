import java.io.*;
import java.util.*;

public class FileReaderUtil {

    public static Map<String, Object> parseInput(String filePath) throws IOException {
        BufferedReader br = new BufferedReader(new FileReader(filePath));
        String algorithm = br.readLine().trim();
        boolean timeFlag = br.readLine().trim().equals("with time");
        boolean openListFlag = br.readLine().trim().equals("with open");

        String[][] startState = new String[3][3];
        for (int i = 0; i < 3; i++) {
            startState[i] = br.readLine().split(",");
        }

        br.readLine(); // Skip "Goal state:"
        String[][] goalState = new String[3][3];
        for (int i = 0; i < 3; i++) {
            goalState[i] = br.readLine().split(",");
        }

        br.close();

        Map<String, Object> result = new HashMap<>();
        result.put("algorithm", algorithm);
        result.put("timeFlag", timeFlag);
        result.put("openListFlag", openListFlag);
        result.put("startState", startState);
        result.put("goalState", goalState);
        return result;
    }

    public static void writeOutput(String output, String filePath) throws IOException {
        try (PrintWriter pw = new PrintWriter(filePath)) {
            pw.println(output);
        }
    }
}
