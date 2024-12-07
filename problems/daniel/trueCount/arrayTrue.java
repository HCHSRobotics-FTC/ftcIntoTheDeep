package trueCount;

public class arrayTrue {
    public static void main(String[] args) {
        final boolean[] inputArray = {true, false, false, true, false, true, true, false, true, true, false, false, false};

        int trueCount = countTrue(inputArray);
        
        System.out.printf("Number of trues in array: %d\n", trueCount);

    }
    public static int countTrue(boolean[] input) {
        int trueCount = 0;
        for (boolean value : input) {
            if (value == true) {
                trueCount++;
            }
        }
        return trueCount;
    }
}
