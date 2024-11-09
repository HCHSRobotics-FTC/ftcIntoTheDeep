public class fizzBuzz {
    public static void main(String[] args) {
        final int start = 0;
        final int end = 100;

        int current = start;

        while (current <= (end)) {
            if ((current % 3) == 0)
                System.out.print("Fizz");
            if ((current % 5) == 0)
                System.out.print("Buzz");
            if ((current % 7) == 0)
                System.out.print("Bazz");
            if ((current % 3) != 0 && (current % 5) != 0 && (current % 7) != 0)
                System.out.print(current);
            System.out.println();
            current++;
        }
    }
}