public class Car {

    private static int totalCarInstances = 0;

    private double wheelDiameterInches;
    private String colour;

    // Constructor with no parameters
    public Car() {
        colour = "White";
        wheelDiameterInches = 14;

        totalCarInstances++;

    }

    // Constructor with one parameter
    public Car(String carColour) {
        colour = carColour;
        wheelDiameterInches = 14;
        System.out.println("This is a car!");

        totalCarInstances++;
    }

    // Gets the wheel diameter in inches
    public double getWheelDiameter() {
        return wheelDiameterInches;
    }

    public String getCarColour() {
        return colour;
    }

    public void setWheelDiameter(double wheelDiam) {
        // Validation
        // This is why we have setters
        if (wheelDiam > 0) {
            wheelDiameterInches = wheelDiam;
        }
    }

    public int getTotalCars() {
        return totalCarInstances;
    }

    public String repeatWord(String word, int repetition) {

        String longWord = "";

        for (int i = 0; i < repetition; i++) {
            longWord += word;
        }

        return longWord;

    }

}
