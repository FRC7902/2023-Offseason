public class App {
    public static void main(String[] args) throws Exception {
        System.out.println("Hello, World!");

        // Create a new car object
        Car genericCar = new Car("Red");
        Car secondCar = new Car();
        Car thirdCar = new Car();

        Ferry carFerry = new Ferry();

        SUV SUVcar = new SUV("Blue");
        SUV van = new SUV("Orange", true);

        System.out.println(SUVcar.getCarColour());
        System.out.println("Is van: " + van.getVan());

        carFerry.loadCar(genericCar);
        carFerry.loadCar(secondCar);
        carFerry.loadCar(van);

        System.out.println(carFerry.unloadCar(2).getCarColour());

        // System.out.println("Total car instances: " + genericCar.getTotalCars());
        // System.out.println("Total car instances: " + secondCar.getTotalCars());

        // // System.out.println(genericCar.getWheelDiameter());

        // genericCar.setWheelDiameter(16);
        // secondCar.setWheelDiameter(20);

        // System.out.println(genericCar.getCarColour());
        // System.out.println(secondCar.getCarColour());

        // System.out.println(genericCar.repeatWord("Honk", 5));

    }
}
