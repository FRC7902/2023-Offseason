public class Ferry {

    private Car carsOnFerry[] = new Car[100];
    int totalCars = 0;

    public Ferry() {

        System.out.println(carsOnFerry[0]);

    }

    public void loadCar(Car loadingCar) {

        if (totalCars < 100) {
            for (int i = 0; i < 100; i++) {
                if (carsOnFerry[i] == null) {
                    carsOnFerry[i] = loadingCar;
                    System.out.println("Car loaded at index: " + i);
                    break;
                }
            }

        } else {
            System.out.println("No more room!");
        }
    }

    public Car unloadCar(int indexNum) {
        return carsOnFerry[indexNum];
    }

}
