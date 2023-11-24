public class SUV extends Car {

    boolean isVan = false;

    public SUV(String SUVColour) {
        super(SUVColour);
    }

    public SUV(String SUVColour, boolean van) {
        super(SUVColour);
        isVan = van;
    }

    public void setVan(boolean van) {
        isVan = van;
    }

    public boolean getVan() {
        return isVan;
    }

}
