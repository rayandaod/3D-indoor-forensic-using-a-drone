/**
 * Class representing a 3D object
 */

public class Point3D {
    public double x;
    public double y;
    public double z;

    Point3D(double x, double y, double z){
        this.x = x;
        this.y = y;
        this.z = z;
    }

    @Override
    public String toString(){
        return "(" + x + ", " + y + ", " + z + ")";
    }
}
