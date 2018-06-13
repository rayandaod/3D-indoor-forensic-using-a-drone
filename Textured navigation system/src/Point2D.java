/**
 * Class representing a 2D point
 */

class Point2D {

    private final double x;
    private final double y;

    Point2D(double x, double y){
        this.x = x;
        this.y = y;
    }

    public double euclidianDistanceTo(Point2D p){
        return Math.sqrt((p.x - this.x) * (p.x - this.x) + (p.y - this.y) * (p.y - this.y));
    }

    @Override
    public String toString(){
        return "(" + x + ", " + y + ")";
    }
}
