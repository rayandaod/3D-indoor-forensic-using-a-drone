/**
 * Class representing a face in a 3D object
 */

public class Face {

    public int v1;
    public int vt1;
    public int vn1;

    public int v2;
    public int vt2;
    public int vn2;

    public int v3;
    public int vt3;
    public int vn3;

    public Face(int v1, int vt1, int vn1, int v2, int vt2, int vn2, int v3, int vt3, int vn3) {
        this.v1 = v1;
        this.vt1 = vt1;
        this.vn1 = vn1;
        this.v2 = v2;
        this.vt2 = vt2;
        this.vn2 = vn2;
        this.v3 = v3;
        this.vt3 = vt3;
        this.vn3 = vn3;
    }

    @Override
    public String toString(){
        return "f " + v1 + "/" + vt1 + "/" + vn1 + " " + v2 + "/" + vt2 + "/" + vn2 + " " + v3 + "/" + vt3 + "/" + vn3 + " ";
    }
}
