import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.Scanner;


/**
 * Class representing a Parser for 3D objects containing the variables v, vt, and f (at least)
 */

class Parser {

    private File file;

    private List<Point2D> textureVertices = new ArrayList<>();
    private List<Point3D> verticesCoordinates = new ArrayList<>();
    private List<Face> faces = new ArrayList<>();

    private List<String> lines = new ArrayList<>();

    Parser(File file){
        this.file = file;
    }

    /**
     * Retrieves the index of the closest texture vertex to the given one
     *
     * @param tPosition the given position in the texture file
     * @return the index of the closest vt point in the given file,
     * or -1 if no vt coordinates were found
     */
    int closestTexturePoint(Point2D tPosition) {
        try {
            fillTheLists();
        } catch (Exception e) {
            e.printStackTrace();
        }

        double minDistance = Double.MAX_VALUE;
        int closestTexturePointIndex = -2;

        for (int i = 0; i < textureVertices.size(); ++i) {
            double distance = tPosition.euclidianDistanceTo(textureVertices.get(i));
            if (distance < minDistance) {
                closestTexturePointIndex = i;
                minDistance = distance;
            }
        }

        return closestTexturePointIndex;
    }

    /**
     * Returns the index of the vertex corresponding to the given closestTexturePointIndex
     * @param closestTexturePointIndex the given index
     * @return the index of the corresponding vertex, else returns -1
     */
    int correspondingVertexIndex(int closestTexturePointIndex){
        for(Face f : faces){
            if(f.vt1 == closestTexturePointIndex) return f.v1 - 1;
            if(f.vt2 == closestTexturePointIndex) return f.v2 - 1;
            if(f.vt3 == closestTexturePointIndex) return f.v3 - 1;
        }
        return -1;
    }

    /**
     * Returns the Point3D at the given index
     * @param correspondingVertexIndex the index of the vertex we want the coordinates from
     * @return the Point3D at the given index
     */
    Point3D findCoordinates(int correspondingVertexIndex){
        return verticesCoordinates.get(correspondingVertexIndex);
    }

    /**
     * Divide the file into vertices, vts, and faces and fill the corresponding lists
     * @throws Exception
     */
    private void fillTheLists() throws Exception {

        Scanner input = new Scanner(file);
        while(input.hasNextLine()){
            lines.add(input.nextLine());
        }

        for (String line : lines) {
            String[] tokens = line.split("\\s+|/");
            switch (tokens[0]){
                case "v":
                    Point3D coordinate = new Point3D(
                            Double.parseDouble(tokens[1]),
                            Double.parseDouble(tokens[2]),
                            Double.parseDouble(tokens[3]));
                    verticesCoordinates.add(coordinate);
                    break;
                case "vt":
                    Point2D tPosition = new Point2D(
                            Double.parseDouble(tokens[1]),
                            Double.parseDouble(tokens[2]));
                    textureVertices.add(tPosition);
                    break;
                case "f":
                    Face face = new Face(
                            Integer.parseInt(tokens[1]),
                            Integer.parseInt(tokens[2]),
                            Integer.parseInt(tokens[3]),
                            Integer.parseInt(tokens[4]),
                            Integer.parseInt(tokens[5]),
                            Integer.parseInt(tokens[6]),
                            Integer.parseInt(tokens[7]),
                            Integer.parseInt(tokens[8]),
                            Integer.parseInt(tokens[9]));
                    faces.add(face);
            }
        }

//        System.out.println(verticesCoordinates);
//        System.out.println(textureVertices);
//        System.out.println(faces);
    }
}
