import java.awt.*;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.io.BufferedReader;
import java.io.File;
import java.io.IOException;
import java.io.InputStreamReader;
import java.text.DecimalFormat;
import javax.swing.*;
import javax.swing.border.Border;
import javax.swing.filechooser.FileNameExtensionFilter;


class Interface extends JFrame {

    // UI variables

    private int wWidth = 1250;
    private int wHeight = 990;

    private int xSideBar = 980;
    private int wSideBar = 250;
    private int hSideBar = 40;
    private int spacing = 10;

    private JLabel textChooseFile;
    private JButton buttonChooseFile;

    private JLabel textureImage;

    private JLabel textSelectPoint;

    private JLabel textCurrentPoint;

    private JButton buttonConvertTo3D;
    private JLabel text3DCoordinates;

    private JButton buttonLaunchDrone;

    private JLabel textAssociatedObj;


    // Coordinates
    private Point2D coordinates2D;
    private Point3D coordinates3D;
    private Point3D coordinates3DAdjusted;

    // Strings

    private String textChooseFileStr = "Choose a png/jpg texture file:";
    private String buttonChooseFileStr = "Browse";
    private String textAssociatedObjStrDefault = "No associated .obj file";
    private String textAssociatedObjStrNotFoud = "Associated mesh found!";
    private String textSelectPointStr = "<html>Click on the chosen texture file<br>to select a destination for the drone:</html>";
    private String textCurrentPointStr = "(no point selected)";
    private String buttonConvertTo3DStr = "Convert to 3D coordinates";
    private String text3DcoordinatesStr = "";
    private String buttonLaunchDroneStr = "Launch drone";

    private String objectFileName = "mesh_gen.obj";


    // Paths
    private String droneLauncherCommand = "python3 " + System.getProperty("user.dir") + "../Pyparrot/demo.py";
    private String textureImagePath;

    Interface() {
        super("Texture to 3D coordinates");

        // Declare the texture image frame
        textureImage = new JLabel();
        textureImage.setBounds(10, 10, 950, 950);
        Border border = BorderFactory.createLineBorder(Color.BLACK, 2);
        textureImage.setBorder(border);

        // Declare a variable to get the next vertical available spot for inserting a text/button/etc
        int nextAvailableSpace = spacing;


        // Declare all the texts/buttons

        textChooseFile = new JLabel(textChooseFileStr);
        Font boldFont = new Font(textChooseFile.getFont().getFontName(), Font.BOLD, textChooseFile.getFont().getSize());
        textChooseFile.setFont(boldFont);
        textChooseFile.setBounds(xSideBar, nextAvailableSpace, wSideBar, hSideBar);

        nextAvailableSpace += textChooseFile.getHeight() + spacing;

        buttonChooseFile = new JButton(buttonChooseFileStr);
        buttonChooseFile.setBounds(xSideBar, nextAvailableSpace, wSideBar, hSideBar);

        nextAvailableSpace += buttonChooseFile.getHeight() + spacing;

        textAssociatedObj = new JLabel(textAssociatedObjStrDefault);
        textAssociatedObj.setBounds(xSideBar, nextAvailableSpace, wSideBar, hSideBar);

        nextAvailableSpace += textAssociatedObj.getHeight() + spacing;

        textSelectPoint = new JLabel(textSelectPointStr);
        textSelectPoint.setFont(boldFont);
        textSelectPoint.setBounds(xSideBar, nextAvailableSpace, wSideBar, hSideBar);

        nextAvailableSpace += textSelectPoint.getHeight() + spacing;

        textCurrentPoint = new JLabel(textCurrentPointStr);
        textCurrentPoint.setBounds(xSideBar, nextAvailableSpace, wSideBar, hSideBar);

        nextAvailableSpace += textCurrentPoint.getHeight() + spacing;

        buttonConvertTo3D = new JButton(buttonConvertTo3DStr);
        buttonConvertTo3D.setBounds(xSideBar, nextAvailableSpace, wSideBar, hSideBar);
        //buttonConvertTo3D.setEnabled(false);

        nextAvailableSpace += buttonConvertTo3D.getHeight() + spacing;

        text3DCoordinates = new JLabel(text3DcoordinatesStr);
        text3DCoordinates.setBounds(xSideBar, nextAvailableSpace, wSideBar, hSideBar);

        nextAvailableSpace += text3DCoordinates.getHeight() + spacing;

        buttonLaunchDrone = new JButton(buttonLaunchDroneStr);
        buttonLaunchDrone.setBounds(xSideBar, nextAvailableSpace, wSideBar, hSideBar);
        buttonLaunchDrone.setEnabled(false);


        // Add all the texts/buttons to the interface
        add(textureImage);

        add(textChooseFile);
        add(buttonChooseFile);

        add(textSelectPoint);
        add(textCurrentPoint);
        add(textAssociatedObj);

        add(buttonConvertTo3D);
        add(text3DCoordinates);

        add(buttonLaunchDrone);


        setListeners();

        setLayout(null);
        setDefaultCloseOperation(WindowConstants.EXIT_ON_CLOSE);
        setLocationRelativeTo(null);
        setSize(wWidth, wHeight);
        setVisible(true);
    }

    // Set the listeners up
    private void setListeners() {

        // Add a mouse listener to retrieve the 2D coordinates of the mouse
        // when clicking on the loaded texture
        textureImage.addMouseListener(new MouseAdapter() {
            @Override
            public void mouseClicked(MouseEvent e) {
                super.mouseClicked(e);
                DecimalFormat numberFormat = new DecimalFormat("#0.000");
                textCurrentPoint.setText("(" + numberFormat.format((double) e.getX() / textureImage.getWidth()) + ", "
                        + numberFormat.format((textureImage.getHeight() - (double) e.getY()) / textureImage.getHeight()) + ")");

                // We modify the y coordinate to match the texture file axis orientation
                coordinates2D = new Point2D((double) e.getX() / textureImage.getWidth(), (textureImage.getHeight() - (double) e.getY()) / textureImage.getHeight());
            }
        });

        // Add an action listener for each button

        buttonChooseFile.addActionListener(e -> {

            JFileChooser file = new JFileChooser();
            file.setCurrentDirectory(new File(System.getProperty("user.home")));
            file.addChoosableFileFilter(new FileNameExtensionFilter(null, "jpg", "png"));
            int result = file.showOpenDialog(null);

            if (result == JFileChooser.APPROVE_OPTION) {
                File selectedFile = file.getSelectedFile();
                textureImagePath = selectedFile.getParent();

                textureImage.setIcon(ResizeImage(selectedFile.getAbsolutePath()));

                if (selectedFile.getName().contains(".png") || selectedFile.getName().contains(".jpg")) {
                    if (new File(textureImagePath + "/" + objectFileName).exists()) {
                        textAssociatedObj.setText(textAssociatedObjStrNotFoud);
                    } else {
                        textAssociatedObj.setText(textAssociatedObjStrDefault);
                    }
                } else {
                    textAssociatedObj.setText(textAssociatedObjStrDefault);
                }
            }
        });

        buttonConvertTo3D.addActionListener(e -> converter3D());

        buttonLaunchDrone.addActionListener(e -> {
            String s;

            try {
                Process p = Runtime.getRuntime().exec(droneLauncherCommand + " " +
                        coordinates3DAdjusted.x + " " + coordinates3DAdjusted.y + " " + coordinates3DAdjusted.z);

                BufferedReader stdInput = new BufferedReader(new
                        InputStreamReader(p.getInputStream()));

                BufferedReader stdError = new BufferedReader(new
                        InputStreamReader(p.getErrorStream()));

                while ((s = stdInput.readLine()) != null) {
                    System.out.println(s);
                }

                while ((s = stdError.readLine()) != null) {
                    System.err.println(s);
                }

            } catch (IOException e1) {
                e1.printStackTrace();
            }
        });
    }

    // Resize imageIcon with the same size of the textureFrame
    private ImageIcon ResizeImage(String ImagePath) {
        ImageIcon MyImage = new ImageIcon(ImagePath);
        Image img = MyImage.getImage();
        Image newImg = img.getScaledInstance(textureImage.getWidth(), textureImage.getHeight(), Image.SCALE_SMOOTH);
        return new ImageIcon(newImg);
    }

    // Call the 3D object parser to convert the selected 2D coordinates into 3D coordinates
    private void converter3D() {
        File file = new File(textureImagePath + "/" + objectFileName);
        Parser parser = new Parser(file);

        int closestTextureIndex = parser.closestTexturePoint(coordinates2D);
        //System.out.println("Closest texture index : " + closestTextureIndex);

        int correspondingVertex = parser.correspondingVertexIndex(closestTextureIndex + 1);
        //System.out.println("Corresponding vertex index : " + correspondingVertex);

        coordinates3D = parser.findCoordinates(correspondingVertex);

        // After some research I did not find what went wrong here
        // but the right coordinates in blender are equal to (x, -z, y) (with x, y, z being the coordinates our algorithm return)
        // so I'm just fixing it here for now
        coordinates3DAdjusted = new Point3D(coordinates3D.x, (-coordinates3D.z), coordinates3D.y);

        if (coordinates3D != null) {
            //System.out.println("The 3D coordinates are " + coordinates3DAdjusted.toString());
            text3DCoordinates.setText(coordinates3DAdjusted.toString());
            buttonLaunchDrone.setEnabled(true);
        }
    }
}