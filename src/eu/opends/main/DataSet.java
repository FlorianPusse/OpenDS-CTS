package eu.opends.main;

import com.jme3.math.Quaternion;

import java.awt.geom.Point2D;
import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;

public class DataSet {

    static String path0 = "assets/DrivingTasks/Projects/EisenbahnNewTrainingData/paris.xml";
    static String path1 = "assets/DrivingTasks/Projects/EisenbahnNewTrainingData1/paris.xml";
    static String path2 = "assets/DrivingTasks/Projects/EisenbahnNewTrainingData2/paris.xml";
    static String path3 = "assets/DrivingTasks/Projects/EisenbahnNewTrainingData3/paris.xml";
    static String path4 = "assets/DrivingTasks/Projects/EisenbahnNewTrainingData4/paris.xml";
    static String path5 = "assets/DrivingTasks/Projects/EisenbahnNewTrainingData5/paris.xml";
    static String path6 = "assets/DrivingTasks/Projects/EisenbahnNewTrainingData6/paris.xml";
    static String path7 = "assets/DrivingTasks/Projects/EisenbahnNewTrainingData7/paris.xml";
    static String path8 = "assets/DrivingTasks/Projects/EisenbahnNewTrainingData8/paris.xml";

    static String testPath = "assets/DrivingTasks/Projects/EisenbahnNewTestData/paris.xml";
    static String testPath1 = "assets/DrivingTasks/Projects/EisenbahnNewTestData2/paris.xml";
    static String testPath2 = "assets/DrivingTasks/Projects/EisenbahnNewTrainingData/paris.xml";
    static String testPath3 = "assets/DrivingTasks/Projects/EisenbahnNewTestData3/paris.xml";
    static String testPath4 = "assets/DrivingTasks/Projects/EisenbahnNewTestData4/paris.xml";
    static String testPath5 = "assets/DrivingTasks/Projects/EisenbahnNewTestData5/paris.xml";

    static String multiplePedsLeftRight = "assets/DrivingTasks/Projects/EisenbahnNewTestDataMultiplePedestriansLeftRight/paris.xml";
    static String approachingIntersecLeft = "assets/DrivingTasks/Projects/EisenbahnNewTrainingDataApproachingIntersecLeft/paris.xml";
    static String approachingIntersecRight = "assets/DrivingTasks/Projects/EisenbahnNewTrainingDataApproachingIntersecRight/paris.xml";
    static String leavingIntersec = "assets/DrivingTasks/Projects/EisenbahnNewTrainingDataLeavingIntersec/paris.xml";

    static String threePedsAllLeft = "assets/DrivingTasks/Projects/EisenbahnNewTestDataThreePedsRight/paris.xml";
    static String fourPedsLeftRight = "assets/DrivingTasks/Projects/EisenbahnNewTestDataMultipleFourLeftRight/paris.xml";


    static String[] paths = new String[]{path0,path1,path2,path3,path4,path5,path6,path7,path8,testPath, testPath1, testPath2, testPath3, testPath3, testPath4, testPath2, testPath5, multiplePedsLeftRight,
            approachingIntersecLeft, approachingIntersecRight, leavingIntersec, threePedsAllLeft, fourPedsLeftRight};
    public static final int NUM_SETS = paths.length;
    public static final int NUM_TRAINING_SETS = 16;

    public static Set<Integer> MULTIPLE_PEDESTRIAN_SETS = new HashSet<>(Arrays.asList(10, 17, 21, 22));

    /* First part of map*/
    static Point2D.Float[] startingPos0 = new Point2D.Float[]{
            new Point2D.Float(-322.29904f, 3.122442f), new Point2D.Float(-311.35413f, -26.293238f)};
    static Quaternion startingOrientation0 =
            new Quaternion(-0.0012575171f, -0.1761959f, -2.5488864E-4f, 0.9843543f);

    /* Second part of map*/
    static Point2D.Float[] startingPos1 = new Point2D.Float[]{
            new Point2D.Float(-296.58386f, -72.15636f), new Point2D.Float(-284.14148f, -100.57097f)
    };
    static Quaternion startingOrientation1 = new Quaternion(-0.0012523904f, -0.20483445f, -2.9179914E-4f, 0.97879577f);

    /* Third part of map*/
    static Point2D.Float[] startingPos2 = new Point2D.Float[]{
            new Point2D.Float(-256.11823f, -162.37689f), new Point2D.Float(-251.98824f, -171.29654f)
    };
    static Quaternion startingOrientation2 = new Quaternion(-0.0012493747f, -0.21467808f, -3.0455622E-4f, 0.97668403f);

    /* Car turning part of map*/
    static Point2D.Float[] startingPos3 = new Point2D.Float[]{
            new Point2D.Float(-305.9476f, -40.451244f), new Point2D.Float(-305.9476f, -40.451244f)
    };
    static Quaternion startingOrientation3 = new Quaternion(-0.0012595586f, -0.17924128f, -2.5918364E-4f, 0.9838043f);

    /* Car approaching intersection*/
    static Point2D.Float[] startingPos4 = new Point2D.Float[]{
            new Point2D.Float(-315.182f, -16.1598f), new Point2D.Float(-315.182f, -16.1598f)
    };
    static Quaternion startingOrientation4 = new Quaternion(-0.0012595586f, -0.17924128f, -2.5918364E-4f, 0.9838043f);

    /* Car leaving intersection*/
    static Point2D.Float[] startingPos5 = new Point2D.Float[]{
            new Point2D.Float(-273.84976f, -125.309296f), new Point2D.Float(-273.84976f, -125.309296f)
    };
    static Quaternion startingOrientation5 = new Quaternion(-0.0012595586f, -0.17924128f, -2.5918364E-4f, 0.9838043f);


    static Quaternion obstacleRotation = new Quaternion(-0.0012566609f, -0.17946866f, -2.5896847E-4f, 0.98376286f);
}



