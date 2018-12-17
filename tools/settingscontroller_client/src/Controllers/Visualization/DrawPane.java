package settingscontroller_client.src.Controllers.Visualization;

import com.jme3.math.Vector2f;
import org.imgscalr.Scalr;
import settingscontroller_client.src.Controllers.OpenDSConnection.SubscribedValues;
import settingscontroller_client.src.PathPlanning.HybridAStarPhysical;
import settingscontroller_client.src.TrafficObject.Obstacle;
import settingscontroller_client.src.TrafficObject.Pedestrian;

import javax.imageio.ImageIO;
import javax.swing.*;
import java.awt.*;
import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

import static settingscontroller_client.src.Parameters.*;
import static settingscontroller_client.src.Parameters.BELIEF_ANGLE_DISCRETIZATION;
import static settingscontroller_client.src.Parameters.MAX_PATH_STEPS;
import static settingscontroller_client.src.Util.MathUtil.toAngle;
import static settingscontroller_client.src.Util.Util.drawCenteredCircle;

/**
 * Pane used to visualize the current POMDP
 */
public class DrawPane extends JPanel {
    /**
     * Background image at 1/5 scale
     */
    private static BufferedImage background = null;

    /**
     * Background image at 1/5 scale
     */
    private static BufferedImage background2 = null;

    /**
     * Background image
     */
    private static BufferedImage backgroundOriginal = null;

    /**
     * Current path to the goal
     */
    private List<HybridAStarPhysical.ContinuousSearchState> path = null;

    /**
     * Current pedestrian state
     */
    private List<Pedestrian> pedestrians = null;

    /**
     * Path traversed by the car
     */
    private LinkedList<Point2D.Float> latestTrajectory = new LinkedList<>();

    /**
     * Obstacle in the scene
     */
    private Obstacle obstacle = null;

    /**
     * Last position of the car
     */
    Point2D.Float lastPos = null;

    /**
     * Belief of the car. Only present in APPL based methods
     */
    Map<Integer,List<Double>> currentBelief = null;

    /**
     * X-coordinate of the goal position
     */
    private float goalX;

    /**
     * Z-coordinate of the goal position
     */
    private float goalZ;

    public DrawPane() {

        try {
            int dWidth = (int) Math.ceil(map_width / 5);
            int dHeight = (int) Math.ceil(map_height / 5);

            backgroundOriginal = ImageIO.read(new File("LearningAssets/map.jpg"));
            background = Scalr.resize(backgroundOriginal, Scalr.Method.QUALITY, dWidth, dHeight);

            dWidth = (int) Math.ceil(map_width / 2);
            dHeight = (int) Math.ceil(map_height / 2);

            background2 = Scalr.resize(backgroundOriginal, Scalr.Method.QUALITY, dWidth, dHeight);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    /**
     * Resets the traversed trajectory
     */
    public void reset() {
        latestTrajectory = new LinkedList<>();
    }

    /**
     * Paints the POMDP on a graphics object g
     */
    public void paintComponent(Graphics g) {
        super.paintComponent(g);

        Graphics2D g2 = (Graphics2D) g;

        int scalingFactor = 5;

        if (path != null) {
            float x = path.get(0).x;
            float z = path.get(0).z;
            g.drawImage(background, 0, 0, null);

            g.setColor(Color.DARK_GRAY);
            drawCenteredCircle(g2, goalX / scalingFactor, goalZ / scalingFactor, (float) ((R_GOAL + 3 / mapResolution) / scalingFactor));

            g.setColor(Color.PINK);
            List<float[]> f = HybridAStarPhysical.getCornerPositions(x, z, path.get(0).theta);

            g2.setStroke(new BasicStroke(2));

            g2.draw(new Line2D.Float(f.get(0)[0] / scalingFactor, f.get(0)[1] / scalingFactor, f.get(1)[0] / scalingFactor, f.get(1)[1] / scalingFactor));
            g2.draw(new Line2D.Float(f.get(1)[0] / scalingFactor, f.get(1)[1] / scalingFactor, f.get(2)[0] / scalingFactor, f.get(2)[1] / scalingFactor));
            g2.draw(new Line2D.Float(f.get(2)[0] / scalingFactor, f.get(2)[1] / scalingFactor, f.get(3)[0] / scalingFactor, f.get(3)[1] / scalingFactor));
            g2.draw(new Line2D.Float(f.get(3)[0] / scalingFactor, f.get(3)[1] / scalingFactor, f.get(0)[0] / scalingFactor, f.get(0)[1] / scalingFactor));

            if (obstacle != null) {
                g2.setColor(Color.GRAY);
                f = HybridAStarPhysical.getCornerPositionsIncreased(obstacle.x, obstacle.z, obstacle.theta);
                g2.draw(new Line2D.Float(f.get(0)[0] / scalingFactor, f.get(0)[1] / scalingFactor, f.get(1)[0] / scalingFactor, f.get(1)[1] / scalingFactor));
                g2.draw(new Line2D.Float(f.get(1)[0] / scalingFactor, f.get(1)[1] / scalingFactor, f.get(2)[0] / scalingFactor, f.get(2)[1] / scalingFactor));
                g2.draw(new Line2D.Float(f.get(2)[0] / scalingFactor, f.get(2)[1] / scalingFactor, f.get(3)[0] / scalingFactor, f.get(3)[1] / scalingFactor));
                g2.draw(new Line2D.Float(f.get(3)[0] / scalingFactor, f.get(3)[1] / scalingFactor, f.get(0)[0] / scalingFactor, f.get(0)[1] / scalingFactor));

                g.setColor(Color.DARK_GRAY);
                f = HybridAStarPhysical.getCornerPositions(obstacle.x, obstacle.z, obstacle.theta);
                g2.draw(new Line2D.Float(f.get(0)[0] / scalingFactor, f.get(0)[1] / scalingFactor, f.get(1)[0] / scalingFactor, f.get(1)[1] / scalingFactor));
                g2.draw(new Line2D.Float(f.get(1)[0] / scalingFactor, f.get(1)[1] / scalingFactor, f.get(2)[0] / scalingFactor, f.get(2)[1] / scalingFactor));
                g2.draw(new Line2D.Float(f.get(2)[0] / scalingFactor, f.get(2)[1] / scalingFactor, f.get(3)[0] / scalingFactor, f.get(3)[1] / scalingFactor));
                g2.draw(new Line2D.Float(f.get(3)[0] / scalingFactor, f.get(3)[1] / scalingFactor, f.get(0)[0] / scalingFactor, f.get(0)[1] / scalingFactor));
            }

            g.setColor(Color.RED);
            drawCenteredCircle(g2, x / scalingFactor, z / scalingFactor, 4);

            for (int i = 0; i < path.size() - 1; ++i) {
                HybridAStarPhysical.ContinuousSearchState x1 = path.get(i);
                HybridAStarPhysical.ContinuousSearchState x2 = path.get(i + 1);
                g2.draw(new Line2D.Float(x1.x / scalingFactor, x1.z / scalingFactor, x2.x / scalingFactor, x2.z / scalingFactor));
            }

            g.setColor(Color.GREEN);
            for (int i = 0; i < latestTrajectory.size() - 2; ++i) {
                Point2D.Float x1 = latestTrajectory.get(i);
                Point2D.Float x2 = latestTrajectory.get(i + 1);
                g2.draw(new Line2D.Float(x1.x / scalingFactor, x1.y / scalingFactor, x2.x / scalingFactor, x2.y / scalingFactor));
            }

            g2.setStroke(new BasicStroke(1));
        }

        g.setColor(Color.BLUE);
        if (pedestrians != null) {
            g2.setStroke(new BasicStroke(6));
            int i = 0;
            for (Pedestrian p : pedestrians) {
                drawCenteredCircle(g2, (float) p.x / scalingFactor, (float) p.z / scalingFactor, 5);

                if (currentBelief != null) {
                    for (int angleIndex = 0; angleIndex < currentBelief.get(i).size() - 1; ++angleIndex) {
                        double belief = currentBelief.get(i).get(angleIndex);
                        Vector2f direction = toAngle(angleIndex * BELIEF_ANGLE_DISCRETIZATION, belief * 1000.0);
                        g2.draw(new Line2D.Double(p.x / scalingFactor, p.z / scalingFactor, (p.x + direction.x) / scalingFactor, (p.z + direction.y) / scalingFactor));
                    }
                }
                ++i;
            }
        }
    }

    /**
     * Paints the POMDP on a graphics object g with a given scaling factor
     */
    public void paintComponent(Graphics g, int scalingFactor) {
        super.paintComponent(g);

        Graphics2D g2 = (Graphics2D) g;

        if (path != null) {
            float x = path.get(0).x;
            float z = path.get(0).z;
            g.drawImage(background2, 0, 0, null);

            g.setColor(Color.DARK_GRAY);
            drawCenteredCircle(g2, goalX / scalingFactor, goalZ / scalingFactor, (float) ((R_GOAL + 3 / mapResolution) / scalingFactor));

            g.setColor(Color.PINK);
            List<float[]> f = HybridAStarPhysical.getCornerPositions(x, z, path.get(0).theta);

            g2.setStroke(new BasicStroke(4));

            g2.draw(new Line2D.Float(f.get(0)[0] / scalingFactor, f.get(0)[1] / scalingFactor, f.get(1)[0] / scalingFactor, f.get(1)[1] / scalingFactor));
            g2.draw(new Line2D.Float(f.get(1)[0] / scalingFactor, f.get(1)[1] / scalingFactor, f.get(2)[0] / scalingFactor, f.get(2)[1] / scalingFactor));
            g2.draw(new Line2D.Float(f.get(2)[0] / scalingFactor, f.get(2)[1] / scalingFactor, f.get(3)[0] / scalingFactor, f.get(3)[1] / scalingFactor));
            g2.draw(new Line2D.Float(f.get(3)[0] / scalingFactor, f.get(3)[1] / scalingFactor, f.get(0)[0] / scalingFactor, f.get(0)[1] / scalingFactor));

            if (obstacle != null) {
                g.setColor(Color.DARK_GRAY);
                f = HybridAStarPhysical.getCornerPositions(obstacle.x, obstacle.z, obstacle.theta);
                g2.draw(new Line2D.Float(f.get(0)[0] / scalingFactor, f.get(0)[1] / scalingFactor, f.get(1)[0] / scalingFactor, f.get(1)[1] / scalingFactor));
                g2.draw(new Line2D.Float(f.get(1)[0] / scalingFactor, f.get(1)[1] / scalingFactor, f.get(2)[0] / scalingFactor, f.get(2)[1] / scalingFactor));
                g2.draw(new Line2D.Float(f.get(2)[0] / scalingFactor, f.get(2)[1] / scalingFactor, f.get(3)[0] / scalingFactor, f.get(3)[1] / scalingFactor));
                g2.draw(new Line2D.Float(f.get(3)[0] / scalingFactor, f.get(3)[1] / scalingFactor, f.get(0)[0] / scalingFactor, f.get(0)[1] / scalingFactor));
            }

            g.setColor(Color.RED);
            drawCenteredCircle(g2, x / scalingFactor, z / scalingFactor, 6);

            for (int i = 0; i < path.size() - 1; ++i) {
                HybridAStarPhysical.ContinuousSearchState x1 = path.get(i);
                HybridAStarPhysical.ContinuousSearchState x2 = path.get(i + 1);
                g2.draw(new Line2D.Float(x1.x / scalingFactor, x1.z / scalingFactor, x2.x / scalingFactor, x2.z / scalingFactor));
            }

            g.setColor(Color.GREEN);
            for (int i = 0; i < latestTrajectory.size() - 2; ++i) {
                Point2D.Float x1 = latestTrajectory.get(i);
                Point2D.Float x2 = latestTrajectory.get(i + 1);
                g2.draw(new Line2D.Float(x1.x / scalingFactor, x1.y / scalingFactor, x2.x / scalingFactor, x2.y / scalingFactor));
            }

            g2.setStroke(new BasicStroke(2));
        }

        if (pedestrians != null) {
            g2.setStroke(new BasicStroke(6));
            g.setColor(Color.BLUE);

            int i = 0;
            for (Pedestrian p : pedestrians) {
                drawCenteredCircle(g2, (float) p.x / scalingFactor, (float) p.z / scalingFactor, 10);

                if (currentBelief != null) {
                    for (int angleIndex = 0; angleIndex < currentBelief.get(i).size() - 1; ++angleIndex) {
                        double belief = currentBelief.get(i).get(angleIndex);
                        Vector2f direction = toAngle(angleIndex * BELIEF_ANGLE_DISCRETIZATION, belief * 1000.0);
                        g2.draw(new Line2D.Double(p.x / scalingFactor, p.z / scalingFactor, (p.x + direction.x) / scalingFactor, (p.z + direction.y) / scalingFactor));
                    }
                }
                ++i;
            }
        }
    }

    /**
     * Updates the current path of the car
     * @param path Path of the car
     */
    public void updatePath(List<HybridAStarPhysical.ContinuousSearchState> path) {
        this.path = path;
        removeAll();
        repaint();
    }

    /**
     * Updates the current path of the car, state of pedestrians, and obstacle
     * @param path Path of the car
     * @param parsedValue state of the peds and the obstacle
     */
    public void updatePath(List<HybridAStarPhysical.ContinuousSearchState> path, SubscribedValues parsedValue) {
        Point2D.Float newPos = new Point2D.Float(path.get(0).x, path.get(0).z);

        if (lastPos == null || lastPos.distance(newPos) >= 1) {
            latestTrajectory.add(newPos);
            if (latestTrajectory.size() >= MAX_PATH_STEPS) {
                latestTrajectory.poll();
            }
        }
        lastPos = newPos;

        this.obstacle = parsedValue.obstacle;
        this.pedestrians = parsedValue.pedestrians;
        this.updatePath(path);
    }

    /**
     * Updates the belief of the car over ped goal
     * @param currentBelief Belief of the car over ped goal
     */
    public void updateBelief(Map<Integer,List<Double>> currentBelief){
        this.currentBelief = currentBelief;
    }

    /**
     * Updates the goal position of the car
     * @param x X-coordinate of the car
     * @param z Z-coordinate of the car
     */
    public void updateGoal(float x, float z){
        this.goalX = x;
        this.goalZ = z;
    }
}