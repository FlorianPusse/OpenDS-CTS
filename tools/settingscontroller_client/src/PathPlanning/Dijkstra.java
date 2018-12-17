package settingscontroller_client.src.PathPlanning;

import com.jme3.math.FastMath;
import java.awt.*;
import java.awt.geom.Point2D;
import java.awt.image.BufferedImage;
import java.io.*;
import java.util.*;
import java.util.List;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.locks.ReentrantLock;

import static settingscontroller_client.src.Parameters.mapResolution;
import static settingscontroller_client.src.Parameters.map_height;
import static settingscontroller_client.src.Parameters.map_width;
import static settingscontroller_client.src.Util.PGMUtils.readPGMFile;

/**
 * Dijkstra algorithm for computing the holonomic-with-obstacles
 * heuristic of Hybrid A*
 */
@SuppressWarnings("Duplicates")
public class Dijkstra {
    private short[][] costMap;
    private ConcurrentHashMap<SimplePositionEntry,float[][]> holonomicWithObstaclesMaps = new ConcurrentHashMap<>();
    private ReentrantLock lock = new ReentrantLock();
    private HashMap<SimplePositionEntry, ReentrantLock> entryLocks = new HashMap<SimplePositionEntry, ReentrantLock>();

    BufferedImage beliefImage = null;

    public Dijkstra(short[][] costMap){
        this.costMap = costMap;

        int goalX = Math.round(544);
        int goalZ = Math.round(3333);

        File f = new File("LearningAssets/dijkstraGoals");
        if(f.exists()){
            try {
                FileInputStream fis = new FileInputStream(f);
                ObjectInputStream ois = new ObjectInputStream(fis);
                holonomicWithObstaclesMaps = (ConcurrentHashMap<SimplePositionEntry, float[][]>) ois.readObject();

            } catch (IOException | ClassNotFoundException e) {
                e.printStackTrace();
            }
            System.out.println("Load existing Dijkstra entries.");
        }else {
            f = new File("LearningAssets/holonomicWithObstaclesMap");
            if (f.exists()) {
                try {
                    FileInputStream fis = new FileInputStream(f);
                    ObjectInputStream ois = new ObjectInputStream(fis);

                    SimplePositionEntry newGoalEntry = new SimplePositionEntry((short) goalX, (short) goalZ);
                    holonomicWithObstaclesMaps.put(newGoalEntry, (float[][]) ois.readObject());
                } catch (IOException | ClassNotFoundException e) {
                    e.printStackTrace();
                }
            } else {
                SimplePositionEntry newGoalEntry = new SimplePositionEntry((short) goalX, (short) goalZ);
                float[][] map = newGoal(goalX, goalZ, this.costMap);
                holonomicWithObstaclesMaps.put(newGoalEntry, map);

                try {
                    FileOutputStream fis = new FileOutputStream(f);
                    ObjectOutputStream ois = new ObjectOutputStream(fis);
                    ois.writeObject(map);
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
        }
    }

    class State implements Comparable<State>{
        final short x; // discretized position in x space
        final short z; // discretized position in z space

        float g; // accumulated total cost until to reach this state

        public State(short x, short z){
            this.x = x;
            this.z = z;
        }

        public State(short x, short z, float g){
            this.x = x;
            this.z = z;
            this.g = g;
        }

        private final float sqrt2Approx = 1.414f;

        @Override
        public int compareTo(State s) {
            return (int) Math.signum(g - s.g);
        }

        private class SuccessorResult {
            public State succ;
            public float cost;

            public SuccessorResult(State succ, float cost) {
                this.succ = succ;
                this.cost = cost;
            }
        }

        int[] xDirections = new int[]{ 0, 1,1,1,0,-1,-1,-1};
        int[] zDirections = new int[]{-1,-1,0,1,1, 1, 0,-1};
        List<SuccessorResult> expand(State goalState){

            // initialize list of successors
            List<SuccessorResult> successors = new LinkedList<>();
            short x_prime;
            short z_prime;
            State succ;

            for(int i = 0; i < 8; ++i){
                x_prime = (short) (x + xDirections[i]);
                z_prime = (short) (z + zDirections[i]);
                if(x_prime >= 0 && z_prime >= 0 && x_prime < map_width*mapResolution && z_prime < map_height*mapResolution) {
                    int mapZIndex = (int) Math.floor(z_prime/mapResolution);
                    int mapXIndex = (int) Math.floor(x_prime/mapResolution);

                    int reward = costMap[mapZIndex][mapXIndex];

                    if(beliefImage != null){
                        int tmpReward = new Color(beliefImage.getRGB(mapXIndex, mapZIndex)).getRed();
                        if(tmpReward < reward){
                            reward = tmpReward;
                        }
                    }

                    int obstacleCost = 255 - reward;

                    float cost = (i % 2 == 0 ? 1 : sqrt2Approx) + obstacleCost;

                    succ = new State(x_prime, z_prime, (short) (g + cost));
                    successors.add(new State.SuccessorResult(succ, cost));
                }
            }

            return successors;
        }

        @Override
        public String toString(){
            return "(" + x + ", " + z + "), g:" + g;
        }

        public float distance(State s){
            return FastMath.sqrt(FastMath.pow(x-s.x,2) + FastMath.pow(z-s.z,2));
        }

        @Override
        public boolean equals(Object o) {
            if (this == o) return true;
            if (o == null || getClass() != o.getClass()) return false;
            State state = (State) o;
            return x == state.x &&
                    z == state.z;
        }

        @Override
        public int hashCode() {
            return Objects.hash(x, z);
        }
    }

    float[][] newGoal(float goalX_, float goalZ_, short[][] costMap) {
        this.costMap = costMap;

        int goalX = Math.round(goalX_*mapResolution);
        int goalZ = Math.round(goalZ_*mapResolution);

        State goalState = new State((short) goalX, (short) goalZ);

        float[][] shortestPath = new float[(int) Math.ceil(map_height*mapResolution)][(int) Math.ceil(map_height*mapResolution)];

        shortestPath[goalZ][goalX] = 0;
        PriorityQueue<State> openList = new PriorityQueue<>();
        Set<SimplePositionEntry> openSet = new HashSet<>();

        for(short i = 0; i < shortestPath.length; ++i){
            for(short j = 0; j < shortestPath[0].length; ++j){
                if(i == goalZ && j == goalX){
                    shortestPath[i][j] = 0;
                    openList.add(new State(j,i,0));
                    openSet.add(new SimplePositionEntry(j,i));
                }else{
                    shortestPath[i][j] = Float.POSITIVE_INFINITY;
                    openList.add(new State(j,i,Float.POSITIVE_INFINITY));
                    openSet.add(new SimplePositionEntry(j,i));
                }
            }
        }

        while (!openList.isEmpty()){
            State u = openList.poll();
            openSet.remove(new SimplePositionEntry(u.x,u.z));

            List<State.SuccessorResult> neighbors = u.expand(goalState);
            for(State.SuccessorResult succ : neighbors){
                if(succ.succ.g < shortestPath[succ.succ.z][succ.succ.x]){
                    shortestPath[succ.succ.z][succ.succ.x] = succ.succ.g;

                    if(openSet.contains(new SimplePositionEntry(succ.succ.x,succ.succ.z))){
                        openList.remove(succ.succ);
                        openList.add(succ.succ);
                    }
                }
            }
        }

        return shortestPath;
    }

    public float search(float startX_, float startZ_, short goalX, short goalZ){
        int startX = (int) Math.floor(startX_*mapResolution);
        int startZ = (int) Math.floor(startZ_*mapResolution);

        SimplePositionEntry newGoalEntry = new SimplePositionEntry(goalX, goalZ);
        if(!holonomicWithObstaclesMaps.containsKey(newGoalEntry)){
            ReentrantLock entryLock;
            lock.lock();
                entryLock = entryLocks.get(newGoalEntry);
                if(entryLock == null){
                    entryLock = new ReentrantLock();
                    entryLocks.put(newGoalEntry,entryLock);
                }
            lock.unlock();

            entryLock.lock();
            if(!holonomicWithObstaclesMaps.containsKey(newGoalEntry)){
                holonomicWithObstaclesMaps.put(newGoalEntry,newGoal(goalX,goalZ,costMap));
            }
            entryLock.unlock();
        }

        return holonomicWithObstaclesMaps.get(newGoalEntry)[startZ][startX];
    }

    public static void main(String[] args){
        List<Point2D.Float> possibleGoals1 = new LinkedList<>(
                Arrays.asList(
                        new Point2D.Float(1403.38f, 1445.41f),
                        new Point2D.Float(1015.30f, 2280.85f)
                ));

        short[][] costMap;

        try {
            costMap = readPGMFile("LearningAssets/combinedmapSimpleDiscretized.pgm");
            HybridAStarPhysical.costMap = costMap;
        } catch (FileNotFoundException e) {
            throw new RuntimeException("Costmap not found!");
        }

        Dijkstra dijkstra = new Dijkstra(costMap);

        int numgoals = possibleGoals1.size();
        for (Point2D.Float aPossibleGoals1 : possibleGoals1) {
            waitingElements.add(aPossibleGoals1);
        }

        int numThreads = 2;

        Thread[] threads = new Thread[numThreads];
        for(int i = 0; i < numThreads; ++i){
            Consumer consumer = new Consumer(dijkstra,costMap);
            Thread t = new Thread(consumer);
            t.start();
            threads[i] = t;
        }

        for(int i = 0; i < numThreads; ++i){
            try {
                threads[i].join();
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        File f = new File("LearningAssets/dijkstraGoals");
        try {
            FileOutputStream fis = new FileOutputStream(f);
            ObjectOutputStream ois = new ObjectOutputStream(fis);
            ois.writeObject(dijkstra.holonomicWithObstaclesMaps);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    static ConcurrentLinkedQueue<Point2D.Float> waitingElements = new ConcurrentLinkedQueue<Point2D.Float>();
    static AtomicInteger counter = new AtomicInteger(0);

    public static class Consumer implements Runnable{
        Dijkstra dijkstra;
        short[][] costMap;

        public Consumer(Dijkstra dijkstra, short[][] costMap) {
            this.dijkstra = dijkstra;
            this.costMap = costMap;
        }

        @Override
        public void run() {
            while(true){
                Point2D.Float entry = waitingElements.poll();
                if(entry == null){
                    break;
                }

                SimplePositionEntry newGoalEntry = new SimplePositionEntry((short) entry.x,(short) entry.y);

                float[][] map = dijkstra.newGoal(entry.x, entry.y, costMap);
                dijkstra.holonomicWithObstaclesMaps.put(newGoalEntry,map);

                int cValue = counter.incrementAndGet();
                System.out.println(cValue);
            }
        }
    }
}
