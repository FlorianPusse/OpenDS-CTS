package settingscontroller_client.src.PathPlanning;

import java.io.*;
import java.util.List;

import static settingscontroller_client.src.Parameters.mapResolution;

/**
 * Represents the non-holonomic without obstacles heuristic of Hybrid A*
 */
public class NonHolonomicWithoutObstacles {

    private static final float goalX = 0;
    private static final float goalZ = 0;
    private static final float goalTheta = 0;

    static short[][][] shortestPaths;

    /**
     * Computes the non-holonomic without obstacles cost
     * @param args
     * @throws IOException
     * @throws ClassNotFoundException
     */
    public static void main(String[] args) throws IOException, ClassNotFoundException {
        int maxX = 400;
        int maxZ = 400;

        float non_holonomic_discretization = 1.0f;

        int numX = (int) Math.ceil(maxX / non_holonomic_discretization);
        int numZ = (int) Math.ceil(maxZ / non_holonomic_discretization);
        int numOrientations = 72;

        shortestPaths = new short[numZ][numX][numOrientations];

        SimpleHybridAStar simpleHybridAStar = new SimpleHybridAStar();

        int c = 0;
        int longestPath = 0;

        long startTime = System.nanoTime();
        for(int iterX = 0; iterX < numX; ++iterX){
            for(int iterZ = 0; iterZ < numZ; ++iterZ){
                for(int iterTheta = 0; iterTheta < numOrientations; ++iterTheta){
                    float x = (iterX * non_holonomic_discretization) / mapResolution;
                    float z = (iterZ * non_holonomic_discretization) / mapResolution;
                    float theta = (float) Math.toRadians(iterTheta*5);
                    SimpleHybridAStar.ContinuousSearchState tmp = simpleHybridAStar.search(x,z,theta,goalX,goalZ,goalTheta,null);
                    List<SimpleHybridAStar.ContinuousSearchState> path = simpleHybridAStar.getPath(tmp);
                    shortestPaths[iterZ][iterX][iterTheta] = (short) path.size();
                    if((short) path.size() > longestPath){
                        longestPath = (short) path.size();
                    }

                    if(c % 1000 == 0){
                        System.out.println(c + ", " + iterX + ", "+ iterZ + ", " + longestPath);
                    }
                    c++;
                }
            }
            if(iterX % 10 == 0){
                System.out.println((iterX / (float) numX) + "% computed");
            }
        }
        long endTime = System.nanoTime();

        System.out.print((endTime - startTime) / 1000000);


        FileOutputStream fos = new FileOutputStream("LearningAssets/hashmap1");
        ObjectOutputStream oos = new ObjectOutputStream(fos);
        oos.writeObject(shortestPaths);
        System.out.println();
    }
}
