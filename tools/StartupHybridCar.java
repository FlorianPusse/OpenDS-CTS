import eu.opends.main.Simulator;
import settingscontroller_client.src.Controllers.HybridController;
import settingscontroller_client.src.Evaluation.Config;
import settingscontroller_client.src.Evaluation.ScenarioConfig;

import java.io.*;

import static eu.opends.main.DataSet.NUM_SETS;

public class StartupHybridCar {

    static Config.ExperimentType type = Config.ExperimentType.SPEED_DISTANCE;
    static Config.SimulationMode mode = Config.SimulationMode.TESTING;
    static int TRAINING_SET = ScenarioConfig.toTrainingSet(ScenarioConfig.PedestrianSide.RIGHT, ScenarioConfig.Intersection.NONE, false);
    static ScenarioConfig.ObstaclePositions obstaclePosition = ScenarioConfig.ObstaclePositions.NONE;

    static boolean headless = false;
    static Process[] openDSProcesses = new Process[1];
    static int startingInstance = 0;
    static int START_PORT = 5000;

    public void destroy(){
        for(Process p : openDSProcesses){
            if(p != null){
                p.destroy();
            }
        }
    }

    public static void main(String[] args){
        float param1 = 18;
        float param2 = 0;

        if(args.length >= 1){
            if (args[0].equals("headless")){
                headless = true;
            }
        }

        if(args.length >= 3){

        }

        if(args.length >= 5){
            param1 = Integer.parseInt(args[3]);
            param2 = Integer.parseInt(args[4]);
        }

        if (1 > 1 || headless){
            Simulator.isHeadLess = true;
        }

        ByteArrayOutputStream baos = new ByteArrayOutputStream();
        PrintStream ps = new PrintStream(baos);
        PrintStream original = System.out;

        System.setOut(ps);

        for(int i = 0; i < 1; ++i){
            original.print("Starting instance " + (i+1) + "...\n");

            int startingSet = ((i + startingInstance) % (NUM_SETS));
            if(mode == Config.SimulationMode.TESTING){
                startingSet = TRAINING_SET;
            }
            Simulator.main(new String[]{String.valueOf(param1),String.valueOf(param2),String.valueOf(START_PORT + i), String.valueOf(startingSet), obstaclePosition.name(), mode.name(), type.name()});

            while(!baos.toString().contains("SettingsControllerServer")){
                try {
                    Thread.sleep(10);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
            baos = new ByteArrayOutputStream();
            ps = new PrintStream(baos);
            System.setOut(ps);

            original.println("Done. Process " + (i+1) + " started.");
        }

        System.setOut(original);

        System.out.print("Start Path planner ...\n");

        HybridController[] controllers = new HybridController[1];

        for(int i = 0; i < 1; ++i){
            System.out.println("Start controller " + (i+1));
            controllers[i] = new HybridController();
            try {
                boolean isHeadless = headless || (i > 5);
                int startingSet = ((i + startingInstance) % (NUM_SETS));
                if(mode == Config.SimulationMode.TESTING){
                    startingSet = TRAINING_SET;
                }
                controllers[i].initController(param1,param2, START_PORT + i,START_PORT + 1 + i, mode, type, startingSet, isHeadless);
            } catch (IOException e) {
                e.printStackTrace();
            }
        }

        System.out.println("... Controllers started");

        for(int i = 0; i < 1; ++i){
            try {

                ProcessBuilder pb = new ProcessBuilder("ISDESPOT/smart-car-sim-master/is-despot/problems/hybridVisual_car/car",String.valueOf(START_PORT + 1 + i));
                //pb.redirectErrorStream(true);

                Process process = pb.start();

                Thread outThread = new Thread(() -> {
                    try (BufferedReader reader = new BufferedReader(new InputStreamReader(process.getInputStream()))) {
                        char[] buf = new char[1024];
                        int read = 0;

                        while (read != -1) {
                            read = reader.read(buf);
                            if(read == 0){
                                Thread.sleep(50);
                            }
                        }
                    } catch (Exception e) {
                    }
                });
                outThread.start();


                Thread outThread1 = new Thread(() -> {
                    try (BufferedReader reader = new BufferedReader(new InputStreamReader(process.getErrorStream()))) {
                        String line;

                        while ((line = reader.readLine()) != null) {
                            System.err.println(line);
                        }
                    } catch (Exception e) {
                    }
                });
                outThread1.start();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }

        System.out.println("... ISDESPOT started");

        while(true){
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

}
