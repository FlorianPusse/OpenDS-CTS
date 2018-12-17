import eu.opends.main.Simulator;
import settingscontroller_client.src.Actions.DRLAction;
import settingscontroller_client.src.Controllers.DiscretizedController;
import settingscontroller_client.src.Evaluation.Config;
import settingscontroller_client.src.Evaluation.ScenarioConfig;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.PrintStream;

import static eu.opends.main.DataSet.NUM_SETS;

public class StartupNavA3Cp {

    static int MAX_INSTANCES = 1;
    static Config.ExperimentType type = Config.ExperimentType.SPEED_DISTANCE;
    static Config.SimulationMode mode = Config.SimulationMode.TESTING;
    static int TRAINING_SET = ScenarioConfig.toTrainingSet(ScenarioConfig.PedestrianSide.RIGHT, ScenarioConfig.Intersection.NONE, false);
    static ScenarioConfig.ObstaclePositions obstaclePosition = ScenarioConfig.ObstaclePositions.NONE;

    static boolean headless = false;
    static Process[] openDSProcesses = new Process[MAX_INSTANCES];
    static int startingInstance = 0;
    static int START_PORT = 4000;
    static boolean setSimple = false;

    public void destroy(){
        for(Process p : openDSProcesses){
            if(p != null){
                p.destroy();
            }
        }
    }

    public static void main(String[] args){
        float param1 = 0;
        float param2 = 0;

        if(args.length >= 1){
            if (args[0].equals("headless")){
                headless = true;
            }
        }

        if(args.length >= 2){
            MAX_INSTANCES = Integer.parseInt(args[1]);
        }

        if(args.length >= 3){

        }

        if(args.length >= 5){
            param1 = Integer.parseInt(args[3]);
            param2 = Integer.parseInt(args[4]);
        }

        if (MAX_INSTANCES > 1){
            Simulator.isHeadLess = true;
        }

        ByteArrayOutputStream baos = new ByteArrayOutputStream();
        PrintStream ps = new PrintStream(baos);
        PrintStream original = System.out;

        System.setOut(ps);

        for(int i = 0; i < MAX_INSTANCES; ++i){
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

        if(setSimple){
            System.out.println("Set action type to simple");
            DRLAction.type = DRLAction.SteeringType.SIMPLE;
        }

        DiscretizedController[] controllers = new DiscretizedController[MAX_INSTANCES];

        for(int i = 0; i < MAX_INSTANCES; ++i){
            System.out.println("Start controller " + (i+1));
            controllers[i] = new DiscretizedController();
            try {
                boolean isHeadless = headless || (i > 5);
                int startingSet = ((i + startingInstance) % (NUM_SETS));
                if(mode == Config.SimulationMode.TESTING){
                    startingSet = TRAINING_SET;
                }
                controllers[i].initController(param1,param2, START_PORT + i,START_PORT + MAX_INSTANCES + i, mode, type, startingSet, isHeadless);
            } catch (IOException e) {
                e.printStackTrace();
            }
        }

        System.out.println("... Controllers started");

        while(true){
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

}
