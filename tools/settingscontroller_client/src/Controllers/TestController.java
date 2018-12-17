package settingscontroller_client.src.Controllers;

import settingscontroller_client.src.Actions.AbstractAction;
import settingscontroller_client.src.Actions.SimpleAction;
import settingscontroller_client.src.Controllers.OpenDSConnection.SubscribedValues;
import java.io.IOException;

import static settingscontroller_client.src.AccelerationType.ACCELERATE;


public class TestController extends AbstractController {

    @Override
    public AbstractAction chooseAction(SubscribedValues currentObservation, float plannedAngle){
        return new SimpleAction(plannedAngle, ACCELERATE);
    }

    public static void main(String[] args) throws IOException {
        new TestController().initController();

        while (true) {
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}
