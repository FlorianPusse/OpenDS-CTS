package settingscontroller_client.src.Controllers.OpenDSConnection;

import java.io.IOException;
import java.io.OutputStream;
import java.io.UnsupportedEncodingException;
import java.util.Random;

import static settingscontroller_client.src.Controllers.OpenDSConnection.MessageBuilder.*;
import static settingscontroller_client.src.Controllers.OpenDSConnection.MessageBuilder.buildNextScenarioMessage;
import static settingscontroller_client.src.Controllers.OpenDSConnection.MessageBuilder.buildSceneResetMessage;

/** Sends messages to Opends **/
public class MessageSender {

    /**
     * Sends the initial message to OpenDS
     */
    public static void sendInitMessage(OutputStream out, int interval) {
        byte[] msg;
        try {
            msg = buildStartUpMessage(interval).getBytes("UTF-8");
            out.write(msg);
            out.flush();

            Thread.sleep(100);
            msg = buildMessage("drivingCar", 0, 0f, 0f, 0).getBytes("UTF-8");
            out.write(msg);
            out.flush();
        } catch (IOException | InterruptedException e) {
            e.printStackTrace();
        }
    }

    /**
     * Continues to the next scene
     */
    public static void sendNextSceneMessage(OutputStream out) {
        byte[] msg;
        try {
            msg = buildNextSceneMessage().getBytes("UTF-8");
            out.write(msg);
            out.flush();
        } catch (UnsupportedEncodingException e) {
            e.printStackTrace();
            return;
        } catch (IOException e) {
            e.printStackTrace();
            return;
        }
        try {
            Thread.sleep(50);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    /**
     * Resets the car position
     */
    public static void sendResetMessage(OutputStream out) {
        byte[] msg;
        try {
            msg = buildResetMessage().getBytes("UTF-8");
            out.write(msg);
            out.flush();
        } catch (UnsupportedEncodingException e) {
            e.printStackTrace();
            return;
        } catch (IOException e) {
            e.printStackTrace();
            return;
        }
        try {
            Thread.sleep(50);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    /**
     * Resets the current scene
     */
    public static void sendSceneResetMessage(OutputStream out) {
        byte[] msg;
        try {
            msg = buildSceneResetMessage().getBytes("UTF-8");
            out.write(msg);
            out.flush();
        } catch (UnsupportedEncodingException e) {
            e.printStackTrace();
            return;
        } catch (IOException e) {
            e.printStackTrace();
            return;
        }
        try {
            Thread.sleep(50);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    /**
     * Switch to the next scenario represented by TRAINING_SET
     */
    public static void sendNextScenarioMessage(OutputStream out, int TRAINING_SET) {
        byte[] msg;
        try {
            msg = buildNextScenarioMessage(TRAINING_SET).getBytes("UTF-8");
            out.write(msg);
            out.flush();
        } catch (UnsupportedEncodingException e) {
            e.printStackTrace();
            return;
        } catch (IOException e) {
            e.printStackTrace();
            return;
        }
        try {
            Thread.sleep(10);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

}
