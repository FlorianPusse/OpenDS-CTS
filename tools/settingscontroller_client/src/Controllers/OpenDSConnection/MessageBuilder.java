package settingscontroller_client.src.Controllers.OpenDSConnection;

import static settingscontroller_client.src.Parameters.NUM_PEDESTRIAN;

/** Builds the messages used by OpenDS **/
public class MessageBuilder {

    public static String buildStartUpMessage(int interval) {
        String message = "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n" +
                "<Message>\n" +
                "  <Event Name=\"Unsubscribe\">/root/thisVehicle</Event>\n" +
                "  <Event Name=\"Unsubscribe\">/root/pedestrians</Event>\n";

        for (int i = 1; i <= NUM_PEDESTRIAN; ++i) {
            message += "  <Event Name=\"Subscribe\">/root/pedestrians/ped" + i + "/physicalAttributes/Properties/speed</Event>\n" +
                    "  <Event Name=\"Subscribe\">/root/pedestrians/ped" + i + "/physicalAttributes/Properties/position</Event>\n";
        }

        message +=
                "  <Event Name=\"Subscribe\">/root/thisVehicle/physicalAttributes/Properties/orientation</Event>\n" +
                        "  <Event Name=\"Subscribe\">/root/thisVehicle/physicalAttributes/Properties/x</Event>\n" +
                        "  <Event Name=\"Subscribe\">/root/thisVehicle/physicalAttributes/Properties/z</Event>\n" +
                        "  <Event Name=\"Subscribe\">/root/thisVehicle/physicalAttributes/Properties/speed</Event>\n" +
                        "  <Event Name=\"Subscribe\">/root/thisVehicle/physicalAttributes/Properties/isCrossing</Event>\n" +
                        "  <Event Name=\"Subscribe\">/root/obstacles/obstacle1/props</Event>\n" +
                        "  <Event Name=\"SetUpdateInterval\">" + interval + "</Event>\n" +
                        "  <Event Name=\"EstablishConnection\"/>\n" +
                        "</Message>\n";

        return message;
    }

    public static String buildMessage(String id, float steering, float acceleration, float braking) {
        String paramString = id + ";" + steering + ";" + acceleration + ";" + braking;

        return "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n" +
                "<Message>\n" +
                "  <Event Name=\"SetVehicleControl\">" + paramString + "</Event>\n" +
                "  <Event Name=\"EstablishConnection\"/>\n" +
                "</Message>\n";
    }

    public static String buildMessage(String id, float steering, float acceleration, float braking, float targetSpeed) {
        String paramString = id + ";" + steering + ";" + acceleration + ";" + braking + ";" + targetSpeed;

        return "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n" +
                "<Message>\n" +
                "  <Event Name=\"SetVehicleControl\">" + paramString + "</Event>\n" +
                "  <Event Name=\"EstablishConnection\"/>\n" +
                "</Message>\n";
    }

    public static String buildResetMessage() {
        return "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n" +
                "<Message>\n" +
                "  <Event Name=\"SetVehicleControl\">RESET_CAR</Event>\n" +
                "  <Event Name=\"EstablishConnection\"/>\n" +
                "</Message>\n";
    }

    public static String buildSceneResetMessage() {
        return "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n" +
                "<Message>\n" +
                "  <Event Name=\"SetVehicleControl\">RESET_SCENE</Event>\n" +
                "  <Event Name=\"EstablishConnection\"/>\n" +
                "</Message>\n";
    }

    public static String buildNextSceneMessage() {
        return "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n" +
                "<Message>\n" +
                "  <Event Name=\"SetVehicleControl\">NEXT_SCENE</Event>\n" +
                "  <Event Name=\"EstablishConnection\"/>\n" +
                "</Message>\n";
    }

    public static String buildNextScenarioMessage(int id) {
        return "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n" +
                "<Message>\n" +
                "  <Event Name=\"SetVehicleControl\">NEXT_SCENARIO " + id +  "</Event>\n" +
                "  <Event Name=\"EstablishConnection\"/>\n" +
                "</Message>\n";
    }

    static String buildPauseMessage() {
        return "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n" +
                "<Message>\n" +
                "  <Event Name=\"SetVehicleControl\">PAUSE</Event>\n" +
                "  <Event Name=\"EstablishConnection\"/>\n" +
                "</Message>\n";
    }

    static String buildUnPauseMessage() {
        return "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n" +
                "<Message>\n" +
                "  <Event Name=\"SetVehicleControl\">UNPAUSE</Event>\n" +
                "  <Event Name=\"EstablishConnection\"/>\n" +
                "</Message>\n";
    }

}
