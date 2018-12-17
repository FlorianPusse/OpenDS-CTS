package settingscontroller_client.src.Controllers.OpenDSConnection;

import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.NodeList;
import org.xml.sax.InputSource;
import settingscontroller_client.src.Controllers.AbstractController;
import settingscontroller_client.src.Controllers.HybridController;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import java.io.DataInputStream;
import java.io.StringReader;
import java.net.SocketException;

import static settingscontroller_client.src.Controllers.OpenDSConnection.SubscribedValues.parseSubscribedValues;
import static settingscontroller_client.src.Parameters.map_height;
import static settingscontroller_client.src.Parameters.map_width;

/** Receives messages from OpenDS and processes them **/
public class MessageReceiver implements Runnable {

    DataInputStream in;
    AbstractController abstractController;

    byte[] buf = new byte[1024];
    String bufferedString = "";

    public MessageReceiver(DataInputStream in, AbstractController abstractController){
        this.in = in;
        this.abstractController = abstractController;
    }

    @Override
    public void run() {
        while (true) {
            try {
                String messageValue = "";

                try {
                    while (true) {
                        int nAvailable = in.available();

                        if (nAvailable > 0) {
                            int nRead = in.read(buf, 0, buf.length);
                            if (nRead == -1) {
                                System.out.println("Connection closed by server.");
                                Thread.sleep(5000);
                                break;
                            }

                            String contentRead = new String(buf, 0, nRead);
                            bufferedString += contentRead;
                        } else {
                            if (bufferedString.trim().endsWith("</Message>")) {
                                messageValue = bufferedString.substring(bufferedString.lastIndexOf("<Message>"));
                                bufferedString = "";
                                break;
                            }
                            Thread.sleep(50);
                        }
                    }
                } catch (SocketException e) {
                    System.out.println("Connection closed by server.");
                    break;
                }

                if (!messageValue.equals("")) {
                    Document document = loadXMLFromString(messageValue);
                    document.getDocumentElement().normalize();

                    Element rootElement = document.getDocumentElement();
                    if (rootElement.getTagName().equals("Message")) {
                        NodeList eventList = rootElement.getElementsByTagName("Event");
                        if (eventList.getLength() > 0) {
                            Element event = (Element) eventList.item(0);
                            if (event.getAttribute("Name").equals("SubscribedValues")) {
                                SubscribedValues parsedValue = parseSubscribedValues(event);
                                abstractController.obstacle = parsedValue.obstacle;

                                if (parsedValue != null && (parsedValue.x > 0 && parsedValue.z > 0 && parsedValue.x < map_width && parsedValue.z < map_height)) {
                                    abstractController.setSubscribedValues(parsedValue);
                                }
                            }
                        }

                    }
                }

            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }

    static Document loadXMLFromString(String xml) throws Exception {
        DocumentBuilderFactory factory = DocumentBuilderFactory.newInstance();
        DocumentBuilder builder = factory.newDocumentBuilder();
        InputSource is = new InputSource(new StringReader(xml));
        return builder.parse(is);
    }
}