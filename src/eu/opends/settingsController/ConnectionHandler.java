/*
*  This file is part of OpenDS (Open Source Driving Simulator).
*  Copyright (C) 2016 Rafael Math
*
*  OpenDS is free software: you can redistribute it and/or modify
*  it under the terms of the GNU General Public License as published by
*  the Free Software Foundation, either version 3 of the License, or
*  (at your option) any later version.
*
*  OpenDS is distributed in the hope that it will be useful,
*  but WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*  GNU General Public License for more details.
*
*  You should have received a copy of the GNU General Public License
*  along with OpenDS. If not, see <http://www.gnu.org/licenses/>.
*/

package eu.opends.settingsController;

import java.io.DataInputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.io.StringReader;
import java.net.SocketException;
import java.nio.charset.Charset;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;

import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.NodeList;
import org.xml.sax.InputSource;

import com.jme3.math.FastMath;

import eu.opends.main.Simulator;
import eu.opends.traffic.Pedestrian;
import eu.opends.traffic.TrafficCar;
import eu.opends.traffic.TrafficObject;

/**
 *
 * @author Daniel Braun
 */
public class ConnectionHandler extends Thread
{
	private Simulator sim;
	private OutputStream out;
	private DataInputStream in;
	private UpdateSender updateSender;
	private APIData data;

	private int updateInterval = 1000; //in ms

	private Lock intervalLock = new ReentrantLock();


	public static int byteArrToInt(byte[] b){
		int value = 0;

		for (int i = 0; i < b.length; i++)
   	 	{
			value += ((long) b[i] & 0xffL) << (8 * i);
   	 	}

		return value;
	}

	public static String byteArrToStr(byte[] b){
		Charset charset = Charset.forName("UTF-8");
		int i;
		for (i = 0; i < b.length && b[i] != 0; i++) { }
		String str = new String(b, 0, i, charset);
		return str;
	}

	private static Document loadXMLFromString(String xml) throws Exception
    {
        DocumentBuilderFactory factory = DocumentBuilderFactory.newInstance();
        DocumentBuilder builder = factory.newDocumentBuilder();
        InputSource is = new InputSource(new StringReader(xml));
        return builder.parse(is);
    }

	public ConnectionHandler(Simulator s, OutputStream o, DataInputStream i){
		sim = s;
		out = o;
		in = i;

		data = new APIData(sim.getCar(),sim);
		updateSender = new UpdateSender(data, this, sim);
	}

    public void run() {
        byte[] buf = new byte[1024];
        String bufferedString = "";

        while (!isInterrupted()) {
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
                        }
                    }
                } catch (SocketException e) {
                    System.out.println("Connection closed by server.");
                    interrupt();
                    continue;
                }

                if (!messageValue.equals("")) {
                    parseXML(messageValue);
                }
            } catch (Exception e) {
                e.printStackTrace();
            }
        }

        try {
            out.close();
            updateSender.interrupt();
            in.close();
        } catch (IOException e) {
            e.printStackTrace();
        }

    }

	public int getUpdateInterval(){
		int value;

		intervalLock.lock();
		try{
			value = updateInterval;
		}
		finally{
			intervalLock.unlock();
		}

		return value;
	}

	public void setUpdateInterval(int ui){
		intervalLock.lock();
		try{
			updateInterval = ui;
		}
		finally{
			intervalLock.unlock();
		}
	}

	public void setVehicleControl(String val)
	{
		try{
			if(val.startsWith("NEXT_SCENARIO")){
				sim.TRAINING_SET = Integer.parseInt(val.split(" ")[1]);

				sim.switchScenario(true);

				if(sim.TRAINING_SET > 9){
					sim.nextScene();
				}else{
					sim.getCar().setToRandomResetPosition();
				}

				Thread.sleep(25);
				updateSender.messageReceived.set(true);
			}else if(val.equals("RESET_CAR")){
				if(sim.TRAINING_SET > 9){
					sim.nextScene();
				}else{
					sim.getCar().setToRandomResetPosition();
				}

				Thread.sleep(25);
				updateSender.messageReceived.set(true);
				//System.out.println("[INFO] Reset car.");
			}else if (val.equals("RESET_SCENE")){
				sim.resetScene();
				Thread.sleep(25);
				updateSender.messageReceived.set(true);
				//System.out.println("[INFO] Reset scene.");
			}else if(val.equals("NEXT_SCENE")){
				sim.nextScene();
				Thread.sleep(25);
				updateSender.messageReceived.set(true);
				//System.out.println("[INFO] Next scene.");
			}else{
				//System.out.println("[INFO] Receive steering command.");

				// val must have format: ID;steering;acceleration;brake
				String[] stringArray = val.split(";");
				String ID = stringArray[0];
				float steering = Float.parseFloat(stringArray[1]);
				float acceleration = Float.parseFloat(stringArray[2]);
				float brake = Float.parseFloat(stringArray[3]);

				TrafficObject trafficObject = sim.getPhysicalTraffic().getTrafficObject(ID);

				if ("drivingCar".equalsIgnoreCase(ID)) {
					sim.getCar().steer(steering);
					sim.getCar().setAcceleratorPedalIntensity(-acceleration);
					sim.getCar().setBrakePedalIntensity(brake);

					if(stringArray.length > 4){
						float targetSpeed = Float.parseFloat(stringArray[4]);
						//sim.getCar().setMaxSpeed(targetSpeed);
					}
				} else if (trafficObject != null && trafficObject instanceof TrafficCar) {
					TrafficCar car = (TrafficCar) trafficObject;
					car.useExternalControl();
					car.getCarControl().steer(steering);
					car.setAcceleratorPedalIntensity(-acceleration);
					car.setBrakePedalIntensity(brake);
				}

				updateSender.messageReceived.set(true);
			}
		} catch (Exception e) {
			System.err.println("Invalid vehicle control data received!");
			e.printStackTrace();
		}
	}

	public void setPedestrianControl(String val)
	{
		try{
			// val must have format: ID;heading;speed
			String[] stringArray = val.split(";");
			String ID = stringArray[0];
			float heading = Float.parseFloat(stringArray[1]) * FastMath.DEG_TO_RAD;
			float speed = Float.parseFloat(stringArray[2]);

			TrafficObject trafficObject = sim.getPhysicalTraffic().getTrafficObject(ID);

			if(trafficObject != null && trafficObject instanceof Pedestrian)
			{
				Pedestrian pedestrian = (Pedestrian) trafficObject;
				pedestrian.useExternalControl();
				pedestrian.setHeading(heading);
				pedestrian.setSpeed(speed);

				//System.err.println("Pedestrian: " +ID + " " + heading + " " + speed);
			}

		} catch (Exception e) {

			System.err.println("Invalid pedestrian control data received!");
			e.printStackTrace();
		}
	}


	private void parseXML(String xml) {
		try {
			Document doc = loadXMLFromString(xml);
			doc.getDocumentElement().normalize();
			String response = "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";

			NodeList nodes = doc.getElementsByTagName("Event");

			response += "<Message>";

			for (int i = 0; i < nodes.getLength(); i++) {
				String eventName = (((Element) nodes.item(i)).getAttribute("Name"));

				if(eventName.equals("EstablishConnection")){
					String val =nodes.item(i).getTextContent();

					if(val.length() > 0){
						try{
							updateInterval = Integer.valueOf(val);
						} catch(Exception e){}
					}

					if(!updateSender.isAlive())
						updateSender.start();

					response += "<Event Name=\"ConnectionEstablished\"/>\n";
				}
				else if(eventName.equals("AbolishConnection")){
					response += "<Event Name=\"ConnectionAbolished\"/>\n";
					this.interrupt();
				}
				else if(eventName.equals("GetDataSchema")){
					response += "<Event Name=\"DataSchema\">\n" + data.getSchema() + "\n</Event>";
				}
				else if(eventName.equals("GetSubscriptions")){
					response += "<Event Name=\"Subscriptions\">\n" + data.getAllSubscribedValues(true) + "\n</Event>";
				}
				else if(eventName.equals("GetSubscribedValues")){
					response += "<Event Name=\"SubscribedValues\">\n" + data.getAllSubscribedValues(false) + "\n</Event>";
				}
				else if(eventName.equals("GetValue")){
					String[] val = new String[]{nodes.item(i).getTextContent()};
					response += "<Event Name=\""+val[0]+"\">\n" + data.getValues(val, false) + "\n</Event>";
				}
				else if(eventName.equals("GetUpdateInterval")){
					response += "<Event Name=\"UpdateInterval\">\n" + String.valueOf(getUpdateInterval()) + "\n</Event>";
				}
				else if(eventName.equals("SetUpdateInterval")){
					String val =nodes.item(i).getTextContent();
					setUpdateInterval(Integer.valueOf(val));
					response += "<Event Name=\"UpdateInterval\">\n" + String.valueOf(getUpdateInterval()) + "\n</Event>";
				}
				else if(eventName.equals("Subscribe")){
					data.subscribe(nodes.item(i).getTextContent());
					response += "<Event Name=\"Subscriptions\">\n" + data.getAllSubscribedValues(true) + "\n</Event>";
				}
				else if(eventName.equals("Unsubscribe")){
					data.unsubscribe(nodes.item(i).getTextContent());
					response += "<Event Name=\"Subscriptions\">\n" + data.getAllSubscribedValues(true) + "\n</Event>";
				}
				else if(eventName.equals("SetVehicleControl")){
					String val = nodes.item(i).getTextContent();
					setVehicleControl(val);
					response += "<Event Name=\"UpdateVehicleControl\">\n" + val + "\n</Event>";
				}
				else if(eventName.equals("SetPedestrianControl")){
					String val = nodes.item(i).getTextContent();
					setPedestrianControl(val);
					response += "<Event Name=\"UpdatePedestrianControl\">\n" + val + "\n</Event>";
				}
				else{
					System.err.println("Unknow event received!");
					return;
				}


			}

			response += "</Message>\n";



			sendResponse(response);


		} catch (Exception e) {;
			System.err.println("No valid XML data received!");
			e.printStackTrace();
		}
	}

	public synchronized void sendResponse(String response) throws IOException {
		byte[] msg = (response).getBytes("UTF-8");
		out.write(msg);
		out.flush();
	}

}
