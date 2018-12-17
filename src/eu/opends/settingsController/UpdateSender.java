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

import eu.opends.main.Simulator;

import java.io.IOException;
import java.util.concurrent.atomic.AtomicBoolean;

import static settingscontroller_client.src.Evaluation.Config.SimulationMode.TESTING;
import static settingscontroller_client.src.Evaluation.Config.SimulationMode.TRAINING;

/**
 * 
 * @author Daniel Braun
 */
public class UpdateSender extends Thread 
{
	
	private APIData  data;
	ConnectionHandler connectionHandler;
	Simulator sim;

	AtomicBoolean messageReceived = new AtomicBoolean(false);

	public UpdateSender(APIData data, ConnectionHandler connectionHandler, Simulator sim){
		this.data = data;
		this.connectionHandler = connectionHandler;
		this.sim = sim;
	}

	final boolean waitLong = true;

	int totalSlept = 0;
	public void run(){

		int waitingTime = 225;

		if(waitLong){
			waitingTime = 350;
		}

		int test = 0;

		while(!isInterrupted()){

			boolean synchronous = true;
			if(synchronous){
				if(messageReceived.get() || (sim.mode == TRAINING && totalSlept >= waitingTime)
						|| sim.mode == TESTING && totalSlept >= 350){
					messageReceived.set(false);

					//long startTime = System.nanoTime();
					sim.setPause(false);
					try {
						Thread.sleep(connectionHandler.getUpdateInterval());
					} catch (InterruptedException e) {
						e.printStackTrace();
						this.interrupt();
					}
					sim.setPause(true);
					//long estimatedTime = System.nanoTime() - startTime;
					//System.out.println("Executed simulator for " + (estimatedTime / 1000000.0) + "ms");

					String response = "<Message><Event Name=\"SubscribedValues\">\n" + data.getAllSubscribedValues(false) + "\n</Event></Message>\n";
					try {
						connectionHandler.sendResponse(response);
					} catch (IOException e) {
						e.printStackTrace();
						this.interrupt();
					}

					totalSlept = 0;
					test = 0;
				}else{
					try {
						Thread.sleep(10);
						totalSlept++;
						//System.out.println("HI " + test);
						++test;
					} catch (InterruptedException e) {
						e.printStackTrace();
                        this.interrupt();
					}
				}
			}else{
				String response = "<Message><Event Name=\"SubscribedValues\">\n" + data.getAllSubscribedValues(false) + "\n</Event></Message>\n";
				try {
					connectionHandler.sendResponse(response);
				} catch (IOException e) {
					this.interrupt();
				}
				try {
					Thread.sleep(connectionHandler.getUpdateInterval());
				} catch (InterruptedException e) {
					this.interrupt();
				}
			}


		}
	}

}
