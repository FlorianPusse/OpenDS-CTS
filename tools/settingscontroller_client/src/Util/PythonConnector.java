package settingscontroller_client.src.Util;

import settingscontroller_client.src.Util.Util;

import java.io.*;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.ReentrantLock;

/**
 * Connects the car controller to the outside world
 */
public class PythonConnector implements Runnable{

    ServerSocket serverSocket;
    boolean running = true;

    /**
     * Sends messages to the connected socket
     */
    Sender sender = null;

    /**
     * Receives messags from the connected socket
     */
    Receiver receiver = null;

    /**
     * Connects the controller to the socket and sends messages
     * to it.
     */
    private class Sender implements Runnable{
        String message = null;
        ReentrantLock messageLock = new ReentrantLock();
        Condition messageSetCondition = messageLock.newCondition();
        Socket clientSocket;
        PrintWriter outputStream;

        Sender(Socket clientSocket) throws IOException {
            this.clientSocket = clientSocket;
            outputStream = new PrintWriter(clientSocket.getOutputStream());
        }

        public void send(String message){
            messageLock.lock();
            this.message = message;
            messageSetCondition.signal();
            messageLock.unlock();
        }

        @Override
        public void run() {
            while(true){
                messageLock.lock();
                while(this.message == null){
                    try {
                        messageSetCondition.await();
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }
                outputStream.write(message);
                outputStream.flush();
                this.message = null;
                messageLock.unlock();
            }
        }
    }

    /**
     * Connects the controller to the socket and receives messages
     * from it.
     */
    private class Receiver implements Runnable {
        Socket clientSocket;
        String message = null;
        ReentrantLock messageLock = new ReentrantLock();
        Condition messageSetCondition = messageLock.newCondition();
        BufferedReader inputStream;
        Util.UnblockingLineReader reader;

        public Receiver(Socket clientSocket) throws IOException {
            this.clientSocket = clientSocket;
            inputStream = new BufferedReader(new InputStreamReader(clientSocket.getInputStream()));
            reader = new Util.UnblockingLineReader(clientSocket.getInputStream());
        }

        @Override
        public void run() {
            while(true){
                try {
                    String message = reader.readLine();
                    if(message.isEmpty()){
                        continue;
                    }
                    messageLock.lock();
                    this.message = message;
                    messageSetCondition.signal();
                    messageLock.unlock();

                    Thread.sleep(10);
                } catch (IOException | InterruptedException e) {
                    //e.printStackTrace();
                    System.err.println("Python connector broken");
                    try {
                        Thread.sleep(5000);
                    } catch (InterruptedException e1) {

                    }
                }
            }
        }

        public String receive(){
            String message;
            messageLock.lock();
            while(this.message == null){
                try {
                    messageSetCondition.await();
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
            message = this.message;
            this.message = null;
            messageLock.unlock();
            return message;
        }
    }

    public PythonConnector(ServerSocket socket){
        this.serverSocket = socket;
    }

    public void stop(){
        running = false;
    }

    @Override
    public void run() {
        while(running){
            try {
                Socket newClient = serverSocket.accept();
                receiver = new Receiver(newClient);
                sender = new Sender(newClient);
                Thread t = new Thread(receiver);
                t.start();
                t = new Thread(sender);
                t.start();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }

    /**
     * Sends a message to the connected socket by queuing
     * it in the sender
     * @param message Message to send
     */
    public void sendMessage(String message){
        while(sender == null){
            try {
                Thread.sleep(50);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        sender.send(message);
    }

    /**
     * Receives a message from the connected socket.
     * @return The received message
     */
    public String receiveMessage(){
        while(receiver == null){
            try {
                Thread.sleep(50);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        return receiver.receive();
    }

    /**
     * Checks the controller is currently connected
     * via socket
     * @return Whether the controller is currently connected
     * via socket
     */
    public boolean connected(){
        return receiver != null && sender != null;
    }
}
