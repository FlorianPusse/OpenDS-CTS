package settingscontroller_client.src.Util;

import com.jme3.math.Vector2f;

import java.awt.*;
import java.awt.geom.Ellipse2D;
import java.io.IOException;
import java.io.InputStream;
import java.nio.charset.Charset;

import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.toRadians;

/**
 * Util functions used for OpenDS-CTS
 */
public class Util {

    /**
     * Draws a circle of radius r centered at (x,z)
     * @param g Graphics2D object to draw on
     * @param x X-coordinate of the circle
     * @param z Z-coordinate of the circle
     * @param r Radius of the circle
     */
    // https://stackoverflow.com/questions/19386951/how-to-draw-a-circle-with-given-x-and-y-coordinates-as-the-middle-spot-of-the-ci
    public static void drawCenteredCircle(Graphics2D g, float x, float z, float r) {
        x = x - (r / 2);
        z = z - (r / 2);
        g.fill(new Ellipse2D.Float(x, z, r, r));
    }

    /**
     * Reads a line from a socket without busy waiting.
     */
    public static class UnblockingLineReader{
        InputStream is;
        String bufferString = "";
        int timeout;

        public UnblockingLineReader(InputStream is){
            this.is = is;
            this.timeout = 10;
        }

        public String readLine() throws IOException {
            byte[] buffer = new byte[1024];

            while(!bufferString.contains("\n")){
                int bytesAvailable = is.available();

                if(bytesAvailable > 0){
                    int bytesRead = is.read(buffer);
                    if (bytesRead == -1){
                        bufferString = "";
                        return null;
                    }else{
                        bufferString += new String(buffer,0,bytesRead);
                    }
                }else{
                    try {
                        Thread.sleep(timeout);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }
            }

            String[] tmp = bufferString.split("\n",2);
            if(tmp.length == 2){
                bufferString = tmp[1];
            }else{
                bufferString = "";
            }
            return tmp[0];
        }
    }

}
