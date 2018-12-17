/*
 * Copyright (c) 2009-2012 jMonkeyEngine
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *
 * * Neither the name of 'jMonkeyEngine' nor the names of its contributors
 *   may be used to endorse or promote products derived from this software
 *   without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package eu.opends.camera;

import com.jme3.app.Application;
import com.jme3.app.state.AbstractAppState;
import com.jme3.app.state.AppStateManager;
import com.jme3.input.InputManager;
import com.jme3.input.KeyInput;
import com.jme3.input.controls.ActionListener;
import com.jme3.input.controls.KeyTrigger;
import com.jme3.post.SceneProcessor;
import com.jme3.profile.AppProfiler;
import com.jme3.renderer.Camera;
import com.jme3.renderer.RenderManager;
import com.jme3.renderer.Renderer;
import com.jme3.renderer.ViewPort;
import com.jme3.renderer.queue.RenderQueue;
import com.jme3.system.JmeSystem;
import com.jme3.texture.FrameBuffer;
import com.jme3.texture.Image.Format;
import com.jme3.util.BufferUtils;

import java.io.BufferedOutputStream;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.io.PrintWriter;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.Arrays;
import java.util.List;
import java.util.logging.Level;
import java.util.logging.Logger;

public class ScreenshotAppState extends AbstractAppState implements ActionListener, SceneProcessor {

    private static final Logger logger = Logger.getLogger(ScreenshotAppState.class.getName());
    private String filePath = null;
    private boolean capture = false;
    private boolean numbered = true;
    private Renderer renderer;
    private RenderManager rm;
    private ByteBuffer outBuf;
    private String shotName;
    private long shotIndex = 0;
    private int width, height;

    /**
     * Using this constructor, the screenshot files will be written sequentially to the system
     * default storage folder.
     */
    public ScreenshotAppState() {
        this(null);
    }

    /**
     * This constructor allows you to specify the output file path of the screenshot.
     * Include the seperator at the end of the path.
     * Use an emptry string to use the application folder. Use NULL to use the system
     * default storage folder.
     * @param filePath The screenshot file path to use. Include the seperator at the end of the path.
     */
    public ScreenshotAppState(String filePath) {
        this.filePath = filePath;
    }

    /**
     * This constructor allows you to specify the output file path of the screenshot.
     * Include the seperator at the end of the path.
     * Use an emptry string to use the application folder. Use NULL to use the system
     * default storage folder.
     * @param filePath The screenshot file path to use. Include the seperator at the end of the path.
     * @param fileName The name of the file to save the screeshot as.
     */
    public ScreenshotAppState(String filePath, String fileName) {
        this.filePath = filePath;
        this.shotName = fileName;
    }

    /**
     * This constructor allows you to specify the output file path of the screenshot and
     * a base index for the shot index.
     * Include the seperator at the end of the path.
     * Use an emptry string to use the application folder. Use NULL to use the system
     * default storage folder.
     * @param filePath The screenshot file path to use. Include the seperator at the end of the path.
     * @param shotIndex The base index for screen shots.  The first screen shot will have
     *                  shotIndex + 1 appended, the next shotIndex + 2, and so on.
     */
    public ScreenshotAppState(String filePath, long shotIndex) {
        this.filePath = filePath;
        this.shotIndex = shotIndex;
    }

    /**
     * This constructor allows you to specify the output file path of the screenshot and
     * a base index for the shot index.
     * Include the seperator at the end of the path.
     * Use an emptry string to use the application folder. Use NULL to use the system
     * default storage folder.
     * @param filePath The screenshot file path to use. Include the seperator at the end of the path.
     * @param fileName The name of the file to save the screeshot as.
     * @param shotIndex The base index for screen shots.  The first screen shot will have
     *                  shotIndex + 1 appended, the next shotIndex + 2, and so on.
     */
    public ScreenshotAppState(String filePath, String fileName, long shotIndex) {
        this.filePath = filePath;
        this.shotName = fileName;
        this.shotIndex = shotIndex;
    }
    
    /**
     * Set the file path to store the screenshot.
     * Include the seperator at the end of the path.
     * Use an emptry string to use the application folder. Use NULL to use the system
     * default storage folder.
     * @param filePath File path to use to store the screenshot. Include the seperator at the end of the path.
     */
    public void setFilePath(String filePath) {
        this.filePath = filePath;
    }

    /**
     * Set the file name of the screenshot.
     * @param fileName File name to save the screenshot as.
     */
    public void setFileName(String fileName) {
        this.shotName = fileName;
    }

    /**
     * Sets the base index that will used for subsequent screen shots. 
     * @param index The index
     */
    public void setShotIndex(long index) {
        this.shotIndex = index;
    }

    /**
     * Sets if the filename should be appended with a number representing the 
     * current sequence.
     * @param numberedWanted If numbering is wanted.
     */
    public void setIsNumbered(boolean numberedWanted) {
        this.numbered = numberedWanted;
    }

    @Override
    public void initialize(AppStateManager stateManager, Application app) {
        if (!super.isInitialized()){
            InputManager inputManager = app.getInputManager();
            inputManager.addMapping("ScreenShot", new KeyTrigger(KeyInput.KEY_SYSRQ));
            inputManager.addListener(this, "ScreenShot");

            List<ViewPort> vps = app.getRenderManager().getPostViews();
            ViewPort last = vps.get(vps.size()-1);
            last.addProcessor(this);

            if (shotName == null) {
                shotName = app.getClass().getSimpleName();
            }
        }

        super.initialize(stateManager, app);
    }

    public void onAction(String name, boolean value, float tpf) {
        if (value){
            capture = true;
        }
    }

    public void takeScreenshot() {
        capture = true;
    }

    public void initialize(RenderManager rm, ViewPort vp) {
        renderer = rm.getRenderer();
        this.rm = rm;
        reshape(vp, vp.getCamera().getWidth(), vp.getCamera().getHeight());
    }

    @Override
    public boolean isInitialized() {
        return super.isInitialized() && renderer != null;
    }

    public void reshape(ViewPort vp, int w, int h) {
        outBuf = BufferUtils.createByteBuffer(w * h * 4);
        width = w;
        height = h;
    }

    public void preFrame(float tpf) {
    }

    public void postQueue(RenderQueue rq) {
    }

    
	public enum ScreenshotFormat
	{
		FLOAT_MAP, PPM_ASCII, PPM_BINARY, PGM_ASCII, PGM_BINARY, PNG, GIF, JPG;
	}

	
	@Override
	public void postFrame(FrameBuffer out) 
	{
		if (capture){
			capture = false;

			Camera curCamera = rm.getCurrentCamera();
			int viewX = (int) (curCamera.getViewPortLeft() * curCamera.getWidth());
			int viewY = (int) (curCamera.getViewPortBottom() * curCamera.getHeight());
			int viewWidth = (int) ((curCamera.getViewPortRight() - curCamera.getViewPortLeft()) * curCamera.getWidth());
			int viewHeight = (int) ((curCamera.getViewPortTop() - curCamera.getViewPortBottom()) * curCamera.getHeight());

			renderer.setViewPort(0, 0, width, height);
			renderer.readFrameBuffer(out, outBuf);
			renderer.setViewPort(viewX, viewY, viewWidth, viewHeight);

			File file;
			String filename;
			if (numbered) {
				shotIndex++;
				filename = shotName + shotIndex;
			} else {
				filename = shotName;
			}

			if (filePath == null) {
				file = new File(JmeSystem.getStorageFolder() + File.separator + filename + ".png").getAbsoluteFile();
			} else {
				file = new File(filePath + filename + ".png").getAbsoluteFile();
			}
			logger.log(Level.FINE, "Saving ScreenShot to: {0}", file.getAbsolutePath());

			try {
				writeImageFile(file);
			} catch (IOException ex) {
				logger.log(Level.SEVERE, "Error while saving screenshot", ex);
			}
		}
	}

	protected void writeImageFile( File file ) throws IOException {
		OutputStream outStream = new FileOutputStream(file);
		try {
			JmeSystem.writeImageFile(outStream, "png", outBuf, width, height);
		} finally {
			outStream.close();
		}
	}

	@Override
	public void setProfiler(AppProfiler appProfiler) {

	}

	public void postFrame(ViewPort vp, Format imageFormat, ScreenshotFormat screenshotFormat, int trimHeight)
    {
    	FrameBuffer out = vp.getOutputFrameBuffer();   	
    	
        if (capture){
            capture = false;

            Camera curCamera = rm.getCurrentCamera();
            int viewX = (int) (curCamera.getViewPortLeft() * curCamera.getWidth());
            int viewY = (int) (curCamera.getViewPortBottom() * curCamera.getHeight());
            int viewWidth = (int) ((curCamera.getViewPortRight() - curCamera.getViewPortLeft()) * curCamera.getWidth());
            int viewHeight = (int) ((curCamera.getViewPortTop() - curCamera.getViewPortBottom()) * curCamera.getHeight());

            renderer.setViewPort(0, 0, width, height);
            //renderer.readFrameBuffer(out, outBuf);
            renderer.readFrameBufferWithFormat(out, outBuf,imageFormat);
            renderer.setViewPort(viewX, viewY, viewWidth, viewHeight);

    		int w = width;
    		int h = height;
    		
    		if(out != null)
    		{
    			w = out.getWidth();
    			h = out.getHeight();
    		}   		
    		
            if(screenshotFormat == ScreenshotFormat.PPM_ASCII || screenshotFormat == ScreenshotFormat.PPM_BINARY ||
               screenshotFormat == ScreenshotFormat.PGM_ASCII || screenshotFormat == ScreenshotFormat.PGM_BINARY ||
               screenshotFormat == ScreenshotFormat.FLOAT_MAP)
            {
            	// convert ByteBuffer to byte[]
        		ByteBuffer byteBuffer = outBuf.duplicate();
        		byte[] byteArray = new byte[byteBuffer.remaining()];
        		byteBuffer.get(byteArray);

        		writePPM(byteArray, w, h, screenshotFormat, vp, trimHeight);
            }
            else
            {
            	writeImageFile(w, h, screenshotFormat);
            }
        }
    }
    
    
	private void writeImageFile(int width, int height, ScreenshotFormat screenshotFormat) 
	{
		// use thread to prevent file access slowing down simulation
        new Thread()
        {
            public void run()
            {
            	String format = screenshotFormat.toString().toLowerCase();
				File file = getOutputFile(format);
		        OutputStream outStream = null;
		        
		        try {
		            outStream = new FileOutputStream(file);
		            JmeSystem.writeImageFile(outStream, format, outBuf,  width, height);
		        } catch (IOException ex) {
		            logger.log(Level.SEVERE, "Error while saving screenshot", ex);
		        } finally {
		            if (outStream != null){
		                try {
		                    outStream.close();
		                } catch (IOException ex) {
		                    logger.log(Level.SEVERE, "Error while saving screenshot", ex);
		                }
		            }
		        }
            }
        }.start();
	}
	
	
	private void writePPM(byte[] origByteArray, int width, int origHeight, ScreenshotFormat screenshotFormat, ViewPort vp, int trimHeight)
	{
		// use thread to prevent file access slowing down simulation
		new Thread()
        {
            public void run() 
            {
            	File file;
		        PrintWriter asciiOutput = null;
		        BufferedOutputStream binaryOutput = null;
		        
		        try {
		        	
		        	int height; 
		        	byte[] byteArray;
		        	
		        	
		    		if(0<trimHeight && trimHeight < 0.5f*origHeight-1)
		    		{
		    			height = origHeight - (2*trimHeight);
		    			byteArray = trimHeight(origByteArray, width, origHeight, trimHeight);
		    		}
		    		else
		    		{
		    			height = origHeight;
		    			byteArray = origByteArray;
		    		}
		    		
		        	
		        	if(screenshotFormat == ScreenshotFormat.FLOAT_MAP)
		        	{
		        		//open ASCII writer
		        		file = getOutputFile("txt");
		        		asciiOutput = new PrintWriter(file);
		        		
		        		// write header (e.g. "Resolution: 1280 x 720 \n")
		        		asciiOutput.print("Resolution: " + width + " x " + height + " \n");
		        	}
		        	else if(screenshotFormat == ScreenshotFormat.PPM_ASCII)
		        	{
		        		// open ASCII writer 
		        		file = getOutputFile("ppm");
		        		asciiOutput = new PrintWriter(file);
		        		
		        		// write header (e.g. "P3 1280 720 255 \n")
		        		asciiOutput.print("P3 " + width + " " + height + " 255 \n");
		        	}
		        	else if(screenshotFormat == ScreenshotFormat.PPM_BINARY)
		        	{
		        		// open Binary writer
		        		file = getOutputFile("ppm");
		        		binaryOutput = new BufferedOutputStream(new FileOutputStream(file));
		        		
		        		// write header (e.g. "P6 1280 720 255 ")
		        		String header = "P6 " + width + " " + height + " 255 ";
		        		binaryOutput.write(header.getBytes());
		        	}
		        	else if(screenshotFormat == ScreenshotFormat.PGM_ASCII)
		        	{
		        		// open ASCII writer 
		        		file = getOutputFile("pgm");
		        		asciiOutput = new PrintWriter(file);
		        		
		        		// write header (e.g. "P2 1280 720 255 \n")
		        		asciiOutput.print("P2 " + width + " " + height + " 255 \n");
		        	}
		        	else if(screenshotFormat == ScreenshotFormat.PGM_BINARY)
		        	{
		        		// open Binary writer
		        		file = getOutputFile("pgm");
		        		binaryOutput = new BufferedOutputStream(new FileOutputStream(file));
		        		
		        		// write header (e.g. "P5 1280 720 255 ")
		        		String header = "P5 " + width + " " + height + " 255 ";
		        		binaryOutput.write(header.getBytes());
		        	}
					
		        	// buffer --> file
		        	if(screenshotFormat == ScreenshotFormat.PPM_ASCII || screenshotFormat == ScreenshotFormat.PPM_BINARY)
		        	{
						for(int h=height-1; h>=0; h--)  //--> flip image vertically
							for(int w=0; w<width; w++)
								for(int color=0; color<3; color++)  // --> remove transparency color value (4# component)
								{
									int position = h*width*4 + w*4 + color;
			
			    	            	if(screenshotFormat == ScreenshotFormat.PPM_ASCII)
			    	            	{
			    	            		int integer = ((255 + (int)byteArray[position])%255);
			    	            		asciiOutput.print(integer + " ");
			    	            	}
			    	            	else if(screenshotFormat == ScreenshotFormat.PPM_BINARY)
			    	            		binaryOutput.write(byteArray[position]);
								}
		        	}
		        	else if(screenshotFormat == ScreenshotFormat.FLOAT_MAP || 
		        			screenshotFormat == ScreenshotFormat.PGM_ASCII || 
		        			screenshotFormat == ScreenshotFormat.PGM_BINARY)
		        	{
		            	float zFar = vp.getCamera().getFrustumFar();
		            	float zNear = vp.getCamera().getFrustumNear();
		            	
						for(int h=height-1; h>=0; h--)  //--> flip image vertically
							for(int w=0; w<width; w++)
							{							
								int pos = h*width*4 + w*4;
								byte[] subByteArray = new byte[] {byteArray[pos], byteArray[pos+1], byteArray[pos+2], byteArray[pos+3]};
								
								// get distance in meters
								float dist = ByteBuffer.wrap(subByteArray).order(ByteOrder.LITTLE_ENDIAN).getFloat();
								float linearDist = linearizeDepthValue(dist, zNear, zFar);							
								
								// divide by zFar value --> 0 <= relativeDist <= 1
								float relativeDist = linearDist/zFar;
								int intVal = (int) (relativeDist*255);
	    	            		byte byteVal = (byte) intVal;
	    	            		
	    	            		if(screenshotFormat == ScreenshotFormat.FLOAT_MAP)
		    	            		asciiOutput.write(linearDist + " ");
	    	            		else if(screenshotFormat == ScreenshotFormat.PGM_ASCII)
	    	            			asciiOutput.print(intVal + " ");
		    	            	else if(screenshotFormat == ScreenshotFormat.PGM_BINARY)
		    	            		binaryOutput.write(byteVal);		    	            	 
							}
		        	}
					
					
					if(screenshotFormat == ScreenshotFormat.PPM_BINARY || screenshotFormat == ScreenshotFormat.PGM_BINARY)
						binaryOutput.flush();
		
		        } catch (Exception ex) {
		        	System.err.println("Screenshot error");
		        } finally {
		            if (asciiOutput != null){
		                try {
		                    asciiOutput.close();
		                } catch (Exception ex) {
		                	System.err.println("Screenshot: could not close ascii output file.");
		                }
		            }
		            if (binaryOutput != null){
		                try {
		                	binaryOutput.close();
		                } catch (Exception ex) {
		                	System.err.println("Screenshot: could not close binary output file.");
		                }
		            }
		        }
            }
        }.start();
	}
    
	
	float linearizeDepthValue(float depthSample, float zNear, float zFar)
	{
	    depthSample = 2.0f * depthSample - 1.0f;
	    float zLinear = 2.0f * zNear * zFar / (zFar + zNear - depthSample * (zFar - zNear));
	    
	    // cap distances exceeding zFar
	    if(zLinear>zFar)
	    	zLinear = zFar;
	    
	    // return linear distance (in meters) to given pixel
	    return zLinear;
	}
	
	
	private File getOutputFile(String extension) 
	{
		String filename;
		if (numbered) 
		{
		    shotIndex++;
		    filename = shotName + shotIndex;
		} 
		else 
		    filename = shotName;

		File file;
		if (filePath == null)
		    file = new File(JmeSystem.getStorageFolder() + File.separator + filename + "." + extension).getAbsoluteFile();
		else
		    file = new File(filePath + filename + "." + extension).getAbsoluteFile();

		logger.log(Level.FINE, "Saving ScreenShot to: {0}", file.getAbsolutePath());
		
		return file;
	}

	
	/**
	 * Removes the amount of pixel rows specified in "trimHeight" at the top AND bottom of the frame  
	 * @param origByteArray
	 * 			Byte array representing frame buffer
	 * @param origWidth
	 * 			Original width of frame buffer
	 * @param origHeight
	 * 			Original height of frame buffer
	 * @param trimHeight
	 * 			Number of pixel rows to remove at top and bottom
	 * @return
	 * 			Byte array (sub-array of the input array)
	 */
	private byte[] trimHeight(byte[] origByteArray, int origWidth, int origHeight, int trimHeight) 
	{
		int startIdx = trimHeight*origWidth*4;
		int endIdx = (origHeight-trimHeight)*origWidth*4;
		return Arrays.copyOfRange(origByteArray, startIdx, endIdx);
	}
}
