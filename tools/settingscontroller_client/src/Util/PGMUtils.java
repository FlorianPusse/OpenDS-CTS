package settingscontroller_client.src.Util;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.util.Scanner;

/**
 * Some utilities for manipulating greyscale images and for
 * reading and writing PGM (Portable Greymap) files.
 * A PGM file is a text file with the general format
 * 
 *   P2
 *   width  height  maxval
 *   (width * height values, separated by whitespace)
 *   
 * The first four entries can be separated by any kind of whitespace,
 * and may be interspersed with comments (lines starting with '#').
 * See the main() method below for an example.  For more information, 
 * about PGM files and how to view them, see
 * 
 * http://www.cs.iastate.edu/~smkautz/cs227f11/examples/week11/pgm_files.pdf
 */
public class PGMUtils
{
  // We'll use a constant maximum greyscale value
  private static final int MAXVAL = 255;

  /**
   * Creates an image of the given width and height 
   * containing a diagonal line.
   * @param width
   * @param height
   * @return
   */
  public static int[][] createDiagonalLine(int width, int height)
  {
    int[][] image = new int[height][width];
    for (int i = 0; i < height; ++i)
    {
      for (int j = 0; j < width; ++j)
      {
        int value;
        if (i == j)
        {
          value = 0;
        }
        else
        {
          value = MAXVAL;
        }
        image[i][j] = value;
      }
    }
    return image;
  }

  /**
   * Creates an image of the given width and height 
   * containing an arc of a circle.
   * @param width
   * @param height
   * @return
   */
  public static int[][] createArc(int width, int height)
  {
    int[][] image = new int[height][width];
    for (int i = 0; i < height; ++i)
    {
      for (int j = 0; j < width; ++j)
      {
        int value;
        double radius = Math.sqrt(i * i + j * j);
        if (radius < width && radius > width - 4)
        {
          value = 0;
        }
        else
        {
          value = MAXVAL;
        }
        image[i][j] = value;
      }
    }
    return image;
  }

  /**
   * Produces a negative of the given image.
   * @param image
   * @return
   */
  public static int[][] invert(int[][] image)
  {
    int width = image[0].length;
    int height = image.length;
    int[][] result = new int[height][width];
    for (int i = 0; i < height; ++i)
    {
      for (int j = 0; j < width; ++j)
      {
        result[i][j] = MAXVAL - image[i][j];
      }
    }
    return result;
  }
  
  /**
   * Creates a PGM file from the given image.
   * @param image
   *   the image
   * @param filename
   *   name of the file to be created
   * @throws FileNotFoundException
   */
  public static void createFile(int[][] image, String filename) throws FileNotFoundException
  {
    PrintWriter pw = new PrintWriter(filename);
    int width = image[0].length;
    int height = image.length;
    
    // magic number, width, height, and maxval
    pw.println("P2");
    pw.println(width + " " + height);
    pw.println(MAXVAL);
    
    // print out the data, limiting the line lengths to 70 characters
    int lineLength = 0;
    for (int i = 0; i < height; ++i)
    {
      for (int j = 0; j < width; ++j)
      {
        int value = image[i][j];
        
        // if we are going over 70 characters on a line,
        // start a new line
        String stringValue = "" + value;
        int currentLength = stringValue.length() + 1;
        if (currentLength + lineLength > 70)
        {
          pw.println();
          lineLength = 0;
        }
        lineLength += currentLength;
        pw.print(value + " ");
      }
    }
    pw.close();  
  }
  
  /**
   * Reads a PGM file and returns the image. The maximum greyscale
   * value is rescaled to be between 0 and 255.
   * @param filename
   * @return
   * @throws FileNotFoundException
   */
  public static short[][] readPGMFile(String filename) throws FileNotFoundException
  {
    Scanner scanner = new Scanner(new File(filename));
    scanner.next(); // magic number
    int width = scanner.nextInt();
    int height = scanner.nextInt();
    int max = scanner.nextInt();

    short[][] image = new short[height][width];
    
    for (int i = 0; i < height; ++i)
    {
      for (int j = 0; j < width; ++j)
      {
        // normalize to 255
        int value = scanner.nextInt();
        value = (int) Math.round( ((double) value) / max * MAXVAL);
        image[i][j] = (short) value;
      }
    }
    return image;
  }
  
  
  
  public static int[][] readPGMFileAlt(String filename) throws FileNotFoundException
  {
    Scanner scanner = new Scanner(new File(filename));
    scanner.next(); // skip the magic number
    int width = scanner.nextInt();
    int height = scanner.nextInt();
    int max = scanner.nextInt();
    
    int[][] image = new int[height][width];
    
    int row = 0;
    int col = 0;
    while(scanner.hasNextInt())
    {
      int value = scanner.nextInt();
      
      // re-scale the value to be between 0 and 255
      value = (int) Math.round( ((double) value) / max * MAXVAL);
      
      image[row][col] = value;
      col += 1;
      if (col == width)
      {
        col = 0;
        row += 1;
      }
    }
    return image;
  }

  
}