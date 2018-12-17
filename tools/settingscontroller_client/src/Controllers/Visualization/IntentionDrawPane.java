package settingscontroller_client.src.Controllers.Visualization;

import javax.swing.*;
import java.awt.*;
import java.awt.image.BufferedImage;

import static settingscontroller_client.src.Parameters.CAR_INTENTION_SIZE;

/** The car intention used for DRL based methods **/
public class IntentionDrawPane extends JPanel {
    int[] content = null;

    public void paintComponent(Graphics g) {
        super.paintComponent(g);

        if (content != null) {
            BufferedImage image = new BufferedImage(CAR_INTENTION_SIZE, CAR_INTENTION_SIZE, BufferedImage.TYPE_INT_RGB);
            image.setRGB(0, 0, CAR_INTENTION_SIZE, CAR_INTENTION_SIZE, content, 0, CAR_INTENTION_SIZE);
            g.drawImage(image, 0, 0, null);
        }
    }

    public void updateContent(int[] content) {
        this.content = content;
        removeAll();
        repaint();
    }
}