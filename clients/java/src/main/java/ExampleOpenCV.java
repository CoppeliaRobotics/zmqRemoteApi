import java.util.List;
import java.util.Arrays;

import java.awt.FlowLayout;
import java.awt.Image;
import java.awt.image.BufferedImage;
import java.awt.image.DataBufferByte;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.core.CvType;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import javax.imageio.ImageIO;
import javax.swing.ImageIcon;
import javax.swing.JFrame;
import javax.swing.JLabel;

import com.coppeliarobotics.remoteapi.zmq.*;

public class ExampleOpenCV
{
    /*
     * Make sure to have the add-on "ZMQ remote API" running in
     * CoppeliaSim and have following scene loaded:
     *
     * scenes/messaging/synchronousImageTransmissionViaRemoteApi.ttt
     *
     * Do not launch simulation, but run this script
     */

    public static void main(String[] _args) throws java.io.IOException, co.nstant.in.cbor.CborException
    {
        // on different Java versions you might need one or another:
        nu.pattern.OpenCV.loadLocally();
        //nu.pattern.OpenCV.loadShared();

        var frame = new JFrame();
        frame.setLayout(new FlowLayout());
        frame.setSize(400, 400);
        var lbl = new JLabel();
        frame.add(lbl);
        frame.setVisible(true);
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

        var client = new RemoteAPIClient();
        var sim = client.getObject().sim();

        var visionSensorHandle = sim.getObject("/VisionSensor");
        var passiveVisionSensorHandle = sim.getObject("/PassiveVisionSensor");

        sim.setStepping(true);
        sim.startSimulation();

        double startTime = sim.getSimulationTime();
        int skip = 2;
        while(sim.getSimulationTime() - startTime < 5)
        {
            Object[] r = sim.getVisionSensorImg(visionSensorHandle);
            byte[] imgData = (byte[])r[0];
            List<Long> res = (List<Long>)r[1];
            int width = res.get(0).intValue();
            int height = res.get(1).intValue();

            // OpenCV processing:
            var src = new Mat(width, height, CvType.CV_8UC3);
            src.put(0, 0, imgData);
            Core.flip(src, src, -1);
            var gray = new Mat(src.rows(), src.cols(), src.type());
            var edges = new Mat(src.rows(), src.cols(), src.type());
            var dst = new Mat(src.rows(), src.cols(), src.type(), new Scalar(0));
            Imgproc.cvtColor(src, gray, Imgproc.COLOR_RGB2GRAY);
            Imgproc.blur(gray, edges, new Size(3, 3));
            Imgproc.Canny(edges, edges, 100, 100 * 3);
            src.copyTo(dst, edges);

            // image display:
            var img = toBufferedImage(dst);
            var icon = new ImageIcon(img);
            lbl.setIcon(icon);

            sim.step();
        }
        sim.stopSimulation();
    }

    public static Image toBufferedImage(Mat m)
    {
        int type = BufferedImage.TYPE_BYTE_GRAY;
        if(m.channels() > 1)
            type = BufferedImage.TYPE_3BYTE_BGR;
        int bufferSize = m.channels() * m.cols() * m.rows();
        byte[] b = new byte[bufferSize];
        m.get(0, 0, b);
        var image = new BufferedImage(m.cols(), m.rows(), type);
        final byte[] targetPixels = ((DataBufferByte)image.getRaster().getDataBuffer()).getData();
        System.arraycopy(b, 0, targetPixels, 0, b.length);
        return image;
    }
}
