package org.usfirst.frc.team3319.robot.vision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.vision.VisionPipeline;

/**
* TapeRecognitionPipeline class.
*
* <p>An OpenCV pipeline generated by GRIP.
*
* @author GRIP
*/
public class ResizingPipeline implements VisionPipeline {

	//Outputs
	private Mat resizeImageOutput = new Mat();

	static {
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
	}

	/**
	 * This is the primary method that runs the entire pipeline and updates the outputs.
	 */
	@Override	public void process(Mat source0) {
		// Step Resize_Image0:
		Mat resizeImageInput = source0;
		double resizeImageWidth = 320.0;
		double resizeImageHeight = 240.0;
		int resizeImageInterpolation = Imgproc.INTER_LINEAR;
		resizeImage(resizeImageInput, resizeImageWidth, resizeImageHeight, resizeImageInterpolation, resizeImageOutput);
	}

	/**
	 * This method is a generated getter for the output of a Resize_Image.
	 * @return Mat output from Resize_Image.
	 */
	public Mat resizeImageOutput() {
		return resizeImageOutput;
	}
	/**
	 * Scales and image to an exact size.
	 * @param input The image on which to perform the Resize.
	 * @param width The width of the output in pixels.
	 * @param height The height of the output in pixels.
	 * @param interpolation The type of interpolation.
	 * @param output The image in which to store the output.
	 */
	private void resizeImage(Mat input, double width, double height,
		int interpolation, Mat output) {
		Imgproc.resize(input, output, new Size(width, height), 0.0, 0.0, interpolation);
	}

}

