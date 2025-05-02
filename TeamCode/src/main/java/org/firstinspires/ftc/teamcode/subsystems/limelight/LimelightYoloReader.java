package org.firstinspires.ftc.teamcode.subsystems.limelight;

import com.acmerobotics.dashboard.config.Config;

import org.json.JSONArray;
import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.net.HttpURLConnection;
import java.net.URL;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.logging.Logger;

@Config
public class LimelightYoloReader {
    public static String baseUrl = "http://172.29.0.1:8000";
    public static Logger logger = Logger.getLogger("LimelightYoloReader");
    public static Map<Integer, Limelight.Sample.Color> clsMap = Map.of(
            0, Limelight.Sample.Color.BLUE,
            1, Limelight.Sample.Color.RED,
            2, Limelight.Sample.Color.YELLOW
    );

    public static int DEFAULT_RETRY_FRAMES = 5;
    public static int RETRY_DELAY_MS = 100;

    public boolean ready = true;

    public LimelightYoloReader() {
    }

    /**
     * Get all detections from the vision coprocessor
     *
     * @return List of sample objects detected
     */
    public List<Limelight.Sample> getDetections() {
        JSONObject json = sendGetRequest("/detection_boxes.json");
        if (json == null) {
            logger.warning("Failed to get detection boxes: JSON null");
            return new ArrayList<>();
        }

        List<Limelight.Sample> samples = new ArrayList<>();

        try {
            JSONArray detectionsArray = json.getJSONArray("detections");

            for (int i = 0; i < detectionsArray.length(); i++) {
                JSONObject detectionObj = detectionsArray.getJSONObject(i);

                int classId = detectionObj.getInt("class");
                double worldX = detectionObj.has("world_x") ? detectionObj.getDouble("world_x") : 0.0;
                double worldY = detectionObj.has("world_y") ? detectionObj.getDouble("world_y") : 0.0;

                Limelight.Sample.Color color = clsMap.getOrDefault(classId, Limelight.Sample.Color.UNKNOWN);

                //logger.info("Detected sample " + classId + ", world: (" + worldX + ", " + worldY + ")");
                samples.add(new Limelight.Sample(color, worldX, worldY, 0.0));
            }
        } catch (Exception e) {
            logger.warning("Error parsing detections: " + e.getMessage());
        }

        return samples;
    }

    public int getFrameTimeDelta() {
        JSONObject json = sendGetRequest("/detection_boxes.json");
        if (json == null) {
            logger.warning("Failed to get time delta: JSON null");
            return -1;
        }
        // Get frame_time_delta field
        try {
            if (json.has("frame_time_delta")) {
                return json.getInt("frame_time_delta");
            } else {
                logger.warning("No frame time delta found in current frame");
            }
        } catch (Exception e) {
            logger.warning("Error parsing frame time delta: " + e.getMessage());
        }
        return -1;
    }

    public void requestFrame() {
        JSONObject json = sendGetRequest("/request_frame");
        if (json == null) {
            logger.info("Failed to request frame: JSON null");
        }
    }

    /**
     * Get the best sample from the current frame
     *
     * @return The best sample or null if none found
     */
    public Limelight.Sample getBestSample() {
        JSONObject json = sendGetRequest("/detection_boxes.json");
        if (json == null) {
            logger.info("Failed to get detection boxes: JSON null");
            return null;
        }

        try {
            if (json.has("best_sample")) {
                JSONObject bestSampleObj = json.getJSONObject("best_sample");
                String colorStr = bestSampleObj.getString("color");
                double x = bestSampleObj.getDouble("x");
                double y = bestSampleObj.getDouble("y");
                double angle = bestSampleObj.has("angle") ? bestSampleObj.getDouble("angle") : 0.0;

                Limelight.Sample.Color color;
                try {
                    color = Limelight.Sample.Color.valueOf(colorStr);
                } catch (IllegalArgumentException e) {
                    color = Limelight.Sample.Color.UNKNOWN;
                }

                logger.info("Found best sample: " + color + " at (" + x + ", " + y + "), angle: " + angle);
                return new Limelight.Sample(color, x, y, angle);
            } else {
                logger.info("No best sample found in current frame");
            }
        } catch (Exception e) {
            logger.warning("Error parsing best sample: " + e.getMessage());
        }

        return null;
    }

    /**
     * Try to get the best sample across multiple frames
     * Will return as soon as a valid sample is found
     *
     * @return The best sample or null if none found after all retries
     */
    public Limelight.Sample getBestSampleWithRetry() {
        return getBestSampleWithRetry(DEFAULT_RETRY_FRAMES);
    }

    /**
     * Try to get the best sample across multiple frames
     *
     * @param maxFrames Maximum number of frames to check
     * @return The best sample or null if none found after all retries
     */
    public Limelight.Sample getBestSampleWithRetry(int maxFrames) {
        for (int i = 0; i < maxFrames; i++) {
            logger.info("Attempting to get best sample, frame " + (i + 1) + "/" + maxFrames);
            Limelight.Sample bestSample = getBestSample();
            if (bestSample != null) {
                return bestSample;
            }

            // Wait a bit before the next attempt
            if (i < maxFrames - 1) {
                try {
                    Thread.sleep(RETRY_DELAY_MS);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    break;
                }
            }
        }
        logger.info("No best sample found after " + maxFrames + " attempts");
        return null;
    }

    /**
     * Set the allowed sample colors for detection
     *
     * @param colors List of colors to allow
     * @return true if successful, false otherwise
     */
    public boolean setAllowedColors(List<Limelight.Sample.Color> colors) {
        if (colors == null || colors.isEmpty()) {
            logger.info("No colors specified");
            return false;
        }

        // Build comma-separated list of colors
        StringBuilder colorParam = new StringBuilder();
        for (int i = 0; i < colors.size(); i++) {
            colorParam.append(colors.get(i).name());
            if (i < colors.size() - 1) {
                colorParam.append(",");
            }
        }

        // Send request to set allowed colors
        String endpoint = "/set_colors?colors=" + colorParam.toString();
        JSONObject response = sendGetRequest(endpoint);

        if (response != null && response.optBoolean("success", false)) {
            logger.info("Successfully set allowed colors: " + colorParam);
            return true;
        } else {
            logger.warning("Failed to set allowed colors");
            return false;
        }
    }

    /**
     * Set to allow only blue samples
     */
    public boolean setBlueOnly() {
        return setAllowedColors(List.of(Limelight.Sample.Color.BLUE));
    }

    /**
     * Set to allow only red samples
     */
    public boolean setRedOnly() {
        return setAllowedColors(List.of(Limelight.Sample.Color.RED));
    }

    /**
     * Set to allow only yellow samples
     */
    public boolean setYellowOnly() {
        return setAllowedColors(List.of(Limelight.Sample.Color.YELLOW));
    }

    /**
     * Allow all colors (default behavior)
     */
    public boolean setAllColors() {
        return setAllowedColors(List.of(
                Limelight.Sample.Color.BLUE,
                Limelight.Sample.Color.RED,
                Limelight.Sample.Color.YELLOW
        ));
    }

    /**
     * Check if the connection to the vision coprocessor is active
     *
     * @return true if ready, false otherwise
     */
    public boolean isReady() {
        return ready;
    }

    /**
     * Send a GET request to the vision coprocessor
     *
     * @param endpoint API endpoint to query
     * @return JSON response or null if failed
     */
    public String getRawJSONRequest(String endpoint) {
        JSONObject jsonObject = sendGetRequest(endpoint);
        if (jsonObject == null) {
            return "NULL";
        }
        return jsonObject.toString();
    }

    private JSONObject sendGetRequest(String endpoint) {
        HttpURLConnection connection = null;
        try {
            String urlString = baseUrl + endpoint;
            URL url = new URL(urlString);
            connection = (HttpURLConnection) url.openConnection();
            connection.setRequestMethod("GET");
            connection.setReadTimeout(100);
            connection.setConnectTimeout(100);

            int responseCode = connection.getResponseCode();
            if (responseCode == HttpURLConnection.HTTP_OK) {
                String response = readResponse(connection);
                logger.info("GET " + endpoint);
                logger.info(response);
                ready = true;
                return new JSONObject(response);
            } else {
                logger.warning("GET Error: " + responseCode);
                ready = false;
            }
        } catch (Exception e) {
            logger.warning("Connection error: " + e.getMessage());
            ready = false;
        } finally {
            if (connection != null) {
                connection.disconnect();
            }
        }
        return null;
    }

    private String readResponse(HttpURLConnection connection) throws IOException {
        BufferedReader reader = new BufferedReader(new InputStreamReader(connection.getInputStream()));
        StringBuilder response = new StringBuilder();
        String line;

        while ((line = reader.readLine()) != null) {
            response.append(line);
        }
        reader.close();

        return response.toString();
    }

    /**
     * For testing purposes - this class should be defined elsewhere in your project
     */
    public static class Limelight {
        public static class Sample {
            public enum Color {
                RED, BLUE, YELLOW, UNKNOWN
            }

            private final Color color;
            private final double x;
            private final double y;
            private final double angle;

            public Sample(Color color, double x, double y, double angle) {
                this.color = color;
                this.x = x;
                this.y = y;
                this.angle = angle;
            }

            public Color getColor() {
                return color;
            }

            public double getX() {
                return x;
            }

            public double getY() {
                return y;
            }

            public double getAngle() {
                return angle;
            }

            @Override
            public String toString() {
                return "Sample{" +
                        "color=" + color +
                        ", x=" + x +
                        ", y=" + y +
                        ", angle=" + angle +
                        '}';
            }
        }
    }
}