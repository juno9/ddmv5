package dji.v5.ux.sample.util.video;

import android.content.Context;
import android.graphics.ImageFormat;
import android.graphics.Rect;
import android.graphics.YuvImage;
import android.media.ExifInterface;
import android.media.MediaCodecInfo;
import android.media.MediaFormat;
import android.os.AsyncTask;
import android.os.Build;
import android.os.Environment;
import android.util.Log;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import android.view.TextureView;
import android.widget.TextView;

import androidx.annotation.NonNull;
import androidx.annotation.RequiresApi;

import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.lang.ref.WeakReference;
import java.nio.ByteBuffer;
import java.time.LocalDateTime;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.ArrayBlockingQueue;

import dji.sdk.keyvalue.value.common.LocationCoordinate3D;
import dji.v5.manager.interfaces.ICameraStreamManager;
import dji.v5.ux.sample.util.DDMMqttClient;
import dji.v5.ux.sample.util.DroneModel;


/**
 *
 */
public class DDMImageHandler implements ICameraStreamManager.CameraFrameListener {
    private static final String TAG = DDMImageHandler.class.getSimpleName();
    private static ArrayBlockingQueue<DDMFrameInfo> processingQueue = new ArrayBlockingQueue<DDMFrameInfo>(1);

    public DDMImageHandler() {

    }

    public DDMImageHandler(Context context, DroneModel model, int videoViewWidth, int videoViewHeight) {
        Log.d(TAG, "DDMImageHandler : " + context);
        this.currentContext = context;
        this.imageWidth = videoViewWidth;
        this.imageHeight = videoViewHeight;
        this.mDrone = model;
        // TODO
//        this.createCodecManager(context, videoViewWidth, videoViewHeight);
    }

//    public synchronized static DDMImageHandler getInstance() {
//        if (instance == null) {
//            instance = new DDMImageHandler();
//        }
//        return instance;
//    }

    public void tick() { // Called ever 100ms...
        ticks += 100;
        Log.e(TAG, "Camera Tick! ");

        this.mDrone.setGimbalRotation(0);

        if (processingQueue == null)
            return;

        if (processingQueue.size() <= 0)
            return;

        try {
            DDMFrameInfo frameInfo = processingQueue.take();
            Log.e(TAG, "Camera Tick: Frame정보 받아옴 :"+frameInfo.getColorFormat());
            switch (frameInfo.getColorFormat()) {
                case MediaCodecInfo.CodecCapabilities.COLOR_FormatYUV420SemiPlanar:
                    Log.e(TAG, "Camera Tick: 케이스 1 ");
                    //NV12
                    if (Build.VERSION.SDK_INT <= 23) {
                        oldSaveYuvDataToJPEG(frameInfo.getRawFrame(), frameInfo.getWidth(), frameInfo.getHeight());
                    } else {
                        newSaveYuvDataToJPEG(frameInfo.getRawFrame(), frameInfo.getWidth(), frameInfo.getHeight());
                    }
                    break;
                case 35:
                    Log.e(TAG, "Camera Tick: 케이스 2 ");

                    newSaveYuvDataToJPEG420P(frameInfo.getRawFrame(), frameInfo.getWidth(), frameInfo.getHeight());
                    break;
                default:
                    break;
            }
        } catch (InterruptedException e) {
            e.printStackTrace();
        }


    }


    public void callYuvDataReceived(int format, final byte[] yuvFrame, int dataSize) {
        //In this demo, we test the YUV data by saving it into JPG files.
        //DJILog.d(TAG, "onYuvDataReceived " + dataSize);
        Log.d(TAG, "callYuvDataReceived recordVideoImage : " + yuvFrame.toString());
        Log.d(TAG, "callYuvDataReceived recordVideoImage format : " + String.valueOf(format) + "w:" + this.imageWidth + "h:" + this.imageHeight);
        if (/*count++ % 30 == 0 && */yuvFrame != null) {
            //DJILog.d(TAG, "onYuvDataReceived2 " + dataSize);
            AsyncTask.execute(new Runnable() {
                @Override
                public void run() {
                    // two samples here, it may has other color format.
                    Log.d(TAG, "callYuvDataReceived run size : " + dataSize);
                    int colorFormat = format; // format.getInteger(MediaFormat.KEY_COLOR_FORMAT);
                    switch (colorFormat) {
                        case MediaCodecInfo.CodecCapabilities.COLOR_FormatYUV420SemiPlanar:
                            //NV12
                            if (Build.VERSION.SDK_INT <= 23) {
                                oldSaveYuvDataToJPEG(yuvFrame, imageWidth, imageHeight);
                            } else {
                                newSaveYuvDataToJPEG(yuvFrame, imageWidth, imageHeight);
                            }
                            break;
                        case MediaCodecInfo.CodecCapabilities.COLOR_FormatYUV420Planar:
                            //YUV420P
                            // newSaveYuvDataToJPEG420P(bytes, width, height);
                            break;
                        default:
                            break;
                    }

                }
            });
        }
    }


    private void newSaveYuvDataToJPEG420P(byte[] yuvFrame, int width, int height) {
        if (yuvFrame.length < width * height) {
            return;
        }
        int length = width * height;

        byte[] u = new byte[width * height / 4];
        byte[] v = new byte[width * height / 4];

        for (int i = 0; i < u.length; i++) {
            u[i] = yuvFrame[length + i];
            v[i] = yuvFrame[length + u.length + i];
        }
        for (int i = 0; i < u.length; i++) {
            yuvFrame[length + 2 * i] = v[i];
            yuvFrame[length + 2 * i + 1] = u[i];
        }

        int seq = 0;
        writeMappingImage(yuvFrame, width, height, false);
    }

    // For android API <= 23
    private void oldSaveYuvDataToJPEG(byte[] yuvFrame, int width, int height) {
        Log.e(TAG, "oldSaveYuvDataToJPEG : " + yuvFrame.toString());
        if (yuvFrame.length < width * height) {
            //DJILog.d(TAG, "yuvFrame size is too small " + yuvFrame.length);
            return;
        }

        byte[] y = new byte[width * height];
        byte[] u = new byte[width * height / 4];
        byte[] v = new byte[width * height / 4];
        byte[] nu = new byte[width * height / 4]; //
        byte[] nv = new byte[width * height / 4];

        System.arraycopy(yuvFrame, 0, y, 0, y.length);
        for (int i = 0; i < u.length; i++) {
            v[i] = yuvFrame[y.length + 2 * i];
            u[i] = yuvFrame[y.length + 2 * i + 1];
        }
        int uvWidth = width / 2;
        int uvHeight = height / 2;
        for (int j = 0; j < uvWidth / 2; j++) {
            for (int i = 0; i < uvHeight / 2; i++) {
                byte uSample1 = u[i * uvWidth + j];
                byte uSample2 = u[i * uvWidth + j + uvWidth / 2];
                byte vSample1 = v[(i + uvHeight / 2) * uvWidth + j];
                byte vSample2 = v[(i + uvHeight / 2) * uvWidth + j + uvWidth / 2];
                nu[2 * (i * uvWidth + j)] = uSample1;
                nu[2 * (i * uvWidth + j) + 1] = uSample1;
                nu[2 * (i * uvWidth + j) + uvWidth] = uSample2;
                nu[2 * (i * uvWidth + j) + 1 + uvWidth] = uSample2;
                nv[2 * (i * uvWidth + j)] = vSample1;
                nv[2 * (i * uvWidth + j) + 1] = vSample1;
                nv[2 * (i * uvWidth + j) + uvWidth] = vSample2;
                nv[2 * (i * uvWidth + j) + 1 + uvWidth] = vSample2;
            }
        }
        //nv21test
        byte[] bytes = new byte[yuvFrame.length];
        System.arraycopy(y, 0, bytes, 0, y.length);
        for (int i = 0; i < u.length; i++) {
            bytes[y.length + (i * 2)] = nv[i];
            bytes[y.length + (i * 2) + 1] = nu[i];
        }

//        screenShot(bytes, Environment.getExternalStorageDirectory() + "/DJI_ScreenShot", width, height);
        int seq = 0;
        writeMappingImage(yuvFrame, width, height, false);
    }

    private void newSaveYuvDataToJPEG(byte[] yuvFrame, int width, int height) {
        Log.e(TAG, "newSaveYuvDataToJPEG : " + yuvFrame.toString());
        if (yuvFrame.length < width * height) {
            //DJILog.d(TAG, "yuvFrame size is too small " + yuvFrame.length);
            return;
        }
        int length = width * height;

        byte[] u = new byte[width * height / 4];
        byte[] v = new byte[width * height / 4];
        for (int i = 0; i < u.length; i++) {
            v[i] = yuvFrame[length + 2 * i];
            u[i] = yuvFrame[length + 2 * i + 1];
        }
        for (int i = 0; i < u.length; i++) {
            yuvFrame[length + 2 * i] = u[i];
            yuvFrame[length + 2 * i + 1] = v[i];
        }
        writeMappingImage(yuvFrame, width, height, false);
    }

    public void writeMappingImage(byte[] yuvFrameBuffer, int imageWidth, int imageHeight, boolean isSdCard) {
        Log.e(TAG, "writeMappingImage : " + yuvFrameBuffer.toString());
        if (!this.isStart) {
            this.isStart = true;
            root = this.createMappingImageDirectory();
        }
//        ContextWrapper cw = new ContextWrapper(this.currentContext);
//        File directory2= cw.getDir("DDM-Img", Context.MODE_PRIVATE);
        Log.e(TAG, "writeMappingImage : recordDirectory : " + recordDirectory.toURI().toString());

        YuvImage yuvImage = new YuvImage(yuvFrameBuffer,
                ImageFormat.NV21,
                imageWidth,
                imageHeight,
                null);
        FileOutputStream outputFile;
        final String path = /*"/storage/emulated/0/DDM-Img"*/ recordDirectory + "/ScreenShot_" + System.currentTimeMillis() + ".jpg";
        try {
            outputFile = new FileOutputStream(new File(path));
        } catch (FileNotFoundException e) {
            Log.e(TAG, "test screenShot: new bitmap output file error: " + e);
            return;
        }
        if (outputFile != null) {
            yuvImage.compressToJpeg(new Rect(0,
                    0,
                    imageWidth,
                    imageHeight), 100, outputFile);
        }
        try {
            outputFile.flush();
            outputFile.close();
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
                this.processingImageInfo(path, imageWidth, imageHeight);
            }
        } catch (IOException e) {
            Log.e(TAG, "test screenShot: compress yuv image error: " + e);
            e.printStackTrace();
        }
    }

    public String createMappingImageDirectory() {
        // TODO Mapping Flight Flag, Start
        try {
            String imgDirName = "IMG_MAPPING_" + android.text.format.DateFormat.format("yyyy-MM-dd-hh-mm-ss", new java.util.Date());



            useDirectory = new File(Environment.getExternalStorageDirectory().getPath() + File.separator + "DDM-Img");

            if (!useDirectory.exists()) {
                useDirectory.mkdir();
            }
            recordDirectory = new File(useDirectory, imgDirName);
            if (!recordDirectory.exists()) {
                recordDirectory.mkdir();
            }
//            ArrayList<File> files = new ArrayList<>(fileNames.length);
//            for (String fileName : fileNames) {
//                files.add(new File(directory, fileName));
//            }
            return imgDirName;
        } catch (Exception e) {
            Log.e(TAG, "ERROR ZIPPING LOGS", e);
        }
        return null;

    }

    public void publishImageInfo(int droneId, Map<String, Object> imageInfo) {
        Log.e(TAG, "publishImageInfo start :" + imageInfo.toString());
        try {
            DDMMqttClient client = DDMMqttClient.getSimpleMqttClient(this.currentContext
                    , "192.168.110.93"
                    , "1883"
                    , "clrobur/mapping/ddm/" // sub

            );
            client.senderJsonMap("clrobur/mapping/drone" + String.valueOf(droneId)
                    , imageInfo
            );
            Log.e(TAG, "TOPIC :" + "clrobur/mapping/drone" + String.valueOf(droneId));
        } catch (Exception e) {
            Log.e(TAG, "publishImageInfo error :" + imageInfo.toString());
            e.printStackTrace();
        }
    }

    private void screenShot(byte[] buf, String shotDir, int width, int height) {
        File dir = new File(shotDir);
        if (!dir.exists() || !dir.isDirectory()) {
            dir.mkdirs();
        }
        YuvImage yuvImage = new YuvImage(buf,
                ImageFormat.NV21,
                width,
                height,
                null);
        FileOutputStream outputFile;
        final String path = dir + "/ScreenShot_" + System.currentTimeMillis() + ".jpg";
        try {
            outputFile = new FileOutputStream(new File(path));
        } catch (FileNotFoundException e) {
            Log.e(TAG, "test screenShot: new bitmap output file error: " + e);
            return;
        }
        if (outputFile != null) {
            yuvImage.compressToJpeg(new Rect(0,
                    0,
                    width,
                    height), 100, outputFile);
        }
        try {
            outputFile.close();
        } catch (IOException e) {
            Log.e(TAG, "test screenShot: compress yuv image error: " + e);
            e.printStackTrace();
        }
//        runOnUiThread(new Runnable() {
//            @Override
//            public void run() {
//                displayPath(path);
//            }
//        });
    }

    @RequiresApi(api = Build.VERSION_CODES.O)
    public void processingImageInfo(String recordedImagePath, int imageWidth, int imageHeight) {

        LocalDateTime time = LocalDateTime.now(); //LocalDateTime.now();

        LocationCoordinate3D coord3d = new LocationCoordinate3D();
        coord3d.setLatitude(mDrone.get_current_lat());
        coord3d.setLongitude(mDrone.get_current_lon());
        coord3d.setAltitude(mDrone.get_current_alt());

        double lat = coord3d.getLatitude();
        double lng = coord3d.getLongitude();
        double alt = coord3d.getAltitude();
        if (Double.isNaN(lat)) {
            lat = DroneModel.DEFUALT_LATITUDE;
        }
        if (Double.isNaN(lng)) {
            lng = DroneModel.DEFUALT_LONGITUDE;
        }
        if (Double.isNaN(alt)) {
            alt = 10;
        }
        float roll = (float) this.mDrone.getmRoll();
        float pitch = (float) this.mDrone.getmPitch();
        float yaw = (float) this.mDrone.getmYaw();

//        alt = 503.393758;
        // TODO exif GPS and Camera Infomation for georeferenced processing at server
        // 차후 기타 정보들을 이미지에 싣자..
        byte[] exifImageByteArray = this.setGeoTag(recordedImagePath
                , lat
                , lng
                , alt
        );
        String imageHex = bytesToHex(exifImageByteArray);
        Map<String, Object> imageMap = new HashMap<String, Object>();
        imageMap.put("dataTime", time);
        imageMap.put("imageWidth", imageWidth);
        imageMap.put("imageHeight", imageHeight);
        imageMap.put("imageSeq", imageIndex++);
        imageMap.put("image", imageHex);
        imageMap.put("lat", (int) (lat * 10000000));
        imageMap.put("lon", (int) (lng * 10000000));
        imageMap.put("alt", (int) (alt * 1000));
        imageMap.put("roll", roll);   // need degree
        imageMap.put("pitch", pitch); // need degree
        imageMap.put("yaw", yaw);     // need degree
        imageMap.put("droneId", this.mDrone.getSystemId());
        imageMap.put("flightId", FLIGHT_ID);
        imageMap.put("storeName", STORE_NAME);
        imageMap.put("layerName", LAYER_NAME);

        this.publishImageInfo(this.mDrone.getSystemId(), imageMap);

    }

    public byte[] setGeoTag(String imagePath, double _latitude, double _longitude, double _altitude) {
//        if (geoTag != null) {
        Log.e(TAG, "setGeoTag : " + imagePath);
        Log.e(TAG, "_latitude : " + _latitude);
        Log.e(TAG, "_longitude : " + _longitude);
        Log.e(TAG, "_altitude : " + _altitude);
        try {
            File image = new File(imagePath);
            ExifInterface exif = new ExifInterface(
                    image.getAbsolutePath());

            double latitude = Math.abs(_latitude);
            double longitude = Math.abs(_longitude);

            int num1Lat = (int) Math.floor(latitude);
            int num2Lat = (int) Math.floor((latitude - num1Lat) * 60);
            double num3Lat = (latitude - ((double) num1Lat + ((double) num2Lat / 60))) * 3600000;

            int num1Lon = (int) Math.floor(longitude);
            int num2Lon = (int) Math.floor((longitude - num1Lon) * 60);
            double num3Lon = (longitude - ((double) num1Lon + ((double) num2Lon / 60))) * 3600000;

            int num1Alt = (int) Math.floor(_altitude);
            int num2Alt = (int) Math.floor((_altitude - num1Alt) * 60);
            double num3Alt = (_altitude - ((double) num1Alt + ((double) num2Alt / 60))) * 3600000;

            String lat = num1Lat + "/1," + num2Lat + "/1," + num3Lat + "/1000";
            String lon = num1Lon + "/1," + num2Lon + "/1," + num3Lon + "/1000";
//            String alt = num1Alt + "/1," + num2Alt + "/1," + num3Alt + "/1000";

            if (_latitude > 0) {
                exif.setAttribute(ExifInterface.TAG_GPS_LATITUDE_REF, "N");
            } else {
                exif.setAttribute(ExifInterface.TAG_GPS_LATITUDE_REF, "S");
            }

            if (_longitude > 0) {
                exif.setAttribute(ExifInterface.TAG_GPS_LONGITUDE_REF, "E");
            } else {
                exif.setAttribute(ExifInterface.TAG_GPS_LONGITUDE_REF, "W");
            }
//            exif.setGpsInfo();
//            TAG_ORIENTATION
//            TAG_GPS_ALTITUDE_REF

            exif.setAttribute(ExifInterface.TAG_GPS_LATITUDE, lat);
            exif.setAttribute(ExifInterface.TAG_GPS_LONGITUDE, lon);
            exif.setAttribute(ExifInterface.TAG_GPS_ALTITUDE, String.valueOf(_altitude * 1000) + "/1000");
            exif.setAttribute(ExifInterface.TAG_GPS_ALTITUDE_REF, Integer.toString(1));
            exif.saveAttributes();

            FileInputStream fis = new FileInputStream(image);
            //create FileInputStream which obtains input bytes from a file in a file system
            //FileInputStream is meant for reading streams of raw bytes such as image data. For reading streams of characters, consider using FileReader.
            ByteArrayOutputStream bos = new ByteArrayOutputStream();
            byte[] buf = new byte[1024];
            try {
                for (int readNum; (readNum = fis.read(buf)) != -1; ) {
                    //Writes to this byte array output stream
                    bos.write(buf, 0, readNum);
//                    System.out.println("bytearray read " + readNum + " bytes,");
                }
            } catch (IOException ex) {
                Log.e(TAG, "image to bytearray error ");
                return null;
            }
            return bos.toByteArray();
        } catch (IOException e) {
            e.printStackTrace();
            return null;
        }
//        } else {
//            return false;
//        }
    }


    public static String bytesToHex(byte[] bytes) {
        char[] hexChars = new char[bytes.length * 2];
        for (int j = 0; j < bytes.length; j++) {
            int v = bytes[j] & 0xFF;
            hexChars[j * 2] = HEX_ARRAY[v >>> 4];
            hexChars[j * 2 + 1] = HEX_ARRAY[v & 0x0F];
        }
        return new String(hexChars);
    }

    private static final char[] HEX_ARRAY = "0123456789ABCDEF".toCharArray();

    /**
     * Data member..
     */
    private long ticks = 0;
    public boolean isStart = false;
    public String root = "";
    public File useDirectory = null;
    public File recordDirectory = null;
    public static String FLIGHT_ID = "FM20210627-1";
    public static String STORE_NAME = "TECHNOPARKV5";
    public static String LAYER_NAME = "PANTOM";
    private static int imageIndex = 0;
    private static final int MSG_WHAT_SHOW_TOAST = 0;
    private static final int MSG_WHAT_UPDATE_TITLE = 1;
    private SurfaceHolder.Callback surfaceCallback;

    @Override
    public void onFrame(@NonNull byte[] frameData, int offset, int length, int width, int height, @NonNull ICameraStreamManager.FrameFormat format) {
        DDMFrameInfo frameInfo = new DDMFrameInfo();
        frameInfo.setColorFormat(ICameraStreamManager.FrameFormat.YUV420_888.value);
        frameInfo.setRawFrame(frameData);
        frameInfo.setWidth(width);
        frameInfo.setHeight(height);

        if (processingQueue.size() >= 1) {//기존에 프레임이 있으면
            processingQueue.remove();
            processingQueue.offer(frameInfo);

        } else//기존에 프레임이 없으면
        {
            processingQueue.offer(frameInfo);

        }
//        Log.e(TAG, "onYuvDataReceived offer processingQueue : " + processingQueue.size());

//        switch (colorFormat) {
//            case MediaCodecInfo.CodecCapabilities.COLOR_FormatYUV420SemiPlanar:
//                //NV12
//                if (Build.VERSION.SDK_INT <= 23) {
//                    oldSaveYuvDataToJPEG(bytes, width, height);
//                } else {
//                    newSaveYuvDataToJPEG(bytes, width, height);
//                }
//                break;
//            case MediaCodecInfo.CodecCapabilities.COLOR_FormatYUV420Planar:
//                //YUV420P
//                 newSaveYuvDataToJPEG420P(bytes, width, height);
//                break;
//            default:
//                break;
//        }

//        if (/*count++ % 30 == 0 && */yuvFrame != null) {
//            final byte[] bytes = new byte[dataSize];
//            yuvFrame.get(bytes);
//            //DJILog.d(TAG, "onYuvDataReceived2 " + dataSize);
//            AsyncTask.execute(new Runnable() {
//                @Override
//                public void run() {
//                    // two samples here, it may has other color format.
//                    int colorFormat = format.getInteger(MediaFormat.KEY_COLOR_FORMAT);
//                    switch (colorFormat) {
//                        case MediaCodecInfo.CodecCapabilities.COLOR_FormatYUV420SemiPlanar:
//                            //NV12
//                            if (Build.VERSION.SDK_INT <= 23) {
//                                oldSaveYuvDataToJPEG(bytes, width, height);
//                            } else {
//                                newSaveYuvDataToJPEG(bytes, width, height);
//                            }
//                            break;
//                        case MediaCodecInfo.CodecCapabilities.COLOR_FormatYUV420Planar:
//                            //YUV420P
//                            // newSaveYuvDataToJPEG420P(bytes, width, height);
//                            break;
//                        default:
//                            break;
//                    }
//
//                }
//            });
//        }
    }

    private enum DemoType {USE_TEXTURE_VIEW, USE_SURFACE_VIEW, USE_SURFACE_VIEW_DEMO_DECODER}

    private static DemoType demoType = DemoType.USE_TEXTURE_VIEW;

    private long createTime;
    private TextureView videostreamPreviewTtView;
    private SurfaceView videostreamPreviewSf;
    private SurfaceHolder videostreamPreviewSh;

    private TextView savePath;
    private StringBuilder stringBuilder;
    private int imageWidth;
    private int imageHeight;
    private int count;
    private DroneModel mDrone;


    private WeakReference<Object> currentWindow;
    private Context currentContext;
}