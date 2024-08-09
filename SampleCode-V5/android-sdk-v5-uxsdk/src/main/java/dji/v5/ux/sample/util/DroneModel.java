package dji.v5.ux.sample.util;


import static dji.sdk.keyvalue.value.camera.CameraMode.PHOTO_NORMAL;
import static dji.sdk.keyvalue.value.camera.CameraStorageLocation.SDCARD;
import static dji.v5.ux.MAVLink.common.msg_mission_ack.MAVLINK_MSG_ID_MISSION_ACK;

import static dji.v5.ux.MAVLink.enums.MAV_COMPONENT.MAV_COMP_ID_AUTOPILOT1;
import static dji.v5.ux.MAVLink.enums.MAV_MISSION_RESULT.MAV_MISSION_ACCEPTED;
import static dji.v5.ux.sample.util.video.DDMImageHandler.bytesToHex;

import android.app.AlertDialog;
import android.content.SharedPreferences;


import android.media.ExifInterface;
import android.os.Build;
import android.os.Environment;
import android.util.Log;


import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.annotation.RequiresApi;


import com.dji.industry.mission.waypointv2.gimbal.Rotation;


import java.io.BufferedOutputStream;
import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetSocketAddress;
import java.net.PortUnreachableException;
import java.net.Socket;
import java.time.LocalDateTime;
import java.util.ArrayList;

import java.util.Collection;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;

import java.util.ListIterator;
import java.util.Map;
import java.util.Timer;
import java.util.concurrent.atomic.AtomicBoolean;


import dji.sdk.keyvalue.key.AirLinkKey;
import dji.sdk.keyvalue.key.BatteryKey;
import dji.sdk.keyvalue.key.CameraKey;

import dji.sdk.keyvalue.key.FlightControllerKey;
import dji.sdk.keyvalue.key.GimbalKey;
import dji.sdk.keyvalue.key.KeyTools;
import dji.sdk.keyvalue.key.RemoteControllerKey;

import dji.sdk.keyvalue.value.camera.CameraStorageLocation;

import dji.sdk.keyvalue.value.camera.MediaFileType;
import dji.sdk.keyvalue.value.common.Attitude;
import dji.sdk.keyvalue.value.common.ComponentIndexType;
import dji.sdk.keyvalue.value.common.EmptyMsg;
import dji.sdk.keyvalue.value.common.LocationCoordinate2D;
import dji.sdk.keyvalue.value.common.LocationCoordinate3D;
import dji.sdk.keyvalue.value.common.Velocity3D;
import dji.sdk.keyvalue.value.flightcontroller.FlightMode;
import dji.sdk.keyvalue.value.flightcontroller.GPSSignalLevel;

import dji.sdk.keyvalue.value.gimbal.GimbalAngleRotation;


import dji.sdk.keyvalue.value.mission.Waypoint;

import dji.sdk.keyvalue.value.mission.WaypointMission;

import dji.sdk.keyvalue.value.remotecontroller.BatteryInfo;

import dji.v5.common.callback.CommonCallbacks;
import dji.v5.common.error.IDJIError;
import dji.v5.manager.KeyManager;
import dji.v5.manager.aircraft.waypoint3.WaylineExecutingInfoListener;
import dji.v5.manager.aircraft.waypoint3.WaypointActionListener;
import dji.v5.manager.aircraft.waypoint3.WaypointMissionExecuteStateListener;
import dji.v5.manager.aircraft.waypoint3.WaypointMissionManager;
import dji.v5.manager.aircraft.waypoint3.model.WaylineExecutingInfo;
import dji.v5.manager.aircraft.waypoint3.model.WaypointMissionExecuteState;
import dji.v5.manager.datacenter.MediaDataCenter;

import dji.v5.manager.datacenter.media.MediaFile;
import dji.v5.manager.datacenter.media.MediaFileDownloadListener;
import dji.v5.manager.datacenter.media.MediaFileListDataSource;
import dji.v5.manager.datacenter.media.MediaFileListState;
import dji.v5.manager.datacenter.media.MediaFileListStateListener;
import dji.v5.manager.datacenter.media.MediaManager;
import dji.v5.manager.datacenter.media.PullMediaFileListParam;
import dji.v5.manager.interfaces.ICameraStreamManager;
import dji.v5.manager.interfaces.IKeyManager;
import dji.v5.manager.interfaces.IMediaManager;
import dji.v5.utils.common.ContextUtil;
import dji.v5.utils.common.DiskUtil;
import dji.v5.utils.common.FileUtils;
import dji.v5.ux.MAVLink.MAVLinkPacket;
import dji.v5.ux.MAVLink.Messages.MAVLinkMessage;
import dji.v5.ux.MAVLink.common.msg_altitude;
import dji.v5.ux.MAVLink.common.msg_attitude;
import dji.v5.ux.MAVLink.common.msg_autopilot_version;
import dji.v5.ux.MAVLink.common.msg_battery_status;
import dji.v5.ux.MAVLink.common.msg_command_ack;
import dji.v5.ux.MAVLink.common.msg_global_position_int;
import dji.v5.ux.MAVLink.common.msg_gps_raw_int;
import dji.v5.ux.MAVLink.common.msg_heartbeat;
import dji.v5.ux.MAVLink.common.msg_home_position;
import dji.v5.ux.MAVLink.common.msg_mission_ack;
import dji.v5.ux.MAVLink.common.msg_mission_count;
import dji.v5.ux.MAVLink.common.msg_mission_item_int;
import dji.v5.ux.MAVLink.common.msg_mission_request_int;
import dji.v5.ux.MAVLink.common.msg_mission_request_list;
import dji.v5.ux.MAVLink.common.msg_param_value;
import dji.v5.ux.MAVLink.common.msg_power_status;
import dji.v5.ux.MAVLink.common.msg_radio_status;
import dji.v5.ux.MAVLink.common.msg_rc_channels;
import dji.v5.ux.MAVLink.common.msg_sys_status;
import dji.v5.ux.MAVLink.common.msg_vfr_hud;
import dji.v5.ux.MAVLink.common.msg_vibration;
import dji.v5.ux.MAVLink.enums.GPS_FIX_TYPE;
import dji.v5.ux.MAVLink.enums.MAV_AUTOPILOT;

import dji.v5.ux.MAVLink.enums.MAV_MISSION_TYPE;
import dji.v5.ux.MAVLink.enums.MAV_MODE_FLAG;
import dji.v5.ux.MAVLink.enums.MAV_PROTOCOL_CAPABILITY;

import dji.v5.ux.MAVLink.enums.MAV_STATE;
import dji.v5.ux.MAVLink.enums.MAV_TYPE;

import dji.v5.ux.sample.showcase.defaultlayout.DefaultLayoutActivity;


public class DroneModel {
    private static final int NOT_USING_GCS_COMMANDED_MODE = -1;

    /*
     *드론 모델 객체가 초기화 되자 마자 listen메소드들을 실행하여 기체로부터 값들을 받아옴
     *이 값들은 변경될 때마다 0.2초 주기로 변경된 값들을 받아 이 클래스 내의 전역변수로 선언되어 있는 값들을 갱신 함
     */
    public DroneModel(DefaultLayoutActivity defaultLayoutActivity) {

        this.parent = defaultLayoutActivity;
        listenIsMotorOn();
        listenIsFly();
        listenAttitude();
        listenDroneBattery();
        listenVelocity();
        listen3DLocation();
        listenRemoteControllerSticks();
        listenControllerBattery();
        listenFlightMode();
        listenAirlinkQuality();
        getfullchargecapacity();

    }


    public void initmediamanager() {
        mediaManager = MediaDataCenter.getInstance().getMediaManager();//전역에 선언되어 있는 미디어 매니저 객체 할당
        parent.Log("mediaManager MediaFileListStateListener initiated");
        mediaManager.addMediaFileListStateListener(new MediaFileListStateListener() {
            @Override
            public void onUpdate(MediaFileListState mediaFileListState) {
                parent.Log("mediaManager MediaFileListStateListener updated");
            }
        });

        mediaManager.enable(new CommonCallbacks.CompletionCallback() {
            @Override
            public void onSuccess() {
                parent.Log("mediaManager enabled");
            }

            @Override
            public void onFailure(@NonNull IDJIError idjiError) {
                parent.Log("mediaManager Not enabled");
            }
        });

        MediaFileListDataSource source = new MediaFileListDataSource(new MediaFileListDataSource.Builder());
        source.setStorageLocation(SDCARD);
        mediaManager.setMediaFileDataSource(source);

    }


    //DDM에서 가져온 메소드들
    public double get_current_lat() {

        return mLatitude;
    }

    public double get_current_lon() {

        return mLongitude;
    }

    public double get_current_alt() {

        return mAlt;
    }

    public void setSystemId(int id) {
        mSystemId = id;
    }

    void setRTLAltitude(final int altitude) {

        keyManager.setValue(KeyTools.createKey(FlightControllerKey.KeyGoHomeHeight), altitude, new CommonCallbacks.CompletionCallback() {
            @Override
            public void onSuccess() {
                ////parent.Log("RTL altitude set to " + altitude + "m");
            }

            @Override
            public void onFailure(@NonNull IDJIError idjiError) {
                ////parent.Log("Error setting RTL altitude " + idjiError.description());
            }
        });


    }

    void setMaxHeight(final int height) {

        keyManager.setValue(KeyTools.createKey(FlightControllerKey.KeyLimitMaxFlightHeightInMeter), height, new CommonCallbacks.CompletionCallback() {
            @Override
            public void onSuccess() {
                ////parent.Log("Max height set to " + height + "m");
            }

            @Override
            public void onFailure(@NonNull IDJIError idjiError) {
                ////parent.Log("Error setting max height " + idjiError.description());
            }
        });

    }

    void do_takeoff(float alt) {
        mAutonomy = false;
        keyManager.setValue(KeyTools.createKey(FlightControllerKey.KeyTakeoffLocationAltitude), (double) alt, new CommonCallbacks.CompletionCallback() {
            @Override
            public void onSuccess() {
                parent.Log("set takeoff alt : " + String.valueOf(alt));
            }

            @Override
            public void onFailure(@NonNull IDJIError idjiError) {
                parent.Log("Error setting takeoff alt " + idjiError.description().toString());
            }
        });//이륙할 고도를 먼저 세팅

        keyManager.performAction(KeyTools.createKey(FlightControllerKey.KeyStartTakeoff), new CommonCallbacks.CompletionCallbackWithParam<EmptyMsg>() {
            @Override
            public void onSuccess(EmptyMsg emptyMsg) {
                parent.Log("start takeoff success ");
            }

            @Override
            public void onFailure(@NonNull IDJIError idjiError) {
                parent.Log("takeoff failed " + idjiError.description().toString());
            }
        });


        Log.d(TAG, "Takeoff started...");
    }

    void armMotors() {

    }

    void disarmMotors() {
    }

    private void send_heartbeat() {

        msg_heartbeat msg = new msg_heartbeat();
        msg.type = MAV_TYPE.MAV_TYPE_QUADROTOR;
        msg.autopilot = MAV_AUTOPILOT.MAV_AUTOPILOT_UDB; // MAV_AUTOPILOT_ARDUPILOTMEGA;  // 차후 dji로 수정하자.// 이거 꼭 써야 함.
//이 값은 드론 펌웨어마다 유형을 분기할 수 있는 값. 현재 DJI용으로 UDB를 쓰고 있다.


        // For base mode logic, see Copter::sendHeartBeat() in ArduCopter/GCS_Mavlink.cpp
        msg.base_mode = MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
        msg.base_mode |= MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;


        //getFlightMode();

        switch (lastMode) {

            case MANUAL: {//조종자가 직접 모든 것을 제어
                msg.custom_mode = ArduCopterFlightModes.ACRO;//수동 바디 프레임 각 속도 및 수동 스로틀, 모든 요소 직접 컨트롤
                break;
            }
            case ATTI: {//태도 안정화만 제공, 위치 유지 불가능
                msg.custom_mode = ArduCopterFlightModes.ALT_HOLD;//고도를 유지하며 롤 피치 요 자동으로 유지, 자세는 유지 되지만 바람에 의해 흘러갈 수 있음
                break;
            }
            case GPS_NORMAL: {//GPS와 비전 시스템으로 위치 설정
                msg.custom_mode = ArduCopterFlightModes.LOITER;//자동 스로틀과 자동 수평 가속도, 자동으로 의치와 자세를 유지함, GPS사용
                break;
            }
            case GPS_SPORT: {//GPS와 비전 시스템으로 위치 설정, 정밀한 hovering 가능, 최대 속도 20m/s
                msg.custom_mode = ArduCopterFlightModes.LOITER;//자동 스로틀과 자동 수평 가속도, 자동으로 의치와 자세를 유지함, GPS사용
                break;
            }
            case GPS_TRIPOD: {//GPS와 비전 시스템으로 위치 설정, 비행 속도와 회전 감도 감소, 정밀한 촬영 가능
                msg.custom_mode = ArduCopterFlightModes.LOITER;//자동 스로틀과 자동 수평 가속도, 자동으로 의치와 자세를 유지함, GPS사용
                break;
            }
            case AUTO_LANDING: {//자동으로 착륙
                msg.custom_mode = ArduCopterFlightModes.LAND;//수평 위치 제어를 사용하는 자동 착륙
                msg.base_mode |= MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED;
                break;
            }
            case FORCE_LANDING: {//강제 착륙
                msg.custom_mode = ArduCopterFlightModes.LAND;//수평 위치 제어를 사용하는 자동 착륙
                msg.base_mode |= MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED;
                break;
            }
            case GO_HOME: {//홈 포인트로 복귀
                msg.custom_mode = ArduCopterFlightModes.RTL;//자동 이륙 지점으로 복귀
                msg.base_mode |= MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED;
                break;
            }
            case WAYPOINT: {//경로 설정
                msg.custom_mode = ArduCopterFlightModes.AUTO;//미션 명령을 사용하는 완전 자동 웨이포인트 제어
                break;
            }
            case POI: {//관심 지점 설정
                break;
            }
            case VIRTUAL_STICK: {//가상 조이스틱 사용
                break;
            }
            case SMART_FLY: {//스마트 비행 기능 사용
                break;
            }
            case AUTO_AVOIDANCE: {//자동 회피 기능 사용
                break;
            }
            case APAS: {//고급 조종사 지원 시스템(Auto Pilot Assistance System) 사용
                break;
            }
            case MOTOR_START: {//모터 시작
                break;
            }
            case TAKE_OFF_READY: {//이륙 준비 완료
                break;
            }
            case AUTO_TAKE_OFF: {//자동으로 이륙
                msg.custom_mode = ArduCopterFlightModes.GUIDED;//즉시 명령을 사용하여 GCS로 좌표 또는 속도/방향으로 완전 자동 비행
                msg.base_mode |= MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED;
                break;
            }
            case TAP_FLY: {
                msg.custom_mode = ArduCopterFlightModes.GUIDED;//즉시 명령을 사용하여 GCS로 좌표 또는 속도/방향으로 완전 자동 비행
                break;
            }
            case SMART_FLIGHT: {
                break;
            }
            case PANO: {
                break;
            }
            case DRAW: {
                msg.base_mode |= MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED;
                break;
            }
            case FOLLOW_ME: {
                msg.custom_mode = ArduCopterFlightModes.GUIDED;//즉시 명령을 사용하여 GCS로 좌표 또는 속도/방향으로 완전 자동 비행
                msg.base_mode |= MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED;
                break;
            }
            case ATTI_LANDING: {
                break;
            }
            case CLICK_GO: {
                break;
            }
            case CINEMATIC: {
                break;
            }
            case GPS_NOVICE: {
                break;
            }
            case QUICK_MOVIE: {
                break;
            }
            case MASTER_SHOT: {
                break;
            }
            case TIME_LAPSE: {
                break;
            }
            case UNKNOWN: {
                break;
            }


        }

        if (mGCSCommandedMode == ArduCopterFlightModes.AUTO)
            msg.custom_mode = ArduCopterFlightModes.AUTO;
        if (mGCSCommandedMode == ArduCopterFlightModes.GUIDED)
            msg.custom_mode = ArduCopterFlightModes.GUIDED;
        if (mGCSCommandedMode == ArduCopterFlightModes.BRAKE)
            msg.custom_mode = ArduCopterFlightModes.BRAKE;

        if (isMotorOn)
            msg.base_mode |= MAV_MODE_FLAG.MAV_MODE_FLAG_SAFETY_ARMED;

        if (mAutonomy)
            msg.base_mode |= MAV_MODE_FLAG.MAV_MODE_FLAG_AUTO_ENABLED;

        // Catches manual landings
        // Automatically disarm motors if aircraft is on the ground and a takeoff is not in progress

        if (!isFlying && mGCSCommandedMode != ArduCopterFlightModes.GUIDED)
            isMotorOn = false;


        // Catches manual takeoffs
        if (isMotorOn)
            isMotorOn = true;

        msg.system_status = MAV_STATE.MAV_STATE_ACTIVE;
        msg.mavlink_version = 3;
        // ////parent.Log("send_heartbeat %9%");
        sendMessage(msg);
    }

    private void send_attitude() {
        //   ////parent.Log("send_attitude %1% ");
        msg_attitude msg = new msg_attitude();
        // TODO: this next line causes an exception
        //msg.time_boot_ms = getTimestampMilliseconds();
        msg.roll = (float) (mRoll * Math.PI / 180);
        msg.pitch = (float) (mPitch * Math.PI / 180);
        msg.yaw = (float) (mYaw * Math.PI / 180);
        // TODO msg.rollspeed = 0;
        // TODO msg.pitchspeed = 0;
        // TODO msg.yawspeed = 0;

        sendMessage(msg);
    }

    private void send_altitude() {
        //  ////parent.Log("send_altitude %2% ");
        msg_altitude msg = new msg_altitude();
        msg.altitude_relative = (int) (mAlt);
        mAlt = msg.altitude_relative;
        sendMessage(msg);
    }

    private void send_vibration() {
        // ////parent.Log("send_vibration %3% ");
        msg_vibration msg = new msg_vibration();
        sendMessage(msg);
    }

    private void send_vfr_hud() {

        msg_vfr_hud msg = new msg_vfr_hud();

        // Mavlink: Current airspeed in m/s
        // DJI: unclear whether getState() returns airspeed or groundspeed
        msg.airspeed = (float) (Math.sqrt(Math.pow(velocityx, 2) +
                Math.pow(velocityy, 2)));

        // Mavlink: Current ground speed in m/s. For now, just echoing airspeed.
        msg.groundspeed = msg.airspeed;

        // Mavlink: Current heading in degrees, in compass units (0..360, 0=north)
        // TODO: unspecified in Mavlink documentation whether this heading is true or magnetic
        // DJI=[-180,180] where 0 is true north, Mavlink=degrees
        double yaw = this.mYaw;
        if (yaw < 0)
            yaw += 360;
        msg.heading = (short) yaw;

        // Mavlink: Current throttle setting in integer percent, 0 to 100
        msg.throttle = mThrottleSetting;

        // Mavlink: Current altitude (MSL), in meters
        // DJI: relative altitude is altitude of the aircraft relative to take off location, measured by barometer, in meters.
        // DJI: home altitude is home point's altitude. Units unspecified in DJI SDK documentation. Presumably meters AMSL.
        msg.alt = (int) (mAlt);

        // Mavlink: Current climb rate in meters/second
        // DJI: m/s, positive values down
        msg.climb = -(short) (velocityz);
        //  ////parent.Log("send_vfr_hud %4% ");
        sendMessage(msg);
    }

    private void send_global_position_int() {
        msg_global_position_int msg = new msg_global_position_int();

        msg.lat = (int) (mLatitude * Math.pow(10, 7));
        msg.lon = (int) (mLongitude * Math.pow(10, 7));
        // DEFUALT_LATITUDE DEFUALT_LONGITUDE
        if (isDev) {
            if (msg.lat <= 0 || msg.lon <= 0) {
                msg.lat = (int) ((DEFUALT_LATITUDE + (mSystemId * 0.001)) * Math.pow(10, 7));
                msg.lon = (int) ((DEFUALT_LONGITUDE + (mSystemId * 0.001)) * Math.pow(10, 7));
            }
        }
//        Log.d(TAG, "GPS :: Lat:: " +  String.valueOf(msg.lat) + " Lon:: " + String.valueOf(msg.lon));
        // NOTE: Commented out this field, because msg.relative_alt seems to be intended for altitude above the current terrain,
        // but DJI reports altitude above home point.
        // Mavlink: Millimeters above ground (unspecified: presumably above home point?)
        // DJI: relative altitude of the aircraft relative to take off location, measured by barometer, in meters.
        msg.relative_alt = (int) (mAlt);

        //        Log.d(TAG, "msg_global_position_int :: ALT :: " +  String.valueOf(msg.relative_alt/1000) + " origin :" + String.valueOf(coord.getAltitude()) );

        // SDK 에서 AMSL 고도를 얻는 방법 찾아야함. 현재 없음.
        // Mavlink: Millimeters AMSL
        // msg.alt = ??? No method in SDK for obtaining MSL altitude.
        // djiAircraft.getFlightController().getState().getHomePointAltitude()) seems promising, but always returns 0

        // Mavlink: m/s*100
        // DJI: m/s
        msg.vx = (short) (velocityx * 100); // positive values N
        msg.vy = (short) (velocityy * 100); // positive values E
        msg.vz = (short) (velocityz * 100); // positive values down

        // DJI=[-180,180] where 0 is true north, Mavlink=degrees
        // TODO unspecified in Mavlink documentation whether this heading is true or magnetic
        double yaw = this.mYaw;
        if (yaw < 0)
            yaw += 360;
        msg.hdg = (int) (yaw * 100);
        // //parent.Log("send_global_position_int %5% ");
        sendMessage(msg);
    }

    private void send_gps_raw_int() {
        msg_gps_raw_int msg = new msg_gps_raw_int();


        msg.time_usec = getTimestampMicroseconds();
        msg.lat = (int) (mLatitude * Math.pow(10, 7));
        msg.lon = (int) (mLongitude * Math.pow(10, 7));

        // DEFUALT_LATITUDE DEFUALT_LONGITUDE
        if (isDev) {
            if (msg.lat <= 0 || msg.lon <= 0) {
                msg.lat = (int) ((DEFUALT_LATITUDE + (getSystemId() * 0.001)) * Math.pow(10, 7));
                msg.lon = (int) ((DEFUALT_LONGITUDE + (getSystemId() * 0.001)) * Math.pow(10, 7));
            }
        }

        // TODO msg.alt
        // TODO msg.eph
        // TODO msg.epv
        // TODO msg.vel
        // TODO msg.cog
        keyManager.getValue(KeyTools.createKey(FlightControllerKey.KeyGPSSatelliteCount), new CommonCallbacks.CompletionCallbackWithParam<Integer>() {
            @Override
            public void onSuccess(Integer integer) {
                msg.satellites_visible = integer.shortValue();
            }

            @Override
            public void onFailure(@NonNull IDJIError idjiError) {
                //////parent.Log("KeyGPSSatelliteCount Failed");
            }
        });

        // DJI reports signal quality on a scale of 1-5
        // Mavlink has separate codes for fix type.
        final GPSSignalLevel[] gpsLevel = new GPSSignalLevel[1];
        keyManager.getValue(KeyTools.createKey(FlightControllerKey.KeyGPSSignalLevel), new CommonCallbacks.CompletionCallbackWithParam<GPSSignalLevel>() {
            @Override
            public void onSuccess(GPSSignalLevel gpsSignalLevel) {
                gpsLevel[0] = gpsSignalLevel;
            }

            @Override
            public void onFailure(@NonNull IDJIError idjiError) {
                //////parent.Log("KeyGPSSignalLevel Failed");
            }
        });

        if (gpsLevel[0] == GPSSignalLevel.LEVEL_NONE)
            msg.fix_type = GPS_FIX_TYPE.GPS_FIX_TYPE_NO_FIX;
        if (gpsLevel[0] == GPSSignalLevel.LEVEL_0 || gpsLevel[0] == GPSSignalLevel.LEVEL_1)
            msg.fix_type = GPS_FIX_TYPE.GPS_FIX_TYPE_NO_FIX;
        if (gpsLevel[0] == GPSSignalLevel.LEVEL_2)
            msg.fix_type = GPS_FIX_TYPE.GPS_FIX_TYPE_2D_FIX;
        if (gpsLevel[0] == GPSSignalLevel.LEVEL_3 || gpsLevel[0] == GPSSignalLevel.LEVEL_4 ||
                gpsLevel[0] == GPSSignalLevel.LEVEL_5)
            msg.fix_type = GPS_FIX_TYPE.GPS_FIX_TYPE_3D_FIX;
        //////parent.Log("send_gps_raw_int %6% ");
        sendMessage(msg);
    }

    private void send_radio_status() {
        msg_radio_status msg = new msg_radio_status();
        msg.rssi = 0; // TODO: work out units conversion (see issue #1)
        msg.remrssi = 0; // TODO: work out units conversion (see issue #1)
        // ////parent.Log("send_radio_status %7%");
        sendMessage(msg);
    }

    private void send_rc_channels() {
        msg_rc_channels msg = new msg_rc_channels();
        msg.rssi = (short) mUplinkQuality;
        msg.chan1_raw = mLeftStickVertical;
        msg.chan2_raw = mLeftStickHorisontal;
        msg.chan3_raw = mRightStickVertical;
        msg.chan4_raw = mRightStickHorisontal;
        msg.chan5_raw = mC1 ? 1000 : 2000;
        msg.chan6_raw = mC2 ? 1000 : 2000;
        msg.chan7_raw = mC3 ? 1000 : 2000;

        // Cancel all AI modes if stick is moved...
        if ((mLeftStickVertical > 1550 || mLeftStickVertical < 1450) ||
                (mLeftStickHorisontal > 1550 || mLeftStickHorisontal < 1450) ||
                (mRightStickVertical > 1550 || mRightStickVertical < 1450) ||
                (mRightStickHorisontal > 1550 || mRightStickHorisontal < 1450)) {
            if (mAIfunction_activation != 0) {
                ////parent.Log("AI Mode Canceled...");
                mAIfunction_activation = 0;
            }

            mAutonomy = false;
            //    pauseWaypointMission();  // TODO:: halt mission for safety...
        }

        msg.chan8_raw = (mAIfunction_activation * 100) + 1000;
        msg.chancount = 8;
        // ////parent.Log("send_rc_channels %8%");
        sendMessage(msg);
    }

    private void send_sys_status() {
        msg_sys_status msg = new msg_sys_status();


        if (mCFullChargeCapacity_mAh > 0) {
            msg.battery_remaining = (byte) ((float) mCChargeRemaining_mAh / (float) mCFullChargeCapacity_mAh * 100.0);
            //       Log.d(TAG, "calc'ed bat remain: " + String.valueOf(msg.battery_remaining));
        } else {
            Log.d(TAG, "mCFullChargeCapacity_mAh == 0...");
            msg.battery_remaining = 100; // Prevent divide by zero
        }

        msg.voltage_battery = mCVoltage_mV;
        msg.current_battery = (short) mCCurrent_mA;
        //  ////parent.Log("send_sys_status %10%");
        sendMessage(msg);
    }

    private void send_power_status() {
        msg_power_status msg = new msg_power_status();
        // ////parent.Log("send_power_status %11%");
        sendMessage(msg);
    }

    private void send_battery_status() {
        msg_battery_status msg = new msg_battery_status();
        msg.current_consumed = mCFullChargeCapacity_mAh - mCChargeRemaining_mAh;
        msg.voltages = mCellVoltages;
        float mBatteryTemp_C = 0;
        msg.temperature = (short) (mCBatteryTemp_C * 100);
        msg.current_battery = (short) (mCCurrent_mA * 10);
        msg.battery_remaining = (byte) ((float) mCChargeRemaining_mAh / (float) mCFullChargeCapacity_mAh * 100.0);
        msg.time_remaining = 0;
        //     Log.d(TAG, "temp: " + String.valueOf(mBatteryTemp_C));
        //      Log.d(TAG, "send_battery_status() complete");
        // TODO cell voltages

        // ////parent.Log("send_battery_status %12%");
        sendMessage(msg);
    }

    void send_home_position() {
        msg_home_position msg = new msg_home_position();

        msg.latitude = (int) (mLatitude * Math.pow(10, 7));
        msg.longitude = (int) (mLongitude * Math.pow(10, 7));
        if (isDev) {
            if (msg.latitude <= 0 || msg.longitude <= 0) {
                msg.latitude = (int) ((DEFUALT_LATITUDE + (mSystemId * 0.001)) * Math.pow(10, 7));
                msg.longitude = (int) ((DEFUALT_LONGITUDE + (mSystemId * 0.001)) * Math.pow(10, 7));
            }
        }

        // msg.altitude = (int) (djiAircraft.getFlightController().getState().getHomePointAltitude());
//        msg.altitude = (int) (djiAircraft.getFlightController().getState().getGoHomeHeight()); 이거 V5에서 똑같은 역할 하는거 찾아야 함

        // msg.x = 0;
        // msg.y = 0;
        // msg.z = 0;
        // msg.approach_x = 0;
        // msg.approach_y = 0;
        // msg.approach_z = 0;
        // ////parent.Log("send_home_position %13%");
        sendMessage(msg);
    }

    void set_home_position(double lat, double lon) {


        keyManager.setValue(KeyTools.createKey(FlightControllerKey.KeyHomeLocation), new LocationCoordinate2D(lat, lon), new CommonCallbacks.CompletionCallback() {
            @Override
            public void onSuccess() {
                parent.Log("set_home_position Done");
            }

            @Override
            public void onFailure(@NonNull IDJIError idjiError) {
                parent.Log("set_home_position Failed");
            }
        });
    }

    void set_RTL_Heigt(int height) {


        keyManager.setValue(KeyTools.createKey(FlightControllerKey.KeyGoHomeHeight), height, new CommonCallbacks.CompletionCallback() {
            @Override
            public void onSuccess() {
                toastinMain("RTL 고도: " + height + "M");
            }

            @Override
            public void onFailure(@NonNull IDJIError idjiError) {
                toastinMain("RTL 고도 설정 실패");
            }
        });
    }

    void set_RTL_speed(double speed) {


        keyManager.setValue(KeyTools.createKey(FlightControllerKey.KeyGoHomeSpeed), speed, new CommonCallbacks.CompletionCallback() {
            @Override
            public void onSuccess() {
                toastinMain("RTL 속도: " + speed + "Km/s");
            }

            @Override
            public void onFailure(@NonNull IDJIError idjiError) {
                toastinMain("RTL 고도 설정 실패");
            }
        });
    }


    void send_autopilot_version() {
        msg_autopilot_version msg = new msg_autopilot_version();
        msg.capabilities = MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_COMMAND_INT;
        msg.capabilities |= MAV_PROTOCOL_CAPABILITY.MAV_PROTOCOL_CAPABILITY_MISSION_INT;
        msg.os_sw_version = 0x040107;
        msg.middleware_sw_version = 0x040107;
        msg.flight_sw_version = 0x040107;
        msg.compid = 0;

        sendMessage(msg);


    }

    //    void do_set_motion_velocity(float x, float y, float z, float yaw, int mask) {
//        //    Log.i(TAG, "do_set_motion_velocity");
//
//        // If we use yaw rate...
//        if ((mask & 0b0000100000000000) == 0) {
//            mYaw = yaw;
//        }
//        if ((mask & 0b0000000000001000) == 0) {
//            mPitch = y;
//        }
//        if ((mask & 0b0000000000010000) == 0) {
//            mRoll = x;
//        }
//        if ((mask & 0b0000000000100000) == 0) {
//            mThrottle = z;
//        }
//
//        // Only allow velocity movement in P mode...
//        if (rcmode == avtivemode) {
//            parent.logMessageDJI(":rcmode != avtivemode " + rcmode + "   " + avtivemode);
//            Log.i(TAG, ":rcmode != avtivemode " + rcmode + "   " + avtivemode);
//            return;
//        }
//
//        mFlightController.getVirtualStickModeEnabled(new CommonCallbacks.CompletionCallbackWith<Boolean>() {
//            @Override
//            public void onSuccess(Boolean aBoolean) {
//                if (aBoolean == false) {
//                    // After a manual mode change, we might loose the JOYSTICK mode...
//                    if (lastMode != FlightMode.MANUAL) {
//                        mFlightController.setVirtualStickModeEnabled(true, djiError -> {
//                                    if (djiError != null) {
//                                        Log.i(TAG, "Velocity Mode not enabled Error: " + djiError.toString());
//                                        parent.logMessageDJI("Velocity Mode not enabled Error: " + djiError.toString());
//                                    } else {
//                                        mFlightController.setVirtualStickAdvancedModeEnabled(true);
//                                    }
//                                }
//                        );
//                    }
//                }
//            }
//
//
//
//
//            @Override
//            public void onFailure(DJIError error) {
//                Log.e(TAG, "Can Not get VirtualStick mode...");
//            }
//        });
//
//        // If first time...
//        if (null == mSendVirtualStickDataTimer) {
//            mSendVirtualStickDataTask = new SendVelocityDataTask();
//            mSendVirtualStickDataTimer = new Timer();
//            mSendVirtualStickDataTimer.schedule(mSendVirtualStickDataTask, 0, 50);
//        } else {
//            mSendVirtualStickDataTask.repeat = 2;
//        }
//    }
//
//    boolean loadParamsFromDJI() {
//        if (getDjiAircraft() == null)//이거는 연결 여부로 바꾸고
//            return false;
//        for (int i = 0; i < getParams().size(); i++) {
//            switch (getParams().get(i).getParamName()) {
//                case "DJI_CTRL_MODE":
//                    getDjiAircraft().getFlightController().getControlMode(new ParamControlModeCallback(i));
//                    break;
//                case "DJI_ENBL_LEDS":
////                    getDjiAircraft().getFlightController().getLEDsEnabled(new ParamBooleanCallback(i));
//                    break;
//                case "DJI_ENBL_QSPIN":
//                    getDjiAircraft().getFlightController().getQuickSpinEnabled(new ParamBooleanCallback(i));
//                    break;
//                case "DJI_ENBL_RADIUS":
//                    getDjiAircraft().getFlightController().getMaxFlightRadiusLimitationEnabled(new ParamBooleanCallback(i));
//                    break;
//                case "DJI_ENBL_TFOLLOW":
//                    getDjiAircraft().getFlightController().getTerrainFollowModeEnabled(new ParamBooleanCallback(i));
//                    break;
//                case "DJI_ENBL_TRIPOD":
//                    getDjiAircraft().getFlightController().getTripodModeEnabled(new ParamBooleanCallback(i));
//                    break;
//                case "DJI_FAILSAFE":
//                    getDjiAircraft().getFlightController().getConnectionFailSafeBehavior(new ParamConnectionFailSafeBehaviorCallback(i));
//                    break;
//                case "DJI_LOW_BAT":
//                    getDjiAircraft().getFlightController().getLowBatteryWarningThreshold(new ParamIntegerCallback(i));
//                    break;
//                case "DJI_MAX_HEIGHT":
//                    getDjiAircraft().getFlightController().getMaxFlightHeight(new ParamIntegerCallback(i));
//                    break;
//                case "DJI_MAX_RADIUS":
//                    getDjiAircraft().getFlightController().getMaxFlightRadius(new ParamIntegerCallback(i));
//                    break;
//                case "DJI_RLPCH_MODE":
//                    if (getDjiAircraft().getFlightController().getRollPitchControlMode() == RollPitchControlMode.ANGLE)
//                        getParams().get(i).setParamValue(0f);
//                    else if (getDjiAircraft().getFlightController().getRollPitchControlMode() == RollPitchControlMode.VELOCITY)
//                        getParams().get(i).setParamValue(1f);
//                    break;
//                case "DJI_RTL_HEIGHT":
//                    getDjiAircraft().getFlightController().getGoHomeHeightInMeters(new ParamIntegerCallback(i));
//                    break;
//                case "DJI_SERIOUS_BAT":
//                    getDjiAircraft().getFlightController().getSeriousLowBatteryWarningThreshold(new ParamIntegerCallback(i));
//                    break;
//                case "DJI_SMART_RTH":
//                    getDjiAircraft().getFlightController().getSmartReturnToHomeEnabled(new ParamBooleanCallback(i));
//                    break;
//                case "DJI_VERT_MODE":
//                    if (getDjiAircraft().getFlightController().getVerticalControlMode() == VerticalControlMode.VELOCITY)
//                        getParams().get(i).setParamValue(0f);
//                    else if (getDjiAircraft().getFlightController().getVerticalControlMode() == VerticalControlMode.POSITION)
//                        getParams().get(i).setParamValue(1f);
//                    break;
//                case "DJI_YAW_MODE":
//                    if (getDjiAircraft().getFlightController().getYawControlMode() == YawControlMode.ANGLE)
//                        getParams().get(i).setParamValue(0f);
//                    else if (getDjiAircraft().getFlightController().getYawControlMode() == YawControlMode.ANGULAR_VELOCITY)
//                        getParams().get(i).setParamValue(1f);
//                    break;
//            }
//        }
//        return true;
//    }
    void send_param(int index) {
        MAVParam param = params.get(index);
        send_param(param.getParamName(),
                param.getParamValue(),
                param.getParamType(),
                params.size(),
                index);
    }

    private void send_param(String key, float value, short type, int count, int index) {

        msg_param_value msg = new msg_param_value();
        msg.setParam_Id(key);
        msg.param_value = value;
        msg.param_type = type;
        msg.param_count = count;
        msg.param_index = index;
        Log.d("Rosetta", "Sending param: " + msg.toString());
        sendMessage(msg);
    }

    void send_all_params() {
        for (int i = 0; i < params.size(); i++)
            send_param(i);
    }

//    void changeParam(MAVParam param) {
//        for (int i = 0; i < getParams().size(); i++) {
//            if (getParams().get(i).getParamName().equals(param.getParamName())) {
//                getParams().get(i).setParamValue(param.getParamValue());
//                switch (param.getParamName()) {
//                    case "DJI_CTRL_MODE":
//                        if (param.getParamValue() == 0)
//
//                        keyManager.setValue(KeyTools.createKey(FlightControllerKey.KeyFlightMode), FlightMode.MANUAL, new CommonCallbacks.CompletionCallback() {
//                            @Override
//                            public void onSuccess() {
//
//                            }
//
//                            @Override
//                            public void onFailure(@NonNull IDJIError idjiError) {
//
//                            }
//                        });
//                        else if (param.getParamValue() == 2)
//                            getDjiAircraft().getFlightController().setControlMode(FlightMode.SMART_FLY, new ParamWriteCompletionCallback(i));
//                        else if (param.getParamValue() == 255)
//                            getDjiAircraft().getFlightController().setControlMode(FlightMode.UNKNOWN, new ParamWriteCompletionCallback(i));
//                        break;
//                    case "DJI_ENBL_LEDS":
////                        getDjiAircraft().getFlightController().setLEDsEnabled(param.getParamValue() > 0, new ParamWriteCompletionCallback(i));
//                        break;
//                    case "DJI_ENBL_QSPIN":
//                        getDjiAircraft().getFlightController().setAutoQuickSpinEnabled(param.getParamValue() > 0, new ParamWriteCompletionCallback(i));
//                        break;
//                    case "DJI_ENBL_RADIUS":
//                        getDjiAircraft().getFlightController().setMaxFlightRadiusLimitationEnabled((param.getParamValue() > 0), new ParamWriteCompletionCallback(i));
//                        break;
//                    case "DJI_ENBL_TFOLLOW":
//                        getDjiAircraft().getFlightController().setTerrainFollowModeEnabled(param.getParamValue() > 0, new ParamWriteCompletionCallback(i));
//                        break;
//                    case "DJI_ENBL_TRIPOD":
//                        getDjiAircraft().getFlightController().setTripodModeEnabled(param.getParamValue() > 0, new ParamWriteCompletionCallback(i));
//                        break;
//                    case "DJI_FAILSAFE":
//                        if (param.getParamValue() == 0)
//                            getDjiAircraft().getFlightController().setConnectionFailSafeBehavior(ConnectionFailSafeBehavior.HOVER, new ParamWriteCompletionCallback(i));
//                        else if (param.getParamValue() == 1)
//                            getDjiAircraft().getFlightController().setConnectionFailSafeBehavior(ConnectionFailSafeBehavior.LANDING, new ParamWriteCompletionCallback(i));
//                        else if (param.getParamValue() == 2)
//                            getDjiAircraft().getFlightController().setConnectionFailSafeBehavior(ConnectionFailSafeBehavior.GO_HOME, new ParamWriteCompletionCallback(i));
//                        else if (param.getParamValue() == 255)
//                            getDjiAircraft().getFlightController().setConnectionFailSafeBehavior(ConnectionFailSafeBehavior.UNKNOWN, new ParamWriteCompletionCallback(i));
//                        break;
//                    case "DJI_LOW_BAT":
//                        getDjiAircraft().getFlightController().setLowBatteryWarningThreshold(Math.round(param.getParamValue()), new ParamWriteCompletionCallback(i));
//                        break;
//                    case "DJI_MAX_HEIGHT":
//                        getDjiAircraft().getFlightController().setMaxFlightHeight(Math.round(param.getParamValue()), new ParamWriteCompletionCallback(i));
//                        break;
//                    case "DJI_MAX_RADIUS":
//                        getDjiAircraft().getFlightController().setMaxFlightRadius(Math.round(param.getParamValue()), new ParamWriteCompletionCallback(i));
//                        break;
//                    case "DJI_RLPCH_MODE":
//                        if (param.getParamValue() == 0)
//                            getDjiAircraft().getFlightController().setRollPitchControlMode(RollPitchControlMode.ANGLE);
//                        else if (param.getParamValue() == 1)
//                            getDjiAircraft().getFlightController().setRollPitchControlMode(RollPitchControlMode.VELOCITY);
//                        break;
//                    case "DJI_RTL_HEIGHT":
//                        getDjiAircraft().getFlightController().setGoHomeHeightInMeters(Math.round(param.getParamValue()), new ParamWriteCompletionCallback(i));
//                        break;
//                    case "DJI_SERIOUS_BAT":
//                        getDjiAircraft().getFlightController().setSeriousLowBatteryWarningThreshold(Math.round(param.getParamValue()), new ParamWriteCompletionCallback(i));
//                        break;
//                    case "DJI_SMART_RTH":
//                        getDjiAircraft().getFlightController().setSmartReturnToHomeEnabled(param.getParamValue() > 0, new ParamWriteCompletionCallback(i));
//                        break;
//                    case "DJI_VERT_MODE":
//                        if (param.getParamValue() == 0)
//                            getDjiAircraft().getFlightController().setVerticalControlMode(VerticalControlMode.VELOCITY);
//                        else if (param.getParamValue() == 1)
//                            getDjiAircraft().getFlightController().setVerticalControlMode(VerticalControlMode.POSITION);
//                        break;
//                    case "DJI_YAW_MODE":
//                        if (param.getParamValue() == 0)
//                            getDjiAircraft().getFlightController().setYawControlMode(YawControlMode.ANGLE);
//                        else if (param.getParamValue() == 1)
//                            getDjiAircraft().getFlightController().setYawControlMode(YawControlMode.ANGULAR_VELOCITY);
//                        break;
//                    default:
//                        parent.logMessageDJI("Unknown parameter name");
//
//                }
//                send_param(i);
//                break;
//            }
//        }
//        Log.d(TAG, "Request to set param that doesn't exist");
//
//    }

    void request_mission_item(int seq) {
        // Reset internal list
        if (seq == 0) {
            stopWaypointMission();
            //m_activeWaypointMission = null; //임시 주석처리, 현재 활성화 되어 있는 웨이포인트 미션을 담아둔 객체, 비슷한 역할을 할 객체를 V5에서 찾아야 한다.
        }

        msg_mission_request_int msg = new msg_mission_request_int();
        msg.sysid = getSystemId();
        msg.seq = seq;
        msg.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
        sendMessage(msg);
        parent.Log("sent message: " + msg);
    }

    public void stopWaypointMission() {
        mAutonomy = false;
        //내용 달아야 함

    }


//    public void goto_position(double Lat, double Lon, float alt, float head) {
//        //TimeLine.TimeLineGoTo(Lat, Lon, alt, (float) 2.0, head);
//        //TimeLine.startTimeline();
//        gotoNoPhoto = true;
//        do_set_motion_absolute(Lat, Lon, alt, head, 2.5f, 2.5f, 2.5f, 2.5f, 0);
//    }
//    public void do_set_motion_absolute(double Lat, double Lon, float alt, float head, float vx, float vy, float vz, float yaw_rate, int mask) {
//        //    Log.i(TAG, "do_set_motion_absolute");
//        photoTaken = false;
//
//        // Set our new destination...
//        mDestinationLat = Lat;
//        mDestinationLon = Lon;
//        mDestinationAlt = alt;
//        mDestinationYaw = head;
//        mDestinationYawrate = yaw_rate;
//        mDestinationSetVx = vx;
//        mDestinationSetVy = vy;
//        mDestinationSetVz = vz;
//        mDestinationMask = mask;
//        m_lastCommand = MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT;
//
//        //------------------------------------------------
//        // To be able to follow a straight line...
//
//        double locallat = mLatitude;
//        double locallon = mLongitude;
//
//        // Find the bearing to wp... return 0-360 deg.
//        mDestinationbrng = getBearingBetweenWaypoints(mDestinationLat, mDestinationLon, locallat, locallon) - 180;
//        if (mDestinationbrng < 0) mDestinationbrng = mDestinationbrng + 360;
//
//        // The direct distance to the wp... return distance in meters...
//        mDestinationhypotenuse = getRangeBetweenWaypointsm(mDestinationLat, mDestinationLon, 0, locallat, locallon, 0);
//        Log.i(TAG, "mDestinationhypotenuse: " + mDestinationhypotenuse + " mDestinationbrng: " + mDestinationbrng);
//
//        do_start_absolute_motion();
//    }
//    private void do_start_absolute_motion() {
//
//        if (mMoveToDataTimer == null) {
//            parent.logMessageDJI("Starting Absolution Mission!");
//
//            if(!m_CruisingMode)
//            {
//                miniPIDFwd.reset();
//                miniPIDSide.reset();
//                miniPIDAlti.reset();
//                miniPIDHeading.reset();
//            }
//
//
//            mMoveToDataTask = new MoveTo();
//            mMoveToDataTimer = new Timer();
//
//            mMoveToDataTimer.schedule(mMoveToDataTask, 100, 50);
//            mAutonomy = true;
//        } else {
//            mMoveToDataTask.detection = 0;
//        }
//    }


    public void initMissionManager() {

        mMissionManager = WaypointMissionManager.getInstance();
    }

    public void toastinMain(String inputfrommodel) {
        parent.toast(inputfrommodel);
    }

    private void SetMesasageBox(String msg) {
        AlertDialog.Builder alertDialog2 = new AlertDialog.Builder(parent);
        alertDialog2.setTitle(msg);
        alertDialog2.setMessage("Please Land !!!");
        alertDialog2.setPositiveButton("Accept",
                (dialog, which) -> {
                    dialog.cancel();
                    //dismiss the dialog
                });

        parent.runOnUiThread(() -> {
            //          Uri notification = RingtoneManager.getDefaultUri(RingtoneManager.TYPE_NOTIFICATION);
            //          Ringtone r = RingtoneManager.getRingtone(getApplicationContext(), notification);
            //          r.play();
            alertDialog2.show();
        });

    }

    public DatagramSocket getSocket() {
        return socket;
    }

    public void setSocket(DatagramSocket socket) {
        this.socket = socket;
    }

    public void setTcpSocket(Socket socket) {
        this.mTcpSocket = socket;
        this.mIsa = (InetSocketAddress) socket.getRemoteSocketAddress();
        this.isTcpWorker = true;
    }

    public void setSecondarySocket(DatagramSocket socket) {
        this.secondarySocket = socket;
    }

    public void sendMessage(MAVLinkMessage msg) {

        if (!this.isTcpWorker && socket == null)
            return;

        if (this.isTcpWorker) {
            if (this.mTcpSocket == null)
                return;
        }

        MAVLinkPacket packet = msg.pack();

        if (packet.msgid == MAVLINK_MSG_ID_MISSION_ACK) {
            packet.sysid = mSystemId;
            Log.i(TAG, "패킷 시스템 id지정");
            packet.compid = MAV_COMP_ID_AUTOPILOT1;
            Log.i(TAG, "패킷 컴포넌트 id지정");
            byte[] bytes = packet.encodePacket();

            try {
                if (!this.isTcpWorker) {
                    DatagramPacket p = new DatagramPacket(bytes, bytes.length, socket.getInetAddress(), socket.getPort());
                    socket.send(p);
                    if (secondarySocket != null) {
                        DatagramPacket secondaryPacket = new DatagramPacket(bytes, bytes.length, secondarySocket.getInetAddress(), secondarySocket.getPort());
                        secondarySocket.send(secondaryPacket);
//                ////parent.Log("SECONDARY PACKET SENT");
                    }
//            if(msg.msgid != MAVLINK_MSG_ID_POWER_STATUS &&
//                    msg.msgid != MAVLINK_MSG_ID_SYS_STATUS &&
//                    msg.msgid != MAVLINK_MSG_ID_VIBRATION &&
//                    msg.msgid != MAVLINK_MSG_ID_ATTITUDE &&
//                    msg.msgid != MAVLINK_MSG_ID_VFR_HUD &&
//                    msg.msgid != MAVLINK_MSG_ID_GLOBAL_POSITION_INT &&
//                    msg.msgid != MAVLINK_MSG_ID_GPS_RAW_INT &&
//                    msg.msgid != MAVLINK_MSG_ID_RADIO_STATUS)
//                //parent.LogMessageToGCS(msg.toString());
                    // //parent.Log("UDP Send :"+msg.toString());
                } else {
                    // TODO TCP Send
                    OutputStream os = this.mTcpSocket.getOutputStream();
                    Log.i(TAG, "아웃풋 스트림 지정");

                    os.write(bytes);
                    Log.i(TAG, "아웃풋 스트림에 바이트 데이터 입력");
                    os.flush();
                    Log.i(TAG, "아웃풋 스트림에 흘려보냄");
//                Log.d(TAG, "Confirm, TCP Packet Config IP : {}, Port : {} Received-IP:"+ this.mIsa.getAddress() + " / Received-localport:" +  this.mIsa.getPort());

                    // //parent.Log("TCP Send :" + msg.toString());
                }

            } catch (PortUnreachableException ignored) {

            } catch (IOException e) {
//            Log.d(TAG, "Write Error ::");
            }
        } else {
            packet.sysid = mSystemId;

            packet.compid = MAV_COMP_ID_AUTOPILOT1;

            byte[] bytes = packet.encodePacket();

            try {
                if (!this.isTcpWorker) {
                    DatagramPacket p = new DatagramPacket(bytes, bytes.length, socket.getInetAddress(), socket.getPort());
                    socket.send(p);


                    if (secondarySocket != null) {
                        DatagramPacket secondaryPacket = new DatagramPacket(bytes, bytes.length, secondarySocket.getInetAddress(), secondarySocket.getPort());
                        secondarySocket.send(secondaryPacket);
//                ////parent.Log("SECONDARY PACKET SENT");
                    }
//            if(msg.msgid != MAVLINK_MSG_ID_POWER_STATUS &&
//                    msg.msgid != MAVLINK_MSG_ID_SYS_STATUS &&
//                    msg.msgid != MAVLINK_MSG_ID_VIBRATION &&
//                    msg.msgid != MAVLINK_MSG_ID_ATTITUDE &&
//                    msg.msgid != MAVLINK_MSG_ID_VFR_HUD &&
//                    msg.msgid != MAVLINK_MSG_ID_GLOBAL_POSITION_INT &&
//                    msg.msgid != MAVLINK_MSG_ID_GPS_RAW_INT &&
//                    msg.msgid != MAVLINK_MSG_ID_RADIO_STATUS)
//                //parent.LogMessageToGCS(msg.toString());
                    // //parent.Log("UDP Send :"+msg.toString());
                } else {

                    // TODO TCP Send
                    OutputStream os = this.mTcpSocket.getOutputStream();
//                Log.d(TAG, "<<<<<<<<<<<<<<<<< TCP Send Start <<<<<<<<<<<<<<<<<<<<< SysID:" + Integer.valueOf(mSystemId) + " / Port:" +  Integer.valueOf(this.mIsa.getPort()) );
//                log.info("TcpWorkerLink :: WriteBytes sysID index: {}, targetIp : {}, targetPort : {}", new Object[] { Integer.valueOf(index), targetIp, Integer.valueOf(targetPort) });
//                Log.d(TAG, "TcpWorkerLink :: Send1  MAVLink TCP Data = "+ bytes + ", length ="+ Integer.valueOf(bytes.length));
//                InetAddress targetInetAddr = InetAddress.getByName(targetI
                    os.write(bytes);
                    os.flush();

//                Log.d(TAG, "Confirm, TCP Packet Config IP : {}, Port : {} Received-IP:"+ this.mIsa.getAddress() + " / Received-localport:" +  this.mIsa.getPort());

                    // //parent.Log("TCP Send :" + msg.toString());
                }

            } catch (PortUnreachableException ignored) {

            } catch (IOException e) {
//            Log.d(TAG, "Write Error ::");
            }

        }
    }

    void send_mission_ack(int status) {

        msg_mission_ack msg = new msg_mission_ack();
        Log.i(TAG, "msg_mission_ack 객체 생성");
        msg.type = (short) status;
        Log.i(TAG, "msg_mission_ack 타입 지정");
        msg.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
        Log.i(TAG, "미션타입 지정");
        sendMessage(msg);
    }

    public void setStorageLocation() {
        MediaFileListDataSource.Builder builder = new MediaFileListDataSource.Builder();
        MediaFileListDataSource source = new MediaFileListDataSource(builder);


        parent.Log("미디어파일소스 수정 전 저장 위치 : " + source.getStorageLocation().toString());


        source.setStorageLocation(CameraStorageLocation.SDCARD);
        parent.Log("미디어파일소스 수정 후 저장 위치 : " + source.getStorageLocation().toString());
    }

    public void uploadKMZfile(DroneModel model, String path) {
        this.kmzOutPath = path;

        WaypointMissionManager.getInstance().pushKMZFileToAircraft(kmzOutPath, new CommonCallbacks.CompletionCallbackWithProgress<Double>() {
            @Override
            public void onProgressUpdate(Double aDouble) {
                Log.i(TAG, "uploading");
            }

            @Override
            public void onSuccess() {
                Log.i(TAG, "Upload success!\npath : " + path);
                toastinMain("Upload success!\npath : " + path);

                Thread sendMessageThread = new Thread(() -> {
                    try {
                        send_mission_ack(MAV_MISSION_ACCEPTED);
                    } catch (Exception e) {
                        Log.i(TAG, "exception : " + e.toString());
                    }
                });//ack를 보낼 쓰레드를 하나 만들고 실행
                sendMessageThread.start();

            }

            @Override
            public void onFailure(@NonNull IDJIError idjiError) {
                Log.i(TAG, "upload failed");
            }
        });
    }

    public int getSystemId() {
        return mSystemId;
    }

    public static long getTimestampMicroseconds() {
        return System.currentTimeMillis() / 10;
    }

    public ArrayList<MAVParam> getParams() {
        return params;
    }


    /*



     */
    public void tick() { // Called ever 100ms...
        ticks += 100;
        try {
            if (ticks % 100 == 0) {
                send_attitude();
                send_altitude();
                send_vibration();
                send_vfr_hud();
            }
            if (ticks % 200 == 0) {
                send_global_position_int(); // We use this for the AI se need 5Hz...
            }
            if (ticks % 300 == 0) {
                send_gps_raw_int();
                send_radio_status();
                send_rc_channels();
            }
            if (ticks % 1000 == 0) {
                send_heartbeat();
                send_sys_status();
                send_power_status();
                send_battery_status();
            }
            if (ticks % 5000 == 0) {
                send_home_position();
            }

        } catch (Exception e) {
            Log.d(TAG, "exception", e);
        }
    }

    private void request_mission_list() {
        msg_mission_request_list msg = new msg_mission_request_list();
        sendMessage(msg);
    }

//

    /**
     * 웨이포인트 실행 정지 관련 메소드들
     */
    //void request_mission_item(int seq) {
//        // Reset internal list
//        if (seq == 0) {
//            stopWaypointMission();
//            mactiveWaypointMission = null;
//        }
//
//        msg_mission_request_int msg = new msg_mission_request_int();
//        msg.seq = seq;
//        msg.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
//        sendMessage(msg);
//    }

    //웨이포인트 미션 시작 메소드
    public void startWaypointMission() {
        String rootDir = DiskUtil.getExternalCacheDirPath(ContextUtil.getContext(), WAYPOINT_SAMPLE_FILE_DIR);
        String kmzOutPath = rootDir + "generate_test.kmz";
        ArrayList wayline = new ArrayList<Integer>();
        wayline.add(0, 0);

        WaypointMissionManager.getInstance().addWaylineExecutingInfoListener(new WaylineExecutingInfoListener() {
            @Override
            public void onWaylineExecutingInfoUpdate(WaylineExecutingInfo excutingWaylineInfo) {
                Log.i(TAG, "excutingWaylineInfo waylineID : " + excutingWaylineInfo.getWaylineID() + "\nmissionfile name : " + excutingWaylineInfo.getMissionFileName() + "\nCurrentWaypointIndex : " + excutingWaylineInfo.getCurrentWaypointIndex());
            }

            @Override
            public void onWaylineExecutingInterruptReasonUpdate(IDJIError error) {
                Log.i(TAG, "WaylineExecutingInterruptReason : " + error.toString());
            }
        });
        WaypointMissionManager.getInstance().addWaypointMissionExecuteStateListener(new WaypointMissionExecuteStateListener() {
            @Override
            public void onMissionStateUpdate(WaypointMissionExecuteState missionState) {
                Log.i(TAG, "onMissionStateUpdate : " + missionState.toString());
            }
        });
        WaypointMissionManager.getInstance().addWaypointActionListener(new WaypointActionListener() {
            @Override
            public void onExecutionStart(int actionId) {
//                Log.i(TAG, "onExecutionStart actionId : " + actionId);
            }

            @Override
            public void onExecutionFinish(int actionId, @Nullable IDJIError error) {
//                Log.i(TAG, "onExecutionFinish error : " + error.toString());
            }

            @Override
            public void onExecutionStart(int actionGroup, int actionId) {
//                Log.i(TAG, "onExecutionStart actionGroup : " + actionGroup + " actionId : " + actionId);
            }

            @Override
            public void onExecutionFinish(int actionGroup, int actionId, @Nullable IDJIError error) {
//                Log.i(TAG, "onExecutionFinish actionGroup : " + actionGroup + " actionId : " + actionId + "error : " + error.toString());
            }
        });
        WaypointMissionManager.getInstance().startMission("generate_test_ddm", new CommonCallbacks.CompletionCallback() {
            @Override
            public void onSuccess() {
                Log.i(TAG, "WayPoint Mission Started" + "\n filename: " + FileUtils.getFileName(kmzOutPath, WAYPOINT_FILE_TAG));
                toastinMain("WayPoint Mission Started");

            }

            @Override
            public void onFailure(@NonNull IDJIError idjiError) {
                Log.i(TAG, "sarting WPM failed : " + idjiError.description().toString() + "\n filename: " + FileUtils.getFileName(kmzOutPath, WAYPOINT_FILE_TAG));
                toastinMain("sarting WPM failed : " + idjiError.description().toString() + "\n filename: " + FileUtils.getFileName(kmzOutPath, WAYPOINT_FILE_TAG));
            }
        });

//
    }

//    public void startImageCaptureIntervalControl(/*float paramInterval*/) {
//        safeSleep(100);
//
//        // MAV_CMD_IMAGE_START_CAPTURE 이 포함되어있었다면, 인터벌 컨트롤로 인지하여 수행됨.
//        if (isIntervalControl()) {
//            if (djiAircraft.getCamera() != null) {
//
//                this.doSetGimbalDown(9, -90);
//
//                safeSleep(100);
//
//
//                // 카메라 모드를 포토로 설정.
//                djiAircraft.getCamera().setMode(SettingsDefinitions.CameraMode.SHOOT_PHOTO, djiError -> {
//                    if (djiError != null) {
//                        parent.logMessageDJI("Error Setting PhotoMode: " + djiError.toString());
//                        SetCustomMesasageBox("1 [setMode Error]", djiError.toString());
//                    } else {
//                        SetCustomMesasageBox("1 [setMode Success]", "setMode Success.");
//                    }
//                });
//                djiAircraft.getCamera().setPhotoFileFormat(SettingsDefinitions.PhotoFileFormat.JPEG, djiError -> {
//                    if (djiError != null) {
//                        parent.logMessageDJI("Error Setting RAW_AND_JPEG: " + djiError.toString());
//                        SetCustomMesasageBox("3 [setPhotoFileFormat Error]", djiError.toString());
//                    } else {
//                        SetCustomMesasageBox("[setPhotoFileFormat Success]", "setPhotoFileFormat Success.");
//                    }
//                });
////                djiAircraft.getCamera().file
//
//                if (djiAircraft.getCamera().isFlatCameraModeSupported()) {
//                    djiAircraft.getCamera().setFlatMode(SettingsDefinitions.FlatCameraMode.PHOTO_INTERVAL, djiError -> {
//                        if (djiError != null) {
//                            parent.logMessageDJI("Error Setting PhotoMode: " + djiError.toString());
//                            SetCustomMesasageBox("2 [setFlatMode Error]", djiError.toString());
//                        } else {
//                            SetCustomMesasageBox("1 [setFlatMode Success]", "setFlatMode Success.");
//                        }
//                    });
//                }
//                //  PHOTO>Intelligent Flight Mode>Hyperlapse>2
//                //                SettingsDefinitions.FileType.JPEG
//                //  파일 캡쳐 형식 JPEG 2초이상  / RAW 5초이상 설정
////                djiAircraft.getCamera().setPhotoFileFormat(SettingsDefinitions.PhotoFileFormat.JPEG, djiError -> {
////                    if (djiError != null) {
////                        parent.logMessageDJI("Error Setting RAW_AND_JPEG: " + djiError.toString());
////                        SetCustomMesasageBox("3 [setPhotoFileFormat Error]", djiError.toString());
////                    } else {
//                SettingsDefinitions.ShootPhotoMode photoMode = SettingsDefinitions.ShootPhotoMode.INTERVAL;
//                djiAircraft.getCamera().setShootPhotoMode(photoMode, djiError2 -> {
//                    if (djiError2 == null) {
//                        SetCustomMesasageBox("3 [setShootPhotoMode Success]", "setShootPhotoMode Success.");
//
//                        int shootInterval = Math.round(this.getCaptureInterval()); // 반올림한다.
//                        Log.e(TAG, "startImageCaptureIntervalControl round shootInterval " + shootInterval);
//                        Log.e(TAG, "startImageCaptureIntervalControl origin paramInterval " + this.getCaptureInterval());
//
//                        SettingsDefinitions.PhotoTimeIntervalSettings mSettings = new SettingsDefinitions.PhotoTimeIntervalSettings(255, shootInterval);
//                        djiAircraft.getCamera().setPhotoTimeIntervalSettings(mSettings, djiError3 -> {
//                            if (djiError3 == null) {
//                                parent.logMessageDJI("Camera interval set to " + mSettings.getTimeIntervalInSeconds() + " seconds");
//                                SetCustomMesasageBox("4 [setPhotoTimeIntervalSettings Success]", "setPhotoTimeIntervalSettings Success.");
//
//                                photoTaken = false;
//                                photoTakenError = false;
//
//                                // setCameraVideoStreamSource
//                                /**
//                                 * 카메라 비디오 스트림 소스를 설정합니다. 다중 렌즈 카메라의 경우 카메라 스트림의 소스가 다릅니다. DJICameraVideoStreamSource카메라 비디오 스트림 소스를 나타내는 데 사용됩니다.
//                                 * 카메라 비디오 스트림의 소스가 설정되면 현재 라이브 뷰 스트림도 해당 소스로 변경됩니다. 젠뮤즈 H20/H20T에서만 지원됩니다.
//                                 */
//                                djiAircraft.getCamera().startShootPhoto(djiError4 -> {
//                                    if (djiError4 == null) {
//                                        parent.logMessageDJI("Requested Photo");
///*
//                    msg_camera_image_captured msg = new msg_camera_image_captured();
//                    msg.lat = (int)(mLatitude*10000000);
//                    msg.lon = (int)(mLongitude*10000000);
//                    msg.alt = (int)mAlt;
//                    msg.camera_id=0;
//                    msg.image_index=0;
//                    msg.capture_result = 1;
//                    sendMessage(msg);
//  */
//                                        //send_command_ack(MAV_CMD_DO_SET_PARAMETER, MAV_RESULT.MAV_RESULT_ACCEPTED);
//                                        send_command_ack(MAV_CMD_DO_DIGICAM_CONTROL, MAV_RESULT.MAV_RESULT_ACCEPTED);
//                                        SetCustomMesasageBox("5 [startShootPhoto Capture!]", "startShootPhoto Success.");
//
//                                        safeSleep(500);
//                                        startWaypointMission();
//
//                                    } else {
//                                        parent.logMessageDJI("Error requesting photo: " + djiError4.toString());
//                                        SetCustomMesasageBox("5 [startShootPhoto Error]", djiError4.toString());
//                                        // try again
////                                                takePhoto();
//                                    }
//                                });
//
//
//                            } else {
//                                parent.logMessageDJI("ERROR! Message: " + djiError3.getDescription());
//                                SetCustomMesasageBox("4 [setPhotoTimeIntervalSettings Error]", djiError3.toString());
//                            }
//                        });
//
//                    } else {
//                        parent.logMessageDJI("ERROR! Message: " + djiError2.getDescription());
//                    }
//                });
//
////                    }
////                });
//
//            }
//        }
//    }

    public void startImageCaptureIntervalControl(/*float paramInterval*/) {
//        safeSleep(100);
//
//        // MAV_CMD_IMAGE_START_CAPTURE 이 포함되어있었다면, 인터벌 컨트롤로 인지하여 수행됨.
//        if (isIntervalControl()) {
//            if (djiAircraft.getCamera() != null) {
//
//                this.doSetGimbalDown(9, -90);
//
//                safeSleep(100);
//
//
//                // 카메라 모드를 포토로 설정.
//                djiAircraft.getCamera().setMode(SettingsDefinitions.CameraMode.SHOOT_PHOTO, djiError -> {
//                    if (djiError != null) {
//                        parent.logMessageDJI("Error Setting PhotoMode: " + djiError.toString());
//                        SetCustomMesasageBox("1 [setMode Error]", djiError.toString());
//                    } else {
//                        SetCustomMesasageBox("1 [setMode Success]", "setMode Success.");
//                    }
//                });
//                djiAircraft.getCamera().setPhotoFileFormat(SettingsDefinitions.PhotoFileFormat.JPEG, djiError -> {
//                    if (djiError != null) {
//                        parent.logMessageDJI("Error Setting RAW_AND_JPEG: " + djiError.toString());
//                        SetCustomMesasageBox("3 [setPhotoFileFormat Error]", djiError.toString());
//                    } else {
//                        SetCustomMesasageBox("[setPhotoFileFormat Success]", "setPhotoFileFormat Success.");
//                    }
//                });
////                djiAircraft.getCamera().file
//
//                if (djiAircraft.getCamera().isFlatCameraModeSupported()) {
//                    djiAircraft.getCamera().setFlatMode(SettingsDefinitions.FlatCameraMode.PHOTO_INTERVAL, djiError -> {
//                        if (djiError != null) {
//                            parent.logMessageDJI("Error Setting PhotoMode: " + djiError.toString());
//                            SetCustomMesasageBox("2 [setFlatMode Error]", djiError.toString());
//                        } else {
//                            SetCustomMesasageBox("1 [setFlatMode Success]", "setFlatMode Success.");
//                        }
//                    });
//                }
//                //  PHOTO>Intelligent Flight Mode>Hyperlapse>2
//                //                SettingsDefinitions.FileType.JPEG
//                //  파일 캡쳐 형식 JPEG 2초이상  / RAW 5초이상 설정
////                djiAircraft.getCamera().setPhotoFileFormat(SettingsDefinitions.PhotoFileFormat.JPEG, djiError -> {
////                    if (djiError != null) {
////                        parent.logMessageDJI("Error Setting RAW_AND_JPEG: " + djiError.toString());
////                        SetCustomMesasageBox("3 [setPhotoFileFormat Error]", djiError.toString());
////                    } else {
//                SettingsDefinitions.ShootPhotoMode photoMode = SettingsDefinitions.ShootPhotoMode.INTERVAL;
//                djiAircraft.getCamera().setShootPhotoMode(photoMode, djiError2 -> {
//                    if (djiError2 == null) {
//                        SetCustomMesasageBox("3 [setShootPhotoMode Success]", "setShootPhotoMode Success.");
//
//                        int shootInterval = Math.round(this.getCaptureInterval()); // 반올림한다.
//                        Log.e(TAG, "startImageCaptureIntervalControl round shootInterval " + shootInterval);
//                        Log.e(TAG, "startImageCaptureIntervalControl origin paramInterval " + this.getCaptureInterval());
//
//                        SettingsDefinitions.PhotoTimeIntervalSettings mSettings = new SettingsDefinitions.PhotoTimeIntervalSettings(255, shootInterval);
//                        djiAircraft.getCamera().setPhotoTimeIntervalSettings(mSettings, djiError3 -> {
//                            if (djiError3 == null) {
//                                parent.logMessageDJI("Camera interval set to " + mSettings.getTimeIntervalInSeconds() + " seconds");
//                                SetCustomMesasageBox("4 [setPhotoTimeIntervalSettings Success]", "setPhotoTimeIntervalSettings Success.");
//
//                                photoTaken = false;
//                                photoTakenError = false;
//
//                                // setCameraVideoStreamSource
//                                /**
//                                 * 카메라 비디오 스트림 소스를 설정합니다. 다중 렌즈 카메라의 경우 카메라 스트림의 소스가 다릅니다. DJICameraVideoStreamSource카메라 비디오 스트림 소스를 나타내는 데 사용됩니다.
//                                 * 카메라 비디오 스트림의 소스가 설정되면 현재 라이브 뷰 스트림도 해당 소스로 변경됩니다. 젠뮤즈 H20/H20T에서만 지원됩니다.
//                                 */
//                                djiAircraft.getCamera().startShootPhoto(djiError4 -> {
//                                    if (djiError4 == null) {
//                                        parent.logMessageDJI("Requested Photo");
///*
//                    msg_camera_image_captured msg = new msg_camera_image_captured();
//                    msg.lat = (int)(mLatitude*10000000);
//                    msg.lon = (int)(mLongitude*10000000);
//                    msg.alt = (int)mAlt;
//                    msg.camera_id=0;
//                    msg.image_index=0;
//                    msg.capture_result = 1;
//                    sendMessage(msg);
//  */
//                                        //send_command_ack(MAV_CMD_DO_SET_PARAMETER, MAV_RESULT.MAV_RESULT_ACCEPTED);
//                                        send_command_ack(MAV_CMD_DO_DIGICAM_CONTROL, MAV_RESULT.MAV_RESULT_ACCEPTED);
//                                        SetCustomMesasageBox("5 [startShootPhoto Capture!]", "startShootPhoto Success.");
//
//                                        safeSleep(500);
//                                        startWaypointMission();
//
//                                    } else {
//                                        parent.logMessageDJI("Error requesting photo: " + djiError4.toString());
//                                        SetCustomMesasageBox("5 [startShootPhoto Error]", djiError4.toString());
//                                        // try again
////                                                takePhoto();
//                                    }
//                                });
//
//
//                            } else {
//                                parent.logMessageDJI("ERROR! Message: " + djiError3.getDescription());
//                                SetCustomMesasageBox("4 [setPhotoTimeIntervalSettings Error]", djiError3.toString());
//                            }
//                        });
//
//                    } else {
//                        parent.logMessageDJI("ERROR! Message: " + djiError2.getDescription());
//                    }
//                });
//
////                    }
////                });
//
//            }
//        }
    }

//    public void stopWaypointMission() {
//        mAutonomy = false;
//        if (getWaypointMissionOperator() == null) {
//            //parent.LogMessageDJI("stopWaypointMission() - mWaypointMissionOperator null");
//            return;
//        }
//
//        if (getWaypointMissionOperator().getCurrentState() == WaypointMissionState.EXECUTING) {
//            //parent.LogMessageDJI("Stopping mission...\n");
//            getWaypointMissionOperator().stopMission(djiError -> {
//                if (djiError != null)
//                    //parent.LogMessageDJI("Error: " + djiError.toString());
//                else
//                    //parent.LogMessageDJI("Mission stopped!\n");
//            });
//        }
//    }
//
//    void pauseWaypointMission() {
//        mAutonomy = false;
//        if (getWaypointMissionOperator() == null) {
//            //parent.LogMessageDJI("pauseWaypointMission() - mWaypointMissionOperator null");
//            return;
//        }
//
//        if (getWaypointMissionOperator().getCurrentState() == WaypointMissionState.EXECUTING) {
//            //parent.LogMessageDJI("Pausing mission...\n");
//            getWaypointMissionOperator().pauseMission(djiError -> {
//                if (djiError != null)
//                    //parent.LogMessageDJI("Error: " + djiError.toString());
//                else
//                    //parent.LogMessageDJI("Mission paused!\n");
//            });
//        }
//    }
//
//    void resumeWaypointMission() {
//        mAutonomy = false;
//        if (isSafetyEnabled()) {
//            //parent.LogMessageDJI(parent.getResources().getString(R.string.safety_launch));
//            send_command_ack(MAV_CMD_NAV_TAKEOFF, MAV_RESULT.MAV_RESULT_DENIED);
//            return;
//        }
//
//        if (getWaypointMissionOperator() == null) {
//            //parent.LogMessageDJI("resumeWaypointMission() - mWaypointMissionOperator null");
//            return;
//        }
//
//        if (getWaypointMissionOperator().getCurrentState() == WaypointMissionState.EXECUTION_PAUSED) {
//            //parent.LogMessageDJI("Resuming mission...\n");
//            getWaypointMissionOperator().resumeMission(djiError -> {
//                if (djiError != null)
//                    //parent.LogMessageDJI("Error: " + djiError.toString());
//                else {
//                    //parent.LogMessageDJI("Mission resumed!\n");
//                    mGCSCommandedMode = NOT_USING_GCS_COMMANDED_MODE;
//                }
//            });
//        }
//    }


    //DDM에서 가져온 메소드들


    void send_command_ack(int message_id, int result) {
        msg_command_ack msg = new msg_command_ack();
        msg.sysid = getSystemId();
        msg.command = message_id;
        msg.result = (short) result;
        parent.Log("send_command_ack");
        sendMessage(msg);
    }


    public void getStorageInfo() {


    }

    public void getmediafileList() {

        PullMediaFileListParam.Builder builder = new PullMediaFileListParam.Builder();
        PullMediaFileListParam listParam = new PullMediaFileListParam(builder);
        mediaManager.pullMediaFileListFromCamera(listParam, new CommonCallbacks.CompletionCallback() {//카메라에 있는 미디어 파일들을 당겨옴
            @Override
            public void onSuccess() {//성공시 콜백
                parent.Log("pullMediaFileListFromCamera done");
            }

            @Override
            public void onFailure(@NonNull IDJIError idjiError) {//실패시 콜백
                parent.Log("pullMediaFileListFromCamera fail");
            }
        });
        downloadPhotofromdrone();

    }


    public void createdir() {
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

            Log.i(TAG, "디렉토리명 지정");
        } catch (Exception e) {
            Log.e(TAG, "ERROR ZIPPING LOGS", e);
        }
    }

    public void downloadPhotofromdrone() {
        parent.Log("이미지 다운로드 메소드 진입");

        mediaManager.getMediaFileListData().getData().get(0);
        ArrayList<MediaFile> listTodelete = new ArrayList<MediaFile>();


        ;


        Log.i(TAG, 0 + "번째 아이템 for문 시작 파일 형식 : " + mediaManager.getMediaFileListData().getData().get(0).getFileType().toString());


        if (mediaManager.getMediaFileListData().getData().get(0).getFileType() == MediaFileType.JPEG && (mediaManager.getMediaFileListData().getData().get(0).getFileName().endsWith("V.JPG") || mediaManager.getMediaFileListData().getData().get(0).getFileName().endsWith("W.JPG"))) {
            Log.i(TAG, 0 + "번째 아이템 파일형식 JPEG, W 또는 V로 끝남");
            FileOutputStream outputStream;
            BufferedOutputStream bos;

            String fileName = mediaManager.getMediaFileListData().getData().get(0).getFileName();
            String path = recordDirectory.getPath() + File.separator + fileName;
            File destPath = new File(path);


            if (!destPath.exists())
            {try {
                    outputStream = new FileOutputStream(destPath);
                    bos = new BufferedOutputStream(outputStream);
                    parent.Log("아웃풋 스트림 생성");
                    mediaManager.getMediaFileListData().getData().get(0).pullOriginalMediaFileFromCamera(0, new MediaFileDownloadListener() {
                        @Override
                        public void onStart() {
                            Log.i(TAG, 0 + "번째 아이템 pull시작");
                        }

                        @Override
                        public void onProgress(long total, long current) {
                            double percentage = ((double) current / total) * 100;
                            Log.i(TAG, String.format(0 + "번째 아이템 onProgress: %.2f%% (%d/%d)", percentage, current, total));
                        }

                        @Override
                        public void onRealtimeDataUpdate(byte[] data, long position) {
                            Log.i(TAG, 0 + "번째 아이템 실시간 업데이트");
                            try {
                                bos.write(data, 0, data.length);

                            } catch (IOException e) {
                                throw new RuntimeException(e);
                            }
                        }

                        @RequiresApi(api = Build.VERSION_CODES.O)
                        @Override
                        public void onFinish() {
                            try {
                                bos.flush();
                            } catch (IOException e) {
                                throw new RuntimeException(e);
                            }

                            processingImageInfo(String.valueOf(destPath), 1920, 1080);
                            Log.i(TAG, 0 + "번째 아이템 pull 종료");


                        }

                        @Override
                        public void onFailure(IDJIError error) {
                            Log.i(TAG, 0 + "번째 아이템 pull 실패");
                        }
                    });
                } catch (FileNotFoundException e) {
                    e.printStackTrace();
                }}
            else {
                Log.i(TAG, 0 + "번째 아이템 이미있음");
            }


        }


    }


    public void deletemediafilefromCamera(List<MediaFile> deletelist) {

        mediaManager.deleteMediaFiles(deletelist, new CommonCallbacks.CompletionCallback() {
            @Override
            public void onSuccess() {
                parent.Log("deleteMediaFiles success");
            }

            @Override
            public void onFailure(@NonNull IDJIError idjiError) {
                parent.Log("deleteMediaFiles fail");
            }
        });

    }

    public void deleteAllmediafilefromCamera() {

        mediaManager.deleteMediaFiles(mediaManager.getMediaFileListData().getData(), new CommonCallbacks.CompletionCallback() {
            @Override
            public void onSuccess() {
                parent.Log("deleteMediaFiles success");
            }

            @Override
            public void onFailure(@NonNull IDJIError idjiError) {
                parent.Log("deleteMediaFiles fail");
            }
        });

    }


    public void takePhoto() {
        // 카메라 모드변경

        initmediamanager();
        keyManager.setValue(KeyTools.createKey(CameraKey.KeyCameraMode), PHOTO_NORMAL, new CommonCallbacks.CompletionCallback() {
            @Override
            public void onSuccess() {
                parent.Log("KeyCameraMode done ");

            }

            @Override
            public void onFailure(@NonNull IDJIError idjiError) {
                parent.Log("KeyCameraMode Error :  " + idjiError);
            }
        });


        //사진 촬영
        keyManager.performAction(KeyTools.createKey(CameraKey.KeyStartShootPhoto), new CommonCallbacks.CompletionCallbackWithParam<EmptyMsg>() {
            @Override
            public void onSuccess(EmptyMsg emptyMsg) {
                parent.Log("KeyStartShootPhoto done");
                keyManager.performAction(KeyTools.createKey(CameraKey.KeyStopShootPhoto), new CommonCallbacks.CompletionCallbackWithParam<EmptyMsg>() {
                    @Override
                    public void onSuccess(EmptyMsg emptyMsg) {
                        parent.Log("KeyStopShootPhoto done");
                    }

                    @Override
                    public void onFailure(@NonNull IDJIError idjiError) {
                        parent.Log("KeyStopShootPhoto Error :  " + idjiError);
                    }
                });
            }

            @Override
            public void onFailure(@NonNull IDJIError idjiError) {
                parent.Log("KeyStartShootPhoto Error :  " + idjiError);
            }
        });

        getmediafileList();
    }

    public void setGimbalRotation(double degree) {//입력한 각도에 따라 짐벌 세팅


        GimbalAngleRotation angleRotation = new GimbalAngleRotation();
        angleRotation.setPitch(degree);
        angleRotation.setYaw((double) 0);
        angleRotation.setRoll((double) 0);
        keyManager.performAction(KeyTools.createKey(GimbalKey.KeyRotateByAngle), angleRotation, new CommonCallbacks.CompletionCallbackWithParam<EmptyMsg>() {
            @Override
            public void onSuccess(EmptyMsg emptyMsg) {
                parent.Log("KeyRotateByAngle done : " + degree);
            }

            @Override
            public void onFailure(@NonNull IDJIError idjiError) {
                parent.Log("KeyRotateByAngle Error :  " + idjiError);
            }
        });


    }

    public boolean isIntervalControl() {
        return this.doImageInterval;
    }

    public void listen3DLocation() {
        keyManager.listen(KeyTools.createKey(FlightControllerKey.KeyAircraftLocation3D), this, new CommonCallbacks.KeyListener<LocationCoordinate3D>() {
            @Override
            public void onValueChange(@Nullable LocationCoordinate3D oldValue, @Nullable LocationCoordinate3D newValue) {
                if (newValue != null) {
                    mLatitude = newValue.getLatitude();
                    mLongitude = newValue.getLongitude();
                    mAlt = newValue.getAltitude();

                    //  parent.toast("LAT: " + mLatitude + " " + "LON: " + mLongitude + " " +"ALT: " + mAlt);
                }
            }
        });
    }

    public void listenRemoteControllerSticks() {

        keyManager.getValue(KeyTools.createKey(RemoteControllerKey.KeyStickLeftVertical), new CommonCallbacks.CompletionCallbackWithParam<Integer>() {
            @Override
            public void onSuccess(Integer integer) {
                mThrottleSetting = (int) integer;
                //parent.Log("listenRemot eControllerSticks() %4% mThrottleSetting " + integer);
            }

            @Override
            public void onFailure(@NonNull IDJIError idjiError) {
                //parent.Log("listenRemoteControllerSticks() %4% mThrottleSetting : failed");
            }
        });

        keyManager.listen(KeyTools.createKey(RemoteControllerKey.KeyStickLeftVertical), this, new CommonCallbacks.KeyListener<Integer>() {
            @Override
            public void onValueChange(@Nullable Integer integer, @Nullable Integer t1) {
                mLeftStickVertical = (int) (t1 * 0.8) + 1500;
                //parent.Log("listenRemoteControllerSticks() %5% mLeftStickVertical : " + mLeftStickVertical);
            }
        });
        keyManager.listen(KeyTools.createKey(RemoteControllerKey.KeyStickLeftHorizontal), this, new CommonCallbacks.KeyListener<Integer>() {
            @Override
            public void onValueChange(@Nullable Integer integer, @Nullable Integer t1) {
                mLeftStickHorisontal = (int) (t1 * 0.8) + 1500;
                //parent.Log("listenRemoteControllerSticks() %6% mLeftStickHorisontal : " + mLeftStickHorisontal);
            }
        });
        keyManager.listen(KeyTools.createKey(RemoteControllerKey.KeyStickRightVertical), this, new CommonCallbacks.KeyListener<Integer>() {
            @Override
            public void onValueChange(@Nullable Integer integer, @Nullable Integer t1) {
                mRightStickVertical = (int) (t1 * 0.8) + 1500;
                //parent.Log("listenRemoteControllerSticks() %7% mRightStickVertical : " + mRightStickVertical);
            }
        });
        keyManager.listen(KeyTools.createKey(RemoteControllerKey.KeyStickRightHorizontal), this, new CommonCallbacks.KeyListener<Integer>() {
            @Override
            public void onValueChange(@Nullable Integer integer, @Nullable Integer t1) {
                mRightStickHorisontal = (int) (t1 * 0.8) + 1500;
                //parent.Log("listenRemoteControllerSticks() %8% mRightStickHorisontal : " + mRightStickHorisontal);
            }
        });
        keyManager.listen(KeyTools.createKey(RemoteControllerKey.KeyCustomButton1Down), this, new CommonCallbacks.KeyListener<Boolean>() {
            @Override
            public void onValueChange(@Nullable Boolean aBoolean, @Nullable Boolean t1) {
                mC1 = t1;
                //parent.Log("listenRemoteControllerSticks() %9% mC1 clicked : " + mC1);
            }
        });
        keyManager.listen(KeyTools.createKey(RemoteControllerKey.KeyCustomButton2Down), this, new CommonCallbacks.KeyListener<Boolean>() {
            @Override
            public void onValueChange(@Nullable Boolean aBoolean, @Nullable Boolean t1) {
                mC2 = t1;
                //parent.Log("listenRemoteControllerSticks() %10% mC2 clicked : " + mC2);
            }
        });

        keyManager.listen(KeyTools.createKey(RemoteControllerKey.KeyCustomButton3Down), this, new CommonCallbacks.KeyListener<Boolean>() {
            @Override
            public void onValueChange(@Nullable Boolean aBoolean, @Nullable Boolean t1) {
                mC3 = t1;
                //parent.Log("listenRemoteControllerSticks() %11% mC3 clicked : " + mC3);
            }
        });

    }

    public void listenIsMotorOn() {
        keyManager.listen(KeyTools.createKey(FlightControllerKey.KeyAreMotorsOn), this, new CommonCallbacks.KeyListener<Boolean>() {
                    @Override
                    public void onValueChange(@Nullable Boolean aBoolean, @Nullable Boolean t1) {
                        if (t1 != null) {
                            isMotorOn = t1;
                            //parent.Log("%12% listenIsMotorOn() : " + isMotorOn);
                        }
                    }
                }


        );
    }


    public void listenAttitude() {
        keyManager.listen(KeyTools.createKey(FlightControllerKey.KeyAircraftAttitude), this, new CommonCallbacks.KeyListener<Attitude>() {
            @Override
            public void onValueChange(@Nullable Attitude attitude, @Nullable Attitude t1) {
                if (t1 != null) {
                    mPitch = t1.getPitch();
                    mRoll = t1.getRoll();
                    mYaw = t1.getYaw();
                    //parent.Log(" %13% listenAttitude() pitch: " + mPitch + " " + " %14% roll: " + mRoll + " " + " %15% yaw: " + mYaw);

                }
            }
        });
    }

    public double getmRoll() {
        return mRoll;
    }

    public double getmPitch() {
        return mPitch;
    }

    public double getmYaw() {
        return mYaw;
    }

    public void listenDroneBattery() {

        keyManager.listen(KeyTools.createKey(BatteryKey.KeyChargeRemaining), this, new CommonCallbacks.KeyListener<Integer>() {
            @Override
            public void onValueChange(@Nullable Integer integer, @Nullable Integer t1) {
                mCChargeRemaining_mAh = (int) t1;
                //parent.Log("%16% mCChargeRemaining_mAh : " + mCChargeRemaining_mAh);
            }
        });
        keyManager.listen(KeyTools.createKey(BatteryKey.KeyCellVoltages), this, new CommonCallbacks.KeyListener<List<Integer>>() {
            @Override
            public void onValueChange(@Nullable List<Integer> integers, @Nullable List<Integer> t1) {
                for (int i = 0; i < t1.size(); i++) {
                    mCellVoltages[i] = t1.get(i);
                    //parent.Log("%17% got cell voltages, v[" + i + "] =" + mCellVoltages[i]);
                }
            }
        });
        keyManager.listen(KeyTools.createKey(BatteryKey.KeyVoltage), this, new CommonCallbacks.KeyListener<Integer>() {
            @Override
            public void onValueChange(@Nullable Integer integer, @Nullable Integer t1) {
                mCVoltage_mV = t1;
                //parent.Log("%18% mCVoltage_mV : " + mCVoltage_mV);
            }
        });
        keyManager.listen(KeyTools.createKey(BatteryKey.KeyChargeRemainingInPercent), this, new CommonCallbacks.KeyListener<Integer>() {
            @Override
            public void onValueChange(@Nullable Integer integer, @Nullable Integer t1) {
                mCVoltage_pr = (int) t1;
                //parent.Log("%19% mCVoltage_pr : " + mCVoltage_pr);
            }
        });
        keyManager.listen(KeyTools.createKey(BatteryKey.KeyBatteryTemperature), this, new CommonCallbacks.KeyListener<Double>() {
            @Override
            public void onValueChange(@Nullable Double aDouble, @Nullable Double t1) {
                mCBatteryTemp_C = t1;
                //parent.Log("%20% mCBatteryTemp_C : " + mCBatteryTemp_C);

            }
        });
        keyManager.listen(KeyTools.createKey(BatteryKey.KeyCurrent), this, new CommonCallbacks.KeyListener<Integer>() {
            @Override
            public void onValueChange(@Nullable Integer integer, @Nullable Integer t1) {
                mCCurrent_mA = t1;
                //parent.Log("%21% mCCurrent_mA : " + mCCurrent_mA);
            }
        });


    }

    public void listenControllerBattery() {
        keyManager.listen(KeyTools.createKey(RemoteControllerKey.KeyBatteryInfo), this, new CommonCallbacks.KeyListener<BatteryInfo>() {
            @Override
            public void onValueChange(@Nullable BatteryInfo batteryInfo, @Nullable BatteryInfo t1) {
                mControllerVoltage_pr = (int) t1.getBatteryPercent();
                //parent.Log("%22% listenControllerBattery() mControllerVoltage_pr : " + mControllerVoltage_pr);
            }
        });
    }


    public void listenVelocity() {
        keyManager.listen(KeyTools.createKey(FlightControllerKey.KeyAircraftVelocity), this, new CommonCallbacks.KeyListener<Velocity3D>() {
                    @Override
                    public void onValueChange(@Nullable Velocity3D velocity3D, @Nullable Velocity3D t1) {
                        if (t1 != null) {
                            velocityx = t1.getX();
                            velocityy = t1.getY();
                            velocityz = t1.getZ();
                            //parent.Log("listenVelocity() %23% velocityx" + velocityx + " " + " %24% velocityy: " + velocityy + " " + " %25% velocityz: " + velocityz);
                        }
                    }
                }

        );
    }


    public void listenIsFly() {
        keyManager.listen(KeyTools.createKey(FlightControllerKey.KeyIsFlying), this, new CommonCallbacks.KeyListener<Boolean>() {
                    @Override
                    public void onValueChange(@Nullable Boolean aBoolean, @Nullable Boolean t1) {
                        if (t1 != null) {
                            isFlying = t1;
                            //parent.Log("listenIsFly() %26% isFlying : " + isFlying);
                        }
                    }
                }


        );
    }

    public void listenFlightMode() {
        keyManager.listen(KeyTools.createKey(FlightControllerKey.KeyFlightMode), this, new CommonCallbacks.KeyListener<FlightMode>() {
            @Override
            public void onValueChange(@Nullable FlightMode flightMode, @Nullable FlightMode t1) {
                lastMode = t1;
                //parent.Log("%27% getFlightMode() : " + lastMode.toString());
            }
        });


    }

    public void listenAirlinkQuality() {
        keyManager.listen(KeyTools.createKey(AirLinkKey.KeyUpLinkQuality), this, new CommonCallbacks.KeyListener<Integer>() {
                    @Override
                    public void onValueChange(@Nullable Integer integer, @Nullable Integer t1) {
                        mUplinkQuality = t1;
                        //parent.Log("%28% mUplinkQuality : " + mUplinkQuality);
                    }
                }

        );
        keyManager.listen(KeyTools.createKey(AirLinkKey.KeyDownLinkQuality), this, new CommonCallbacks.KeyListener<Integer>() {
                    @Override
                    public void onValueChange(@Nullable Integer integer, @Nullable Integer t1) {
                        mDownlinkQuality = t1;
                        //parent.Log("%29% mDownlinkQuality : " + mDownlinkQuality);
                    }
                }

        );

    }


    public void getFlightMode() {
        keyManager.getValue(KeyTools.createKey(FlightControllerKey.KeyFlightMode), new CommonCallbacks.CompletionCallbackWithParam<FlightMode>() {
            @Override
            public void onSuccess(FlightMode flightMode) {
                lastMode = flightMode;
                ////parent.Log("getFlightMode() : " + flightMode.name());
            }

            @Override
            public void onFailure(@NonNull IDJIError idjiError) {
                ////parent.Log("getFlightMode() Failed ");
            }
        });


    }

    public void getfullchargecapacity() {


        keyManager.getValue(KeyTools.createKey(BatteryKey.KeyFullChargeCapacity), new CommonCallbacks.CompletionCallbackWithParam<Integer>() {
            @Override
            public void onSuccess(Integer integer) {
                mCFullChargeCapacity_mAh = (int) integer;
                ////parent.Log("%23% mFullChargeCapacity_mAh : " + mCFullChargeCapacity_mAh);
            }

            @Override
            public void onFailure(@NonNull IDJIError idjiError) {
                ////parent.Log("getfullchargecapacity() Failed ");
            }
        });


    }


    public void getIsMotorOn() {

        keyManager.getValue(KeyTools.createKey(FlightControllerKey.KeyAreMotorsOn), new CommonCallbacks.CompletionCallbackWithParam<Boolean>() {
                    @Override
                    public void onSuccess(Boolean aBoolean) {
                        isMotorOn = aBoolean;
                    }

                    @Override
                    public void onFailure(@NonNull IDJIError idjiError) {
                        ////parent.Log("Get isMotorOn Failed ");
                    }
                }


        );
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


    @RequiresApi(api = Build.VERSION_CODES.O)
    public void processingImageInfo(String recordedImagePath, int imageWidth, int imageHeight) {

        LocalDateTime time = LocalDateTime.now(); //LocalDateTime.now();

        LocationCoordinate3D coord3d = new LocationCoordinate3D();
        coord3d.setLatitude(this.get_current_lat());
        coord3d.setLongitude(this.get_current_lon());
        coord3d.setAltitude(this.get_current_alt());

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
        float roll = (float) this.getmRoll();
        float pitch = (float) this.getmPitch();
        float yaw = (float) this.getmYaw();

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
        imageMap.put("lat", (int) (lat * 10000000));
        imageMap.put("lon", (int) (lng * 10000000));
        imageMap.put("alt", (int) (alt * 1000));
        imageMap.put("roll", roll);   // need degree
        imageMap.put("pitch", pitch); // need degree
        imageMap.put("yaw", yaw);     // need degree
        imageMap.put("droneId", this.getSystemId());
        imageMap.put("flightId", FLIGHT_ID);
        imageMap.put("storeName", STORE_NAME);
        imageMap.put("layerName", LAYER_NAME);

        this.publishImageInfo(this.getSystemId(), imageMap);

    }

    public void publishImageInfo(int droneId, Map<String, Object> imageInfo) {
        Log.e(TAG, "publishImageInfo start :" + imageInfo.toString());
        try {
            DDMMqttClient client = DDMMqttClient.getSimpleMqttClient(parent.getApplicationContext()
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


    MediaFileListState mMediaFileListState;
    //region Fields

    public void test() {
        mMissionManager = new WaypointMissionManager();
    }

    //자바 기본, 엑셀에 정리할 필요 없는 것들
    private final String TAG = DroneModel.class.getSimpleName(); //클래스명
    public DatagramSocket socket;//통신에 사용할 UDP 소켓
    public Socket mTcpSocket;//통신에 사용할 TCP소켓
    public InetSocketAddress mIsa;//IP주소
    public boolean isTcpWorker;//TCP 현재 작동중인지를 확인하기 위해 만들어둔 boolean
    public DatagramSocket secondarySocket;//통신에 사용할 두번째 UDP 소켓
    private final DefaultLayoutActivity parent;// 이 클래스가 선언되고 여러 함수들이 호출될 부모 액티비티
    protected SharedPreferences sharedPreferences;// 안드로이드 내부 저장소
    private final AlertDialog uploadMessageBox = null; // 액티비티에 띄울 다이얼로그
    private long ticks = 0;//일정 시간마다 메소드를 실행시킬때 사용되는 수, 메소드가 1회 돌 때마다 100씩 늘어난다
    public static boolean isDev = false;//개발중인지 여부를 식별하기 위한 boolean값
    public static double DEFUALT_LATITUDE = 37.3895865; // 37.3946352; // 37.3901804;//개발중일 때 사용할 임의의 위도값

    public static double DEFUALT_LONGITUDE = 126.641773; //126.6381979; // 126.6413591;//개발중일 때 사용할 임의의 경도값
    private final boolean doImageInterval = false;//기체가 현재 웨이포인트와 다음 웨이포인트 사이를 이동할 때 두 장의 사진이 촬영되는 시간 간격(초)
    private final float captureInterval = 0.0f;//사진 찍는 간격, DDM내에서 주석처리 되어있는거 보니 안쓰이는듯
    private final int batteryIndex = 0;
    private final ArrayList<MAVParam> params = new ArrayList<>();
    private int mGCSCommandedMode;
    private static int imageIndex = 0;

    private final int mlastState = 100;//임의로 지정한 배터리 전력량, 배터리 전력 정보를 받으면 동기화 하여 배터리 잔량 경보를 부모 액티비티에 보냄

    private final int mVoltage_pr = 0;//종합적인 배터리 잔량을 퍼센트로 나타냄(?)


    private final int m_lastCommand = 0;//do_set_motion_relative 에서 쓰이는 변수, 가장 최근에 받은 마브링크 메시지의 command값을 int로 저장
    private final Timer mSendVirtualStickDataTimer = null;//가상 컨트롤러의 스틱에 명령을 전달할 때 사용되는 타이머
    public Timer mMoveToDataTimer = null;//커스텀 웨이포인트 인터프리터에서 사용하는 타이머,
    private Rotation m_ServoSet;//짐벌 제어에 쓰이는 값들을 다 담는 객체
    private final float m_ServoPos_pitch = 0;//짐벌제어에 쓸 pitch값(좌우)
    private final float m_ServoPos_yaw = 0;//짐벌제어에 쓸 Yaw값(위아래)
    public boolean photoTaken = false;//사진 저장중에 다른 사진을 찍지 못하도록 제어하는 변수, 이 값이 true일 때는 사진을 찍지 못하도록 짜여져 있음
    public boolean photoTakenError = false;//setDjiAircraft 메소드 내에 boolean err = systemState.isOverheating() || systemState.hasError(); 값을 담는 변수
    public boolean gotoNoPhoto = false;//사진을 찍는중이라면 false, 찍는중이 아니면 true를 할당하도록 만든 변수
    public double m_Curvesize = 0.0; //커브가 총 몇미터인지를 나타내는 변수
    public double m_POI_Lat = 0;//커스텀 웨이포인트 실행중에 바라볼 위도
    public double m_POI_Lon = 0;//커스텀 웨이포인트 실행중에 바라볼 경도
    public boolean m_CruisingMode = false;//커스텀 웨이포인트 실행중에 속도가 3미터/s면 true가 할당됨
    public boolean m_Stay = false;//웨이포인트 미션 중에 현재 수행중인 행동을 얻을 수 있는데 그값이 STAY일 때 true를 할당하는 변수
    public AtomicBoolean gimbalReady = null;//do_set_gimbal에서 m_ServoSet을 기반으로 짐벌을 세팅한 다음 true를 할당
    public boolean pauseWaypointMission = false;//웨이포인트 미션 일시정지 여부 저장
    public boolean stopWaypointMission = false;//웨이포인트 여부 중지 여부 저장
    private MiniPID miniPIDSide;//initFlightController()에서 할당됨, Moto timertask에서도 사용됨
    private MiniPID miniPIDFwd;//initFlightController()에서 할당됨, Moto timertask에서도 사용됨
    private MiniPID miniPIDAlti;//initFlightController()에서 할당됨, Moto timertask에서도 사용됨
    private MiniPID miniPIDHeading;//initFlightController()에서 할당됨, Moto timertask에서도 사용됨
    private final boolean mSafetyEnabled = true;//부모 액티비티에서 세이프티모드 버튼의 값을 받아옴
    private final Rotation mRotation = null;//쓰이는곳 없음
    private final boolean m_Sim = false;//시뮬레이션 여부
    private int mAIfunction_activation = 0;//send_rc_channels()에서  msg.chan8_raw = (mAIfunction_activation * 100) + 1000; 코드에 활용
    public boolean mAutonomy = false;//자율비행 모드 여부
    public int mAirBorn = 0;//공중에 뜸 여부  do_land에 달려있음
    int mission_loaded = -1;
    public WaypointMissionManager mMissionManager;
    float mBatteryTemp_C;
    //드론으로부터 받아오는 값

    private double mLatitude; //위도
    private double mLongitude;//경도
    private double mAlt;//고도
    private int mThrottleSetting = 0;//현재 쓰로틀(상승 하강)퍼센트
    private int mLeftStickVertical = 0;//왼쪽 스틱 수직이동값
    private int mLeftStickHorisontal = 0;//왼쪽스틱 수평 이동값
    private int mRightStickVertical = 0;//오른쪽 스틱 수직이동값
    private int mRightStickHorisontal = 0;//오른쪽스틱 수평 이동값
    private boolean mC1 = false;//C1버튼 눌림 여부
    private boolean mC2 = false;//C2버튼 눌림 여부
    private boolean mC3 = false;//C3버튼 눌림 여부
    private boolean isMotorOn = false;//모터 작동 여부
    private double mPitch;//피치
    private double mRoll;//롤
    private double mYaw;//요
    private int mCChargeRemaining_mAh = 0;//기체 남은 배터리 (mAh)
    private int mControllerVoltage_pr = 0;//컨트롤러 남은 배터리 (퍼센트)
    private double velocityx;//x축 속도
    private double velocityy;//y축 속도
    private double velocityz;//z축 속도
    private boolean isFlying;//비행중 여부
    private FlightMode lastMode = FlightMode.UNKNOWN;//비행모드
    private int mCFullChargeCapacity_mAh = 0;//기체 배터리 최대 충전량
    private double mCBatteryTemp_C = 0;//기체 배터리 온도
    private final int[] mCellVoltages = new int[12];//기체 배터리 셀당 전압 리스트
    private int mCVoltage_mV = 0;//배터리 하나의 전압을 mA단위로 받음
    private int mCVoltage_pr = 0;//배터리 하나의 전압을 퍼센트 단위로 받음
    private int mCCurrent_mA = 0;//현재 배터리 소비량
    private int mDownlinkQuality = 0;//컨트롤러에서 드론이 보내는 데이터 받는 신호의 강도
    private int mUplinkQuality = 0;//컨트롤러에서 드론으로 데이터 보내는 신호의 강도
    //드론으로부터 받아오는 값

    //DDM에서 생성, 지정하는 값
    public int mSystemId = 3;//시스템 ID
    public WaypointMission mactiveWaypointMission = null;//웨이포인트 미션
    public Waypoint waypoint = null;

    private final float mThrottle = 0;//설정해주는 쓰로틀 값
    private double mDestinationLat = 0;//관제에서 전달받은 임무 명령의 위도
    private double mDestinationLon = 0;//관제에서 전달받은 임무 명령의 경도
    private float mDestinationAlt = 0;//관제에서 전달받은 임무 명령의 고도
    private double mDestinationYaw = 0;//관제에서 전달받은 임무 명령의 YAW값
    private double mDestinationYawrate = 0; //관제에서 전달받은 임무 명령의 YAW 비율(?)
    private float mDestinationSetVx = -1;//관제에서 전달받은 임무 명령의 x축속도
    private float mDestinationSetVy = -1;//관제에서 전달받은 임무 명령의 y축속도
    private float mDestinationSetVz = -1;//관제에서 전달받은 임무 명령의 z축속도
    private int mDestinationMask = 0; //?
    private final double mDestinationbrng = 0; //웨이포인트와 현재 기체위치의 방위각
    private final double mDestinationhypotenuse = 0; //목적지까지 거리
    String WAYPOINT_SAMPLE_FILE_NAME = "waypointsample.kmz";
    String WAYPOINT_SAMPLE_FILE_DIR = "waypoint/";
    String rootDir = DiskUtil.getExternalCacheDirPath(ContextUtil.getContext(), WAYPOINT_SAMPLE_FILE_DIR);
    String kmzOutPath;
    String WAYPOINT_FILE_TAG = ".kmz";
    boolean missionUploaded;
    String cur = DiskUtil.getExternalCacheDirPath(
            ContextUtil.getContext(),
            WAYPOINT_SAMPLE_FILE_DIR + WAYPOINT_SAMPLE_FILE_NAME
    );
    private KeyManager keyManager = KeyManager.getInstance();
    public File useDirectory = null;
    public File recordDirectory = null;
    private BufferedOutputStream bos;
    private FileOutputStream outputStream;
    public String root = "";
    IMediaManager mediaManager;

    List<MediaFile> mediafilelist;

    public static String FLIGHT_ID = "FM20210627-1";
    public static String STORE_NAME = "TECHNOPARKV5";
    public static String LAYER_NAME = "PANTOM";
    //

//    private TimeLineMissionControlView TimeLine = new TimeLineMissionControlView();
//    private Model m_model;
//    private FollowMeMissionOperator fmmo;
//    public FlightController mFlightController;
//    private Gimbal mGimbal = null;
//    private SystemState m_lastSystemState = null;
//    public MoveTo mMoveToDataTask = null;
//    private HardwareState.FlightModeSwitch rcmode = HardwareState.FlightModeSwitch.POSITION_ONE;
//    private static HardwareState.FlightModeSwitch avtivemode = HardwareState.FlightModeSwitch.POSITION_ONE; // Change this to decide what position the mode switch should be inn.
//    private SendVelocityDataTask mSendVirtualStickDataTask = null;
//    private Aircraft djiAircraft;
    //endregion


}


