package dji.v5.ux.sample.util;

import static java.lang.Thread.sleep;
import static dji.sdk.keyvalue.value.flightcontroller.FCFlightMode.ACTIVE_TRACK;
import static dji.sdk.keyvalue.value.flightcontroller.FCFlightMode.ATTI_HOVER;
import static dji.sdk.keyvalue.value.flightcontroller.FCFlightMode.ATTI_LIMITED;
import static dji.sdk.keyvalue.value.flightcontroller.FCFlightMode.FARMING;
import static dji.sdk.keyvalue.value.flightcontroller.FCFlightMode.GPS_ATTI;
import static dji.sdk.keyvalue.value.flightcontroller.FCFlightMode.GPS_ATTI_WRISTBAND;
import static dji.v5.ux.MAVLink.enums.MAV_CMD.MAV_CMD_COMPONENT_ARM_DISARM;
import static dji.v5.ux.MAVLink.enums.MAV_COMPONENT.MAV_COMP_ID_AUTOPILOT1;

import android.app.AlertDialog;
import android.content.SharedPreferences;
import android.media.RemoteController;
import android.util.Log;
import android.widget.Toast;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.dji.industry.mission.natives.util.NativeCallbackUtils;
import com.dji.industry.mission.waypointv2.gimbal.Rotation;

import java.io.IOException;
import java.io.OutputStream;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetSocketAddress;
import java.net.PortUnreachableException;
import java.net.Socket;
import java.util.ArrayList;
import java.util.List;
import java.util.Timer;
import java.util.concurrent.atomic.AtomicBoolean;


import dji.sdk.keyvalue.converter.SingleValueConverter;
import dji.sdk.keyvalue.key.AirLinkKey;
import dji.sdk.keyvalue.key.BatteryKey;
import dji.sdk.keyvalue.key.DJIKey;
import dji.sdk.keyvalue.key.DJIKeyInfo;
import dji.sdk.keyvalue.key.FlightControllerKey;
import dji.sdk.keyvalue.key.GimbalKey;
import dji.sdk.keyvalue.key.KeyTools;
import dji.sdk.keyvalue.key.RemoteControllerKey;
import dji.sdk.keyvalue.value.common.Attitude;
import dji.sdk.keyvalue.value.common.LocationCoordinate2D;
import dji.sdk.keyvalue.value.common.LocationCoordinate3D;
import dji.sdk.keyvalue.value.common.Velocity3D;
import dji.sdk.keyvalue.value.flightcontroller.FlightMode;
import dji.sdk.keyvalue.value.flightcontroller.GPSSignalLevel;
import dji.sdk.keyvalue.value.flightcontroller.RollPitchControlMode;
import dji.sdk.keyvalue.value.flightcontroller.VerticalControlMode;
import dji.sdk.keyvalue.value.flightcontroller.VirtualStickFlightControlParam;
import dji.sdk.keyvalue.value.flightcontroller.YawControlMode;
import dji.sdk.keyvalue.value.mission.Waypoint;
import dji.sdk.keyvalue.value.mission.Wayline;
import dji.sdk.keyvalue.value.mission.WaypointMission;
import dji.sdk.keyvalue.value.mission.WaypointMissionState;
import dji.sdk.keyvalue.value.remotecontroller.BatteryInfo;
import dji.sdk.keyvalue.value.remotecontroller.ControlMode;
import dji.v5.common.callback.CommonCallbacks;
import dji.v5.common.error.IDJIError;
import dji.v5.manager.KeyManager;
import dji.v5.manager.aircraft.waypoint3.WaypointMissionManager;
import dji.v5.manager.interfaces.IKeyManager;
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
import dji.v5.ux.MAVLink.enums.MAV_MISSION_RESULT;
import dji.v5.ux.MAVLink.enums.MAV_MISSION_TYPE;
import dji.v5.ux.MAVLink.enums.MAV_MODE_FLAG;
import dji.v5.ux.MAVLink.enums.MAV_PROTOCOL_CAPABILITY;
import dji.v5.ux.MAVLink.enums.MAV_RESULT;
import dji.v5.ux.MAVLink.enums.MAV_STATE;
import dji.v5.ux.MAVLink.enums.MAV_TYPE;
import dji.v5.ux.R;
import dji.v5.ux.core.base.DJISDKModel;
import dji.v5.ux.core.communication.ObservableInMemoryKeyedStore;
import dji.v5.ux.core.util.DataProcessor;
import dji.v5.ux.core.widget.battery.BatteryInfoWidgetModel;
import dji.v5.ux.obstacle.AvoidanceShortcutWidget;
import dji.v5.ux.sample.showcase.defaultlayout.DefaultLayoutActivity;
import io.reactivex.rxjava3.core.Flowable;

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


    //DDM에서 가져온 메소드들
    public double get_current_lat() {

        return mLatitude;
    }

    public double get_current_lon() {

        return mLongitude;
    }

    public float get_current_alt() {

        return (float) mAlt;
    }

    public void setSystemId(int id) {
        mSystemId = id;
    }

    void setRTLAltitude(final int altitude) {

        KeyManager.getInstance().setValue(KeyTools.createKey(FlightControllerKey.KeyGoHomeHeight), altitude, new CommonCallbacks.CompletionCallback() {
            @Override
            public void onSuccess() {
                //parent.Log("RTL altitude set to " + altitude + "m");
            }

            @Override
            public void onFailure(@NonNull IDJIError idjiError) {
                //parent.Log("Error setting RTL altitude " + idjiError.description());
            }
        });


    }

    void setMaxHeight(final int height) {

        KeyManager.getInstance().setValue(KeyTools.createKey(FlightControllerKey.KeyLimitMaxFlightHeightInMeter), height, new CommonCallbacks.CompletionCallback() {
            @Override
            public void onSuccess() {
                //parent.Log("Max height set to " + height + "m");
            }

            @Override
            public void onFailure(@NonNull IDJIError idjiError) {
                //parent.Log("Error setting max height " + idjiError.description());
            }
        });

    }

    private void send_heartbeat() {

        msg_heartbeat msg = new msg_heartbeat();
        msg.type = MAV_TYPE.MAV_TYPE_QUADROTOR;
        msg.autopilot = MAV_AUTOPILOT.MAV_AUTOPILOT_UDB; // MAV_AUTOPILOT_ARDUPILOTMEGA;  // 차후 dji로 수정하자.

        // For base mode logic, see Copter::sendHeartBeat() in ArduCopter/GCS_Mavlink.cpp
        msg.base_mode = MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
        msg.base_mode |= MAV_MODE_FLAG.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;


        //getFlightMode();

        switch (lastMode) {
//            case MANUAL:
//                msg.custom_mode = ArduCopterFlightModes.STABILIZE;
//                break;
//            case ATTI_HOVER:
//                break;
//            case HOVER:
//                break;
//            case GPS_BLAKE:
//                break;
//            case ATTI_LANDING:
//                break;
//            case CLICK_GO:
//                break;
//            case CINEMATIC:
//                break;
//            case ATTI_LIMITED:
//                break;
//            case PANO:
//                break;
//            case FARMING:
//                break;
//            case FPV:
//                break;
//            case PALM_CONTROL:
//                break;
//            case QUICK_SHOT:
//                break;
//            case DETOUR:
//                break;
//            case TIME_LAPSE:
//                break;
//            case POI2:
//                break;
//            case OMNI_MOVING:
//                break;
//            case ADSB_AVOIDING:
//                break;
//            case ATTI:
//                msg.custom_mode = ArduCopterFlightModes.LOITER;
//                break;
//            case ATTI_COURSE_LOCK:
//                break;
//            case GPS_ATTI:
//                msg.custom_mode = ArduCopterFlightModes.STABILIZE;
//                break;
//            case GPS_COURSE_LOCK:
//                break;
//            case GPS_HOME_LOCK:
//                break;
//            case GPS_HOT_POINT:
//                break;
//            case ASSISTED_TAKEOFF:
//                break;
//            case AUTO_TAKE_OFF:
//                msg.custom_mode = ArduCopterFlightModes.GUIDED;
//                msg.base_mode |= MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED;
//                break;
//            case AUTO_LANDING:
//                msg.custom_mode = ArduCopterFlightModes.LAND;
//                msg.base_mode |= MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED;
//                break;
//            case GPS_WAYPOINT:
//                msg.custom_mode = ArduCopterFlightModes.AUTO;
//                msg.base_mode |= MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED;
//                break;
//            case GO_HOME:
//                msg.custom_mode = ArduCopterFlightModes.RTL;
//                msg.base_mode |= MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED;
//                break;
//            case JOYSTICK:
//                msg.custom_mode = ArduCopterFlightModes.GUIDED;
//                msg.base_mode |= MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED;
//                break;
//            case GPS_ATTI_WRISTBAND:
//                break;
//            case DRAW:
//                msg.base_mode |= MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED;
//                break;
//            case GPS_FOLLOW_ME:
//                msg.custom_mode = ArduCopterFlightModes.GUIDED;
//                msg.base_mode |= MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED;
//                break;
//            case ACTIVE_TRACK:
//                msg.custom_mode = ArduCopterFlightModes.GUIDED;
//                msg.base_mode |= MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED;
//                break;
//            case TAP_FLY:
//                msg.custom_mode = ArduCopterFlightModes.GUIDED;
//                break;
//            case GPS_SPORT:
//                break;
//            case GPS_NOVICE:
//                break;
//            case UNKNOWN:
//                break;
            case MANUAL: {
                msg.custom_mode = ArduCopterFlightModes.STABILIZE;
                break;
            }
            case ATTI: {
                msg.custom_mode = ArduCopterFlightModes.LOITER;
                break;
            }
            case GPS_NORMAL: {
                break;
            }
            case POI: {
                break;
            }
            case TAKE_OFF_READY: {
                break;
            }
            case AUTO_TAKE_OFF: {
                msg.custom_mode = ArduCopterFlightModes.GUIDED;
                msg.base_mode |= MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED;
                break;
            }
            case AUTO_LANDING: {
                msg.custom_mode = ArduCopterFlightModes.LAND;
                msg.base_mode |= MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED;
                break;
            }
            case WAYPOINT: {
                break;
            }
            case GO_HOME: {
                msg.custom_mode = ArduCopterFlightModes.RTL;
                msg.base_mode |= MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED;
                break;
            }
            case VIRTUAL_STICK: {
                break;
            }
            case SMART_FLIGHT: {
                break;
            }
            case PANO: {
                break;
            }
            case GPS_SPORT: {
                break;
            }
            case GPS_TRIPOD: {
                break;
            }
            case AUTO_AVOIDANCE: {
                break;
            }
            case SMART_FLY: {
                break;
            }
            case FORCE_LANDING: {
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
            case DRAW: {
                msg.base_mode |= MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED;
                break;
            }
            case FOLLOW_ME: {
                msg.custom_mode = ArduCopterFlightModes.GUIDED;
                msg.base_mode |= MAV_MODE_FLAG.MAV_MODE_FLAG_GUIDED_ENABLED;
                break;
            }
            case GPS_NOVICE: {
                break;
            }
            case QUICK_MOVIE: {
                break;
            }
            case TAP_FLY: {
                msg.custom_mode = ArduCopterFlightModes.GUIDED;
                break;
            }
            case MASTER_SHOT: {
                break;
            }
            case APAS: {
                break;
            }
            case TIME_LAPSE: {
                break;
            }
            case MOTOR_START: {
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
        // //parent.Log("send_heartbeat %9%");
        sendMessage(msg);
    }

    private void send_attitude() {
        //   //parent.Log("send_attitude %1% ");
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
        //  //parent.Log("send_altitude %2% ");
        msg_altitude msg = new msg_altitude();
        msg.altitude_relative = (int) (mAlt);
        mAlt = msg.altitude_relative;
        sendMessage(msg);
    }

    private void send_vibration() {
        // //parent.Log("send_vibration %3% ");
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
        //  //parent.Log("send_vfr_hud %4% ");
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
       // parent.Log("send_global_position_int %5% ");
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
        KeyManager.getInstance().getValue(KeyTools.createKey(FlightControllerKey.KeyGPSSatelliteCount), new CommonCallbacks.CompletionCallbackWithParam<Integer>() {
            @Override
            public void onSuccess(Integer integer) {
                msg.satellites_visible = integer.shortValue();
            }

            @Override
            public void onFailure(@NonNull IDJIError idjiError) {
                ////parent.Log("KeyGPSSatelliteCount Failed");
            }
        });

        // DJI reports signal quality on a scale of 1-5
        // Mavlink has separate codes for fix type.
        final GPSSignalLevel[] gpsLevel = new GPSSignalLevel[1];
        KeyManager.getInstance().getValue(KeyTools.createKey(FlightControllerKey.KeyGPSSignalLevel), new CommonCallbacks.CompletionCallbackWithParam<GPSSignalLevel>() {
            @Override
            public void onSuccess(GPSSignalLevel gpsSignalLevel) {
                gpsLevel[0] = gpsSignalLevel;
            }

            @Override
            public void onFailure(@NonNull IDJIError idjiError) {
                ////parent.Log("KeyGPSSignalLevel Failed");
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
        ////parent.Log("send_gps_raw_int %6% ");
        sendMessage(msg);
    }

    private void send_radio_status() {
        msg_radio_status msg = new msg_radio_status();
        msg.rssi = 0; // TODO: work out units conversion (see issue #1)
        msg.remrssi = 0; // TODO: work out units conversion (see issue #1)
        // //parent.Log("send_radio_status %7%");
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
                //parent.Log("AI Mode Canceled...");
                mAIfunction_activation = 0;
            }

            mAutonomy = false;
            //    pauseWaypointMission();  // TODO:: halt mission for safety...
        }

        msg.chan8_raw = (mAIfunction_activation * 100) + 1000;
        msg.chancount = 8;
        // //parent.Log("send_rc_channels %8%");
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
        //  //parent.Log("send_sys_status %10%");
        sendMessage(msg);
    }

    private void send_power_status() {
        msg_power_status msg = new msg_power_status();
        // //parent.Log("send_power_status %11%");
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

        // //parent.Log("send_battery_status %12%");
        sendMessage(msg);
    }

    void send_home_position() {
        msg_home_position msg = new msg_home_position();

        msg.latitude = (int) (mLatitude * Math.pow(10, 7));
        msg.longitude = (int) (mLongitude * Math.pow(10, 7));
        // msg.altitude = (int) (djiAircraft.getFlightController().getState().getHomePointAltitude());
//        msg.altitude = (int) (djiAircraft.getFlightController().getState().getGoHomeHeight()); 이거 V5에서 똑같은 역할 하는거 찾아야 함

        // msg.x = 0;
        // msg.y = 0;
        // msg.z = 0;
        // msg.approach_x = 0;
        // msg.approach_y = 0;
        // msg.approach_z = 0;
        // //parent.Log("send_home_position %13%");
        sendMessage(msg);
    }
    void set_home_position(double lat, double lon) {


        KeyManager.getInstance().setValue(KeyTools.createKey(FlightControllerKey.KeyHomeLocation), new LocationCoordinate2D(lat, lon), new CommonCallbacks.CompletionCallback() {
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
//                        KeyManager.getInstance().setValue(KeyTools.createKey(FlightControllerKey.KeyFlightMode), FlightMode.MANUAL, new CommonCallbacks.CompletionCallback() {
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


    public void initMissionManager() {

        mMissionManager = WaypointMissionManager.getInstance();
    }

    public void toastinMain(String inputfrommodel) {
        //parent.Log(inputfrommodel);
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
//                //parent.Log("SECONDARY PACKET SENT");
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
               // parent.Log("UDP Send :"+msg.toString());
            } else {

                // TODO TCP Send
                OutputStream os = this.mTcpSocket.getOutputStream();

//                Log.d(TAG, "<<<<<<<<<<<<<<<<< TCP Send Start <<<<<<<<<<<<<<<<<<<<< SysID:" + Integer.valueOf(mSystemId) + " / Port:" +  Integer.valueOf(this.mIsa.getPort()) );
//                log.info("TcpWorkerLink :: WriteBytes sysID index: {}, targetIp : {}, targetPort : {}", new Object[] { Integer.valueOf(index), targetIp, Integer.valueOf(targetPort) });
//                Log.d(TAG, "TcpWorkerLink :: Send1  MAVLink TCP Data = "+ bytes + ", length ="+ Integer.valueOf(bytes.length));
//                InetAddress targetInetAddr = InetAddress.getByName(targetIp);
                os.write(bytes);
                os.flush();
//                Log.d(TAG, "Confirm, TCP Packet Config IP : {}, Port : {} Received-IP:"+ this.mIsa.getAddress() + " / Received-localport:" +  this.mIsa.getPort());

               // parent.Log("TCP Send :" + msg.toString());
            }

        } catch (PortUnreachableException ignored) {

        } catch (IOException e) {
//            Log.d(TAG, "Write Error ::");
        }
    }

    void send_mission_ack(int status) {

        msg_mission_ack msg = new msg_mission_ack();
        msg.type = (short) status;
        msg.mission_type = MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
        sendMessage(msg);
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
//    void startWaypointMission() {
//        mAutonomy = false;
//        //parent.LogMessageDJI("start WaypointMission()");
//
//        if (getWaypointMissionOperator() == null) {
//            //parent.LogMessageDJI("start WaypointMission() - WaypointMissionOperator null");
//            return;
//        }
//        if (getWaypointMissionOperator().getCurrentState() == WaypointMissionState.READY_TO_EXECUTE) {
//            //parent.LogMessageDJI("Ready to execute mission!\n");
//        } else {
//            //parent.LogMessageDJI("Not ready to execute mission\n");
//            //parent.LogMessageDJI(getWaypointMissionOperator().getCurrentState().getName());
//            return;
//        }
//        if (mSafetyEnabled) {
//            //parent.LogMessageDJI("You must turn off the safety to start mission");
//        } else {
//            getWaypointMissionOperator().startMission(djiError -> {
//                if (djiError != null) {
//                    //parent.LogMessageDJI("Error: " + djiError.toString());
//                    // TODO: 여기서 start image interval 할지, 데이터 들어오는 고도 체킹해서 할지 MAV_CMD_IMAGE_START_CAPTURE 동작 유무 확인 후 수행.
//                    //        this.mDrone.do_set_Gimbal(9, 120);
//                } else
//                    //parent.LogMessageDJI("Mission started!");
//            });
//        }
//    }
//
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
        msg.command = message_id;
        msg.result = (short) result;
        sendMessage(msg);
    }


    public void listen3DLocation() {
        KeyManager.getInstance().listen(KeyTools.createKey(FlightControllerKey.KeyAircraftLocation3D), this, new CommonCallbacks.KeyListener<LocationCoordinate3D>() {
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

        KeyManager.getInstance().getValue(KeyTools.createKey(RemoteControllerKey.KeyStickLeftVertical), new CommonCallbacks.CompletionCallbackWithParam<Integer>() {
            @Override
            public void onSuccess(Integer integer) {
                mThrottleSetting = (int) integer;
                parent.Log("listenRemoteControllerSticks() %4% mThrottleSetting " + integer);
            }

            @Override
            public void onFailure(@NonNull IDJIError idjiError) {
                parent.Log("listenRemoteControllerSticks() %4% mThrottleSetting : failed");
            }
        });

        KeyManager.getInstance().listen(KeyTools.createKey(RemoteControllerKey.KeyStickLeftVertical), this, new CommonCallbacks.KeyListener<Integer>() {
            @Override
            public void onValueChange(@Nullable Integer integer, @Nullable Integer t1) {
                mLeftStickVertical = (int) (t1 * 0.8) + 1500;
                parent.Log("listenRemoteControllerSticks() %5% mLeftStickVertical : " + mLeftStickVertical);
            }
        });
        KeyManager.getInstance().listen(KeyTools.createKey(RemoteControllerKey.KeyStickLeftHorizontal), this, new CommonCallbacks.KeyListener<Integer>() {
            @Override
            public void onValueChange(@Nullable Integer integer, @Nullable Integer t1) {
                mLeftStickHorisontal = (int) (t1 * 0.8) + 1500;
                parent.Log("listenRemoteControllerSticks() %6% mLeftStickHorisontal : " + mLeftStickHorisontal);
            }
        });
        KeyManager.getInstance().listen(KeyTools.createKey(RemoteControllerKey.KeyStickRightVertical), this, new CommonCallbacks.KeyListener<Integer>() {
            @Override
            public void onValueChange(@Nullable Integer integer, @Nullable Integer t1) {
                mRightStickVertical = (int) (t1 * 0.8) + 1500;
                parent.Log("listenRemoteControllerSticks() %7% mRightStickVertical : " + mRightStickVertical);
            }
        });
        KeyManager.getInstance().listen(KeyTools.createKey(RemoteControllerKey.KeyStickRightHorizontal), this, new CommonCallbacks.KeyListener<Integer>() {
            @Override
            public void onValueChange(@Nullable Integer integer, @Nullable Integer t1) {
                mRightStickHorisontal = (int) (t1 * 0.8) + 1500;
                parent.Log("listenRemoteControllerSticks() %8% mRightStickHorisontal : " + mRightStickHorisontal);
            }
        });
        KeyManager.getInstance().listen(KeyTools.createKey(RemoteControllerKey.KeyCustomButton1Down), this, new CommonCallbacks.KeyListener<Boolean>() {
            @Override
            public void onValueChange(@Nullable Boolean aBoolean, @Nullable Boolean t1) {
                mC1 = t1;
                parent.Log("listenRemoteControllerSticks() %9% mC1 clicked : " + mC1);
            }
        });
        KeyManager.getInstance().listen(KeyTools.createKey(RemoteControllerKey.KeyCustomButton2Down), this, new CommonCallbacks.KeyListener<Boolean>() {
            @Override
            public void onValueChange(@Nullable Boolean aBoolean, @Nullable Boolean t1) {
                mC2 = t1;
                parent.Log("listenRemoteControllerSticks() %10% mC2 clicked : " + mC2);
            }
        });

        KeyManager.getInstance().listen(KeyTools.createKey(RemoteControllerKey.KeyCustomButton3Down), this, new CommonCallbacks.KeyListener<Boolean>() {
            @Override
            public void onValueChange(@Nullable Boolean aBoolean, @Nullable Boolean t1) {
                mC3 = t1;
                parent.Log("listenRemoteControllerSticks() %11% mC3 clicked : " + mC3);
            }
        });

    }

    public void listenIsMotorOn() {
        KeyManager.getInstance().listen(KeyTools.createKey(FlightControllerKey.KeyAreMotorsOn), this, new CommonCallbacks.KeyListener<Boolean>() {
                    @Override
                    public void onValueChange(@Nullable Boolean aBoolean, @Nullable Boolean t1) {
                        if (t1 != null) {
                            isMotorOn = t1;
                            parent.Log("%12% listenIsMotorOn() : " + isMotorOn);
                        }
                    }
                }


        );
    }


    public void listenAttitude() {
        KeyManager.getInstance().listen(KeyTools.createKey(FlightControllerKey.KeyAircraftAttitude), this, new CommonCallbacks.KeyListener<Attitude>() {
            @Override
            public void onValueChange(@Nullable Attitude attitude, @Nullable Attitude t1) {
                if (t1 != null) {
                    mPitch = t1.getPitch();
                    mRoll = t1.getRoll();
                    mYaw = t1.getYaw();
                    parent.Log(" %13% listenAttitude() pitch: " + mPitch + " " + " %14% roll: " + mRoll + " " + " %15% yaw: " + mYaw);

                }
            }
        });
    }


    public void listenDroneBattery() {

        KeyManager.getInstance().listen(KeyTools.createKey(BatteryKey.KeyChargeRemaining), this, new CommonCallbacks.KeyListener<Integer>() {
            @Override
            public void onValueChange(@Nullable Integer integer, @Nullable Integer t1) {
                mCChargeRemaining_mAh = (int) t1;
                parent.Log("%16% mCChargeRemaining_mAh : " + mCChargeRemaining_mAh);
            }
        });
        KeyManager.getInstance().listen(KeyTools.createKey(BatteryKey.KeyCellVoltages), this, new CommonCallbacks.KeyListener<List<Integer>>() {
            @Override
            public void onValueChange(@Nullable List<Integer> integers, @Nullable List<Integer> t1) {
                for (int i = 0; i < t1.size(); i++) {
                    mCellVoltages[i] = t1.get(i);
                    parent.Log("%17% got cell voltages, v[" + i + "] =" + mCellVoltages[i]);
                }
            }
        });
        KeyManager.getInstance().listen(KeyTools.createKey(BatteryKey.KeyVoltage), this, new CommonCallbacks.KeyListener<Integer>() {
            @Override
            public void onValueChange(@Nullable Integer integer, @Nullable Integer t1) {
                mCVoltage_mV = t1;
                parent.Log("%18% mCVoltage_mV : " + mCVoltage_mV);
            }
        });
        KeyManager.getInstance().listen(KeyTools.createKey(BatteryKey.KeyChargeRemainingInPercent), this, new CommonCallbacks.KeyListener<Integer>() {
            @Override
            public void onValueChange(@Nullable Integer integer, @Nullable Integer t1) {
                mCVoltage_pr = (int) t1;
                parent.Log("%19% mCVoltage_pr : " + mCVoltage_pr);
            }
        });
        KeyManager.getInstance().listen(KeyTools.createKey(BatteryKey.KeyBatteryTemperature), this, new CommonCallbacks.KeyListener<Double>() {
            @Override
            public void onValueChange(@Nullable Double aDouble, @Nullable Double t1) {
                mCBatteryTemp_C = t1;
                parent.Log("%20% mCBatteryTemp_C : " + mCBatteryTemp_C);

            }
        });
        KeyManager.getInstance().listen(KeyTools.createKey(BatteryKey.KeyCurrent), this, new CommonCallbacks.KeyListener<Integer>() {
            @Override
            public void onValueChange(@Nullable Integer integer, @Nullable Integer t1) {
                mCCurrent_mA = t1;
                parent.Log("%21% mCCurrent_mA : " + mCCurrent_mA);
            }
        });


    }

    public void listenControllerBattery() {
        KeyManager.getInstance().listen(KeyTools.createKey(RemoteControllerKey.KeyBatteryInfo), this, new CommonCallbacks.KeyListener<BatteryInfo>() {
            @Override
            public void onValueChange(@Nullable BatteryInfo batteryInfo, @Nullable BatteryInfo t1) {
                mControllerVoltage_pr = (int) t1.getBatteryPercent();
                parent.Log("%22% listenControllerBattery() mControllerVoltage_pr : " + mControllerVoltage_pr);
            }
        });
    }


    public void listenVelocity() {
        KeyManager.getInstance().listen(KeyTools.createKey(FlightControllerKey.KeyAircraftVelocity), this, new CommonCallbacks.KeyListener<Velocity3D>() {
                    @Override
                    public void onValueChange(@Nullable Velocity3D velocity3D, @Nullable Velocity3D t1) {
                        if (t1 != null) {
                            velocityx = t1.getX();
                            velocityy = t1.getY();
                            velocityz = t1.getZ();
                            parent.Log("listenVelocity() %23% velocityx" + velocityx + " " + " %24% velocityy: " + velocityy + " " + " %25% velocityz: " + velocityz);
                        }
                    }
                }

        );
    }


    public void listenIsFly() {
        KeyManager.getInstance().listen(KeyTools.createKey(FlightControllerKey.KeyIsFlying), this, new CommonCallbacks.KeyListener<Boolean>() {
                    @Override
                    public void onValueChange(@Nullable Boolean aBoolean, @Nullable Boolean t1) {
                        if (t1 != null) {
                            isFlying = t1;
                            parent.Log("listenIsFly() %26% isFlying : " + isFlying);
                        }
                    }
                }


        );
    }

    public void listenFlightMode() {
        KeyManager.getInstance().listen(KeyTools.createKey(FlightControllerKey.KeyFlightMode), this, new CommonCallbacks.KeyListener<FlightMode>() {
            @Override
            public void onValueChange(@Nullable FlightMode flightMode, @Nullable FlightMode t1) {
                lastMode = t1;
                parent.Log("%27% getFlightMode() : " + lastMode.toString());
            }
        });


    }

    public void listenAirlinkQuality() {
        KeyManager.getInstance().listen(KeyTools.createKey(AirLinkKey.KeyUpLinkQuality), this, new CommonCallbacks.KeyListener<Integer>() {
                    @Override
                    public void onValueChange(@Nullable Integer integer, @Nullable Integer t1) {
                        mUplinkQuality = t1;
                        parent.Log("%28% mUplinkQuality : " + mUplinkQuality);
                    }
                }

        );
        KeyManager.getInstance().listen(KeyTools.createKey(AirLinkKey.KeyDownLinkQuality), this, new CommonCallbacks.KeyListener<Integer>() {
                    @Override
                    public void onValueChange(@Nullable Integer integer, @Nullable Integer t1) {
                        mDownlinkQuality = t1;
                        parent.Log("%29% mDownlinkQuality : " + mDownlinkQuality);
                    }
                }

        );

    }


    public void getFlightMode() {
        KeyManager.getInstance().getValue(KeyTools.createKey(FlightControllerKey.KeyFlightMode), new CommonCallbacks.CompletionCallbackWithParam<FlightMode>() {
            @Override
            public void onSuccess(FlightMode flightMode) {
                lastMode = flightMode;
                //parent.Log("getFlightMode() : " + flightMode.name());
            }

            @Override
            public void onFailure(@NonNull IDJIError idjiError) {
                //parent.Log("getFlightMode() Failed ");
            }
        });


    }

    public void getfullchargecapacity() {


        KeyManager.getInstance().getValue(KeyTools.createKey(BatteryKey.KeyFullChargeCapacity), new CommonCallbacks.CompletionCallbackWithParam<Integer>() {
            @Override
            public void onSuccess(Integer integer) {
                mCFullChargeCapacity_mAh = (int) integer;
                //parent.Log("%23% mFullChargeCapacity_mAh : " + mCFullChargeCapacity_mAh);
            }

            @Override
            public void onFailure(@NonNull IDJIError idjiError) {
                //parent.Log("getfullchargecapacity() Failed ");
            }
        });


    }


    public void getIsMotorOn() {

        KeyManager.getInstance().getValue(KeyTools.createKey(FlightControllerKey.KeyAreMotorsOn), new CommonCallbacks.CompletionCallbackWithParam<Boolean>() {
                    @Override
                    public void onSuccess(Boolean aBoolean) {
                        isMotorOn = aBoolean;
                    }

                    @Override
                    public void onFailure(@NonNull IDJIError idjiError) {
                        //parent.Log("Get isMotorOn Failed ");
                    }
                }


        );
    }


    //region Fields

    public void test(){
        mMissionManager=new WaypointMissionManager();
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

    public static double DEFUALT_LONGITUDE = 126.6417732; //126.6381979; // 126.6413591;//개발중일 때 사용할 임의의 경도값
    private final boolean doImageInterval = false;//기체가 현재 웨이포인트와 다음 웨이포인트 사이를 이동할 때 두 장의 사진이 촬영되는 시간 간격(초)
    private final float captureInterval = 0.0f;//사진 찍는 간격, DDM내에서 주석처리 되어있는거 보니 안쓰이는듯
    private final int batteryIndex = 0;
    private final ArrayList<MAVParam> params = new ArrayList<>();
    private int mGCSCommandedMode;


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
    private final double mDestinationLat = 0;//관제에서 전달받은 임무 명령의 위도
    private final double mDestinationLon = 0;//관제에서 전달받은 임무 명령의 경도
    private final float mDestinationAlt = 0;//관제에서 전달받은 임무 명령의 고도
    private final double mDestinationYaw = 0;//관제에서 전달받은 임무 명령의 YAW값
    private final double mDestinationYawrate = 0; //관제에서 전달받은 임무 명령의 YAW 비율(?)
    private final float mDestinationSetVx = -1;//관제에서 전달받은 임무 명령의 x축속도
    private final float mDestinationSetVy = -1;//관제에서 전달받은 임무 명령의 y축속도
    private final float mDestinationSetVz = -1;//관제에서 전달받은 임무 명령의 z축속도
    private final int mDestinationMask = 0; //?
    private final double mDestinationbrng = 0; //웨이포인트와 현재 기체위치의 방위각
    private final double mDestinationhypotenuse = 0; //목적지까지 거리


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
