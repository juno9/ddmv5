package dji.v5.ux.sample.util;

import static dji.sdk.wpmz.value.mission.WaylineWaypointTurnMode.TO_POINT_AND_STOP_WITH_DISCONTINUITY_CURVATURE;
import static dji.v5.ux.MAVLink.ardupilotmega.msg_autopilot_version_request.MAVLINK_MSG_ID_AUTOPILOT_VERSION_REQUEST;

import static dji.v5.ux.MAVLink.common.msg_command_int.MAVLINK_MSG_ID_COMMAND_INT;
import static dji.v5.ux.MAVLink.common.msg_command_long.MAVLINK_MSG_ID_COMMAND_LONG;
import static dji.v5.ux.MAVLink.common.msg_heartbeat.MAVLINK_MSG_ID_HEARTBEAT;
import static dji.v5.ux.MAVLink.common.msg_manual_control.MAVLINK_MSG_ID_MANUAL_CONTROL;
import static dji.v5.ux.MAVLink.common.msg_mission_ack.MAVLINK_MSG_ID_MISSION_ACK;
import static dji.v5.ux.MAVLink.common.msg_mission_clear_all.MAVLINK_MSG_ID_MISSION_CLEAR_ALL;
import static dji.v5.ux.MAVLink.common.msg_mission_count.MAVLINK_MSG_ID_MISSION_COUNT;
import static dji.v5.ux.MAVLink.common.msg_mission_item.MAVLINK_MSG_ID_MISSION_ITEM;
import static dji.v5.ux.MAVLink.common.msg_mission_item_int.MAVLINK_MSG_ID_MISSION_ITEM_INT;
import static dji.v5.ux.MAVLink.common.msg_mission_request.MAVLINK_MSG_ID_MISSION_REQUEST;
import static dji.v5.ux.MAVLink.common.msg_mission_request_list.MAVLINK_MSG_ID_MISSION_REQUEST_LIST;
import static dji.v5.ux.MAVLink.common.msg_mission_request_partial_list.MAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST;
import static dji.v5.ux.MAVLink.common.msg_mission_set_current.MAVLINK_MSG_ID_MISSION_SET_CURRENT;
import static dji.v5.ux.MAVLink.common.msg_param_request_list.MAVLINK_MSG_ID_PARAM_REQUEST_LIST;
import static dji.v5.ux.MAVLink.common.msg_param_request_read.MAVLINK_MSG_ID_PARAM_REQUEST_READ;
import static dji.v5.ux.MAVLink.common.msg_param_set.MAVLINK_MSG_ID_PARAM_SET;
import static dji.v5.ux.MAVLink.common.msg_set_mode.MAVLINK_MSG_ID_SET_MODE;
import static dji.v5.ux.MAVLink.common.msg_set_position_target_global_int.MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT;
import static dji.v5.ux.MAVLink.common.msg_set_position_target_local_ned.MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED;
import static dji.v5.ux.MAVLink.enums.MAV_CMD.MAV_CMD_COMPONENT_ARM_DISARM;
import static dji.v5.ux.MAVLink.enums.MAV_CMD.MAV_CMD_CONDITION_YAW;
import static dji.v5.ux.MAVLink.enums.MAV_CMD.MAV_CMD_DO_DIGICAM_CONTROL;
import static dji.v5.ux.MAVLink.enums.MAV_CMD.MAV_CMD_DO_JUMP;
import static dji.v5.ux.MAVLink.enums.MAV_CMD.MAV_CMD_DO_SET_HOME;
import static dji.v5.ux.MAVLink.enums.MAV_CMD.MAV_CMD_DO_SET_MODE;
import static dji.v5.ux.MAVLink.enums.MAV_CMD.MAV_CMD_DO_SET_SERVO;
import static dji.v5.ux.MAVLink.enums.MAV_CMD.MAV_CMD_GET_HOME_POSITION;
import static dji.v5.ux.MAVLink.enums.MAV_CMD.MAV_CMD_MISSION_START;
import static dji.v5.ux.MAVLink.enums.MAV_CMD.MAV_CMD_NAV_LAND;
import static dji.v5.ux.MAVLink.enums.MAV_CMD.MAV_CMD_NAV_LOITER_UNLIM;
import static dji.v5.ux.MAVLink.enums.MAV_CMD.MAV_CMD_NAV_RETURN_TO_LAUNCH;
import static dji.v5.ux.MAVLink.enums.MAV_CMD.MAV_CMD_NAV_TAKEOFF;
import static dji.v5.ux.MAVLink.enums.MAV_CMD.MAV_CMD_NAV_WAYPOINT;
import static dji.v5.ux.MAVLink.enums.MAV_CMD.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES;
import static dji.v5.ux.MAVLink.enums.MAV_CMD.MAV_CMD_VIDEO_START_CAPTURE;
import static dji.v5.ux.MAVLink.enums.MAV_CMD.MAV_CMD_VIDEO_STOP_CAPTURE;
import static dji.v5.ux.MAVLink.enums.MAV_MISSION_RESULT.MAV_MISSION_ACCEPTED;
import static dji.v5.ux.MAVLink.enums.MAV_MISSION_TYPE.MAV_MISSION_TYPE_MISSION;
import static dji.v5.ux.MAVLink.ardupilotmega.msg_autopilot_version_request.MAVLINK_MSG_ID_AUTOPILOT_VERSION_REQUEST;
import static dji.v5.ux.MAVLink.common.msg_heartbeat.MAVLINK_MSG_ID_HEARTBEAT;
import static dji.v5.ux.MAVLink.enums.MAV_CMD.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES;
import static dji.v5.ux.utils.KMZTestUtil.DEF_GLOBAL_FLIGHT_HEIGHT;
import static dji.v5.ux.utils.KMZTestUtil.transformActionsFrom;


import android.util.Log;

import androidx.annotation.NonNull;

import com.dji.wpmzsdk.common.data.Template;

import dji.sdk.keyvalue.value.mission.WaylineActionInfo;
import dji.sdk.keyvalue.value.mission.WaypointMission;
import dji.sdk.wpmz.value.mission.WaylineLocationCoordinate2D;
import dji.sdk.wpmz.value.mission.WaylineTemplateWaypointInfo;
import dji.sdk.wpmz.value.mission.WaylineWaypoint;
import dji.sdk.wpmz.value.mission.WaylineWaypointPitchMode;
import dji.sdk.wpmz.value.mission.WaylineWaypointYawMode;
import dji.sdk.wpmz.value.mission.WaylineWaypointYawParam;
import dji.v5.common.callback.CommonCallbacks;
import dji.v5.common.error.IDJIError;
import dji.v5.manager.aircraft.waypoint3.WaypointMissionManager;
import dji.v5.ux.MAVLink.Messages.MAVLinkMessage;
import dji.v5.ux.MAVLink.common.msg_command_long;
import dji.v5.ux.MAVLink.common.msg_heartbeat;
import dji.v5.ux.MAVLink.common.msg_manual_control;
import dji.v5.ux.MAVLink.common.msg_mission_ack;
import dji.v5.ux.MAVLink.common.msg_mission_count;
import dji.v5.ux.MAVLink.common.msg_mission_item;
import dji.v5.ux.MAVLink.common.msg_mission_item_int;
import dji.v5.ux.MAVLink.common.msg_mission_request;
import dji.v5.ux.MAVLink.common.msg_param_request_read;
import dji.v5.ux.MAVLink.common.msg_param_set;
import dji.v5.ux.MAVLink.common.msg_set_mode;
import dji.v5.ux.MAVLink.common.msg_set_position_target_global_int;
import dji.v5.ux.MAVLink.common.msg_set_position_target_local_ned;
import dji.v5.ux.MAVLink.enums.MAV_CMD;
import dji.v5.ux.MAVLink.enums.MAV_RESULT;
import dji.v5.ux.MAVLink.enums.MAV_TYPE;

import java.util.ArrayList;
import java.util.List;
//import static sq.rogue.rosettadrone.util.TYPE_WAYPOINT_DISTANCE;
//import static sq.rogue.rosettadrone.util.TYPE_WAYPOINT_MAX_ALTITUDE;
//import static sq.rogue.rosettadrone.util.TYPE_WAYPOINT_MAX_SPEED;
//import static sq.rogue.rosettadrone.util.TYPE_WAYPOINT_MIN_ALTITUDE;
//import static sq.rogue.rosettadrone.util.TYPE_WAYPOINT_MIN_SPEED;
//import static sq.rogue.rosettadrone.util.safeSleep;
//import dji.common.mission.waypoint.Waypoint;
//import dji.common.mission.waypoint.WaypointAction;
//import dji.common.mission.waypoint.WaypointActionType;
//import dji.common.mission.waypoint.WaypointMission;
//import dji.common.mission.waypoint.WaypointMissionFinishedAction;
//import dji.common.mission.waypoint.WaypointMissionFlightPathMode;
//import dji.common.mission.waypoint.WaypointMissionGotoWaypointMode;
//import dji.common.mission.waypoint.WaypointMissionHeadingMode;
//import dji.common.mission.waypoint.WaypointMissionState;

import dji.v5.ux.sample.showcase.defaultlayout.DefaultLayoutActivity;
import dji.v5.ux.utils.KMZTestUtil;
import dji.v5.ux.utils.wpml.WaypointInfoModel;

public class MAVLinkReceiver {
    private static final int MAV_MISSION_ACCEPTED = 0;
    private final String TAG = this.getClass().getSimpleName();

    private float m_autoFlightSpeed = 2.0f;
    private float m_maxFlightSpeed = 5.0f;


    private final int WP_STATE_INACTIVE = 0;
    private final int WP_STATE_REQ_COUNT = 1;
    private final int WP_STATE_REQ_WP = 2;
    private final int MAX_WAYPOINT_DISTANCE = 475;
    public boolean curvedFlightPath = true;
    public float flightPathRadius = .2f;
    DroneModel mModel;
    private long mTimeStampLastGCSHeartbeat = 0;
    private int mNumGCSWaypoints = 0;
    private int wpState = 0;
    private DefaultLayoutActivity parent;
    //private WaypointMission.Builder mBuilder;

    private WpMissionManager wpMissionManager;
    String kmzOutPath;
    private boolean isHome = true;


    public MAVLinkReceiver(DefaultLayoutActivity parent, DroneModel model) {
        this.parent = parent;
        this.mModel = model;

    }

    public void process(MAVLinkMessage msg) {


        WaypointMissionManager WPMmanager = WaypointMissionManager.getInstance();

        // IS 0 is hart beat...
        if (msg.msgid != 0) {
            Log.d(TAG, msg.toString());
        }

        switch (msg.msgid) {
//            case MAVLINK_MSG_ID_AUTOPILOT_VERSION_REQUEST:
//                Log.d(TAG, "send_autopilot_version...");
//                mModel.send_command_ack(MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES, MAV_RESULT.MAV_RESULT_ACCEPTED);
//                mModel.send_autopilot_version();
//                break;
//
            case MAVLINK_MSG_ID_HEARTBEAT:
                Log.d(TAG, "<<<<<<<<<<<<<<<<<<<<<<<<<<<<< MAVLINK_MSG_ID_HEARTBEAT FROM DROW <<<<<<<<<<<<<<<<<<<<<<<<<<<<<");
                this.mTimeStampLastGCSHeartbeat = System.currentTimeMillis();
                msg_heartbeat heartbeatMsg = (msg_heartbeat) msg;
                switch (heartbeatMsg.type) {
                    case MAV_TYPE.MAV_TYPE_GCS:
                    case MAV_TYPE.MAV_TYPE_GIMBAL:
                    case MAV_TYPE.MAV_TYPE_ADSB:
                    case MAV_TYPE.MAV_TYPE_ONBOARD_CONTROLLER:
                        return;
                    default:
                        break;
                }
                // DJI PRODUCT NUMBER는 MAVLINK SYSID 랑은 다르기 때문에, 사용자가 매핑하도록 UI의 값을 지정한 것으로 통일.
                // 만일 UI에서 중간에 변경한다면, 경고창을 띄우고 재시작 하도록함.
//                mModel.setSystemId(Integer.parseInt(Objects.requireNonNull(prefs.getString("pref_drone_id", "1"))));
//                mModel.getSharedPreferences().getStringSet("pref_drone_id", )
                break;
//
            case MAVLINK_MSG_ID_COMMAND_LONG:
                msg_command_long msg_cmd = (msg_command_long) msg;

                if (mModel.getSystemId() != msg_cmd.target_system) {
                    return;
                }

//                if (mModel.isSafetyEnabled()) {
//                    Log.d(TAG, parent.getResources().getString(R.string.safety_launch));
//                    return;
//                }

                switch (msg_cmd.command) {
//                    case MAV_CMD_COMPONENT_ARM_DISARM:
//                        if (msg_cmd.param1 == 1)
//                            mModel.armMotors();
//                        else
//                            mModel.disarmMotors();
//                        // TEST
////                        mModel.echoLoadedMission();
//                        break;
//                    case MAV_CMD_DO_SET_MODE:
//                        changeFlightMode((int) msg_cmd.param2);
//                        break;
                    case MAV_CMD_NAV_LOITER_UNLIM:
                        //                     mModel.set_flight_mode(ATTI);
                        break;
                    case MAV_CMD_NAV_TAKEOFF:
                        Log.d(TAG, "ALT = " + msg_cmd.param7);
                        mModel.mAirBorn = 0;
//                        mModel.do_takeoff(msg_cmd.param7 * (float) 1000.0);
                        mModel.do_takeoff(msg_cmd.param7);
                        mModel.send_command_ack(MAV_CMD_NAV_TAKEOFF, MAV_RESULT.MAV_RESULT_IN_PROGRESS);
                        break;
//                    case MAV_CMD_NAV_LAND:
//                        mModel.do_land();
//                        break;
                    case MAV_CMD_DO_SET_HOME:
                        Log.d(TAG, "LAT = " + msg_cmd.param5);
                        Log.d(TAG, "LONG = " + msg_cmd.param6);
                        Log.d(TAG, "ALT = " + msg_cmd.param7);
                        mModel.set_home_position(msg_cmd.param5, msg_cmd.param6);
                        break;
//                    case MAV_CMD_NAV_RETURN_TO_LAUNCH:
//                        Log.d(TAG, "MAV_CMD_NAV_RETURN_TO_LAUNCH...");
//                        mModel.do_go_home();
//                        mModel.send_command_ack(MAV_CMD_NAV_RETURN_TO_LAUNCH, MAV_RESULT.MAV_RESULT_ACCEPTED);
//                        break;
                    case MAV_CMD_GET_HOME_POSITION:
                        mModel.send_home_position();
                        break;
                    case MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES:
                        Log.d(TAG, "send_autopilot_version...");
                        mModel.send_autopilot_version();
                        break;
//                    case MAV_CMD_VIDEO_START_CAPTURE:
//                        mModel.startRecordingVideo();
//                        break;
//                    case MAV_CMD_VIDEO_STOP_CAPTURE:
//                        mModel.stopRecordingVideo();
//                        break;
//                    case MAV_CMD_DO_DIGICAM_CONTROL:
//                        // DEPRECATED but still used by QGC
//                        mModel.takePhoto();
//                        break;
                    case MAV_CMD_MISSION_START:
//                        if( !mModel.isIntervalControl() ) {
                        Log.i(TAG,"mission_start_Message_taken");
                            mModel.startWaypointMission();
//                        } else {
//                            mModel.startImageCaptureIntervalControl();
//                        }
                        break;
                    case MAV_CMD_CONDITION_YAW:
//                        Log.d(TAG, "Yaw = " + msg_cmd.param1);
//
//                        // If absolute yaw...
//                        if (msg_cmd.param4 == 0) {
//                            mModel.send_command_ack(MAV_CMD_CONDITION_YAW, MAV_RESULT.MAV_RESULT_IN_PROGRESS);
//                            mModel.do_set_motion_absolute(
//                                    0,
//                                    0,
//                                    0,
//                                    msg_cmd.param1 * (float) (Math.PI / 180.0),
//                                    0,
//                                    0,
//                                    0,
//                                    10,
//                                    0b1111001111111111);
//                        } else {
//                            mModel.send_command_ack(MAV_CMD_CONDITION_YAW, MAV_RESULT.MAV_RESULT_UNSUPPORTED);
//                        }
//                        break;
                    case MAV_CMD_DO_SET_SERVO:
//                        mModel.do_set_Gimbal(msg_cmd.param1, msg_cmd.param2);
//                        break;
                        // JUMP is just a test function to enter the Timeline...
                    case MAV_CMD_DO_JUMP:
                        Log.d(TAG, "Start Timeline...");
                        //    mModel.echoLoadedMission();
                        break;
                }
                break;

            case MAVLINK_MSG_ID_COMMAND_INT:
                // TODO I don't understand what this message is, but ArduCopter handles it.
                // See ArduCopter/GCS_Mavlink.cpp
                break;

            case MAVLINK_MSG_ID_SET_MODE:
//                msg_set_mode msg_set_mode = (msg_set_mode) msg;
//                Log.d(TAG, "MAVLINK_MSG_ID_SET_MODE: " + msg_set_mode.custom_mode);
//                changeFlightMode((int) msg_set_mode.custom_mode);
//                break;

                /**************************************************************
                 * These messages are used when GCS sends Velocity commands   *
                 **************************************************************/
//            case MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED:
//                msg_set_position_target_local_ned msg_param = (msg_set_position_target_local_ned) msg;
//                if (((msg_param.type_mask & 0b0000100000000111) == 0x007)) {  // If no move and we use yaw rate...
//                    mModel.mAutonomy = false; // Velocity must halt autonomy (as Autonomy tries to reach a point, whule veliciity tries to set a speed)
//                    mModel.do_set_motion_velocity(msg_param.vx, msg_param.vy, msg_param.vz, (float) Math.toDegrees(msg_param.yaw_rate), msg_param.type_mask);
//                    mModel.send_command_ack(MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED, MAV_RESULT.MAV_RESULT_ACCEPTED);
//                } else {
//                    mModel.send_command_ack(MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED, MAV_RESULT.MAV_RESULT_IN_PROGRESS);
//                    mModel.do_set_motion_relative(
//                            MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED,
//                            (double) msg_param.x,
//                            (double) msg_param.y,
//                            msg_param.z,
//                            msg_param.yaw,
//                            msg_param.vx,
//                            msg_param.vy,
//                            msg_param.vz,
//                            msg_param.yaw_rate,
//                            msg_param.type_mask);
//                }
//                break;

//            case MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT:
//                // This command must be sent every second...
//                msg_set_position_target_global_int msg_param_4 = (msg_set_position_target_global_int) msg;
//
//                // If position is set to zero then it must be a velocity command... We should use rather the mask ...
//                if (((msg_param_4.type_mask & 0b0000100000000111) == 0x007)) {  // If no move and we use yaw rate...
//                    mModel.mAutonomy = false;
//                    mModel.do_set_motion_velocity_NED(msg_param_4.vx, msg_param_4.vy, msg_param_4.vz, (float) Math.toDegrees(msg_param_4.yaw_rate), msg_param_4.type_mask);
//                    mModel.send_command_ack(MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT, MAV_RESULT.MAV_RESULT_ACCEPTED);
//                } else {
//                    mModel.send_command_ack(MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT, MAV_RESULT.MAV_RESULT_IN_PROGRESS);
//                    mModel.do_set_motion_absolute(
//                            (double) msg_param_4.lat_int / 10000000,
//                            (double) msg_param_4.lon_int / 10000000,
//                            msg_param_4.alt,
//                            msg_param_4.yaw,
//                            msg_param_4.vx,
//                            msg_param_4.vy,
//                            msg_param_4.vz,
//                            msg_param_4.yaw_rate,
//                            msg_param_4.type_mask);
//                }
//                break;

                // This command must be sent at 1Hz minimum...
//            case MAVLINK_MSG_ID_MANUAL_CONTROL:
//                msg_manual_control msg_param_5 = (msg_manual_control) msg;
//
//                mModel.do_set_motion_velocity(
//                        msg_param_5.x / (float) 100.0,
//                        msg_param_5.y / (float) 100.0,
//                        msg_param_5.z / (float) 260.0,
//                        msg_param_5.r / (float) 50.0,
//                        0b0000011111000111);
//
//                mModel.send_command_ack(MAVLINK_MSG_ID_MANUAL_CONTROL, MAV_RESULT.MAV_RESULT_ACCEPTED);
//                break;


                /**************************************************************
                 * These messages are used when GCS requests params from MAV  *
                 **************************************************************/

            case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
                mModel.send_all_params();
                break;

            case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
                msg_param_request_read msg_param_3 = (msg_param_request_read) msg;
                //String paramStr = msg_param.getParam_Id();
                //parent.logMessageFromGCS("***" + paramStr);
                mModel.send_param(msg_param_3.param_index);
                // TODO I am not able to convert the param_id bytearray into String
//                for(int i = 0; i < mModel.getParams().size(); i++)
//                    if(mModel.getParams().get(i).getParamName().equals(msg_param.getParam_Id())) {
//                        mModel.send_param(i);
//                        break;
//                    }
                Log.d(TAG, "Request to read param that doesn't exist");
                break;

//            case MAVLINK_MSG_ID_PARAM_SET:
//                msg_param_set msg_param2 = (msg_param_set) msg;
//                MAVParam param = new MAVParam(msg_param2.getParam_Id(),
//                        msg_param2.param_value,
//                        msg_param2.param_type);
//                mModel.changeParam(param);
//                break;

            /**************************************************************
             * These messages are used when GCS downloads mission from MAV *
             **************************************************************/

//            case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
//                mModel.send_mission_count();
//
//                break;

            case MAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST:
                // TODO
                break;

//            case MAVLINK_MSG_ID_MISSION_REQUEST:
//                msg_mission_request msg_request = (msg_mission_request) msg;;
//                Log.d(TAG, "Request: "+String.valueOf(msg_request));
//                mModel.send_mission_item(msg_request.seq);
//                break;

            case MAVLINK_MSG_ID_MISSION_ACK:
                msg_mission_ack msg_ack = new msg_mission_ack();
                if (msg_ack.type == MAV_MISSION_TYPE_MISSION) {
                    // TODO success
                } else {
                    // TODO fail
                }
                break;

            /**************************************************************
             * These messages are used when GCS uploads a mission to MAV  *
             **************************************************************/

            // Start load new mission... ddm이 대신 수행.
            case MAVLINK_MSG_ID_MISSION_COUNT:

                //최초 GCS에서 나 이제 너한테 웨이포인트 몇개 줄게 를 먼저 메시지로 전달함
                msg_mission_count msg_count = (msg_mission_count) msg;
                if (mModel.getSystemId() != msg_count.target_system) {
                    return;
                }


                Log.d(TAG, "Expect: Mission Counter: " + msg_count.count);

                mNumGCSWaypoints = msg_count.count;//받을 아이템 갯수 먼저 마브링크 리시버 내에 저장
                wpState = WP_STATE_REQ_WP;//현재 웨이포인트 관련 진행상황 저장
                wpMissionManager.iniMissionItemListt();//전달받은 미션 아이템 메시지를 담아둘 어레이 하나 초기화
                wpMissionManager.initmWLIMList();//미션 아이템 메시지를 바탕으로 웨이포인트인포모델을 만들 어레이를 하나 초기화

                Log.d(TAG, "Mission REQ: 0...");
                mModel.request_mission_item(0);//카운트 수신했으니 0번 아이템 요청
                // 차후, 연결링크 혼잡이나 손실되었을 경우 GCS에서 받지못할면 rqeust 타임아웃이 DROW에 있지만,
                // DDM에서 Firmware 역할하도록 reqeust에 따른 item/itemint의 timer가 필요함.
                break;

            case MAVLINK_MSG_ID_MISSION_ITEM_INT:  // 0x73
            {

                msg_mission_item_int msg_item = (msg_mission_item_int) msg;//수신한 메시지 item 메시지 로 변환
                Log.d(TAG, "Add mission: " + msg_item.seq);

                if (mModel.getSystemId() != msg_item.target_system) {//메시지의 타겟이 이 드론이 맞는지 먼저 확인, 아니라면 바로 조건문 탈출
                    break;
                }

                if (wpMissionManager.getmMissionItemintList() == null) {//아이템 리스트가 선언되어 있지 않다면 == 카운트를 받지 않았다면
                    Log.d(TAG, "Error Sequence error!");
                    mModel.send_command_ack(MAVLINK_MSG_ID_MISSION_ACK, MAV_RESULT.MAV_RESULT_DENIED);//gcs에 ACk전송
                } else {
                    wpMissionManager.getmMissionItemintList().add(msg_item);//아이템을 리스트에 담음
                    if (msg_item.seq == mNumGCSWaypoints - 1) {// seq값이 (count-1) 과 같다면 == 마지막 아이템이라면
                        wpState = WP_STATE_INACTIVE;
                        finalizeNewMission();
                    } else {//마지막 아이템이 아니라면
                        Log.d(TAG, "Mission REQ: " + (msg_item.seq + 1));// 받은 아이템의 seq+1 번째 아이템을 요청해야
                        mModel.request_mission_item((msg_item.seq + 1));
                    }
                }

            }
            break;

            case MAVLINK_MSG_ID_MISSION_ITEM:  // 39

                msg_mission_item msg_item = (msg_mission_item) msg;//수신한 메시지 item 메시지 로 변환
                Log.d(TAG, "Add mission: " + msg_item.seq);

                if (mModel.getSystemId() != msg_item.target_system) {//메시지의 타겟이 이 드론이 맞는지 먼저 확인, 아니라면 바로 조건문 탈출
                    break;
                }

                if (wpMissionManager.getmMissionItemList() == null) {//아이템 리스트가 선언되어 있지 않다면 == 카운트를 받지 않았다면
                    Log.d(TAG, "Error Sequence error!");
                    mModel.send_command_ack(MAVLINK_MSG_ID_MISSION_ACK, MAV_RESULT.MAV_RESULT_DENIED);//gcs에 ACk전송
                } else {
                    wpMissionManager.getmMissionItemList().add(msg_item);//아이템을 리스트에 담음
                    if (msg_item.seq == mNumGCSWaypoints - 1) {// seq값이 (count-1) 과 같다면 == 마지막 아이템이라면
                        wpState = WP_STATE_INACTIVE;
                        finalizeNewMission();
                    } else {//마지막 아이템이 아니라면
                        Log.d(TAG, "Mission REQ: " + (msg_item.seq + 1));// 받은 아이템의 seq+1 번째 아이템을 요청해야
                        mModel.request_mission_item((msg_item.seq + 1));
                    }
                }


//            {
//                // Is this message to this system...
//                msg_mission_item msg_item = (msg_mission_item) msg;
//                if (mModel.getSystemId() != msg_item.target_system) {
//                    break;
//                }
//                Log.d(TAG, "Add mission: " + msg_item.seq);
//
//                // Somehow the GOTO from QGroundControl does not issue a mission count...
//                if (mMissionItemList == null && msg_item.command == MAV_CMD_NAV_WAYPOINT) {
//                    Log.d(TAG, "Goto... ");
//                    mModel.goto_position(msg_item.x, msg_item.y, msg_item.z, 0);
//                    mModel.send_command_ack(MAVLINK_MSG_ID_MISSION_ACK, MAV_RESULT.MAV_RESULT_ACCEPTED);
//
//                } else {
//                    if (mMissionItemList == null) {
//                        Log.d(TAG, "Error Sequence error!");
//                        mModel.send_command_ack(MAVLINK_MSG_ID_MISSION_ACK, MAV_RESULT.MAV_RESULT_DENIED);
//                    }
//                    else {
//                        msg_mission_item_int msg_item_f = new msg_mission_item_int();
//
//                        msg_item_f.param1=msg_item.param1;
//                        msg_item_f.param2=msg_item.param2;
//                        msg_item_f.param3=msg_item.param3;
//                        msg_item_f.param4=msg_item.param4;
//                        msg_item_f.x=(int)(msg_item.x*10000000.0);
//                        msg_item_f.y=(int)(msg_item.y*10000000.0);
//                        msg_item_f.z=msg_item.z;
//                        msg_item_f.seq=msg_item.seq;
//                        msg_item_f.command=msg_item.command;
//                        msg_item_f.target_system=msg_item.target_system;
//                        msg_item_f.target_component=msg_item.target_component;
//                        msg_item_f.frame=msg_item.frame;
//                        msg_item_f.current=msg_item.current;
//                        msg_item_f.autocontinue=msg_item.autocontinue;
//                        msg_item_f.mission_type=msg_item.mission_type;
//
//                        mMissionItemList.add(msg_item_f);
//                        // We are done fetching a complete mission from the GCS...
//                        if (msg_item.seq == mNumGCSWaypoints - 1) {
//                            wpState = WP_STATE_INACTIVE;
//                            finalizeNewMission();
//                        } else {
//                            Log.d(TAG, "Mission REQ: " +msg_item.seq + 1);
//                            mModel.request_mission_item((msg_item.seq + 1));
//                        }
//                    }
//                }
//            }
//            break;


                /**************************************************************
                 * These messages from GCS direct a mission-related action    *
                 **************************************************************/

            case MAVLINK_MSG_ID_MISSION_SET_CURRENT:
                Log.d(TAG, "MSN: received set_current from GCS");
                // TODO::::::
                break;

            // Clear all mission states...
            case MAVLINK_MSG_ID_MISSION_CLEAR_ALL:
                Log.d(TAG, "MSN: received clear_all from GCS");


//                WaypointMission ym = mModel.getWaypointMissionOperator().getLoadedMission();
//                if (ym != null) {
//                    ym.getWaypointList().clear();
//                }
                break;
        }
    }

    // Support function...
    public long getTimestampLastGCSHeartbeat() {
        return mTimeStampLastGCSHeartbeat;
    }

    //
//    private void changeFlightMode(int flightMode) {
//        mModel.setGCSCommandedMode(flightMode);
//
////        if (flightMode == ArduCopterFlightModes.AUTO) {
////            if (mModel.m_activeWaypointMission != null && mModel.pauseWaypointMission) {
////                Log.d(TAG, "Resuming mission");
////                mModel.resumeWaypointMission();
////            } else if(mModel.m_activeWaypointMission != null) {
////                mModel.startWaypointMission();
////            }
//        if (flightMode == ArduCopterFlightModes.AUTO) {
//            if (mModel.getWaypointMissionOperator().getCurrentState() == WaypointMissionState.EXECUTION_PAUSED) {
//                Log.d(TAG, "Resuming mission");
//                mModel.resumeWaypointMission();
//            } else if (mModel.getWaypointMissionOperator().getCurrentState() == WaypointMissionState.READY_TO_EXECUTE)
//                mModel.startWaypointMission();
//        } else if (flightMode == ArduCopterFlightModes.BRAKE) {
//            mModel.pauseWaypointMission();
//            mModel.setGCSCommandedMode(flightMode);
//        } else if (flightMode == ArduCopterFlightModes.RTL)
//            mModel.do_go_home();
//        else if (flightMode == ArduCopterFlightModes.LAND)
//            mModel.do_land();
//
//        mModel.send_command_ack(MAV_CMD_DO_SET_MODE, MAV_RESULT.MAV_RESULT_ACCEPTED);
//
//    }
//
//    // Generate a new Mission element, with default speed...


    public DroneModel getmModel() {
        return this.mModel;
    }

    //
    protected void finalizeNewMission() {
        ArrayList<WaylineWaypoint> dji_wps = new ArrayList<WaylineWaypoint>();

        wpMissionManager.generateWLIMlist();//받은 웨이포인트미션 아이템 메시지를 바탕으로 웨이포인트 인포 모델을 만들고 그것들을 리스트에 담음
        wpMissionManager.saveKMZfile();//만들어진 웨이포인트 인포 모델을 활용하여 kmz파일을 생성
        kmzOutPath=wpMissionManager.getKmzOutPath();//만들어 둔
        mModel.uploadKMZfile(mModel,kmzOutPath);
        mModel.send_mission_ack(MAV_MISSION_ACCEPTED);


//
    }

    public void setWpMissionManager(WpMissionManager wpMissionManager) {
        this.wpMissionManager = wpMissionManager;
    }

//
//    private void logWaypointstoRD(ArrayList<Waypoint> wps) {
//        Log.d(TAG, "==============================");
//        Log.d(TAG, "Waypoints with intermediate wps");
//        Log.d(TAG, "==============================");
//        for (Waypoint wp : wps)
//            Log.d(TAG, "WP: "+wp.coordinate.getLatitude() + ", " + wp.coordinate.getLongitude() + ", " + wp.altitude + ", " + wp.heading);
//    }
//
//    private ArrayList<Waypoint> addSurveyWaypoints(ArrayList<Waypoint> wpIn, float triggerDistance) {
//        ArrayList<Waypoint> wpOut = new ArrayList<>();
//        float distanceRemainder = 0;
//
//        Waypoint previousWaypoint = wpIn.get(0);
//
//        for (Waypoint currentWaypoint : wpIn) {
//            if (currentWaypoint == wpIn.get(0)) {
//                currentWaypoint.addAction(new WaypointAction(WaypointActionType.STAY, 1));
//                currentWaypoint.addAction(new WaypointAction(WaypointActionType.START_TAKE_PHOTO, 0));
//                wpOut.add(currentWaypoint);
//                continue;
//            }
//            float distanceBetweenPoints = (float) getRangeBetweenWaypoints_m(currentWaypoint, previousWaypoint);
//            float currentDistance;
//
//            if (distanceRemainder != 0) {
//                currentDistance = distanceRemainder;
//            } else {
//                currentDistance = triggerDistance;
//            }
//
//            float prevDistance = currentDistance;
//            int numSurveyWaypoints = 0;
//
//            while (prevDistance < distanceBetweenPoints) {
//                prevDistance += triggerDistance;
//                numSurveyWaypoints++;
//            }
//
//            for (int i = 1; currentDistance < distanceBetweenPoints; i++) {
//                Log.d(TAG, String.valueOf("WAYPOINT ADDED AT " + currentDistance));
//
//                Waypoint surveyWaypoint = createIntermediateWaypoint(previousWaypoint, currentWaypoint, i, numSurveyWaypoints, currentWaypoint.altitude, 0);
//                surveyWaypoint.addAction(new WaypointAction(WaypointActionType.STAY, 1));
//                surveyWaypoint.addAction(new WaypointAction(WaypointActionType.START_TAKE_PHOTO, 0));
//                wpOut.add(surveyWaypoint);
//
//                currentDistance += triggerDistance;
//            }
//
//            distanceRemainder = (currentDistance - distanceBetweenPoints);
//            currentWaypoint.addAction(new WaypointAction(WaypointActionType.STAY, 1));
//            currentWaypoint.addAction(new WaypointAction(WaypointActionType.START_TAKE_PHOTO, 0));
//            wpOut.add(currentWaypoint);
//            previousWaypoint = currentWaypoint;
//        }
//        return wpOut;
//    }
//
//    private ArrayList<Waypoint> addIntermediateWaypoints(ArrayList<Waypoint> wpIn) {
//
//        // No need for intermediate waypoints if only 0, if 1 then we MUST as a waypoint (min 2 waypoints in DJI)
//        if (wpIn.size() < 1)
//            return wpIn;
//
//        ArrayList<Waypoint> wpOut = new ArrayList<>();
//
//        // If this is a singlepoint goto... we must add at least one more waypoint (minimum 2 on DJI)
//        // For now return error...
//        if (wpIn.size() == 1) {
//            Log.d(TAG, "Single point WP error ");
//            return null;
//        }
//
//        Waypoint wpPrevious = wpIn.get(0);
//        boolean shouldNotify = false;
//
//        // If the distance between waypoints are larget than MAX_WAYPOINT_DISTANCE...
//        for (Waypoint wpCurrent : wpIn) {
//            // Add the first one...  TODO:: Check the distance...
//            if (wpCurrent == wpIn.get(0)) {
//                wpOut.add(wpCurrent);
//                continue;
//            }
//
//            Log.d(TAG, "WP dist x: " + String.valueOf(getRangeBetweenWaypoints_m(wpCurrent, wpPrevious)));
//            // 중간 wp 를 자동 생성하는 것임.
//            if (getRangeBetweenWaypoints_m(wpCurrent, wpPrevious) > MAX_WAYPOINT_DISTANCE) {
//                int numIntermediateWps = (int) getRangeBetweenWaypoints_m(wpCurrent, wpPrevious) / MAX_WAYPOINT_DISTANCE;
//                shouldNotify = true;
//                Log.d(TAG, "creating " + numIntermediateWps + " intermediate wps");
//                float waypointIncrement = (wpCurrent.altitude - wpPrevious.altitude) / (numIntermediateWps + 1);
//                Log.d(TAG, "WAYPOINT INCREMENT + " + waypointIncrement);
//                float prevIntermediaryAltitude = 0;
//                for (int i = 0; i < numIntermediateWps; i++) {
//                    Waypoint intermediateWaypoint = createIntermediateWaypoint(wpPrevious, wpCurrent, i + 1, numIntermediateWps, prevIntermediaryAltitude, waypointIncrement);
//                    prevIntermediaryAltitude = intermediateWaypoint.altitude;
//                    wpOut.add(intermediateWaypoint);
//                }
//            }
//            wpOut.add(wpCurrent);
//            wpPrevious = wpCurrent;
//        }
//
//        if (shouldNotify) {
//            parent.runOnUiThread(() -> NotificationHandler.notifyAlert(parent, TYPE_WAYPOINT_DISTANCE,
//                    null, null));
//        }
//
//        return wpOut;
//    }
//
//    private Waypoint createIntermediateWaypoint(Waypoint wp1, Waypoint wp2, int intermediateWpNum, int numIntermediateWaypoints, float prevIntermediaryAltitude, float waypointIncrement) {
//        // If we need to add one intermediate waypoint, it is (1/2) between wp1 and wp2
//        // If we need to add two intermediate waypoints, they are (1/3) and (2/3) between wp1 and wp2
//        // etc.
//        double new_lat = wp1.coordinate.getLatitude() + (wp2.coordinate.getLatitude() - wp1.coordinate.getLatitude()) * intermediateWpNum / (numIntermediateWaypoints + 1);
//        double new_lon = wp1.coordinate.getLongitude() + (wp2.coordinate.getLongitude() - wp1.coordinate.getLongitude()) * intermediateWpNum / (numIntermediateWaypoints + 1);
//        float new_alt;
//        if (prevIntermediaryAltitude == 0) {
//            new_alt = wp1.altitude + waypointIncrement;
//        } else {
//            new_alt = prevIntermediaryAltitude + waypointIncrement;
//        }
////        float new_alt = Math.max(wp1.altitude, wp2.altitude);
//
//        return new Waypoint(new_lat, new_lon, new_alt);
//    }
//
//    // Based on: https://stackoverflow.com/questions/3694380/calculating-distance-between-two-points-using-latitude-longitude-what-am-i-doi
//    private double getRangeBetweenWaypoints_m(Waypoint wp1, Waypoint wp2) {
//
//        double lat1 = wp1.coordinate.getLatitude();
//        double lon1 = wp1.coordinate.getLongitude();
//        float el1 = 0;  //wp1.altitude;
//        double lon2 = wp2.coordinate.getLongitude();
//        double lat2 = wp2.coordinate.getLatitude();
//        float el2 =  0; //wp2.altitude;
//
//        return mModel.getRangeBetweenWaypoints_m(lat1,lon1,el1,lat2,lon2,el2);
//    }
}
