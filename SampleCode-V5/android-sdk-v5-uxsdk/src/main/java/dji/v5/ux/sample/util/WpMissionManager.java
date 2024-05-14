package dji.v5.ux.sample.util;

import android.util.Log;

import androidx.annotation.NonNull;

import com.dji.wpmzsdk.common.data.Template;
import com.dji.wpmzsdk.manager.WPMZManager;

import java.util.ArrayList;


import dji.sdk.keyvalue.value.mission.WaypointMission;
import dji.sdk.wpmz.value.mission.WaylineActionInfo;
import dji.sdk.wpmz.value.mission.WaylineLocationCoordinate2D;
import dji.sdk.wpmz.value.mission.WaylineMission;
import dji.sdk.wpmz.value.mission.WaylineMissionConfig;
import dji.sdk.wpmz.value.mission.WaylineWaypoint;
import dji.v5.common.callback.CommonCallbacks;
import dji.v5.common.error.IDJIError;
import dji.v5.manager.aircraft.waypoint3.WaypointMissionManager;
import dji.v5.utils.common.ContextUtil;
import dji.v5.utils.common.DiskUtil;
import dji.v5.ux.MAVLink.common.msg_mission_item;
import dji.v5.ux.MAVLink.common.msg_mission_item_int;
import dji.v5.ux.MAVLink.enums.MAV_RESULT;
import dji.v5.ux.utils.KMZTestUtil;
import dji.v5.ux.utils.wpml.WaypointInfoModel;

public class WpMissionManager {
    private final String TAG = this.getClass().getSimpleName();
    private MAVLinkReceiver receiver;
    private ArrayList<msg_mission_item> mMissionItemList;
    private ArrayList<msg_mission_item_int> mMissionItemintList;
    private ArrayList<WaypointInfoModel> mWLIMList;
    String WAYPOINT_SAMPLE_FILE_NAME = "waypointsample.kmz";
    String WAYPOINT_SAMPLE_FILE_DIR = "waypoint/";
    String WAYPOINT_SAMPLE_FILE_CACHE_DIR = "waypoint/cache/";
    String WAYPOINT_FILE_TAG = ".kmz";
    String unzipChildDir = "temp/";
    String unzipDir = "wpmz/";
    String curMissionPath = DiskUtil.getExternalCacheDirPath(ContextUtil.getContext(), WAYPOINT_SAMPLE_FILE_DIR + WAYPOINT_SAMPLE_FILE_NAME);
    String rootDir = DiskUtil.getExternalCacheDirPath(ContextUtil.getContext(), WAYPOINT_SAMPLE_FILE_DIR);
    String kmzOutPath = rootDir + "generate_test.kmz";
    DroneModel model;
    int status = 0;

    WpMissionManager(MAVLinkReceiver MAVLinkReceiver, DroneModel model) {
        this.receiver = MAVLinkReceiver;
        this.model = model;
    }


    public ArrayList<WaypointInfoModel> initmWLIMList() {
        this.mWLIMList = new ArrayList<>();
        return mWLIMList;
    }

    public ArrayList<msg_mission_item> iniMissionItemListt() {
        this.mMissionItemList = new ArrayList<>();
        return mMissionItemList;
    }

    public void setmMissionItemintList(ArrayList<msg_mission_item_int> mMissionItemintList) {
        this.mMissionItemintList = mMissionItemintList;
    }

    public void setmMissionItemList(ArrayList<msg_mission_item> mMissionItemList) {
        this.mMissionItemList = mMissionItemList;
    }

    public void setmWLIMList(ArrayList<WaypointInfoModel> mWLIMList) {
        this.mWLIMList = mWLIMList;
    }

    public ArrayList<msg_mission_item> getmMissionItemList() {
        return mMissionItemList;
    }

    public ArrayList<WaypointInfoModel> getmWLIMList() {
        return mWLIMList;
    }

    public ArrayList<msg_mission_item_int> getmMissionItemintList() {
        return mMissionItemintList;
    }


    public ArrayList<msg_mission_item> clearmMissionItemList() {
        mMissionItemList.clear();
        return mMissionItemList;
    }

    public ArrayList<WaypointInfoModel> clearmWLIMList() {
        mWLIMList.clear();
        return mWLIMList;
    }

    public ArrayList<msg_mission_item_int> clearmMissionItemintList() {
        mMissionItemintList.clear();
        return mMissionItemintList;
    }


    protected ArrayList<WaypointInfoModel> generateWLIMlist() {


        for (int i = 0; i < this.getmMissionItemList().size(); i++) {
            msg_mission_item msg = this.getmMissionItemList().get(i);//마브링크 미션 아이템 목록에서 DJI미션으로 변환할 메시지 꺼냄

            Log.i(TAG, msg.toString());

            WaypointInfoModel wpInfomodel = new WaypointInfoModel();//웨이포인트 인포 모델 객체 초기화
            WaylineWaypoint waypoint = new WaylineWaypoint();//웨이포인트 객체 초기화, 여기에 아이템의값들을 넣어줄것
            waypoint.setWaypointIndex(i);//첫 웨이포인트 미션 생성할거니까
            WaylineLocationCoordinate2D location = new WaylineLocationCoordinate2D((double) msg.x, (double) msg.y);//메시지 바탕으로 위치값 생성
            waypoint.setLocation(location);// 생성한 위치값을 웨이포인트의 위도와 경도로 설정
            waypoint.setHeight((double) msg.z);//웨이포인트의 고도 설정
            waypoint.setEllipsoidHeight((double) msg.z);//웨이포인트의 Ellipsoid고도 설정(이 항목이 뭔지 모르나 샘플에서 그냥 고도값을 넣은것을 보고 일단 넣어둠)
            waypoint.setSpeed((double) msg.param2);//웨이포인트 속도 설정(이거는 메시지에서 어떤 파라미터를 가져와야 하는지 모르겠음)
            waypoint.setUseGlobalTurnParam(true);
            ArrayList<WaylineActionInfo> actionlist = new ArrayList<>();
            wpInfomodel.setWaylineWaypoint(waypoint);
            wpInfomodel.setActionInfos(actionlist);
            this.mWLIMList.add(i, wpInfomodel);

        }


        return this.mWLIMList;

//        Log.d(TAG, "==============================");
//        Log.d(TAG, "Waypoint Mission Uploading");
//        Log.d(TAG, "==============================");
//
//        boolean stopUpload = false;
//        int errorCode = 0;
//
//        boolean triggerDistanceEnabled = false;
//        float triggerDistance = 0;
//
//        if(this.mMissionItemList.size() > 0) {
////            if(m.seq == 0) {
////                // DROW System에서는, Seq 0은 홈포인트 위경고도, Seq1은 APM/PX4를 호환하기위해 자동 takeoff 를 apm에는 넣고, px4는 넣지 않고 있다.
////                // DJI드론은 DDM APM이라고 인식하고 운용 될 것으로 seq 0과 1의 위경도 차이는 0.5 close이상 두면서 500을 넘지 않고,
////                // 고도를 0.5차이 나도록 한다.
////                m.z = 0.0f;
////                Log.d(TAG, "msg__item_int First......" + String.valueOf(m));
////                // 위 주석의 이유의  대안으로 index 0을 제외한다. 홈포인트 고도조절 px4/apm과 apm의 takeoff 명령을 위해..
////                continue;
////            }
//            this.mMissionItemList.remove(0);
//        }
//        waypoint_loop:
//
//        for (msg_mission_item_int m : this.mMissionItemList) {
//            Log.d(TAG, "Command: " +String.valueOf(m));
//
//            switch (m.command) {
//
//                case MAV_CMD.MAV_CMD_NAV_TAKEOFF:
//                    Log.d(TAG, "Takeoff...");
//
//                    // if we got an item (Start item) already we got a position, now we just add altitude.
//                    if(currentWP != null){
//                        currentWP.altitude = m.z;
//                    }else{
//                        if(m.x == 0 || m.y == 0) {
//                            currentWP = new Waypoint(mModel.get_current_lat(), mModel.get_current_lon(), m.z);
//                        }else {
////                            currentWP = new Waypoint(m.x,m.z, m.z);
//                            currentWP = new Waypoint(m.x/10000000.0, m.y/10000000.0, m.z);
//                        }
//                    }
//                    dji_wps.add(currentWP);
//                    Log.d(TAG, "DJI TAKEOFF :: " + currentWP.coordinate.getLatitude() + "/" + currentWP.coordinate.getLongitude() );
//                    break;
//
//                case MAV_CMD.MAV_CMD_NAV_WAYPOINT:
//                    Log.d(TAG, "Waypoint: " + m.x/10000000.0 + ", " + m.y/10000000.0 + " at " + m.z + " m " + m.param2 + " Yaw " + m.param1 + " Delay ");
////                    WaypointMission.Builder waypointMissionBuilder = new WaypointMission.Builder().autoFlightSpeed(5f)
////                            .maxFlightSpeed(10f)
////                            .setExitMissionOnRCSignalLostEnabled(false)
////                            .finishedAction(
////                                    WaypointMissionFinishedAction.NO_ACTION)
////                            .flightPathMode(
////                                    WaypointMissionFlightPathMode.NORMAL)
////                            .gotoFirstWaypointMode(
////                                    WaypointMissionGotoWaypointMode.SAFELY)
////                            .headingMode(
////                                    WaypointMissionHeadingMode.AUTO)
////                            .repeatTimes(1);;
//                    // If this is a start item let's store the position ...
//                    if(m.frame != 3 || Float.isNaN(m.z) ) {
//                        if(DroneModel.isDev)
//                            currentWP = new Waypoint((DroneModel.DEFUALT_LATITUDE + (mModel.getSystemId()*0.001) ), (DroneModel.DEFUALT_LONGITUDE + (mModel.getSystemId()*0.001) ), 0);
//                        else
//                            currentWP = new Waypoint(mModel.get_current_lat(), mModel.get_current_lon(), 0);
//                        break;
//                    }
//
//                    if ((m.z) > 500) {  // TODO:  Shuld reqest max altitude not assume 500...
//                        parent.runOnUiThread(() -> NotificationHandler.notifyAlert(parent, TYPE_WAYPOINT_MAX_ALTITUDE, null, null));
//                        stopUpload = true;
//                        mModel.send_command_ack(MAVLINK_MSG_ID_MISSION_ACK, MAV_RESULT.MAV_RESULT_DENIED);
//                        break waypoint_loop;
//
//                    } else if ((m.z) < -200) {  // TODO:  hmm so we can not take off from a mountain and fly down?? ...
//                        parent.runOnUiThread(() -> NotificationHandler.notifyAlert(parent, TYPE_WAYPOINT_MIN_ALTITUDE, null, null));
//                        stopUpload = true;
//                        mModel.send_command_ack(MAVLINK_MSG_ID_MISSION_ACK, MAV_RESULT.MAV_RESULT_DENIED);
//                        break waypoint_loop;
//                    }
//
//                    currentWP = new Waypoint(m.x/10000000.0, m.y/10000000.0, m.z); // TODO check altitude conversion
//
//                    // If delay at wp...
//                    if (m.param1 > 0)
//                        currentWP.addAction(new WaypointAction(WaypointActionType.STAY, (int) m.param1 * 1000)); // milliseconds...
//
//                    // If rotate at wp...
//                    if (m.param2 > 0)
//                        currentWP.addAction(new WaypointAction(WaypointActionType.ROTATE_AIRCRAFT, (int) (m.param2 * 180.0 / 3.141592))); // +-180 deg...
//
//                    // This is set in the RosettaDrone2 settings...
//                    if (curvedFlightPath) {
//                        currentWP.cornerRadiusInMeters = flightPathRadius;
//                    }
//
//                    // If we use trigger distance, for now just add a stay & photo action
//                    // Ideally feed them pre-processed for now
//                    if(triggerDistanceEnabled)
//                    {
//                        currentWP.addAction(new WaypointAction(WaypointActionType.STAY, 0));
//                        currentWP.addAction(new WaypointAction(WaypointActionType.START_TAKE_PHOTO, 0));
//                    }
//
//                    dji_wps.add(currentWP);
//                    Log.d(TAG, "DJI WP :: " + currentWP.coordinate.getLatitude() + "/" + currentWP.coordinate.getLongitude() );
//                break;
//
//                case MAV_CMD.MAV_CMD_DO_CHANGE_SPEED:
//                    Log.d(TAG, "Change Speed: " + m.x/10000000.0 + ", " + m.y/10000000.0 + " at " + m.z + " m " + m.param2 + " Yaw " + m.param1 + " Delay ");
//
//                    if (m.param2 < -15) {
//                        parent.runOnUiThread(() -> NotificationHandler.notifyAlert(parent, TYPE_WAYPOINT_MIN_SPEED,
//                                null, null));
//                        stopUpload = true;
//                        break waypoint_loop;
//                    }
//                    else if (m.param2 > 15) {
//                        parent.runOnUiThread(() -> NotificationHandler.notifyAlert(parent, TYPE_WAYPOINT_MAX_SPEED,
//                                null, null));
//                        stopUpload = true;
//                        break waypoint_loop;
//                    }
//                    else {
//                        mBuilder.autoFlightSpeed(m.param2);
//                    }
//                    break;
//
//                case MAV_CMD.MAV_CMD_DO_MOUNT_CONTROL:
//                    Log.d(TAG, "Set gimbal pitch: " + m.param1);
//                    currentWP = new Waypoint(m.x/10000000.0, m.y/10000000.0, m.z); // TODO check altitude conversion
//                    currentWP.addAction(new WaypointAction(WaypointActionType.GIMBAL_PITCH, (int) m.param1));
//                    dji_wps.add(currentWP);
//                    break;
//                case MAV_CMD.MAV_CMD_IMAGE_STOP_CAPTURE:
//                    Log.d(TAG, "MAV_CMD_IMAGE_STOP_CAPTURE ");
////                    mModel.stopImageCaptureIntervalControl();
//                    break;
//                case MAV_CMD.MAV_CMD_IMAGE_START_CAPTURE:
//                    Log.d(TAG, "MAV_CMD_IMAGE_START_CAPTURE");
//                    // Cant add a new WP since wp.xyz are garbage & NaN
//                    // Add to last WP entry
//                    int lastIndex = dji_wps.indexOf(currentWP);
//                    currentWP.addAction(new WaypointAction(WaypointActionType.START_TAKE_PHOTO, 0));  // 한번 촬영하는듯
//                    /**
//                     * 기체가 현재 웨이포인트와 다음 웨이포인트 사이를 이동할 때 두 장의 사진이 촬영되는 시간 간격(초)입니다. 첫 번째 사진은 기체가 현재 웨이포인트를 떠날 때 촬영됩니다.
//                     * 최대값은 6,000.0입니다. 최소값은 0.0 이상이며 카메라 유형 및 카메라 매개변수에 따라 다릅니다. 사진 파일 형식이 JPEG인 경우 권장되는 최소값은 2.0입니다.
//                     * 사진 파일 형식이 RAW인 경우 최소값은 10.0입니다. 입력이 카메라의 용량을 초과하면 사진이 가능한 최대 속도로 촬영됩니다. 기본값은 0.0이며 사진이 촬영되지 않습니다.
//                     */
//
//                    // dji 임무웨이포인트자체에 설정하여 start image capture mavlink 시, dji wp마다 찍도록 설정해야함
//                    // 플래그로 세팅된 시점부터 MAV_CMD_NAV_WAYPOINT 시 해당 세팅을 넣도록 하면 될듯.
////                    currentWP.shootPhotoTimeInterval = m.param2;
//
////                    northPoint.addAction(new WaypointAction(WaypointActionType.GIMBAL_PITCH, -60));
////                    southPoint.addAction(new WaypointAction(WaypointActionType.ROTATE_AIRCRAFT, 60));
////                    (ShootPhotoAction.newShootIntervalPhotoAction(3,2)); MissionAction v2 waypoint..
//                    dji_wps.set(lastIndex, currentWP);
//                    mModel.setIntervalControl(true);
//                    mModel.setCaptureInterval(m.param2);
//                    break;
//
//                case MAV_CMD.MAV_CMD_DO_SET_CAM_TRIGG_DIST:
//                    Log.d(TAG, "MAV_CMD_DO_SET_CAM_TRIGG_DIST");
//
//                    if (!triggerDistanceEnabled) {
//                        if (m.param1 != 0) {
//                            triggerDistanceEnabled = true;
//                            triggerDistance = m.param1;
//                        } else {
//                            triggerDistance = 0;
//                            triggerDistanceEnabled = false;
//                        }
//                    }
//                    break;
//
//                case MAV_CMD.MAV_CMD_NAV_RETURN_TO_LAUNCH:
//                    Log.d(TAG, "MAV_CMD_NAV_RETURN_TO_LAUNCH");
//                    mBuilder.finishedAction(WaypointMissionFinishedAction.GO_HOME);
//                    break;
//
//                case MAV_CMD.MAV_CMD_NAV_DELAY:
//                    Log.d(TAG, "MAV_CMD_NAV_DELAY");
//                    break;
//
//                case MAV_CMD.MAV_CMD_VIDEO_START_CAPTURE:
//                    Log.d(TAG, "MAV_CMD_VIDEO_START_CAPTURE");
//                    break;
//
//                case MAV_CMD.MAV_CMD_VIDEO_STOP_CAPTURE:
//                    Log.d(TAG, "MAV_CMD_VIDEO_STOP_CAPTURE");
//                    break;
//
//                case MAV_CMD.MAV_CMD_CONDITION_YAW:
//                    Log.d(TAG, "MAV_CMD_CONDITION_YAW");
//                    break;
//
//                case MAV_CMD.MAV_CMD_DO_DIGICAM_CONTROL:
//                    Log.d(TAG, "MAV_CMD_DO_DIGICAM_CONTROL");
//                    break;
//
//                case MAV_CMD.MAV_CMD_SET_CAMERA_ZOOM:
//                    Log.d(TAG, "MAV_CMD_SET_CAMERA_ZOOM");
//                    break;
//
//                case MAV_CMD.MAV_CMD_SET_CAMERA_FOCUS:
//                    Log.d(TAG, "MAV_CMD_SET_CAMERA_FOCUS");
//                    break;
//                }
//        }
//
//        if (stopUpload) {
//            Log.d(TAG, "Waypoint upload aborted due to invalid parameters");
//        } else {
//            Log.d(TAG, "Speed for mission will be " + mBuilder.getAutoFlightSpeed() + " m/s");
//            Log.d(TAG, "==============================");
//
//            ArrayList<Waypoint> correctedWps;
//
//            /*if (triggerDistanceEnabled) {
//                Log.d(TAG, "ADDING SURVEY WAYPOINTS");
//                correctedWps = addSurveyWaypoints(dji_wps, triggerDistance);
//                mBuilder.flightPathMode(WaypointMissionFlightPathMode.NORMAL);
//            } else {
//                // If distances are to large, add inteermediet...
//                correctedWps = addIntermediateWaypoints(dji_wps);
//                if( correctedWps == null){
//                    Log.d(TAG, "Only 1 Waypoint error...");
//                    mModel.send_command_ack(MAVLINK_MSG_ID_MISSION_ACK, MAV_RESULT.MAV_RESULT_FAILED);
//                    return;
//                }
//            }*/
//            // mission item/int list를 parse해서 dji wp로 비슷하게 만들었으니,
//            // 거리검사(500m limit, 0.5m close) 후 builder에 size를 세팅하고
//            // sdk에 set한다..
//            logWaypointstoRD(dji_wps);
//            Log.d(TAG, "WP size " + dji_wps.size());
//            safeSleep(200);
//            mBuilder.waypointList(dji_wps).waypointCount(dji_wps.size());
//            safeSleep(200);
//            WaypointMission builtMission = mBuilder.build();
//            safeSleep(200);
//            mModel.setWaypointMission(builtMission);
//            safeSleep(200);
//        }
//
//        mMissionItemList=null;  // Flush the mission list...
//        isHome = true;
    }

    public void saveKMZfile() {
        WaylineMission wlm = KMZTestUtil.createWaylineMission();
        WaylineMissionConfig wlmc = KMZTestUtil.createMissionConfig();
        Template template = KMZTestUtil.createTemplate(this.mWLIMList);
        WPMZManager.getInstance().generateKMZFile(kmzOutPath, wlm, wlmc, template);
        Log.i(TAG, "kmzFile saved directory: " + rootDir);
    }

    public int uploadKMZfile(CommonCallbacks.CompletionCallbackWithProgress<Double> callback) {


        WaypointMissionManager.getInstance().pushKMZFileToAircraft(kmzOutPath, callback);
        return status;

    }


}

