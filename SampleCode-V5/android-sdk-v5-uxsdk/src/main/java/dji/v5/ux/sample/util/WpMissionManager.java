package dji.v5.ux.sample.util;

import android.util.Log;

import com.dji.wpmzsdk.common.data.Template;
import com.dji.wpmzsdk.manager.WPMZManager;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

import dji.sdk.wpmz.value.mission.ActionAircraftHoverParam;
import dji.sdk.wpmz.value.mission.ActionAircraftRotateYawParam;
import dji.sdk.wpmz.value.mission.ActionTakePhotoParam;
import dji.sdk.wpmz.value.mission.WaylineActionInfo;
import dji.sdk.wpmz.value.mission.WaylineActionType;
import dji.sdk.wpmz.value.mission.WaylineLocationCoordinate2D;
import dji.sdk.wpmz.value.mission.WaylineMission;
import dji.sdk.wpmz.value.mission.WaylineMissionConfig;
import dji.sdk.wpmz.value.mission.WaylineWaypoint;
import dji.sdk.wpmz.value.mission.WaylineWaypointYawPathMode;
import dji.v5.common.callback.CommonCallbacks;
import dji.v5.manager.aircraft.waypoint3.WaypointMissionManager;
import dji.v5.utils.common.ContextUtil;
import dji.v5.utils.common.DiskUtil;
import dji.v5.ux.MAVLink.common.msg_mission_item;
import dji.v5.ux.MAVLink.common.msg_mission_item_int;
import dji.v5.ux.MAVLink.enums.MAV_CMD;
import dji.v5.ux.sample.showcase.defaultlayout.DefaultLayoutActivity;
import dji.v5.ux.utils.KMZTestUtil;
import dji.v5.ux.utils.wpml.WaypointInfoModel;
import io.reactivex.disposables.Disposable;
import kotlin.jvm.internal.markers.KMutableList;


//미션 매니저 따로 생성하여 미션 목록 관리를 맡겼습니다.
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

    String kmzOutPath = rootDir + "generate_test_ddm.kmz";
    DroneModel model;
    int status = 0;
    DefaultLayoutActivity mainActivity;
    Disposable mDisposable;

    public WpMissionManager(MAVLinkReceiver MAVLinkReceiver, DroneModel model, DefaultLayoutActivity defaultLayoutActivity) {
        this.receiver = MAVLinkReceiver;
        this.model = model;
        this.mainActivity = defaultLayoutActivity;
    }

    public void setReceiver(MAVLinkReceiver receiver) {
        this.receiver = receiver;
    }

    public MAVLinkReceiver getReceiver() {
        return receiver;
    }

    public ArrayList<WaypointInfoModel> initmWLIMList() {
        if (mWLIMList != null) {
            this.mWLIMList = null;
        }
        this.mWLIMList = new ArrayList<>();
        return mWLIMList;
    }

    public ArrayList<msg_mission_item> iniMissionItemList() {
        if (mMissionItemList != null) {
            this.mMissionItemList = null;
        }
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

    //clear는 임무 종료시점에  호출하여 지워야 함, 시점은 회의를 통해 특정해야
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

    public void setKmzOutPath(String kmzOutPath) {
        this.kmzOutPath = kmzOutPath;
    }

    public String getKmzOutPath() {
        return kmzOutPath;
    }

    protected void generateWLIMlist() {
        // Initialize lists to store waypoints and actions
        List<WaylineWaypoint> waypointList = new ArrayList<>();
        List<WaylineActionInfo> actionInfos = new ArrayList<>(this.getmMissionItemList().size());
        List<List<WaylineActionInfo>> actionInfosList = new ArrayList<>(this.getmMissionItemList().size());

        boolean speedChanged = false;
        double defaultSpeed = 3.0;
        double changedSpeed = 0.0;

        // Loop through the mission items
        for (int i = 0; i < this.getmMissionItemList().size(); i++) {
            Log.d(TAG, "\n===================================================" + i + "번째 아이템 " + "=======================================================");
            msg_mission_item msg = this.getmMissionItemList().get(i);

            switch (msg.command) {
                case MAV_CMD.MAV_CMD_NAV_WAYPOINT:
<<<<<<< HEAD
                    if (msg.param1 > 0 && (msg.x != 0 || msg.y != 0 || msg.z != 0)) {
=======
                     if (msg.param1 > 0 && (msg.x != 0 && msg.y != 0 && msg.z !=0)) {
>>>>>>> 0ef43b67cf7a148e9597035d4b6fb150e91a05b4
                        Log.d(TAG, "Delay: " + msg.param1);
                        WaylineActionInfo hoverAction = createHoverAction(msg.param1);
                        actionInfos.add(hoverAction);
                        actionInfosList.add(actionInfos);

                        WaylineWaypoint waypoint = createWaypoint(msg, speedChanged ? changedSpeed : defaultSpeed);
                        addWaypointToList(waypoint, actionInfos);
                    } else if (msg.z != 0) {
                        Log.d(TAG, "Waypoint LAT: " + msg.x + ", LAN: " + msg.y + ", ALT: " + msg.z);
                        WaylineWaypoint waypoint = createWaypoint(msg, speedChanged ? changedSpeed : defaultSpeed);
                        List<WaylineActionInfo> nullactionInfos = new ArrayList<>(this.getmMissionItemList().size());
                        addWaypointToList(waypoint, nullactionInfos);
                        actionInfosList.add(nullactionInfos);
                    }
                    break;

                case MAV_CMD.MAV_CMD_CONDITION_YAW:
                    Log.d(TAG, "Condition Yaw: " + msg.param1);
                    WaylineActionInfo yawAction = createYawAction(msg.param1);
                    List<WaylineActionInfo> latestActions = actionInfosList.get(mWLIMList.size() - 1);
                    latestActions.add(yawAction);
                    updateWaypointActions(latestActions);
                    break;

                case MAV_CMD.MAV_CMD_DO_CHANGE_SPEED:
                    Log.d(TAG, "Change Speed: " + msg.param2);
                    changedSpeed = msg.param2;
                    speedChanged = true;
                    break;

                case MAV_CMD.MAV_CMD_NAV_TAKEOFF:
                    Log.d(TAG, "Takeoff...");
                    break;

                case MAV_CMD.MAV_CMD_DO_MOUNT_CONTROL:
                    Log.d(TAG, "Set gimbal pitch: " + msg.param1);
                    break;

                case MAV_CMD.MAV_CMD_IMAGE_STOP_CAPTURE:
                    Log.d(TAG, "MAV_CMD_IMAGE_STOP_CAPTURE");
                    break;

                case MAV_CMD.MAV_CMD_IMAGE_START_CAPTURE:
                    Log.d(TAG, "MAV_CMD_IMAGE_START_CAPTURE");
                    break;

                case MAV_CMD.MAV_CMD_DO_SET_CAM_TRIGG_DIST:
                    Log.d(TAG, "MAV_CMD_DO_SET_CAM_TRIGG_DIST");
                    break;

                case MAV_CMD.MAV_CMD_NAV_RETURN_TO_LAUNCH:
                    Log.d(TAG, "MAV_CMD_NAV_RETURN_TO_LAUNCH");
                    break;

                case MAV_CMD.MAV_CMD_NAV_DELAY:
                    Log.d(TAG, "MAV_CMD_NAV_DELAY");
                    break;

                case MAV_CMD.MAV_CMD_VIDEO_START_CAPTURE:
                    Log.d(TAG, "MAV_CMD_VIDEO_START_CAPTURE");
                    break;

                case MAV_CMD.MAV_CMD_VIDEO_STOP_CAPTURE:
                    Log.d(TAG, "MAV_CMD_VIDEO_STOP_CAPTURE");
                    break;

                case MAV_CMD.MAV_CMD_DO_DIGICAM_CONTROL:
                    Log.d(TAG, "MAV_CMD_DO_DIGICAM_CONTROL");
                    break;

                case MAV_CMD.MAV_CMD_SET_CAMERA_ZOOM:
                    Log.d(TAG, "MAV_CMD_SET_CAMERA_ZOOM");
                    break;

                case MAV_CMD.MAV_CMD_SET_CAMERA_FOCUS:
                    Log.d(TAG, "MAV_CMD_SET_CAMERA_FOCUS");
                    break;
            }

            Log.d(TAG, "===================================================" + i + "번째 아이템 " + "=======================================================\n");
        }

        this.saveKMZfile();
    }

    // Helper method to create hover action
    private WaylineActionInfo createHoverAction(double hoverTime) {
        WaylineActionInfo info = new WaylineActionInfo();
        info.setActionType(WaylineActionType.HOVER);
        ActionAircraftHoverParam param = new ActionAircraftHoverParam();
        param.setHoverTime(hoverTime);
        info.setAircraftHoverParam(param);
        Log.d(TAG, "Hovering Action Created");
        return info;
    }

    // Helper method to create waypoint
    private WaylineWaypoint createWaypoint(msg_mission_item msg, double speed) {
        WaylineWaypoint waypoint = new WaylineWaypoint();
        WaylineLocationCoordinate2D location = new WaylineLocationCoordinate2D((double) msg.x, (double) msg.y);
        waypoint.setLocation(location);
        waypoint.setHeight((double) msg.z);
        waypoint.setEllipsoidHeight((double) msg.z);
        waypoint.setSpeed(speed);
        waypoint.setUseGlobalTurnParam(true);
        waypoint.setWaypointIndex(this.mWLIMList.size());
        return waypoint;
    }

    // Helper method to add waypoint to list
    private void addWaypointToList(WaylineWaypoint waypoint, List<WaylineActionInfo> actionInfos) {
        WaypointInfoModel wpInfoModel = new WaypointInfoModel();
        wpInfoModel.setWaylineWaypoint(waypoint);
        wpInfoModel.setActionInfos(actionInfos);
        mWLIMList.add(wpInfoModel);
        Log.d(TAG, mWLIMList.size() - 1 + "번째 웨이포인트에 설정된 액션 : " + actionInfos.toString());
    }

    // Helper method to create yaw action
    private WaylineActionInfo createYawAction(double heading) {
        WaylineActionInfo info = new WaylineActionInfo();
        info.setActionType(WaylineActionType.ROTATE_YAW);
        ActionAircraftRotateYawParam param = new ActionAircraftRotateYawParam();
        param.setHeading(heading);
        param.setPathMode(WaylineWaypointYawPathMode.CLOCKWISE);
        info.setAircraftRotateYawParam(param);
        Log.d(TAG, "Condition Yaw Action Created");
        return info;
    }

    // Helper method to update actions of the latest waypoint
    private void updateWaypointActions(List<WaylineActionInfo> wactionInfos) {
        mWLIMList.get(mWLIMList.size() - 1).setActionInfos(wactionInfos);
        Log.d(TAG, mWLIMList.size() - 1 + "번째 웨이포인트에 설정된 액션 : " + wactionInfos.toString());
    }

    public void saveKMZfile() {
        WPMZManager manager = WPMZManager.getInstance();
        WaylineMission wlm = KMZTestUtil.createWaylineMission();
        WaylineMissionConfig wlmc = KMZTestUtil.createMissionConfig();
        Template template = KMZTestUtil.createTemplate(this.mWLIMList);


        File file = new File(kmzOutPath);
        if (file.exists()) {//기존 경로에 파일 있으면 삭제
            Log.i(TAG, "kmzFile exists, so Deleted it: " + rootDir);
            file.delete();
        }
        manager.generateKMZFile(kmzOutPath, wlm, wlmc, template);
        this.mWLIMList.clear();

    }


    public int uploadKMZfile(CommonCallbacks.CompletionCallbackWithProgress<Double> callback) {


        WaypointMissionManager.getInstance().pushKMZFileToAircraft(kmzOutPath, callback);
        return status;

    }


}

