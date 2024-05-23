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
        this.mWLIMList = new ArrayList<>();
        return mWLIMList;
    }

    public ArrayList<msg_mission_item> iniMissionItemList() {
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
        WaylineWaypoint waypoint = null; // 웨이포인트 객체 초기화
        WaylineActionInfo waitnginfo = new WaylineActionInfo();

        boolean triggerDistanceEnabled = false;
        int currentWPIMindex = 0; // WPIM 리스트에 추가될 인덱스
        boolean speedChanged = false;
        double defaultSpeed = 3.0;
        double changedSpeed = 0.0;
        boolean isInfoWaiting = false;

        for (int i = 0; i < this.getmMissionItemList().size(); i++) {
            Log.d(TAG, "==================================================="+i+"번째 아이템 " + "=======================================================");
            msg_mission_item msg = this.getmMissionItemList().get(i); // 마브링크 미션 아이템 목록에서 DJI미션으로 변환할 메시지 꺼냄
            waypoint = new WaylineWaypoint(); // 웨이포인트 객체 생성
            WaylineActionInfo info = new WaylineActionInfo(); // 액션인포 객체 생성
            List<WaylineActionInfo> actionInfos = new ArrayList<>(); // 여러 개의 actionInfo가 들어갈 리스트
            switch (msg.command) {
                case MAV_CMD.MAV_CMD_NAV_WAYPOINT:
                    Log.d(TAG, msg.toString());
                    if (msg.param1 > 0) { // 멈춤 시간이 1보다 크면
                        info.setActionType(WaylineActionType.HOVER);
                        ActionAircraftHoverParam param = new ActionAircraftHoverParam();
                        param.setHoverTime((double) msg.param1);
                        info.setAircraftHoverParam(param);
                        Log.d(TAG, "actionInfo: " + info.toString());
                        actionInfos.add(info);
                    }
                    break;
                case MAV_CMD.MAV_CMD_CONDITION_YAW:
                    Log.d(TAG, msg.toString());
                    if (msg.param1 > 0) { // Yaw 각도가 0보다 크면
                        info.setActionType(WaylineActionType.ROTATE_YAW);
                        ActionAircraftRotateYawParam param = new ActionAircraftRotateYawParam();
                        param.setHeading((double) msg.param1);
                        param.setPathMode(WaylineWaypointYawPathMode.CLOCKWISE);
                        info.setAircraftRotateYawParam(param);
                        Log.d(TAG, "actionInfo: " + info.toString());
                        isInfoWaiting =true;
                        waitnginfo=info;
                    }
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

            WaylineLocationCoordinate2D location = new WaylineLocationCoordinate2D((double) msg.x, (double) msg.y); // 위치값 생성
            waypoint.setLocation(location); // 생성한 위치값을 웨이포인트의 위도와 경도로 설정
            waypoint.setHeight((double) msg.z); // 웨이포인트의 고도 설정
            waypoint.setEllipsoidHeight((double) msg.z); // 웨이포인트의 Ellipsoid 고도 설정

            if (speedChanged) {
                waypoint.setSpeed(changedSpeed);
            } else {
                waypoint.setSpeed(defaultSpeed);
            }

            waypoint.setUseGlobalTurnParam(true);
            waypoint.setWaypointIndex(currentWPIMindex); // 웨이포인트 인덱스 설정
            Log.i(TAG,  "waypoint에 설정한 index : "+currentWPIMindex);
            WaypointInfoModel wpInfoModel = new WaypointInfoModel(); // 웨이포인트 인포 모델 객체 초기화

            if (msg.command == MAV_CMD.MAV_CMD_NAV_WAYPOINT && (msg.x != 0 || msg.y != 0)) {//웨이포인트 명령어이고 위도, 경도가 0이 아닐 때
                wpInfoModel.setWaylineWaypoint(waypoint);


                if(isInfoWaiting){
                    actionInfos.add(waitnginfo);
                    waitnginfo=null;
                    waitnginfo=new WaylineActionInfo();
                    isInfoWaiting=false;
                }
//                if(actionInfos.size()==0 ){
//                    info.setActionType(WaylineActionType.TAKE_PHOTO);
//                    ActionTakePhotoParam param = new ActionTakePhotoParam();
//                    actionInfos.add(info);
//                }


                wpInfoModel.setActionInfos(actionInfos);
                Log.i(TAG,  "wpInfoModel에 담긴"+wpInfoModel.getActionInfos());
                mWLIMList.add(wpInfoModel); // 웨이포인트 인포 모델 리스트에 추가

                currentWPIMindex++;

            } else if((msg.x == 0 || msg.y == 0) &&msg.command == MAV_CMD.MAV_CMD_CONDITION_YAW ) {//위도 경도가 0이고
                Log.i(TAG,  "condition YAW waypoint에 설정한 index : "+currentWPIMindex);
                Log.i(TAG,  "info : "+info.toString());
//                actionInfos.add(info);
            } else{

//                actionInfos.add(info);
            }
            Log.d(TAG, "==================================================="+i+"번째 아이템 " + "=======================================================");
        }


        this.saveKMZfile();
    }


    public void saveKMZfile() {

        for (int i = 0; i < mWLIMList.size(); i++) {
            Log.i(TAG, "savekmzfile "+i + "번째 WaylineInfoModel의 Actioninfos: " + mWLIMList.get(i).getActionInfos().toString() + "\nWaylineWaypoint: " + mWLIMList.get(i).getWaylineWaypoint().toString());
        }

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
        Log.i(TAG, "kmzFile saved directory: " + rootDir);
        Log.i(TAG, "kmzFile validity check : " + manager.checkValidation(kmzOutPath).getValue().toString());
        Log.i(TAG, "WaylineMissionParseInfo" + manager.getKMZInfo("/storage/emulated/0/Android/data/com.dji.sampleV5.aircraft/files/DJI/waypoint/generate_test.kmz").getWaylineMissionParseInfo().toString());
        Log.i(TAG, "WaylineMissionConfigParseInfo" + manager.getKMZInfo("/storage/emulated/0/Android/data/com.dji.sampleV5.aircraft/files/DJI/waypoint/generate_test.kmz").getWaylineMissionConfigParseInfo().toString());
        Log.i(TAG, "WaylineTemplatesParseInfo" + manager.getKMZInfo("/storage/emulated/0/Android/data/com.dji.sampleV5.aircraft/files/DJI/waypoint/generate_test.kmz").getWaylineTemplatesParseInfo().toString());
        Log.i(TAG, "WaylineWaylinesParseInfo" + manager.getKMZInfo("/storage/emulated/0/Android/data/com.dji.sampleV5.aircraft/files/DJI/waypoint/generate_test.kmz").getWaylineWaylinesParseInfo().toString());
        this.mWLIMList.clear();

    }


    public int uploadKMZfile(CommonCallbacks.CompletionCallbackWithProgress<Double> callback) {


        WaypointMissionManager.getInstance().pushKMZFileToAircraft(kmzOutPath, callback);
        return status;

    }


}

