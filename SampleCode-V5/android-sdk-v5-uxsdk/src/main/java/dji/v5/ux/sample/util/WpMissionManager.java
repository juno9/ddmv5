package dji.v5.ux.sample.util;

import static com.autonavi.base.amap.mapcore.tools.GLFileUtil.getFilesDir;

import static dji.v5.ux.MAVLink.common.msg_mission_ack.MAVLINK_MSG_ID_MISSION_ACK;

import android.os.Environment;
import android.util.Log;

import androidx.annotation.NonNull;

import com.dji.wpmzsdk.common.data.Template;
import com.dji.wpmzsdk.manager.WPMZManager;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;


import dji.sdk.keyvalue.value.mission.Waypoint;
import dji.sdk.keyvalue.value.mission.WaypointAction;
import dji.sdk.keyvalue.value.mission.WaypointActionType;
import dji.sdk.keyvalue.value.mission.WaypointMission;
import dji.sdk.wpmz.value.mission.ActionAircraftHoverParam;
import dji.sdk.wpmz.value.mission.ActionTakePhotoParam;
import dji.sdk.wpmz.value.mission.CameraLensType;
import dji.sdk.wpmz.value.mission.WaylineActionInfo;
import dji.sdk.wpmz.value.mission.WaylineActionType;
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
import dji.v5.ux.MAVLink.enums.MAV_CMD;
import dji.v5.ux.MAVLink.enums.MAV_RESULT;
import dji.v5.ux.sample.showcase.defaultlayout.DefaultLayoutActivity;
import dji.v5.ux.utils.KMZTestUtil;
import dji.v5.ux.utils.wpml.WaypointInfoModel;


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
    String kmzOutPath = rootDir + "generate_test.kmz";
    DroneModel model;
    int status = 0;
    DefaultLayoutActivity mainActivity;

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

    protected ArrayList<WaypointInfoModel> generateWLIMlist() {//미션아이템 목록을 웨이라인 인포 모델 목록으로 변환하는 메소드


        //웨이라인 인포 모델을 만들 때 들어가는 웨이라인웨이포인트 객체가 있다.
        //이 웨이포인트 객체는 어느 위치로 갈지, 속도는 어떻게 할지 등을 정의한다. 메시지의 command 값에 따라서 웨이포인트 객체를 생성하고 설정해준다.
        //웨이포인트 하나에는 여러개의 액션을 할당할 수 있다.


        WaylineWaypoint waypoint = null;//웨이포인트 객체 초기화, 여기에 아이템의값들을 넣어줄것
        ArrayList<WaylineActionInfo> actionlist = new ArrayList<>();
        WaylineLocationCoordinate2D location = null;

        boolean triggerDistanceEnabled = false;
        float triggerDistance = 0;
        boolean speedchanged = false;
        double changedSpeed = 0;

        for (int i = 0; i < this.getmMissionItemList().size(); i++) {

            msg_mission_item msg = this.getmMissionItemList().get(i);//마브링크 미션 아이템 목록에서 DJI미션으로 변환할 메시지 꺼냄
            Log.i(TAG, msg.toString());
            waypoint = new WaylineWaypoint();//웨이포인트 객체 생성
            switch (msg.command) {
                case MAV_CMD.MAV_CMD_NAV_WAYPOINT:
                    Log.d(TAG, "Waypoint: " + msg.x / 1000000.0 + ", " + msg.y / 1000000.0 + " at " + msg.z + " m " + msg.param2 + " Yaw " + msg.param1 + " Delay ");

                    location = new WaylineLocationCoordinate2D((double) msg.x, (double) msg.y);//메시지 바탕으로 위치값 생성
                    waypoint.setLocation(location);// 생성한 위치값을 웨이포인트의 위도와 경도로 설정
                    waypoint.setHeight((double) msg.z);//웨이포인트의 고도 설정
                    waypoint.setEllipsoidHeight((double) msg.z);//웨이포인트의 Ellipsoid고도 설정(이 항목이 뭔지 모르나 샘플에서 그냥 고도값을 넣은것을 보고 일단 넣어둠)


                    if (speedchanged) {
                        waypoint.setSpeed(changedSpeed);
                    }


                    if (msg.param1 > 0) {//멈춤 시간이 0보다 크면
                        WaylineActionInfo info = new WaylineActionInfo();//웨이포인트 액션 정보 객체 생성
                        info.setActionType(WaylineActionType.HOVER);//액션 타입을 호버로 설정
                        ActionAircraftHoverParam param = new ActionAircraftHoverParam();//
                        param.setHoverTime((double) msg.param1);//얼마나 멈춰 있을지 지정해 둔 파라미터를 생성
                        info.setAircraftHoverParam(param);//액션인포의 액션에 생성한 파라미터를 넣음
                        actionlist.add(info);//액션리스트에 생성한 액션인포를 넣음
                    }

                    if (msg.param2 > 0) {//돌리기 각도가 0보다 크면
                        WaylineActionInfo info = new WaylineActionInfo();//웨이포인트 액션 정보 객체 생성
                        info.setActionType(WaylineActionType.ROTATE_YAW);//액션 타입을 돌리기로 설정
                        ActionAircraftHoverParam param = new ActionAircraftHoverParam();//액션 파라미터 객체 생성
                        param.setHoverTime((double) msg.param2);//돌리기 각도를 파라미터에 넣음
                        info.setAircraftHoverParam(param);//액션인포의 액션에 생성한 파라미터를 넣음
                        actionlist.add(info);//액션리스트에 생성한 액션인포를 넣음
                    }


                    break;

                case MAV_CMD.MAV_CMD_DO_CHANGE_SPEED://이 스위치문이 작동한 다음번 아이템 부터는 여기서 설정한 속도가 들어가줘야 한다.
                    Log.d(TAG, "Change Speed: " + msg.x / 10000000.0 + ", " + msg.y / 10000000.0 + " at " + msg.z + " m " + msg.param2 + " Yaw " + msg.param1 + " Delay ");
                    speedchanged = true;
                    if (speedchanged) {
                        waypoint.setSpeed((double) msg.param1);
                        changedSpeed = (double) msg.param1;
                    }
                    break;


                case MAV_CMD.MAV_CMD_NAV_TAKEOFF:
                    Log.d(TAG, "Takeoff...");
                    // if we got an item (Start item) already we got a position, now we just add altitude.

                    // if we got an item (Start item) already we got a position, now we just add altitude.
                    if (waypoint != null) {
                        waypoint.setHeight((double) msg.z);
                    } else {
                        if (msg.x == 0 || msg.y == 0) {
                            location = new WaylineLocationCoordinate2D(model.get_current_lat(), model.get_current_lon());//메시지 바탕으로 위치값 생성
                            waypoint.setLocation(location);
                            waypoint.setHeight((double) msg.z);


                        } else {
//                            currentWP = new Waypoint(m.x,m.z, m.z);
                            location = new WaylineLocationCoordinate2D(msg.x / 1000000.0, msg.y / 1000000.0);//메시지 바탕으로 위치값 생성
                            waypoint.setLocation(location);
                            waypoint.setHeight((double) msg.z);
                        }
                    }


                    break;


                case MAV_CMD.MAV_CMD_DO_MOUNT_CONTROL:
                    Log.d(TAG, "Set gimbal pitch: " + msg.param1);
                    break;
                case MAV_CMD.MAV_CMD_IMAGE_STOP_CAPTURE:
                    Log.d(TAG, "MAV_CMD_IMAGE_STOP_CAPTURE ");
//                    mModel.stopImageCaptureIntervalControl();
                    break;
                case MAV_CMD.MAV_CMD_IMAGE_START_CAPTURE:
                    Log.d(TAG, "MAV_CMD_IMAGE_START_CAPTURE");
                    // Cant add a new WP since wp.xyz are garbage & NaN
                    // Add to last WP entry
                    /**
                     * 기체가 현재 웨이포인트와 다음 웨이포인트 사이를 이동할 때 두 장의 사진이 촬영되는 시간 간격(초)입니다. 첫 번째 사진은 기체가 현재 웨이포인트를 떠날 때 촬영됩니다.
                     * 최대값은 6,000.0입니다. 최소값은 0.0 이상이며 카메라 유형 및 카메라 매개변수에 따라 다릅니다. 사진 파일 형식이 JPEG인 경우 권장되는 최소값은 2.0입니다.
                     * 사진 파일 형식이 RAW인 경우 최소값은 10.0입니다. 입력이 카메라의 용량을 초과하면 사진이 가능한 최대 속도로 촬영됩니다. 기본값은 0.0이며 사진이 촬영되지 않습니다.
                     */

                    // dji 임무웨이포인트자체에 설정하여 start image capture mavlink 시, dji wp마다 찍도록 설정해야함
                    // 플래그로 세팅된 시점부터 MAV_CMD_NAV_WAYPOINT 시 해당 세팅을 넣도록 하면 될듯.
                    break;

                case MAV_CMD.MAV_CMD_DO_SET_CAM_TRIGG_DIST:
                    Log.d(TAG, "MAV_CMD_DO_SET_CAM_TRIGG_DIST");

                    if (!triggerDistanceEnabled) {
                        if (msg.param1 != 0) {
                            triggerDistanceEnabled = true;
                            triggerDistance = msg.param1;
                        } else {
                            triggerDistance = 0;
                            triggerDistanceEnabled = false;
                        }
                    }
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

                case MAV_CMD.MAV_CMD_CONDITION_YAW:
                    Log.d(TAG, "MAV_CMD_CONDITION_YAW");
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


            waypoint.setWaypointIndex(i);//첫 웨이포인트 미션 생성할거니까

            waypoint.setSpeed((double) msg.param2);//웨이포인트 속도 설정(이거는 메시지에서 어떤 파라미터를 가져와야 하는지 모르겠음)
            waypoint.setUseGlobalTurnParam(true);


            WaypointInfoModel wpInfomodel = new WaypointInfoModel();//웨이포인트 인포 모델 객체 초기화
            wpInfomodel.setWaylineWaypoint(waypoint);
            wpInfomodel.setActionInfos(actionlist);
            this.mWLIMList.add(i, wpInfomodel);
            waypoint = null;
        }


        return this.mWLIMList;

    }

    public void saveKMZfile() {
        WaylineMission wlm = KMZTestUtil.createWaylineMission();
        WaylineMissionConfig wlmc = KMZTestUtil.createMissionConfig();
        Template template = KMZTestUtil.createTemplate(this.mWLIMList);
        WPMZManager.getInstance().generateKMZFile(kmzOutPath, wlm, wlmc, template);
        Log.i(TAG, "kmzFile saved directory: " + rootDir);

        File file = new File(getFilesDir(mainActivity.getApplicationContext()), "example.txt");
        try (FileOutputStream fos = new FileOutputStream(file)) {
            fos.write("Hello, World!".getBytes());
        } catch (IOException e) {
            e.printStackTrace();

        }
    }


    public int uploadKMZfile(CommonCallbacks.CompletionCallbackWithProgress<Double> callback) {


        WaypointMissionManager.getInstance().pushKMZFileToAircraft(kmzOutPath, callback);
        return status;

    }


}

