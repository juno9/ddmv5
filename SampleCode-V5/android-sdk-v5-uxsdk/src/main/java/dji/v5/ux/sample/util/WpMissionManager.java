package dji.v5.ux.sample.util;

import android.os.Environment;
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

