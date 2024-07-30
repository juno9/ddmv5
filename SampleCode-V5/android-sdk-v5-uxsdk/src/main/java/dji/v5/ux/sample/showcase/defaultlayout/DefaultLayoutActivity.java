/*
 * Copyright (c) 2018-2020 DJI
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

package dji.v5.ux.sample.showcase.defaultlayout;

import static dji.sdk.keyvalue.value.flightassistant.ActiveTrackMode.QUICK_SHOT;
import static dji.v5.ux.MAVLink.enums.MAV_MISSION_RESULT.MAV_MISSION_ACCEPTED;

import android.annotation.SuppressLint;
import android.app.AlertDialog;
import android.content.DialogInterface;
import android.content.SharedPreferences;
import android.content.res.AssetManager;
import android.graphics.Color;
import android.graphics.drawable.ColorDrawable;
import android.graphics.drawable.Drawable;
import android.os.AsyncTask;
import android.os.Bundle;
import android.preference.PreferenceManager;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.ImageView;
import android.widget.RadioButton;
import android.widget.RadioGroup;
import android.widget.SeekBar;
import android.widget.TextView;
import android.widget.Toast;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.appcompat.app.AppCompatActivity;
import androidx.constraintlayout.widget.ConstraintLayout;
import androidx.core.view.GravityCompat;
import androidx.drawerlayout.widget.DrawerLayout;
import androidx.fragment.app.FragmentContainerView;
import androidx.fragment.app.FragmentManager;

import com.naver.maps.geometry.LatLng;
import com.naver.maps.map.MapFragment;
import com.naver.maps.map.NaverMap;
import com.naver.maps.map.OnMapReadyCallback;
import com.naver.maps.map.UiSettings;
import com.naver.maps.map.overlay.Marker;

import java.io.BufferedInputStream;
import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.lang.ref.WeakReference;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.net.Socket;
import java.net.SocketException;
import java.net.UnknownHostException;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Timer;
import java.util.TimerTask;
import java.util.concurrent.TimeUnit;

import dji.sdk.keyvalue.value.airlink.VideoSourceEntity;
import dji.sdk.keyvalue.value.common.CameraLensType;
import dji.sdk.keyvalue.value.common.ComponentIndexType;
import dji.sdk.keyvalue.value.flightassistant.ActiveTrackMode;
import dji.v5.common.callback.CommonCallbacks;
import dji.v5.common.error.IDJIError;
import dji.v5.common.video.channel.VideoChannelState;
import dji.v5.common.video.channel.VideoChannelType;
import dji.v5.common.video.interfaces.IVideoChannel;
import dji.v5.common.video.interfaces.VideoChannelStateChangeListener;
import dji.v5.common.video.stream.PhysicalDevicePosition;
import dji.v5.common.video.stream.StreamSource;
import dji.v5.manager.datacenter.MediaDataCenter;
import dji.v5.manager.datacenter.camera.CameraStreamManager;
import dji.v5.manager.datacenter.livestream.LiveStreamManager;
import dji.v5.manager.datacenter.livestream.LiveStreamSettings;
import dji.v5.manager.datacenter.livestream.LiveStreamType;
import dji.v5.manager.datacenter.livestream.LiveVideoBitrateMode;
import dji.v5.manager.datacenter.livestream.StreamQuality;
import dji.v5.manager.datacenter.livestream.settings.RtmpSettings;
import dji.v5.manager.interfaces.ICameraStreamManager;
import dji.v5.network.DJINetworkManager;
import dji.v5.network.IDJINetworkStatusListener;
import dji.v5.utils.common.JsonUtil;
import dji.v5.utils.common.LogUtils;
import dji.v5.ux.MAVLink.MAVLinkPacket;
import dji.v5.ux.MAVLink.Messages.MAVLinkMessage;
import dji.v5.ux.MAVLink.Parser;
import dji.v5.ux.R;
import dji.v5.ux.accessory.RTKStartServiceHelper;
import dji.v5.ux.cameracore.widget.autoexposurelock.AutoExposureLockWidget;
import dji.v5.ux.cameracore.widget.cameracontrols.CameraControlsWidget;
import dji.v5.ux.cameracore.widget.cameracontrols.exposuresettings.ExposureSettingsPanel;
import dji.v5.ux.cameracore.widget.cameracontrols.lenscontrol.LensControlWidget;
import dji.v5.ux.cameracore.widget.focusexposureswitch.FocusExposureSwitchWidget;
import dji.v5.ux.cameracore.widget.focusmode.FocusModeWidget;
import dji.v5.ux.cameracore.widget.fpvinteraction.FPVInteractionWidget;
import dji.v5.ux.core.base.SchedulerProvider;
import dji.v5.ux.core.communication.BroadcastValues;
import dji.v5.ux.core.communication.GlobalPreferenceKeys;
import dji.v5.ux.core.communication.ObservableInMemoryKeyedStore;
import dji.v5.ux.core.communication.UXKeys;
import dji.v5.ux.core.extension.ViewExtensions;
import dji.v5.ux.core.panel.systemstatus.SystemStatusListPanelWidget;
import dji.v5.ux.core.panel.topbar.TopBarPanelWidget;
import dji.v5.ux.core.util.CameraUtil;
import dji.v5.ux.core.util.CommonUtils;
import dji.v5.ux.core.util.DataProcessor;
import dji.v5.ux.core.util.ViewUtil;
import dji.v5.ux.core.widget.fpv.FPVWidget;
import dji.v5.ux.core.widget.hsi.HorizontalSituationIndicatorWidget;
import dji.v5.ux.core.widget.hsi.PrimaryFlightDisplayWidget;
import dji.v5.ux.core.widget.setting.SettingWidget;
import dji.v5.ux.core.widget.simulator.SimulatorIndicatorWidget;
import dji.v5.ux.core.widget.systemstatus.SystemStatusWidget;
import dji.v5.ux.gimbal.GimbalFineTuneWidget;
import dji.v5.ux.sample.util.DDMMqttClient;
import dji.v5.ux.sample.util.DroneModel;
import dji.v5.ux.sample.util.MAVLinkReceiver;
import dji.v5.ux.sample.util.MAVParam;
import dji.v5.ux.sample.util.StreamDialog;
import dji.v5.ux.sample.util.WpMissionManager;
import dji.v5.ux.sample.util.video.DDMImageHandler;
import dji.v5.ux.training.simulatorcontrol.SimulatorControlWidget;
import dji.v5.ux.visualcamera.CameraNDVIPanelWidget;
import dji.v5.ux.visualcamera.CameraVisiblePanelWidget;
import dji.v5.ux.visualcamera.zoom.FocalZoomWidget;
import io.reactivex.rxjava3.android.schedulers.AndroidSchedulers;
import io.reactivex.rxjava3.disposables.CompositeDisposable;

/**
 * Displays a sample layout of widgets similar to that of the various DJI apps.
 */
public class DefaultLayoutActivity extends AppCompatActivity {

    //region Fields
    private final String TAG = LogUtils.getTag(this);
    protected FPVWidget primaryFpvWidget;
    protected FPVInteractionWidget fpvInteractionWidget;
    protected FPVWidget secondaryFPVWidget;
    protected SystemStatusListPanelWidget systemStatusListPanelWidget;
    protected SimulatorControlWidget simulatorControlWidget;
    protected LensControlWidget lensControlWidget;
    protected AutoExposureLockWidget autoExposureLockWidget;
    protected FocusModeWidget focusModeWidget;
    protected FocusExposureSwitchWidget focusExposureSwitchWidget;
    protected CameraControlsWidget cameraControlsWidget;
    protected HorizontalSituationIndicatorWidget horizontalSituationIndicatorWidget;
    protected ExposureSettingsPanel exposureSettingsPanel;
    protected PrimaryFlightDisplayWidget pfvFlightDisplayWidget;
    protected CameraNDVIPanelWidget ndviCameraPanel;
    protected CameraVisiblePanelWidget visualCameraPanel;
    protected FocalZoomWidget focalZoomWidget;
    protected SettingWidget settingWidget;
    //    protected MapWidget mapWidget;
    protected FragmentContainerView mapWidget;
    protected TopBarPanelWidget topBarPanel;
    protected ConstraintLayout fpvParentView;
    private DrawerLayout mDrawerLayout;
    private TextView gimbalAdjustDone;
    private ImageView drowIcon;
    private GimbalFineTuneWidget gimbalFineTuneWidget;
    private ActiveTrackMode mode = QUICK_SHOT;
    private CompositeDisposable compositeDisposable;
    private final DataProcessor<CameraSource> cameraSourceProcessor = DataProcessor.create(new CameraSource(PhysicalDevicePosition.UNKNOWN,
            CameraLensType.UNKNOWN));
    private VideoChannelStateChangeListener primaryChannelStateListener = null;
    private VideoChannelStateChangeListener secondaryChannelStateListener = null;
    private final IDJINetworkStatusListener networkStatusListener = isNetworkAvailable -> {
        if (isNetworkAvailable) {
            LogUtils.d(TAG, "isNetworkAvailable=" + true);
            RTKStartServiceHelper.INSTANCE.startRtkService(false);
        }
    };
    DroneModel mModel;
    MAVLinkReceiver mReceiver;
    WpMissionManager wpMissionmanager;
    public static boolean FLAG_TELEMETRY_ADDRESS_CHANGED = false;
    private Parser mMavlinkParser;
    private DatagramSocket socket;
    private Socket mTcpSocket;
    private InetSocketAddress isa;
    private InputStream in;
    private BufferedInputStream bufferIn;
    private GCSCommunicatorAsyncTask mGCSCommunicator;
    private final int GCS_TIMEOUT_mSEC = 2000;
    private boolean shouldConnect = false;
    private boolean connectivityHasChanged = false;
    private SharedPreferences prefs;
    private String mNewInbound = "";
    private String mNewDJI = "";
    String id, gcsip, gcsport;
    private Button streamingButton;
    StreamDialog streamDialog;
    String streamAddress;
    DDMImageHandler ddmImageHandler;

    private LiveStreamManager iLiveStreamManager;
    private CameraStreamManager iCameraStreamManager;
    List<VideoSourceEntity> cameraList;
    //private DDMImageHandler mDDMImageHandler;
    //endregion

    //region Lifecycle
    @SuppressLint("MissingInflatedId")
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        setContentView(R.layout.uxsdk_activity_default_layout);
        fpvParentView = findViewById(R.id.fpv_holder);
        mDrawerLayout = findViewById(R.id.root_view);
        topBarPanel = findViewById(R.id.panel_top_bar);
        settingWidget = topBarPanel.getSettingWidget();
        primaryFpvWidget = findViewById(R.id.widget_primary_fpv);
        fpvInteractionWidget = findViewById(R.id.widget_fpv_interaction);
        secondaryFPVWidget = findViewById(R.id.widget_secondary_fpv);
        systemStatusListPanelWidget = findViewById(R.id.widget_panel_system_status_list);
        simulatorControlWidget = findViewById(R.id.widget_simulator_control);
        lensControlWidget = findViewById(R.id.widget_lens_control);
        ndviCameraPanel = findViewById(R.id.panel_ndvi_camera);
        visualCameraPanel = findViewById(R.id.panel_visual_camera);
        autoExposureLockWidget = findViewById(R.id.widget_auto_exposure_lock);
        focusModeWidget = findViewById(R.id.widget_focus_mode);
        focusExposureSwitchWidget = findViewById(R.id.widget_focus_exposure_switch);
        exposureSettingsPanel = findViewById(R.id.panel_camera_controls_exposure_settings);
        pfvFlightDisplayWidget = findViewById(R.id.widget_fpv_flight_display_widget);
        focalZoomWidget = findViewById(R.id.widget_focal_zoom);
        cameraControlsWidget = findViewById(R.id.widget_camera_controls);
        horizontalSituationIndicatorWidget = findViewById(R.id.widget_horizontal_situation_indicator);
        gimbalAdjustDone = findViewById(R.id.fpv_gimbal_ok_btn);
        gimbalFineTuneWidget = findViewById(R.id.setting_menu_gimbal_fine_tune);
        mapWidget = findViewById(R.id.map_fragment);
        streamingButton = findViewById(R.id.btn_streaming);
        cameraControlsWidget.getExposureSettingsIndicatorWidget().setStateChangeResourceId(R.id.panel_camera_controls_exposure_settings);
        iCameraStreamManager = (CameraStreamManager) MediaDataCenter.getInstance().getCameraStreamManager();
        iLiveStreamManager = (LiveStreamManager) MediaDataCenter.getInstance().getLiveStreamManager();
        streamDialog = new StreamDialog(this);
        drowIcon = findViewById(R.id.imageView);
        cameraList = iCameraStreamManager.getAvailableCameraSourceList();//사용 가능한 카메라 목록을 가져옴




        streamDialog.setDialogListener(new StreamDialog.StreamDialogInterface() {
            @Override
            public void startBtnClicked() {
                Log.i(TAG, "스타트버튼 눌림");
                //이거 눌리면 ip값이랑 카메라값, 품질값 받아와서 실행해야 함.
                startLiveStream();
                streamDialog.dismiss();
            }

            @Override
            public void stopBtnClicked() {
                Log.i(TAG, "스탑버튼 눌림");
                stopStream();
                streamDialog.dismiss();
            }

            @Override
            public void etdStreamAddress() {

            }


        });

        streamDialog.setOnEditTextChangedListener(new StreamDialog.OnEditTextChangedListener() {
            @Override
            public void onEditTextChanged(String address) {
                streamAddress = address;
                prefs.edit().putString("pref_stream_address", address).apply();
                prefs.edit().apply();
            }
        });

        initClickListener();
        MediaDataCenter.getInstance().getVideoStreamManager().addStreamSourcesListener(sources -> runOnUiThread(() -> updateFPVWidgetSource(sources)));
        primaryFpvWidget.setOnFPVStreamSourceListener((devicePosition, lensType) -> {
            cameraSourceProcessor.onNext(new CameraSource(devicePosition, lensType));
        });


        //小surfaceView放置在顶部，避免被大的遮挡
        secondaryFPVWidget.setSurfaceViewZOrderOnTop(true);
        secondaryFPVWidget.setSurfaceViewZOrderMediaOverlay(true);

//        mapWidget.initAMap(map -> {
//            // map.setOnMapClickListener(latLng -> onViewClick(mapWidget));
//            DJIUiSettings uiSetting = map.getUiSettings();
//            if (uiSetting != null) {
//                uiSetting.setZoomControlsEnabled(false);//hide zoom widget
//            }
//        });
//        mapWidget.onCreate(savedInstanceState);
        FragmentManager fm = getSupportFragmentManager();
        MapFragment mapFragment = (MapFragment) fm.findFragmentById(R.id.map_fragment);
        if (mapFragment == null) {
            mapFragment = MapFragment.newInstance();
            fm.beginTransaction().add(R.id.map_fragment, mapFragment).commit();
        }

        mapFragment.getMapAsync(new OnMapReadyCallback() {//네이버 지도는 일단 달아는 두지만 나중에 손 봐야 함. Vworld나 다른 무료 지도들을 활용해야하기 때문
            @Override
            public void onMapReady(@NonNull NaverMap naverMap) {
                UiSettings uiSettings = naverMap.getUiSettings();
                uiSettings.setCompassEnabled(false);
                uiSettings.setLocationButtonEnabled(false);
                uiSettings.setZoomControlEnabled(false);
                uiSettings.setLogoClickEnabled(false);
                Marker marker = new Marker();//마커 생성
                marker.setPosition(new LatLng(mModel.get_current_lat(), mModel.get_current_lon()));//마커
            }
        });


        getWindow().setBackgroundDrawable(new ColorDrawable(Color.BLACK));

        //实现RTK监测网络，并自动重连机制
        DJINetworkManager.getInstance().addNetworkStatusListener(networkStatusListener);


        prefs = PreferenceManager.getDefaultSharedPreferences(DefaultLayoutActivity.this);
        id = prefs.getString("pref_drone_id", "19");
        streamAddress = "rtmp://drowdev.skymap.kr:1935/live/drone" + id + ".stream";
        gcsip = prefs.getString("pref_gcs_ip", "127.0.0.1");
        gcsport = prefs.getString("pref_telem_port", "6760");

        mModel = new DroneModel(this);
        mModel.setSystemId(Integer.parseInt(id));


        //mqtt메시지 보내는 코드 정상동작 확인 240711자
        Thread mqttTestThread = new Thread(() -> {
            try {
                //DDMMqttClient client = DDMMqttClient.getSimpleMqttClient(this, "192.168.110.93", "1883", "clrobur/mapping/drone" + id);

                while (true) {

                    mModel.setGimbalRotation((double) 0.0);//짐벌 각도 0도 세팅
                    mModel.takePhoto();
                    Thread.sleep(1000 * 15); //15 초마다

                }
            } catch (Exception e) {
                Log.i(TAG, "exception : " + e.toString());
            }
        });
        mqttTestThread.start();
        drowIcon.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                mModel.downloadPhotofromdrone();
            }
        });

        String streamAddress = prefs.getString("pref_stream_address", "rtmp://drowdev.skymap.kr:1935/live/drone" + id + ".stream");
        prefs.edit().putString("pref_stream_address", streamAddress).apply();
        prefs.edit().apply();
        mReceiver = new MAVLinkReceiver(this, mModel);

        wpMissionmanager = new WpMissionManager(mReceiver, mModel, this);
        mReceiver.setWpMissionManager(wpMissionmanager);

        mGCSCommunicator = new GCSCommunicatorAsyncTask(this);
        mGCSCommunicator.execute();

        ddmImageHandler = new DDMImageHandler(this, mModel, primaryFpvWidget.getWidth(), primaryFpvWidget.getHeight());
        CameraStreamManager.getInstance().addFrameListener(ComponentIndexType.LEFT_OR_MAIN, ICameraStreamManager.FrameFormat.YUV420_888, ddmImageHandler);


        mModel.initmediamanager();


    }

    private void isGimableAdjustClicked(BroadcastValues broadcastValues) {
        if (mDrawerLayout.isDrawerOpen(GravityCompat.END)) {
            mDrawerLayout.closeDrawers();
        }
        horizontalSituationIndicatorWidget.setVisibility(View.GONE);
        if (gimbalFineTuneWidget != null) {
            gimbalFineTuneWidget.setVisibility(View.VISIBLE);
        }
    }

    private void initClickListener() {
        secondaryFPVWidget.setOnClickListener(v -> swapVideoSource());
        initChannelStateListener();

        if (settingWidget != null) {
            settingWidget.setOnClickListener(v -> toggleRightDrawer());
        }

        // Setup top bar state callbacks
        SystemStatusWidget systemStatusWidget = topBarPanel.getSystemStatusWidget();
        if (systemStatusWidget != null) {
            systemStatusWidget.setOnClickListener(v -> ViewExtensions.toggleVisibility(systemStatusListPanelWidget));
        }

        SimulatorIndicatorWidget simulatorIndicatorWidget = topBarPanel.getSimulatorIndicatorWidget();
        if (simulatorIndicatorWidget != null) {
            simulatorIndicatorWidget.setOnClickListener(v -> ViewExtensions.toggleVisibility(simulatorControlWidget));
        }
        gimbalAdjustDone.setOnClickListener(view -> {
            horizontalSituationIndicatorWidget.setVisibility(View.VISIBLE);
            if (gimbalFineTuneWidget != null) {
                gimbalFineTuneWidget.setVisibility(View.GONE);
            }

        });
        streamingButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {

                streamDialog.show();
                streamDialog.setStreamAddress(streamAddress);
                streamDialog.getCameraGroup().removeAllViews();//카메라 리스트에 표출할 카메라 목록을 만들어주는거
                for (int i = 0; i < cameraList.size(); i++) {
                    RadioButton radioButton = new RadioButton(getApplicationContext());
                    radioButton.setText(cameraList.get(i).getPosition().toString());
                    streamDialog.getCameraGroup().addView(radioButton);
                    if (i == 0) {
                        radioButton.setChecked(true);
                    }
                }
                streamDialog.getCameraGroup().setOnCheckedChangeListener(new RadioGroup.OnCheckedChangeListener() {//카메라 목록중에 하나 선택했을때 어떻게 동작할지 정의하는 곳
                    @Override
                    public void onCheckedChanged(RadioGroup group, int checkedId) {
                        for (int i = 0; i < group.getChildCount(); i++) {
                            RadioButton radioButton = (RadioButton) group.getChildAt(i);
                            if (radioButton.getId() == checkedId) {
                                Log("RadioGroup :" + group + "  checked ID :" + checkedId + "  checked tag :" + radioButton.getText());
                                iLiveStreamManager.setCameraIndex(cameraList.get(i).getPosition());
                                Log("checked position :" + cameraList.get(i).getPosition().toString());


                            }
                        }
                    }
                });
                streamDialog.getQualityGroup().setOnCheckedChangeListener(new RadioGroup.OnCheckedChangeListener() {
                    @Override
                    public void onCheckedChanged(RadioGroup group, int checkedId) {
                        for (int i = 0; i < group.getChildCount(); i++) {
                            RadioButton radioButton = (RadioButton) group.getChildAt(i);
                            if (radioButton.getId() == checkedId) {
                                Log("RadioGroup :" + group + "  checked ID :" + checkedId + "  checked tag :" + radioButton.getTag());
                                switch (radioButton.getTag().toString()) {
                                    case "1": {
                                        iLiveStreamManager.setLiveStreamQuality(StreamQuality.SD);
                                        break;
                                    }
                                    case "2": {
                                        iLiveStreamManager.setLiveStreamQuality(StreamQuality.HD);
                                        break;
                                    }
                                    case "3": {
                                        iLiveStreamManager.setLiveStreamQuality(StreamQuality.FULL_HD);
                                        break;
                                    }
                                }
                            }
                        }


                    }
                });
                streamDialog.getBitrateGroup().setOnCheckedChangeListener(new RadioGroup.OnCheckedChangeListener() {
                    @Override
                    public void onCheckedChanged(RadioGroup group, int checkedId) {
                        for (int i = 0; i < group.getChildCount(); i++) {
                            RadioButton radioButton = (RadioButton) group.getChildAt(i);
                            if (radioButton.getId() == checkedId) {
                                Log("RadioGroup :" + group + "  checked ID :" + checkedId + "  checked text:" + radioButton.getText());
                                switch (radioButton.getText().toString()) {
                                    case "MANUAL": {
                                        streamDialog.getsbBitrate().setVisibility(View.VISIBLE);
                                        streamDialog.getTvBitrate().setVisibility(View.VISIBLE);
                                        iLiveStreamManager.setLiveVideoBitrateMode(LiveVideoBitrateMode.MANUAL);

                                        streamDialog.getsbBitrate().setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
                                            @Override
                                            public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                                                int bitrate = (int) (8 * 1024 * 2048 * (0.1 + 0.9 * streamDialog.getsbBitrate().getProgress() / (double) streamDialog.getsbBitrate().getMax()));
                                                streamDialog.getTvBitrate().setText("Bitrate : " + bitrate + " kbps");
                                                Log("bitrate : " + bitrate);

                                            }

                                            @Override
                                            public void onStartTrackingTouch(SeekBar seekBar) {

                                            }

                                            @Override
                                            public void onStopTrackingTouch(SeekBar seekBar) {
                                                int bitrate = (int) (8 * 1024 * 2048 * (0.1 + 0.9 * streamDialog.getsbBitrate().getProgress() / (double) streamDialog.getsbBitrate().getMax()));
                                                iLiveStreamManager.setLiveVideoBitrate(bitrate);

                                            }
                                        });
                                        break;
                                    }
                                    case "AUTO": {
                                        streamDialog.getsbBitrate().setVisibility(View.GONE);
                                        streamDialog.getTvBitrate().setVisibility(View.GONE);
                                        iLiveStreamManager.setLiveVideoBitrateMode(LiveVideoBitrateMode.AUTO);
                                        break;
                                    }
                                }


                            }
                        }
                    }
                });
            }
        });


    }

    private void toggleRightDrawer() {
        mDrawerLayout.openDrawer(GravityCompat.END);
    }


    @Override
    protected void onDestroy() {
        super.onDestroy();
//        mapWidget.onDestroy();
        MediaDataCenter.getInstance().getVideoStreamManager().clearAllStreamSourcesListeners();
        removeChannelStateListener();
        DJINetworkManager.getInstance().removeNetworkStatusListener(networkStatusListener);
        closeGCSCommunicator();
        iLiveStreamManager.stopStream(new CommonCallbacks.CompletionCallback() {
            @Override
            public void onSuccess() {
                Log("스트리밍 멈춤");
            }

            @Override
            public void onFailure(@NonNull IDJIError idjiError) {
                Log("스트리밍 멈춤 실패");
            }
        });

    }

    @Override
    protected void onResume() {
        super.onResume();
//        mapWidget.onResume();
        compositeDisposable = new CompositeDisposable();
        compositeDisposable.add(systemStatusListPanelWidget.closeButtonPressed()
                .observeOn(AndroidSchedulers.mainThread())
                .subscribe(pressed -> {
                    if (pressed) {
                        ViewExtensions.hide(systemStatusListPanelWidget);
                    }
                }));
        compositeDisposable.add(simulatorControlWidget.getUIStateUpdates()
                .observeOn(AndroidSchedulers.mainThread())
                .subscribe(simulatorControlWidgetState -> {
                    if (simulatorControlWidgetState instanceof SimulatorControlWidget.UIState.VisibilityUpdated) {
                        if (((SimulatorControlWidget.UIState.VisibilityUpdated) simulatorControlWidgetState).isVisible()) {
                            hideOtherPanels(simulatorControlWidget);
                        }
                    }
                }));
        compositeDisposable.add(cameraSourceProcessor.toFlowable()
                .observeOn(SchedulerProvider.io())
                .throttleLast(500, TimeUnit.MILLISECONDS)
                .subscribeOn(SchedulerProvider.io())
                .subscribe(result -> runOnUiThread(() -> onCameraSourceUpdated(result.devicePosition, result.lensType)))
        );
        compositeDisposable.add(ObservableInMemoryKeyedStore.getInstance()
                .addObserver(UXKeys.create(GlobalPreferenceKeys.GIMBAL_ADJUST_CLICKED))
                .observeOn(SchedulerProvider.ui())
                .subscribe(this::isGimableAdjustClicked));
        ViewUtil.setKeepScreen(this, true);
    }

    @Override
    protected void onPause() {
        if (compositeDisposable != null) {
            compositeDisposable.dispose();
            compositeDisposable = null;
        }
//        mapWidget.onPause();
        super.onPause();
        ViewUtil.setKeepScreen(this, false);
    }
    //endregion

    private void hideOtherPanels(@Nullable View widget) {
        View[] panels = {
                simulatorControlWidget
        };

        for (View panel : panels) {
            if (widget != panel) {
                panel.setVisibility(View.GONE);
            }
        }
    }

    private void updateFPVWidgetSource(List<StreamSource> streamSources) {
        LogUtils.i(TAG, JsonUtil.toJson(streamSources));
        if (streamSources == null) {
            return;
        }

        //没有数据
        if (streamSources.isEmpty()) {
            secondaryFPVWidget.setVisibility(View.GONE);
            return;
        }

        //仅一路数据
        if (streamSources.size() == 1) {
            //这里仅仅做Widget的显示与否，source和channel的获取放到widget中
            secondaryFPVWidget.setVisibility(View.GONE);
            return;
        }
        secondaryFPVWidget.setVisibility(View.VISIBLE);
    }

    private void initChannelStateListener() {
        IVideoChannel primaryChannel =
                MediaDataCenter.getInstance().getVideoStreamManager().getAvailableVideoChannel(VideoChannelType.PRIMARY_STREAM_CHANNEL);
        IVideoChannel secondaryChannel =
                MediaDataCenter.getInstance().getVideoStreamManager().getAvailableVideoChannel(VideoChannelType.SECONDARY_STREAM_CHANNEL);
        if (primaryChannel != null) {
            primaryChannelStateListener = (from, to) -> {
                StreamSource primaryStreamSource = primaryChannel.getStreamSource();
                if (VideoChannelState.ON == to && primaryStreamSource != null) {
                    runOnUiThread(() -> primaryFpvWidget.updateVideoSource(primaryStreamSource, VideoChannelType.PRIMARY_STREAM_CHANNEL));
                }
            };
            primaryChannel.addVideoChannelStateChangeListener(primaryChannelStateListener);
        }
        if (secondaryChannel != null) {
            secondaryChannelStateListener = (from, to) -> {
                StreamSource secondaryStreamSource = secondaryChannel.getStreamSource();
                if (VideoChannelState.ON == to && secondaryStreamSource != null) {
                    runOnUiThread(() -> secondaryFPVWidget.updateVideoSource(secondaryStreamSource, VideoChannelType.SECONDARY_STREAM_CHANNEL));
                }
            };
            secondaryChannel.addVideoChannelStateChangeListener(secondaryChannelStateListener);
        }
    }

    private void removeChannelStateListener() {
        IVideoChannel primaryChannel =
                MediaDataCenter.getInstance().getVideoStreamManager().getAvailableVideoChannel(VideoChannelType.PRIMARY_STREAM_CHANNEL);
        IVideoChannel secondaryChannel =
                MediaDataCenter.getInstance().getVideoStreamManager().getAvailableVideoChannel(VideoChannelType.SECONDARY_STREAM_CHANNEL);
        if (primaryChannel != null) {
            primaryChannel.removeVideoChannelStateChangeListener(primaryChannelStateListener);
        }
        if (secondaryChannel != null) {
            secondaryChannel.removeVideoChannelStateChangeListener(secondaryChannelStateListener);
        }
    }

    private void onCameraSourceUpdated(PhysicalDevicePosition devicePosition, CameraLensType lensType) {
        LogUtils.i(TAG, devicePosition, lensType);
        ComponentIndexType cameraIndex = CameraUtil.getCameraIndex(devicePosition);
        updateViewVisibility(devicePosition, lensType);
        updateInteractionEnabled();
        //如果无需使能或者显示的，也就没有必要切换了。
        if (fpvInteractionWidget.isInteractionEnabled()) {
            fpvInteractionWidget.updateCameraSource(cameraIndex, lensType);
            fpvInteractionWidget.updateGimbalIndex(CommonUtils.getGimbalIndex(devicePosition));
        }
        if (lensControlWidget.getVisibility() == View.VISIBLE) {
            lensControlWidget.updateCameraSource(cameraIndex, lensType);
        }
        if (ndviCameraPanel.getVisibility() == View.VISIBLE) {
            ndviCameraPanel.updateCameraSource(cameraIndex, lensType);
        }
        if (visualCameraPanel.getVisibility() == View.VISIBLE) {
            visualCameraPanel.updateCameraSource(cameraIndex, lensType);
        }
        if (autoExposureLockWidget.getVisibility() == View.VISIBLE) {
            autoExposureLockWidget.updateCameraSource(cameraIndex, lensType);
        }
        if (focusModeWidget.getVisibility() == View.VISIBLE) {
            focusModeWidget.updateCameraSource(cameraIndex, lensType);
        }
        if (focusExposureSwitchWidget.getVisibility() == View.VISIBLE) {
            focusExposureSwitchWidget.updateCameraSource(cameraIndex, lensType);
        }
        if (cameraControlsWidget.getVisibility() == View.VISIBLE) {
            cameraControlsWidget.updateCameraSource(cameraIndex, lensType);
        }
        if (exposureSettingsPanel.getVisibility() == View.VISIBLE) {
            exposureSettingsPanel.updateCameraSource(cameraIndex, lensType);
        }
        if (focalZoomWidget.getVisibility() == View.VISIBLE) {
            focalZoomWidget.updateCameraSource(cameraIndex, lensType);
        }
        if (horizontalSituationIndicatorWidget.getVisibility() == View.VISIBLE) {
            horizontalSituationIndicatorWidget.updateCameraSource(cameraIndex, lensType);
        }
    }

    private void updateViewVisibility(PhysicalDevicePosition devicePosition, CameraLensType lensType) {
        //只在fpv下显示
        pfvFlightDisplayWidget.setVisibility(devicePosition == PhysicalDevicePosition.NOSE ? View.VISIBLE : View.INVISIBLE);

        //fpv下不显示
        lensControlWidget.setVisibility(devicePosition == PhysicalDevicePosition.NOSE ? View.INVISIBLE : View.VISIBLE);
        ndviCameraPanel.setVisibility(devicePosition == PhysicalDevicePosition.NOSE ? View.INVISIBLE : View.VISIBLE);
        visualCameraPanel.setVisibility(devicePosition == PhysicalDevicePosition.NOSE ? View.INVISIBLE : View.VISIBLE);
        autoExposureLockWidget.setVisibility(devicePosition == PhysicalDevicePosition.NOSE ? View.INVISIBLE : View.VISIBLE);
        focusModeWidget.setVisibility(devicePosition == PhysicalDevicePosition.NOSE ? View.INVISIBLE : View.VISIBLE);
        focusExposureSwitchWidget.setVisibility(devicePosition == PhysicalDevicePosition.NOSE ? View.INVISIBLE : View.VISIBLE);
        cameraControlsWidget.setVisibility(devicePosition == PhysicalDevicePosition.NOSE ? View.INVISIBLE : View.VISIBLE);
        focalZoomWidget.setVisibility(devicePosition == PhysicalDevicePosition.NOSE ? View.INVISIBLE : View.VISIBLE);
        horizontalSituationIndicatorWidget.setSimpleModeEnable(devicePosition != PhysicalDevicePosition.NOSE);

        //有其他的显示逻辑，这里确保fpv下不显示
        if (devicePosition == PhysicalDevicePosition.NOSE) {
            exposureSettingsPanel.setVisibility(View.INVISIBLE);
        }

        //只在部分len下显示
        ndviCameraPanel.setVisibility(CameraUtil.isSupportForNDVI(lensType) ? View.VISIBLE : View.INVISIBLE);
    }

    /**
     * Swap the video sources of the FPV and secondary FPV widgets.
     */
    private void swapVideoSource() {
        VideoChannelType primaryVideoChannel = primaryFpvWidget.getVideoChannelType();
        StreamSource primaryStreamSource = primaryFpvWidget.getStreamSource();
        VideoChannelType secondaryVideoChannel = secondaryFPVWidget.getVideoChannelType();
        StreamSource secondaryStreamSource = secondaryFPVWidget.getStreamSource();
        //两个source都存在的情况下才进行切换
        if (secondaryStreamSource != null && primaryStreamSource != null) {
            primaryFpvWidget.updateVideoSource(secondaryStreamSource, secondaryVideoChannel);
            secondaryFPVWidget.updateVideoSource(primaryStreamSource, primaryVideoChannel);
        }
    }

    private void updateInteractionEnabled() {
        StreamSource newPrimaryStreamSource = primaryFpvWidget.getStreamSource();
        fpvInteractionWidget.setInteractionEnabled(false);
        if (newPrimaryStreamSource != null) {
            fpvInteractionWidget.setInteractionEnabled(newPrimaryStreamSource.getPhysicalDevicePosition() != PhysicalDevicePosition.NOSE);
        }
    }

    private static class CameraSource {
        PhysicalDevicePosition devicePosition;
        CameraLensType lensType;

        public CameraSource(PhysicalDevicePosition devicePosition, CameraLensType lensType) {
            this.devicePosition = devicePosition;
            this.lensType = lensType;
        }
    }

    @Override
    public void onBackPressed() {
        if (mDrawerLayout.isDrawerOpen(GravityCompat.END)) {
            mDrawerLayout.closeDrawers();
        } else {
            super.onBackPressed();
        }
    }

    private void loadMockParamFile() {
        mModel.getParams().clear();
        try {

            AssetManager am = getAssets();
            InputStream is = am.open("DJIMock.txt");
            InputStreamReader inputStreamReader = new InputStreamReader(is);
            BufferedReader br = new BufferedReader(inputStreamReader);

            String line;
            while ((line = br.readLine()) != null) {
                if (line.startsWith("#"))
                    continue;
                String[] paramData = line.split("\t");
                String paramName = paramData[2];
                float paramValue = Float.parseFloat(paramData[3]);
                short paramType = Short.parseShort(paramData[4]);

                mModel.getParams().add(new MAVParam(paramName, paramValue, paramType));
            }
        } catch (IOException e) {
            Log.d(TAG, "exception", e);
        }
    }

    //added
    private static class GCSSenderTimerTask extends TimerTask {

        private WeakReference<DefaultLayoutActivity> mainActivityWeakReference;

        GCSSenderTimerTask(WeakReference<DefaultLayoutActivity> mainActivityWeakReference) {
            this.mainActivityWeakReference = mainActivityWeakReference;

        }

        @Override
        public void run() {
            if (mainActivityWeakReference.get().mModel == null && mainActivityWeakReference.get().mModel.isTcpWorker) {

            } else {
                mainActivityWeakReference.get().mModel.tick();
            }
        }
    }

    private static class GCSCommunicatorAsyncTask extends AsyncTask<Integer, Integer, Integer> {

        private static final String TAG = GCSSenderTimerTask.class.getSimpleName();
        boolean request_renew_datalinks = true;
        private Timer timer;
        private WeakReference<DefaultLayoutActivity> mainActivityWeakReference;

        GCSCommunicatorAsyncTask(DefaultLayoutActivity mainActivity) {
            mainActivityWeakReference = new WeakReference<>(mainActivity);
        }

        void renewDatalinks() {
//            Log.d(TAG, "renewDataLinks");
            request_renew_datalinks = true;
            FLAG_TELEMETRY_ADDRESS_CHANGED = false;
        }

        private void onRenewDatalinks() {
//            Log.d(TAG, "onRenewDataLinks");
//            createTelemetrySocket();
            createTelemfetryTcpOutSocket();
//            mainActivityWeakReference.get().sendRestartVideoService();
        }

        @Override

        protected Integer doInBackground(Integer... ints2) {
            Log.d("RDTHREADS", "doInBackground()");

            try {
                createTelemetrySocket();
//                createTelemfetryTcpOutSocket();
                mainActivityWeakReference.get().mMavlinkParser = new Parser();

                GCSSenderTimerTask gcsSender = new GCSSenderTimerTask(mainActivityWeakReference);
                timer = new Timer(true);
                timer.scheduleAtFixedRate(gcsSender, 0, 100);


//                CameraImageSenderTimerTask cameraImageSenderTimerTask = new CameraImageSenderTimerTask(mainActivityWeakReference);
//                timer = new Timer(true);
//                timer.scheduleAtFixedRate(cameraImageSenderTimerTask, 0, 5000); // 2000


                while (!isCancelled()) {
                    // Listen for packets
                    try {
                        if (request_renew_datalinks) {//최초 실행시 이 조건문이 작동
                            request_renew_datalinks = false;
                            onRenewDatalinks();
                        }
                        if (System.currentTimeMillis() - mainActivityWeakReference.get().mReceiver.getTimestampLastGCSHeartbeat() <= mainActivityWeakReference.get().GCS_TIMEOUT_mSEC) {
                            if (!mainActivityWeakReference.get().shouldConnect) {
                                mainActivityWeakReference.get().shouldConnect = true;
                                mainActivityWeakReference.get().connectivityHasChanged = true;
                            }
                        } else {
                            if (mainActivityWeakReference.get().shouldConnect) {
                                mainActivityWeakReference.get().shouldConnect = false;
                                mainActivityWeakReference.get().connectivityHasChanged = true;
                            }
                        }

                        if (mainActivityWeakReference.get().connectivityHasChanged) {
                            if (mainActivityWeakReference.get().shouldConnect) {
                                final Drawable connectedDrawable = mainActivityWeakReference.get().getResources().getDrawable(R.drawable.ic_baseline_connected_24px, null);//연결 완료 UI 표시
                                mainActivityWeakReference.get().runOnUiThread(() -> {
                                    ImageView imageView = mainActivityWeakReference.get().findViewById(R.id.gcs_conn);
                                    imageView.setBackground(connectedDrawable);
                                    imageView.invalidate();


                                });
                                //라이브스트림 매니저는 setquality, setbitrate등의 메소드를 제공
                            } else {
                                final Drawable disconnectedDrawable = mainActivityWeakReference.get().getResources().getDrawable(R.drawable.ic_outline_disconnected_24px, null);

                                mainActivityWeakReference.get().runOnUiThread(() -> {
                                    ImageView imageView = mainActivityWeakReference.get().findViewById(R.id.gcs_conn);
                                    imageView.setBackground(disconnectedDrawable);
                                    imageView.invalidate();
                                });
                            }

                            mainActivityWeakReference.get().connectivityHasChanged = false;
                        }

                        if (mainActivityWeakReference.get().mModel.isTcpWorker) {
                            // TODO TCP Read.

                            int bytesAvailable = 0; // mainActivityWeakReference.get().in.available();
//                            int bytesAvailable = mainActivityWeakReference.get().bufferIn.available();
                            if (bytesAvailable == 0) {
//                                byte[] input = new byte[bytesAvailable];
                                byte[] input = new byte[16 * 1024];

                                int bytesRead = mainActivityWeakReference.get().bufferIn.read(input);


                                for (int i = 0; i < bytesRead; i++) {
                                    MAVLinkPacket packet = mainActivityWeakReference.get().mMavlinkParser.mavlink_parse_char(input[i] & 0xff);

                                    if (packet != null) {

                                        MAVLinkMessage msg = packet.unpack();
                                        if (mainActivityWeakReference.get().prefs.getBoolean("pref_log_mavlink", false))
                                            mainActivityWeakReference.get().logMessageFromGCS(msg.toString());
                                        mainActivityWeakReference.get().mReceiver.process(msg);
//                                        AsyncReadContextThread asyncContextThread = new AsyncReadContextThread(this.tcpWorkerLink
//                                                , packet
//                                                , addressTable);

//                                        //log.info("[--AsyncReadContextThread--] :: accept tcp Data...");
//                                        Thread thread = new Thread(asyncContextThread, "AsyncReadContextThread");
//                                        thread.start(); // async
                                    }
                                }
                            }
                        } else {
                            byte[] buf = new byte[1000];
                            DatagramPacket dp = new DatagramPacket(buf, buf.length);
                            mainActivityWeakReference.get().socket.receive(dp);

                            byte[] bytes = dp.getData();
                            int[] ints = new int[bytes.length];
                            for (int i = 0; i < bytes.length; i++)
                                ints[i] = bytes[i] & 0xff;

                            for (int i = 0; i < bytes.length; i++) {
                                MAVLinkPacket packet = mainActivityWeakReference.get().mMavlinkParser.mavlink_parse_char(ints[i]);

                                if (packet != null) {
                                    MAVLinkMessage msg = packet.unpack();
                                    if (mainActivityWeakReference.get().prefs.getBoolean("pref_log_mavlink", false))
                                        mainActivityWeakReference.get().logMessageFromGCS(msg.toString());
                                    mainActivityWeakReference.get().mReceiver.process(msg);
                                }
                            }
                        }

                    } catch (IOException e) {
                        //logMessageDJI("IOException: " + e.toString());
                    }
                }

            } catch (Exception e) {
                Log.d(TAG, "exception", e);
            } finally {
                if (mainActivityWeakReference.get().socket.isConnected()) {
                    mainActivityWeakReference.get().socket.disconnect();
                }
                if (timer != null) {
                    timer.cancel();
                }
                if (mainActivityWeakReference.get().mTcpSocket != null) {
                    try {
                        if (mainActivityWeakReference.get().mTcpSocket.isConnected()) {
                            mainActivityWeakReference.get().mTcpSocket.close();
                        }
                    } catch (IOException e) {
                        e.printStackTrace();
                    }
                    mainActivityWeakReference.get().mModel.isTcpWorker = false;
                }
//                Log.d("RDTHREADS", "doInBackground() complete");

            }
            return 0;
        }


        @Override
        protected void onPostExecute(Integer integer) {
            /*
            TODO Not sure what to do here...
             */
            if (mainActivityWeakReference.get() == null || mainActivityWeakReference.get().isFinishing())
                return;

            mainActivityWeakReference.clear();

        }

        @Override
        protected void onCancelled(Integer result) {
            super.onCancelled();

            close();

            final Drawable disconnectedDrawable = mainActivityWeakReference.get().getResources().getDrawable(R.drawable.ic_outline_disconnected_24px, null);

            mainActivityWeakReference.get().runOnUiThread(() -> {
                ImageView imageView = mainActivityWeakReference.get().findViewById(R.id.gcs_conn);
                imageView.setBackground(disconnectedDrawable);
                imageView.invalidate();
            });
        }

        @Override
        protected void onProgressUpdate(Integer... progress) {

        }

        private void createTelemfetryTcpOutSocket() {// GCS와 연결하는 부분
            close();

            String gcsIPString = "223.130.163.167";

            if (mainActivityWeakReference.get().prefs.getBoolean("pref_external_gcs", false)) {
                gcsIPString = mainActivityWeakReference.get().prefs.getString("pref_gcs_ip", "223.130.163.167");
            }
            int telemIPPort = Integer.parseInt(Objects.requireNonNull(mainActivityWeakReference.get().prefs.getString("pref_telem_port", "14550")));


            Log.d(TAG, "gcsIPString :: Host-IP:" + gcsIPString + " HostPort:" + telemIPPort);


            try {
                // TODO Server Socket.
//                InetAddress gcsIp = InetAddress.getByName(gcsIPString);
//                SocketAddress addr = new InetSocketAddress(gcsIp, telemIPPort);
//                mainActivityWeakReference.get().mTcpServerSocket = new ServerSocket();
                //                System.out.println("SO_REUSEADDR is enabled: " + this.myServerSocket.getReuseAddress());
//                System.out.println("SO_REUSEADDR is enabled: " + this.myServerSocket.getReuseAddress());
//                System.out.println("TCPLink :: is enabled IP :" + gcsIp.getHostAddress() + " port :" + this.myTcpConfiguration.get_localPort());
//                this.myServerSocket.setReuseAddress(true);
//                mainActivityWeakReference.get().mTcpServerSocket.bind(addr);
//                mainActivityWeakReference.get().mTcpServerSocket.setSoTimeout(880000);

                // TODO Client Socket.
                mainActivityWeakReference.get().mTcpSocket = new Socket();
                mainActivityWeakReference.get().mTcpSocket.setSoTimeout(5000);
                mainActivityWeakReference.get().mTcpSocket.connect(new InetSocketAddress(gcsIPString, telemIPPort));
                mainActivityWeakReference.get().isa = (InetSocketAddress) mainActivityWeakReference.get().mTcpSocket.getRemoteSocketAddress();
                mainActivityWeakReference.get().in = mainActivityWeakReference.get().mTcpSocket.getInputStream();
                mainActivityWeakReference.get().bufferIn = new BufferedInputStream(mainActivityWeakReference.get().in);

                Log.d(TAG, "mTcpSocket :: Success. Received-IP:" + mainActivityWeakReference.get().isa.getAddress() + " / Received-localport:" + mainActivityWeakReference.get().isa.getPort());


                mainActivityWeakReference.get().mModel.isTcpWorker = true;
                // exist Udp Out
//                mainActivityWeakReference.get().socket = new DatagramSocket();
//                mainActivityWeakReference.get().socket.connect(InetAddress.getByName(gcsIPString), telemIPPort);
//                mainActivityWeakReference.get().socket.setSoTimeout(10);

                mainActivityWeakReference.get().logMessageDJI("Starting GCS TCP Out telemetry link: " + gcsIPString + ":" + telemIPPort);
            } catch (SocketException e) {
                Log.d(TAG, "createTelemetrySocket() - socket exception");
                Log.d(TAG, "exception", e);
                mainActivityWeakReference.get().logMessageDJI("Telemetry socket exception: " + gcsIPString + ":" + telemIPPort);
            } // TODO
            catch (UnknownHostException e) {
                Log.d(TAG, "createTelemetrySocket() - unknown host exception");
                Log.d(TAG, "exception", e);
                mainActivityWeakReference.get().logMessageDJI("Unknown telemetry host: " + gcsIPString + ":" + telemIPPort);
            } // TODO
            catch (IOException e) {
                Log.d(TAG, "createTelemetrySocket() - io socket exception  unknown host exception");
                Log.d(TAG, "exception", e);
                mainActivityWeakReference.get().logMessageDJI("Unknown telemetry host: " + gcsIPString + ":" + telemIPPort);
                return;
            }
            if (mainActivityWeakReference.get() != null) {
                // Set Socket On Drone Model Object
//                mainActivityWeakReference.get().mModel.setSocket(mainActivityWeakReference.get().socket);
                mainActivityWeakReference.get().mModel.setTcpSocket(mainActivityWeakReference.get().mTcpSocket);

                if (mainActivityWeakReference.get().prefs.getBoolean("pref_secondary_telemetry_enabled", false)) {
                    String secondaryIP = mainActivityWeakReference.get().prefs.getString("pref_secondary_telemetry_ip", "127.0.0.1");
                    int secondaryPort = Integer.parseInt(Objects.requireNonNull(mainActivityWeakReference.get().prefs.getString("pref_secondary_telemetry_port", "18990")));
                    try {
                        DatagramSocket secondarySocket = new DatagramSocket();
                        secondarySocket.connect(InetAddress.getByName(secondaryIP), secondaryPort);
                        mainActivityWeakReference.get().logMessageDJI("Starting secondary telemetry link: " + secondaryIP + ":" + secondaryPort);

//                        mainActivityWeakReference.get().logMessageDJI(secondaryIP + ":" + secondaryPort);
                        mainActivityWeakReference.get().mModel.setSecondarySocket(secondarySocket);
                    } catch (SocketException | UnknownHostException e) {
                        e.printStackTrace();
                    }
                }
            }
        }

        private void createTelemetrySocket() {
            close();

            String gcsIPString = "223.130.163.167";

//            임시 주석처리
            if (mainActivityWeakReference.get().prefs.getBoolean("pref_external_gcs", false))
                gcsIPString = mainActivityWeakReference.get().prefs.getString("pref_gcs_ip", "127.0.0.1");
            int telemIPPort = Integer.parseInt(Objects.requireNonNull(mainActivityWeakReference.get().prefs.getString("pref_telem_port", "14550")));


            try {
                mainActivityWeakReference.get().socket = new DatagramSocket();
                mainActivityWeakReference.get().socket.connect(InetAddress.getByName(gcsIPString), telemIPPort);
                mainActivityWeakReference.get().socket.setSoTimeout(10);

                mainActivityWeakReference.get().logMessageDJI("Starting GCS telemetry link: " + gcsIPString + ":" + telemIPPort);
            } catch (SocketException e) {
                Log.d(TAG, "createTelemetrySocket() - socket exception");
                Log.d(TAG, "exception", e);
                mainActivityWeakReference.get().logMessageDJI("Telemetry socket exception: " + gcsIPString + ":" + telemIPPort);
            } // TODO
            catch (UnknownHostException e) {
                Log.d(TAG, "createTelemetrySocket() - unknown host exception");
                Log.d(TAG, "exception", e);
                mainActivityWeakReference.get().logMessageDJI("Unknown telemetry host: " + gcsIPString + ":" + telemIPPort);
            } // TODO

            if (mainActivityWeakReference.get() != null) {
                mainActivityWeakReference.get().mModel.setSocket(mainActivityWeakReference.get().socket);
                if (mainActivityWeakReference.get().prefs.getBoolean("pref_secondary_telemetry_enabled", false)) {
                    String secondaryIP = mainActivityWeakReference.get().prefs.getString("pref_secondary_telemetry_ip", "127.0.0.1");
                    int secondaryPort = Integer.parseInt(Objects.requireNonNull(mainActivityWeakReference.get().prefs.getString("pref_secondary_telemetry_port", "18990")));
                    try {
                        DatagramSocket secondarySocket = new DatagramSocket();
                        secondarySocket.connect(InetAddress.getByName(secondaryIP), secondaryPort);
                        mainActivityWeakReference.get().logMessageDJI("Starting secondary telemetry link: " + secondaryIP + ":" + secondaryPort);

//                        mainActivityWeakReference.get().logMessageDJI(secondaryIP + ":" + secondaryPort);
                        mainActivityWeakReference.get().mModel.setSecondarySocket(secondarySocket);
                    } catch (SocketException | UnknownHostException e) {
                        e.printStackTrace();
                    }
                }
            }
        }

        protected void close() {
            if (mainActivityWeakReference.get().socket != null) {
                mainActivityWeakReference.get().socket.disconnect();
                mainActivityWeakReference.get().socket.close();
            }
            if (mainActivityWeakReference.get().mTcpSocket != null) {
//                mainActivityWeakReference.get().mTcpSocket.disconnect();
                try {
                    mainActivityWeakReference.get().mTcpSocket.close();
                } catch (IOException e) {
                    e.printStackTrace();
                }
                mainActivityWeakReference.get().mModel.isTcpWorker = false;
            }
            if (mainActivityWeakReference.get().mModel != null) {
                if (mainActivityWeakReference.get().mModel.secondarySocket != null) {
                    Thread thread = new Thread(() -> {
                        mainActivityWeakReference.get().mModel.secondarySocket.disconnect();
                        mainActivityWeakReference.get().mModel.secondarySocket.close();
                    });
                    thread.start();

                }

            }
        }
    }

    private static class CameraImageSenderTimerTask extends TimerTask {

        private WeakReference<DefaultLayoutActivity> mainActivityWeakReference;

        CameraImageSenderTimerTask(WeakReference<DefaultLayoutActivity> mainActivityWeakReference) {
            this.mainActivityWeakReference = mainActivityWeakReference;
        }

        @Override
        public void run() {
            if (mainActivityWeakReference.get().ddmImageHandler != null)
                mainActivityWeakReference.get().ddmImageHandler.tick();
        }
    }

    public void startLiveStream() {
        //RTMP 스트리밍 기능 추가


        LiveStreamSettings rtmpSettings = new LiveStreamSettings.Builder()//라이브스트림세팅 객체 생성, 영상 프로토콜, URL정보가 입력되어 있음
                .setLiveStreamType(LiveStreamType.RTMP)
                .setRtmpSettings(new RtmpSettings.Builder()

                        .setUrl(streamAddress)
                        .build()
                ).build();

        iLiveStreamManager.setLiveStreamSettings(rtmpSettings);


        iLiveStreamManager.startStream(new CommonCallbacks.CompletionCallback() {
            @Override
            public void onSuccess() {
                streamingButton.setText("now\nstreaming");
                toast("스트리밍 시작함");
                Log("스트리밍 시작함");
            }

            @Override
            public void onFailure(@NonNull IDJIError idjiError) {
                toast("스트리밍 못함");
                Log("스트리밍 못함");
            }
        });
    }

    public void stopStream() {
        iLiveStreamManager.stopStream(new CommonCallbacks.CompletionCallback() {
            @Override
            public void onSuccess() {
                streamingButton.setText("start\nstream");
                toast("스트리밍 멈춤");

            }

            @Override
            public void onFailure(@NonNull IDJIError idjiError) {
                toast("스트리밍 멈춤 실패");

            }
        });
    }

    private void closeGCSCommunicator() {
        if (mGCSCommunicator != null) {
            mGCSCommunicator.cancel(true);
            mGCSCommunicator = null;
            Log("closeGCSCommunicator");
        }
    }

    public void logMessageFromGCS(String msg) {
        Log(msg);
    }

    public void logMessageDJI(String msg) {
        Log.d(TAG, msg);
        if (mNewDJI.length() > 1000)
            mNewDJI = mNewDJI.substring(500, 1000);

        mNewDJI += "\n" + msg;
    }

    public void Log(String input) {
        Log.i(TAG, input);
    }

    public void toast(String input) {
        Toast.makeText(getApplicationContext(), input, Toast.LENGTH_SHORT).show();
    }


}
