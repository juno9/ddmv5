package dji.v5.ux.sample.util;

import android.annotation.SuppressLint;
import android.app.Dialog;
import android.content.Context;
import android.content.SharedPreferences;
import android.os.Bundle;
import android.text.Editable;
import android.text.TextWatcher;
import android.util.Log;
import android.view.View;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.EditText;
import android.widget.RadioButton;
import android.widget.RadioGroup;
import android.widget.SeekBar;
import android.widget.Spinner;
import android.widget.TextView;

import androidx.annotation.NonNull;

import java.util.List;

import dji.sdk.keyvalue.value.airlink.VideoSourceEntity;
import dji.v5.common.callback.CommonCallbacks;
import dji.v5.common.error.IDJIError;
import dji.v5.manager.datacenter.MediaDataCenter;
import dji.v5.manager.datacenter.camera.CameraStreamManager;
import dji.v5.manager.datacenter.livestream.LiveStreamManager;
import dji.v5.manager.datacenter.livestream.LiveStreamSettings;
import dji.v5.manager.datacenter.livestream.LiveStreamType;
import dji.v5.manager.datacenter.livestream.settings.RtmpSettings;
import dji.v5.ux.R;
import dji.v5.ux.sample.showcase.defaultlayout.DefaultLayoutActivity;

public class StreamDialog extends Dialog {
    private StreamDialogInterface streamDialogInterface;
    private static final String TAG = "StreamDialog";
    private EditText edtAddress;

    private Button start, stop;
    SeekBar sbBitrate;
    private TextView tvBitrate;

    private RadioGroup cameraGroup,qualityGroup, bitrateGroup;

    private OnEditTextChangedListener editTextChangedListener;


    public interface StreamDialogInterface {
        void startBtnClicked();

        void stopBtnClicked();

        void etdStreamAddress();

    }

    public StreamDialog(@NonNull Context context) {
        super(context);

    }

    public void setDialogListener(StreamDialogInterface streamDialogInterface) {
        this.streamDialogInterface = streamDialogInterface;
    }


    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        setContentView(R.layout.uxsdk_dialog_streaming);
        start = findViewById(R.id.btn_start);
        start.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                streamDialogInterface.startBtnClicked();
            }
        });
        stop = findViewById(R.id.btn_stop);
        stop.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                streamDialogInterface.stopBtnClicked();
            }
        });

        edtAddress = findViewById(R.id.edt_rtmpaddress);
        edtAddress.addTextChangedListener(new TextWatcher() {
            @Override
            public void beforeTextChanged(CharSequence s, int start, int count, int after) { }

            @Override
            public void onTextChanged(CharSequence s, int start, int before, int count) {
                if (editTextChangedListener != null) {
                    editTextChangedListener.onEditTextChanged(s.toString());
                }
            }

            @Override
            public void afterTextChanged(Editable s) { }
        });
        cameraGroup = findViewById(R.id.rg_camera);
        qualityGroup = findViewById(R.id.rg_quality);
        bitrateGroup = findViewById(R.id.rg_bit_rate);
        sbBitrate = findViewById(R.id.sb_bit_rate);
        tvBitrate= findViewById(R.id.tv_bit_rate);

    }


    public SeekBar getsbBitrate() {
        return sbBitrate;
    }

    public void setTvBitrate(TextView tvBitrate) {
        this.tvBitrate = tvBitrate;
    }

    public TextView getTvBitrate() {
        return tvBitrate;
    }

    public void setOnEditTextChangedListener(OnEditTextChangedListener listener) {
        this.editTextChangedListener = listener;
    }
    public interface OnEditTextChangedListener {
        void onEditTextChanged(String newText);
    }





    public void setStreamAddress(String Address) {
       edtAddress.setText(Address);
    }

    public RadioGroup getCameraGroup() {
        return cameraGroup;
    }


    public RadioGroup getQualityGroup() {
        return qualityGroup;
    }

    public RadioGroup getBitrateGroup() {
        return bitrateGroup;
    }
}
