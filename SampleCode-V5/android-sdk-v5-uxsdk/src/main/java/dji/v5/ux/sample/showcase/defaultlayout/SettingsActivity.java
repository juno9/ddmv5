package dji.v5.ux.sample.showcase.defaultlayout;

import android.content.SharedPreferences;
import android.os.Bundle;
import android.preference.PreferenceManager;
import android.widget.EditText;
import android.widget.Switch;

import androidx.appcompat.app.AppCompatActivity;

import dji.v5.ux.R;

public class SettingsActivity extends AppCompatActivity {
    private EditText edt_id, edt_ip, edt_port;
    private Switch sw_exgcs;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.uxsdk_activity_settings);

        SharedPreferences prefs = PreferenceManager.getDefaultSharedPreferences(SettingsActivity.this);
        SharedPreferences.Editor editor = prefs.edit();

        String id = prefs.getString("pref_drone_id", "3");
        String ip = prefs.getString("pref_gcs_ip", "223.130.163.167");
        String port = prefs.getString("pref_telem_port", "6760");
        boolean use_external_gcs = prefs.getBoolean("pref_external_gcs", false);
        edt_id = findViewById(R.id.drone_id);
        edt_ip = findViewById(R.id.drone_gcs_ip);
        edt_port = findViewById(R.id.drone_gcs_port);
        sw_exgcs =findViewById(R.id.drone_use_external_gcs);
        edt_id.setText(id);
        edt_ip.setText(ip);
        edt_port.setText(port);
        sw_exgcs.setChecked(use_external_gcs);

    }

    @Override
    protected void onPause() {
        super.onPause();
        SharedPreferences prefs = PreferenceManager.getDefaultSharedPreferences(SettingsActivity.this);
        SharedPreferences.Editor editor = prefs.edit();
        editor.putString("pref_drone_id", edt_id.getText().toString());
        editor.putString("pref_gcs_ip", edt_ip.getText().toString());
        editor.putString("pref_telem_port", edt_port.getText().toString());
        editor.putBoolean("pref_external_gcs",sw_exgcs.isChecked());
        editor.commit();
    }
}
