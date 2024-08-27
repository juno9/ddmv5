package dji.v5.ux.sample.util;


import android.app.Dialog;
import android.content.Context;
import android.os.Bundle;

import android.view.View;
import android.widget.Button;

import androidx.annotation.NonNull;

import dji.v5.ux.R;

public class ImageDialog extends Dialog {

    private ImageDialogInterface ImageDialogInterface;
    private Button download, delete;

    public ImageDialog(@NonNull Context context) {
        super(context);
    }

    public interface ImageDialogInterface {
        void startBtnClicked();
        void stopBtnClicked();

    }

    public void setDialogListener(ImageDialog.ImageDialogInterface streamDialogInterface) {
        this.ImageDialogInterface = streamDialogInterface;
    }

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        setContentView(R.layout.uxsdk_dialog_image);
        download = findViewById(R.id.btn_download_image);
        download.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                ImageDialogInterface.startBtnClicked();
            }
        });

        delete = findViewById(R.id.btn_delete_images);
        delete.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                ImageDialogInterface.stopBtnClicked();
            }
        });



    }


}
