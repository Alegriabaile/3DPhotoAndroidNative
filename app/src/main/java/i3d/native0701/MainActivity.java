package i3d.native0701;

import android.content.Intent;
import android.graphics.Bitmap;
import android.os.Bundle;
import android.support.v7.app.AppCompatActivity;
import android.util.Log;
import android.view.MotionEvent;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import android.widget.ImageView;
import android.widget.TextView;


public class MainActivity extends AppCompatActivity {

    static {
        System.loadLibrary("native-lib");
    }

    public static int COLS;
    public static int ROWS;

    private EditText editText;
    private ImageView imageView;
    private Bitmap bitmap;
    private TextView textView;

    private Button mButtonGenerate3d, mButtonGLViewer;

    String javaString = "/sdcard/000i3dc3";

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        editText = findViewById(R.id.edittext_dir);
        editText.setText(javaString);

        imageView = findViewById(R.id.image_view);
        mButtonGenerate3d = findViewById(R.id.button_generate3d);

        mButtonGenerate3d.setOnClickListener(new View.OnClickListener(){

            @Override
            public void onClick(View v) {

                javaString = editText.getText().toString();

                int szOfFrame = processI3d(javaString);
                ROWS = getImageH();
                COLS = getImageW();
                if(ROWS <= 0 || COLS <= 0)
                {
                    Log.e("MainActivity.OnCreate: ", "ROWS or COLS not positive...");
                    return;
                }

                bitmap = Bitmap.createBitmap(COLS, ROWS, Bitmap.Config.ARGB_8888);
                szOfFrame = getImage(bitmap);
                imageView.setImageBitmap(bitmap);

                imageView.setOnClickListener(new View.OnClickListener() {
                    @Override
                    public void onClick(View v) {
                        bitmap = Bitmap.createBitmap(COLS, ROWS, Bitmap.Config.ARGB_8888);
                        getImage(bitmap);
                        imageView.setImageBitmap(bitmap);
                    }
                });

            }
        });


        mButtonGLViewer = findViewById(R.id.button_glviewer);
        mButtonGLViewer.setOnClickListener(new View.OnClickListener(){
            @Override
            public void onClick(View v) {
                Intent intent = new Intent(MainActivity.this, GLViewerActivity.class);
                startActivity(intent);
            }
        });

    }


    public static native int processI3d(String javaString);

    public static native int getImageW();
    public static native int getImageH();

    public static native int getImage(Bitmap bitmap);
}
