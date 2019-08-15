package i3d.native0701;

import android.content.Intent;
import android.graphics.Bitmap;
import android.os.Bundle;
import android.support.v7.app.AppCompatActivity;
import android.view.MotionEvent;
import android.view.View;
import android.widget.Button;
import android.widget.ImageView;
import android.widget.TextView;

public class MainActivity extends AppCompatActivity {

    static {
        System.loadLibrary("native-lib");
    }

    private ImageView imageView;
    private Bitmap bitmap;
    private TextView textView;

    private Button mButtonGLViewer;

    String javaString = "/sdcard/000i3d2";
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);



        textView = findViewById(R.id.text_view);
        imageView = findViewById(R.id.image_view);
        bitmap = Bitmap.createBitmap(903, 1203, Bitmap.Config.ARGB_8888);
        int szOfFrame = getImage(bitmap, javaString);
        imageView.setImageBitmap(bitmap);

        String str = ("frames.size(): ");
        textView.setText(str + szOfFrame);

        imageView.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                bitmap = Bitmap.createBitmap(903, 1203, Bitmap.Config.ARGB_8888);
                getImage(bitmap, javaString);
                imageView.setImageBitmap(bitmap);
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


    public static native int getImage(Bitmap bitmap, String javaString);
}
